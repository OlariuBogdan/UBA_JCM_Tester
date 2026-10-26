#!/usr/bin/env python3
"""
UBA Tester - testare bill acceptor JCM UBA (protocol ID-003) prin ESP32
=======================================================================
PC  <--USB-->  ESP32 (punte, firmware esp32_id003_bridge.ino)  <--UART 9600 8E1-->  UBA

Instalare:   pip install pyserial
Pornire:     python uba_tester.py

Ce face:
  - comunicație, reset, inițializare, citire versiune firmware și tabel valută
  - activează acceptarea și urmărește fiecare bancnotă introdusă
  - în modul "Returnare" bancnota recunoscută este dată înapoi (pentru capete fără casetă)
  - contorizează pe valori, motivele de respingere și erorile hardware
  - checklist de teste OK / EROARE și raport salvabil
"""

import queue
import threading
import time
import datetime
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # pragma: no cover
    raise SystemExit("Lipsește pyserial. Rulează:  pip install pyserial")

# ---------------------------------------------------------------------------
# Protocol ID-003
# ---------------------------------------------------------------------------
SYNC = 0xFC

# Comenzi host -> acceptor
CMD_STATUS = 0x11
CMD_RESET = 0x40
CMD_STACK1 = 0x41
CMD_STACK2 = 0x42
CMD_RETURN = 0x43
CMD_HOLD = 0x44
CMD_WAIT = 0x45
CMD_ACK = 0x50
CMD_SET_ENABLE = 0xC0       # 2 octeți, bit=1 -> valoare inhibată
CMD_SET_SECURITY = 0xC1     # 2 octeți
CMD_SET_COMM_MODE = 0xC2    # 1 octet, 0 = polling
CMD_SET_INHIBIT = 0xC3      # 1 octet, 0 = acceptare permisă, 1 = inhibat
CMD_SET_DIRECTION = 0xC4    # 1 octet, 0 = toate direcțiile
CMD_SET_OPTIONAL = 0xC5     # 2 octeți
CMD_VERSION = 0x88
CMD_BOOT_VERSION = 0x89
CMD_CURRENCY = 0x8A

# Stări / răspunsuri acceptor
ST = {
    0x11: "Idle (așteaptă bancnotă)",
    0x12: "Accepting (trage bancnota)",
    0x13: "Escrow (bancnotă recunoscută)",
    0x14: "Stacking (stivuire)",
    0x15: "Vend valid",
    0x16: "Stacked (stivuită)",
    0x17: "Rejecting (respinge)",
    0x18: "Returning (returnează)",
    0x19: "Holding",
    0x1A: "Disabled (inhibat)",
    0x1B: "Initialize (inițializare)",
    0x40: "Power up",
    0x41: "Power up cu bancnotă în cap",
    0x42: "Power up cu bancnotă în stacker",
    0x43: "EROARE: stacker plin",
    0x44: "EROARE: stacker/casetă deschisă",
    0x45: "EROARE: blocaj în acceptor (JAM)",
    0x46: "EROARE: blocaj în stacker (JAM)",
    0x47: "Pauză (a doua bancnotă introdusă)",
    0x48: "EROARE: tentativă de fraudă (cheated)",
    0x49: "EROARE: defect (failure)",
    0x4A: "EROARE: comunicație",
    0x4B: "Comandă invalidă",
    0x50: "ACK",
}
POWER_UP = (0x40, 0x41, 0x42)
ERRORS = (0x43, 0x44, 0x45, 0x46, 0x48, 0x49, 0x4A)

REJECT_REASONS = {
    0x71: "Eroare la introducere",
    0x72: "Eroare model magnetic",
    0x73: "Bancnotă rămasă în cap",
    0x74: "Eroare de calibrare / amplificare",
    0x75: "Eroare de transport",
    0x76: "Eroare discriminare valoare",
    0x77: "Eroare model optic 1",
    0x78: "Eroare nivel optic",
    0x79: "Returnată - valoare inhibată",
    0x7A: "Respinsă la escrow",
    0x7B: "Eroare de operare",
    0x7C: "Bancnotă rămasă în stacker",
    0x7D: "Eroare lungime",
    0x7E: "Eroare model optic 2",
    0x7F: "Eroare caracteristică de autenticitate",
}

FAILURE_CODES = {
    0xA2: "Defect motor stacker",
    0xA5: "Viteză motor transport incorectă",
    0xA6: "Defect motor transport",
    0xA8: "Defect solenoid",
    0xA9: "Defect unitate PB",
    0xAB: "Caseta nu e pregătită",
    0xAF: "Capul acceptorului scos/deschis",
    0xB0: "Defect BOOT ROM",
    0xB1: "Defect ROM extern",
    0xB2: "Defect RAM",
    0xB3: "Eroare scriere ROM extern",
}


def crc16(data: bytes) -> int:
    """CRC-16 CCITT reflectat (Kermit), init 0 - folosit de ID-003."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc


def build_frame(cmd: int, data: bytes = b"") -> bytes:
    body = bytes([SYNC, 5 + len(data), cmd]) + data
    c = crc16(body)
    return body + bytes([c & 0xFF, c >> 8])


def hexs(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


class FrameReader:
    """Extrage cadre valide din fluxul de octeți (resincronizare pe 0xFC + CRC)."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf += data
        frames = []
        while True:
            i = self.buf.find(bytes([SYNC]))
            if i < 0:
                self.buf.clear()
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < 2:
                break
            ln = self.buf[1]
            if ln < 5 or ln > 250:
                del self.buf[0]
                continue
            if len(self.buf) < ln:
                break
            fr = bytes(self.buf[:ln])
            c = crc16(fr[:-2])
            if fr[-2] == (c & 0xFF) and fr[-1] == (c >> 8):
                frames.append(fr)
                del self.buf[:ln]
            else:
                del self.buf[0]
        return frames


# ---------------------------------------------------------------------------
# Logica de test (rulează într-un thread separat)
# ---------------------------------------------------------------------------
class UbaWorker(threading.Thread):
    POLL_S = 0.15
    TIMEOUT_S = 0.4

    def __init__(self, port, out_q: queue.Queue, cmd_q: queue.Queue, return_mode_fn):
        super().__init__(daemon=True)
        self.port = port
        self.out = out_q
        self.cmds = cmd_q
        self.return_mode = return_mode_fn
        self.ser = None
        self.reader = FrameReader()
        self.running = True
        self.enabled = False
        self.prev_state = None
        self.escrow_handled = False
        self.denoms = {}          # cod escrow -> text valoare
        self.comm_fail = 0

    # ---- utilitare ----
    def emit(self, kind, **kw):
        self.out.put((kind, kw))

    def log(self, msg, level="info"):
        self.emit("log", msg=msg, level=level)

    def transact(self, cmd, data=b"", expect_reply=True):
        frame = build_frame(cmd, data)
        self.ser.reset_input_buffer()
        self.reader.buf.clear()
        self.ser.write(frame)
        self.emit("raw", dir=">>", data=frame)
        if not expect_reply:
            return None
        t0 = time.time()
        while time.time() - t0 < self.TIMEOUT_S:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if chunk:
                frames = self.reader.feed(chunk)
                if frames:
                    fr = frames[0]
                    self.emit("raw", dir="<<", data=fr)
                    self.comm_fail = 0
                    return fr[2], fr[3:-2]
        self.comm_fail += 1
        return None

    def set_cmd(self, cmd, data, name):
        r = self.transact(cmd, data)
        ok = r is not None and r[0] == cmd
        self.log(f"Setare {name}: {'confirmată' if ok else 'FĂRĂ CONFIRMARE'}",
                 "ok" if ok else "warn")
        return ok

    # ---- secvențe ----
    def initialize(self):
        self.emit("test", key="comm", state="run")
        r = self.transact(CMD_STATUS)
        if r is None:
            r = self.transact(CMD_STATUS)
        if r is None:
            self.emit("test", key="comm", state="fail",
                      note="Niciun răspuns. Verifică TX/RX, alimentarea, jumperii 3.3V.")
            self.log("UBA nu răspunde la Status Request.", "err")
            return False
        self.emit("test", key="comm", state="ok", note=f"Stare: {ST.get(r[0], hex(r[0]))}")
        self.log(f"Comunicație OK - stare inițială: {ST.get(r[0], hex(r[0]))}", "ok")

        # Reset
        self.emit("test", key="reset", state="run")
        r = self.transact(CMD_RESET)
        if r is None or r[0] != CMD_ACK:
            self.log("Reset fără ACK (continui oricum).", "warn")
        t0 = time.time()
        st = None
        while time.time() - t0 < 20:
            time.sleep(0.3)
            r = self.transact(CMD_STATUS)
            if r is None:
                continue
            st = r[0]
            self.emit("state", code=st, data=r[1])
            if st in (0x1A, 0x11):
                break
            if st in ERRORS:
                break
        if st in (0x1A, 0x11):
            self.emit("test", key="reset", state="ok",
                      note=f"Inițializat în {time.time() - t0:.1f}s")
            self.log("Reset și inițializare reușite (motor/senzori verificați la pornire).", "ok")
        else:
            reason = self.describe(st, b"") if st else "fără răspuns"
            self.emit("test", key="reset", state="fail", note=reason)
            self.log(f"Inițializarea a eșuat: {reason}", "err")
            if st == 0x49:
                r = self.transact(CMD_STATUS)
                if r:
                    self.log(self.describe(r[0], r[1]), "err")
            return False

        # Configurare
        self.emit("test", key="config", state="run")
        ok = all([
            self.set_cmd(CMD_SET_ENABLE, b"\x00\x00", "valori permise (toate)"),
            self.set_cmd(CMD_SET_SECURITY, b"\x00\x00", "securitate (standard)"),
            self.set_cmd(CMD_SET_COMM_MODE, b"\x00", "mod comunicație (polling)"),
            self.set_cmd(CMD_SET_INHIBIT, b"\x01", "inhibare (oprit până la Start)"),
            self.set_cmd(CMD_SET_DIRECTION, b"\x00", "direcții (toate 4)"),
            self.set_cmd(CMD_SET_OPTIONAL, b"\x00\x00", "funcții opționale"),
        ])
        self.emit("test", key="config", state="ok" if ok else "warn",
                  note="Toate setările confirmate" if ok else "Unele setări neconfirmate")

        self.read_version()
        self.read_currency()
        self.enabled = False
        return True

    def read_version(self):
        self.emit("test", key="version", state="run")
        r = self.transact(CMD_VERSION)
        if r and r[0] == CMD_VERSION:
            v = r[1].decode("ascii", "replace").strip()
            self.emit("version", text=v)
            self.emit("test", key="version", state="ok", note=v)
            self.log(f"Firmware: {v}", "ok")
        else:
            self.emit("test", key="version", state="fail", note="Nu s-a putut citi")

    def read_currency(self):
        self.emit("test", key="currency", state="run")
        r = self.transact(CMD_CURRENCY)
        if not r or r[0] != CMD_CURRENCY or len(r[1]) < 4:
            self.emit("test", key="currency", state="warn",
                      note="Tabel indisponibil - valorile apar ca cod escrow")
            return
        d = r[1]
        self.denoms = {}
        for i in range(0, len(d) - 3, 4):
            code, country, base, exp = d[i], d[i + 1], d[i + 2], d[i + 3]
            if base == 0:
                continue
            value = base * (10 ** exp)
            self.denoms[code] = f"{value} (țară 0x{country:02X})"
        self.emit("denoms", table=dict(self.denoms))
        self.emit("test", key="currency", state="ok",
                  note=f"{len(self.denoms)} valori: " +
                       ", ".join(v.split()[0] for v in self.denoms.values()))
        self.log("Tabel valută: " + "; ".join(f"0x{k:02X}={v}" for k, v in self.denoms.items()), "ok")

    def describe(self, st, data):
        text = ST.get(st, f"Stare necunoscută 0x{st:02X}")
        if st == 0x17 and data:
            text += " - " + REJECT_REASONS.get(data[0], f"motiv 0x{data[0]:02X}")
        if st == 0x49 and data:
            text += " - " + FAILURE_CODES.get(data[0], f"cod 0x{data[0]:02X}")
        if st == 0x13 and data:
            text += " - valoare " + self.denoms.get(data[0], f"cod 0x{data[0]:02X}")
        return text

    def handle_state(self, st, data):
        changed = st != self.prev_state
        self.emit("state", code=st, data=data)

        if st == 0x13 and data:                          # Escrow
            if not self.escrow_handled:
                code = data[0]
                self.emit("escrow", code=code, name=self.denoms.get(code, f"cod 0x{code:02X}"))
                self.log(f"Recunoscută: {self.denoms.get(code, f'cod 0x{code:02X}')}", "ok")
                self.escrow_handled = True
            cmd = CMD_RETURN if self.return_mode() else CMD_STACK1
            self.transact(cmd)
        elif st == 0x15:                                 # Vend valid -> ACK
            self.transact(CMD_ACK, expect_reply=False)
        elif st == 0x16 and changed:
            self.emit("cycle", what="stacked")
        elif st == 0x17 and changed:                     # Rejecting
            reason = data[0] if data else 0
            self.emit("reject", code=reason,
                      name=REJECT_REASONS.get(reason, f"motiv 0x{reason:02X}"))
            self.log("Respinsă: " + REJECT_REASONS.get(reason, f"motiv 0x{reason:02X}"), "warn")
        elif st in ERRORS and changed:
            msg = self.describe(st, data)
            self.emit("error", code=st, text=msg)
            self.log(msg, "err")
        elif st in POWER_UP and changed:
            self.log("Acceptorul a repornit (power up) - reinițializez.", "warn")
            if self.initialize() and self.enabled:
                self.set_cmd(CMD_SET_INHIBIT, b"\x00", "acceptare activă")

        if changed and self.prev_state in (0x13, 0x18, 0x14, 0x15, 0x16) and st in (0x11, 0x1A):
            self.emit("cycle", what="done")
        if st not in (0x13, 0x14, 0x15, 0x18, 0x19):
            self.escrow_handled = False
        self.prev_state = st

    def run(self):
        try:
            self.ser = serial.Serial()
            self.ser.port = self.port
            self.ser.baudrate = 115200
            self.ser.timeout = 0.05
            self.ser.dtr = False      # evită resetarea ESP32 la deschidere
            self.ser.rts = False
            self.ser.open()
        except Exception as e:
            self.emit("conn", ok=False, msg=str(e))
            return
        self.emit("conn", ok=True, msg=self.port)
        time.sleep(1.5)               # ESP32 poate porni după deschiderea portului
        self.ser.reset_input_buffer()

        self.initialize()

        while self.running:
            try:
                cmd = self.cmds.get_nowait()
            except queue.Empty:
                cmd = None
            try:
                if cmd == "reset":
                    was = self.enabled
                    if self.initialize() and was:
                        self.set_cmd(CMD_SET_INHIBIT, b"\x00", "acceptare activă")
                elif cmd == "start":
                    if self.set_cmd(CMD_SET_INHIBIT, b"\x00", "acceptare activă"):
                        self.enabled = True
                        self.emit("enabled", on=True)
                elif cmd == "stop":
                    self.set_cmd(CMD_SET_INHIBIT, b"\x01", "acceptare oprită")
                    self.enabled = False
                    self.emit("enabled", on=False)
                elif cmd == "version":
                    self.read_version()
                    self.read_currency()

                r = self.transact(CMD_STATUS)
                if r:
                    self.handle_state(r[0], r[1])
                elif self.comm_fail == 5:
                    self.log("Comunicația s-a pierdut (5 interogări fără răspuns).", "err")
                    self.emit("error", code=0, text="Pierdere comunicație")
            except serial.SerialException as e:
                self.emit("conn", ok=False, msg=str(e))
                break
            time.sleep(self.POLL_S)
        try:
            self.ser.close()
        except Exception:
            pass

    def stop(self):
        self.running = False


# ---------------------------------------------------------------------------
# Interfața grafică
# ---------------------------------------------------------------------------
TESTS = [
    ("comm", "1. Comunicație serială ID-003"),
    ("reset", "2. Reset și inițializare (motor + senzori la pornire)"),
    ("config", "3. Configurare acceptor"),
    ("version", "4. Versiune firmware"),
    ("currency", "5. Tabel valută (ce bancnote cunoaște)"),
    ("transport", "6. Transport bancnotă (ciclu complet fără blocaj)"),
    ("recognition", "7. Rată de recunoaștere"),
    ("hardware", "8. Fără erori hardware în sesiune"),
]
STATE_LABEL = {"ok": "✔ OK", "fail": "✖ EROARE", "warn": "⚠ ATENȚIE", "run": "… rulează", "": "—"}
STATE_TAG = {"ok": "ok", "fail": "fail", "warn": "warn", "run": "run", "": "idle"}


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("UBA Tester - JCM ID-003")
        self.geometry("1180x760")
        self.minsize(980, 640)

        self.out_q = queue.Queue()
        self.cmd_q = queue.Queue()
        self.worker = None
        self.return_mode = tk.BooleanVar(value=True)
        self.show_hex = tk.BooleanVar(value=False)
        self.threshold = tk.IntVar(value=90)
        self.head_sn = tk.StringVar()
        self.port_var = tk.StringVar()
        self.reset_stats()

        self.build_ui()
        self.refresh_ports()
        self.after(50, self.pump)

    # ---- statistică ----
    def reset_stats(self):
        self.stats = {"recognized": 0, "rejected": 0, "cycles": 0, "errors": 0}
        self.by_denom = {}
        self.by_reason = {}
        self.error_list = []
        self.firmware = ""
        self.test_state = {k: ("", "") for k, _ in TESTS}

    # ---- UI ----
    def build_ui(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Big.TLabel", font=("Segoe UI", 15, "bold"))
        style.configure("Num.TLabel", font=("Segoe UI", 20, "bold"))
        style.configure("Treeview", rowheight=24)

        top = ttk.Frame(self, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="Port:").pack(side="left")
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var, width=28, state="readonly")
        self.port_cb.pack(side="left", padx=4)
        ttk.Button(top, text="⟳", width=3, command=self.refresh_ports).pack(side="left")
        self.btn_conn = ttk.Button(top, text="Conectare", command=self.toggle_conn)
        self.btn_conn.pack(side="left", padx=6)
        ttk.Label(top, text="Serie cap:").pack(side="left", padx=(16, 2))
        ttk.Entry(top, textvariable=self.head_sn, width=18).pack(side="left")
        ttk.Checkbutton(top, text="Returnează bancnota (cap fără casetă)",
                        variable=self.return_mode).pack(side="left", padx=16)

        bar = ttk.Frame(self, padding=(8, 0, 8, 6))
        bar.pack(fill="x")
        self.btn_start = ttk.Button(bar, text="▶ Start acceptare", command=lambda: self.send("start"))
        self.btn_stop = ttk.Button(bar, text="■ Stop", command=lambda: self.send("stop"))
        self.btn_reset = ttk.Button(bar, text="Reset UBA", command=lambda: self.send("reset"))
        self.btn_ver = ttk.Button(bar, text="Citește versiune/valută", command=lambda: self.send("version"))
        for b in (self.btn_start, self.btn_stop, self.btn_reset, self.btn_ver):
            b.pack(side="left", padx=3)
        ttk.Button(bar, text="Golește statistica", command=self.clear_stats).pack(side="left", padx=12)
        ttk.Button(bar, text="Salvează raport", command=self.save_report).pack(side="left")
        ttk.Label(bar, text="Prag OK recunoaștere %:").pack(side="left", padx=(16, 2))
        ttk.Spinbox(bar, from_=50, to=100, textvariable=self.threshold, width=5,
                    command=self.update_recognition).pack(side="left")
        self.set_buttons(False)

        # Stare curentă
        st = ttk.Frame(self, padding=(8, 2))
        st.pack(fill="x")
        self.lbl_state = tk.Label(st, text="Neconectat", font=("Segoe UI", 15, "bold"),
                                  bg="#555", fg="white", padx=14, pady=8, anchor="w")
        self.lbl_state.pack(fill="x")

        main = ttk.PanedWindow(self, orient="horizontal")
        main.pack(fill="both", expand=True, padx=8, pady=6)

        # Stânga: teste + totaluri
        left = ttk.Frame(main)
        main.add(left, weight=3)

        lf = ttk.LabelFrame(left, text="Teste", padding=4)
        lf.pack(fill="x")
        self.tv_tests = ttk.Treeview(lf, columns=("test", "res", "note"), show="headings", height=8)
        for col, txt, w in (("test", "Test", 330), ("res", "Rezultat", 100), ("note", "Detalii", 300)):
            self.tv_tests.heading(col, text=txt)
            self.tv_tests.column(col, width=w, anchor="w")
        self.tv_tests.tag_configure("ok", foreground="#127a2e")
        self.tv_tests.tag_configure("fail", foreground="#c0161c")
        self.tv_tests.tag_configure("warn", foreground="#b56a00")
        self.tv_tests.tag_configure("run", foreground="#1d5fbf")
        self.tv_tests.tag_configure("idle", foreground="#777")
        self.tv_tests.pack(fill="x")
        for k, name in TESTS:
            self.tv_tests.insert("", "end", iid=k, values=(name, "—", ""), tags=("idle",))

        tot = ttk.LabelFrame(left, text="Totaluri", padding=6)
        tot.pack(fill="x", pady=6)
        self.lbl_tot = {}
        for i, (k, t) in enumerate((("recognized", "Recunoscute"), ("rejected", "Respinse"),
                                    ("rate", "Rată recunoaștere"), ("errors", "Erori hardware"))):
            f = ttk.Frame(tot)
            f.grid(row=0, column=i, sticky="ew", padx=10)
            tot.columnconfigure(i, weight=1)
            ttk.Label(f, text=t).pack()
            self.lbl_tot[k] = ttk.Label(f, text="0", style="Num.TLabel")
            self.lbl_tot[k].pack()

        tables = ttk.Frame(left)
        tables.pack(fill="both", expand=True)
        d = ttk.LabelFrame(tables, text="Pe valori", padding=4)
        d.pack(side="left", fill="both", expand=True, padx=(0, 4))
        self.tv_denom = ttk.Treeview(d, columns=("val", "code", "n"), show="headings")
        for col, txt, w in (("val", "Valoare", 170), ("code", "Cod", 60), ("n", "Recunoscute", 90)):
            self.tv_denom.heading(col, text=txt)
            self.tv_denom.column(col, width=w)
        self.tv_denom.pack(fill="both", expand=True)

        r = ttk.LabelFrame(tables, text="Motive de respingere", padding=4)
        r.pack(side="left", fill="both", expand=True)
        self.tv_reason = ttk.Treeview(r, columns=("reason", "code", "n"), show="headings")
        for col, txt, w in (("reason", "Motiv", 220), ("code", "Cod", 60), ("n", "Nr.", 50)):
            self.tv_reason.heading(col, text=txt)
            self.tv_reason.column(col, width=w)
        self.tv_reason.pack(fill="both", expand=True)

        # Dreapta: jurnal
        right = ttk.LabelFrame(main, text="Jurnal", padding=4)
        main.add(right, weight=2)
        ttk.Checkbutton(right, text="Arată cadrele hex", variable=self.show_hex).pack(anchor="w")
        self.txt = tk.Text(right, wrap="word", font=("Consolas", 9), bg="#111", fg="#ddd")
        sb = ttk.Scrollbar(right, command=self.txt.yview)
        self.txt.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.txt.pack(fill="both", expand=True)
        for tag, col in (("info", "#ddd"), ("ok", "#5fd17a"), ("warn", "#f0b53c"),
                         ("err", "#ff6464"), ("raw", "#7f8ea3")):
            self.txt.tag_configure(tag, foreground=col)

    def set_buttons(self, connected):
        s = "normal" if connected else "disabled"
        for b in (self.btn_start, self.btn_stop, self.btn_reset, self.btn_ver):
            b.configure(state=s)

    def refresh_ports(self):
        ports = [f"{p.device} - {p.description}" for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_conn(self):
        if self.worker and self.worker.is_alive():
            self.worker.stop()
            self.worker = None
            self.btn_conn.configure(text="Conectare")
            self.set_buttons(False)
            self.set_state_label("Deconectat", "#555")
            return
        if not self.port_var.get():
            messagebox.showwarning("Port", "Alege portul ESP32-ului.")
            return
        port = self.port_var.get().split(" - ")[0]
        self.worker = UbaWorker(port, self.out_q, self.cmd_q, self.return_mode.get)
        self.worker.start()
        self.btn_conn.configure(text="Deconectare")

    def send(self, cmd):
        self.cmd_q.put(cmd)

    # ---- actualizare UI ----
    def log(self, msg, level="info"):
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        self.txt.insert("end", f"[{ts}] {msg}\n", level)
        self.txt.see("end")

    def set_test(self, key, state, note=""):
        self.test_state[key] = (state, note)
        name = dict(TESTS)[key]
        self.tv_tests.item(key, values=(name, STATE_LABEL[state], note), tags=(STATE_TAG[state],))

    def set_state_label(self, text, color):
        self.lbl_state.configure(text=text, bg=color)

    def update_totals(self):
        s = self.stats
        total = s["recognized"] + s["rejected"]
        rate = (100.0 * s["recognized"] / total) if total else 0.0
        self.lbl_tot["recognized"].configure(text=str(s["recognized"]))
        self.lbl_tot["rejected"].configure(text=str(s["rejected"]))
        self.lbl_tot["rate"].configure(text=f"{rate:.1f}%" if total else "—")
        self.lbl_tot["errors"].configure(text=str(s["errors"]))
        self.update_recognition()

    def update_recognition(self):
        s = self.stats
        total = s["recognized"] + s["rejected"]
        if total == 0:
            return
        rate = 100.0 * s["recognized"] / total
        try:
            thr = int(self.threshold.get())
        except (tk.TclError, ValueError):
            thr = 90
        note = f"{rate:.1f}% din {total} bancnote (prag {thr}%)"
        if total < 20:
            self.set_test("recognition", "run", note + " - introdu min. 20")
        else:
            self.set_test("recognition", "ok" if rate >= thr else "fail", note)

    def refresh_tables(self):
        self.tv_denom.delete(*self.tv_denom.get_children())
        for code, (name, n) in sorted(self.by_denom.items()):
            self.tv_denom.insert("", "end", values=(name, f"0x{code:02X}", n))
        self.tv_reason.delete(*self.tv_reason.get_children())
        for code, (name, n) in sorted(self.by_reason.items(), key=lambda x: -x[1][1]):
            self.tv_reason.insert("", "end", values=(name, f"0x{code:02X}", n))

    def clear_stats(self):
        keep = {k: self.test_state[k] for k in ("comm", "reset", "config", "version", "currency")}
        fw = self.firmware
        self.reset_stats()
        self.firmware = fw
        for k, (s, n) in keep.items():
            self.test_state[k] = (s, n)
        for k in ("transport", "recognition", "hardware"):
            self.set_test(k, "")
        self.refresh_tables()
        self.update_totals()
        self.lbl_tot["rate"].configure(text="—")

    def pump(self):
        try:
            while True:
                kind, kw = self.out_q.get_nowait()
                self.handle(kind, kw)
        except queue.Empty:
            pass
        self.after(50, self.pump)

    def handle(self, kind, kw):
        if kind == "log":
            self.log(kw["msg"], kw["level"])
        elif kind == "raw":
            if self.show_hex.get():
                self.log(f"{kw['dir']} {hexs(kw['data'])}", "raw")
        elif kind == "conn":
            if kw["ok"]:
                self.log(f"Port deschis: {kw['msg']}", "ok")
                self.set_buttons(True)
                self.set_state_label("Conectat - inițializare...", "#1d5fbf")
                if self.test_state["hardware"][0] == "":
                    self.set_test("hardware", "ok", "Nicio eroare până acum")
            else:
                self.log(f"Eroare port: {kw['msg']}", "err")
                self.set_buttons(False)
                self.btn_conn.configure(text="Conectare")
                self.set_state_label("Eroare port serial", "#c0161c")
        elif kind == "test":
            self.set_test(kw["key"], kw["state"], kw.get("note", ""))
        elif kind == "version":
            self.firmware = kw["text"]
        elif kind == "denoms":
            for code, name in kw["table"].items():
                n = self.by_denom.get(code, (name, 0))[1]
                self.by_denom[code] = (name, n)
            self.refresh_tables()
        elif kind == "state":
            code = kw["code"]
            text = ST.get(code, f"Stare 0x{code:02X}")
            if code == 0x17 and kw["data"]:
                text += " - " + REJECT_REASONS.get(kw["data"][0], f"0x{kw['data'][0]:02X}")
            color = ("#127a2e" if code == 0x11 else "#555" if code == 0x1A else
                     "#c0161c" if code in ERRORS else "#b56a00" if code in (0x17, 0x47) else "#1d5fbf")
            self.set_state_label(text, color)
        elif kind == "enabled":
            self.log("Acceptare ACTIVĂ - introdu bancnotele." if kw["on"] else "Acceptare oprită.",
                     "ok" if kw["on"] else "info")
            if kw["on"] and self.test_state["transport"][0] == "":
                self.set_test("transport", "run", "Introdu o bancnotă")
        elif kind == "escrow":
            self.stats["recognized"] += 1
            name, n = self.by_denom.get(kw["code"], (kw["name"], 0))
            self.by_denom[kw["code"]] = (name, n + 1)
            self.refresh_tables()
            self.update_totals()
        elif kind == "reject":
            self.stats["rejected"] += 1
            name, n = self.by_reason.get(kw["code"], (kw["name"], 0))
            self.by_reason[kw["code"]] = (name, n + 1)
            self.refresh_tables()
            self.update_totals()
            self.stats["cycles"] += 1
            if self.test_state["transport"][0] in ("", "run"):
                self.set_test("transport", "ok", "Bancnota a fost trasă și returnată fără blocaj")
        elif kind == "cycle":
            self.stats["cycles"] += 1
            if self.test_state["transport"][0] in ("", "run"):
                self.set_test("transport", "ok", f"Cicluri complete: {self.stats['cycles']}")
            elif self.test_state["transport"][0] == "ok":
                self.set_test("transport", "ok", f"Cicluri complete: {self.stats['cycles']}")
        elif kind == "error":
            self.stats["errors"] += 1
            self.error_list.append(f"{datetime.datetime.now():%H:%M:%S} {kw['text']}")
            self.set_test("hardware", "fail", kw["text"])
            if kw["code"] in (0x45, 0x46, 0x49):
                self.set_test("transport", "fail", kw["text"])
            self.update_totals()

    # ---- raport ----
    def save_report(self):
        now = datetime.datetime.now()
        sn = self.head_sn.get().strip() or "fara_serie"
        default = f"UBA_{sn}_{now:%Y%m%d_%H%M}.txt"
        path = filedialog.asksaveasfilename(defaultextension=".txt", initialfile=default,
                                            filetypes=[("Text", "*.txt")])
        if not path:
            return
        s = self.stats
        total = s["recognized"] + s["rejected"]
        rate = (100.0 * s["recognized"] / total) if total else 0.0
        lines = [
            "RAPORT TEST BILL ACCEPTOR JCM UBA (ID-003)",
            "=" * 46,
            f"Data:            {now:%d.%m.%Y %H:%M}",
            f"Serie cap:       {self.head_sn.get().strip() or '-'}",
            f"Firmware:        {self.firmware or '-'}",
            f"Mod test:        {'returnare' if self.return_mode.get() else 'stivuire'}",
            "",
            "TESTE",
        ]
        for k, name in TESTS:
            st, note = self.test_state[k]
            lines.append(f"  {name:<52} {STATE_LABEL[st]:<10} {note}")
        lines += ["", "TOTALURI",
                  f"  Bancnote introduse: {total}",
                  f"  Recunoscute:        {s['recognized']}",
                  f"  Respinse:           {s['rejected']}",
                  f"  Rată recunoaștere:  {rate:.1f}%",
                  f"  Erori hardware:     {s['errors']}",
                  "", "PE VALORI"]
        for code, (name, n) in sorted(self.by_denom.items()):
            lines.append(f"  {name:<30} cod 0x{code:02X}   {n}")
        lines += ["", "MOTIVE DE RESPINGERE"]
        if not self.by_reason:
            lines.append("  -")
        for code, (name, n) in sorted(self.by_reason.items(), key=lambda x: -x[1][1]):
            lines.append(f"  {name:<40} cod 0x{code:02X}   {n}")
        lines += ["", "ERORI"]
        lines += [f"  {e}" for e in self.error_list] or ["  -"]
        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        self.log(f"Raport salvat: {path}", "ok")

    def destroy(self):
        if self.worker:
            self.worker.stop()
        super().destroy()


if __name__ == "__main__":
    App().mainloop()
