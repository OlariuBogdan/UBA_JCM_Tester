/*
 * ESP32 - punte USB <-> ID-003 pentru bill acceptor JCM UBA
 * ---------------------------------------------------------
 * PC (programul uba_tester.py)  <--USB 115200-->  ESP32  <--UART2 9600 8E1-->  UBA
 *
 * ESP32-ul doar transferă octeții în ambele sensuri; toată logica
 * protocolului ID-003 este în programul de pe PC.
 *
 * Conexiuni (vezi README.md):
 *   GPIO16 (RX2)  <- UBA TXD   (cu pull-up 4.7k la 3.3V)
 *   GPIO17 (TX2)  -> UBA RXD   (prin rezistență 330 ohm)
 *   GND           -- GND UBA / GND sursă (comun)
 */

#include <Arduino.h>

// ---- Setări ----
#define PIN_RX2      16
#define PIN_TX2      17
#define PIN_LED      2        // LED-ul de pe DevKit (la unele plăci e alt pin)
#define USB_BAUD     115200
#define UBA_BAUD     9600     // ID-003: 9600 baud, 8 biți, paritate pară, 1 stop

// Dacă UBA nu răspunde deloc și ai verificat TX/RX, încearcă true
// (unele interfețe pe optocuplor inversează nivelul logic).
#define INVERT_UART  false

static unsigned long ledOffAt = 0;

static void blink() {
  digitalWrite(PIN_LED, HIGH);
  ledOffAt = millis() + 20;
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  Serial.begin(USB_BAUD);
  Serial2.begin(UBA_BAUD, SERIAL_8E1, PIN_RX2, PIN_TX2, INVERT_UART);
  // Nu apela pinMode() pe GPIO16/17 după begin() - ar deconecta pinii de UART.
  // Pull-up-ul extern de 4.7k pe RX este obligatoriu.
}

void loop() {
  // PC -> UBA
  while (Serial.available()) {
    Serial2.write((uint8_t)Serial.read());
    blink();
  }
  // UBA -> PC
  while (Serial2.available()) {
    Serial.write((uint8_t)Serial2.read());
    blink();
  }
  if (ledOffAt && millis() > ledOffAt) {
    digitalWrite(PIN_LED, LOW);
    ledOffAt = 0;
  }
}
