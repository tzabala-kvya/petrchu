// PetrChu — Pump Control Test (Arduino Uno)
// MS5837-02BA depth sensor → MOSFET → contactor → pump
//
// SERIAL COMMANDS (9600 baud, set "No line ending" or "Newline" in Serial Monitor):
//   on        — start auto control loop
//   off       — pause auto control loop, pump forced off
//   low=XX    — set pump ON threshold in cm  (e.g. low=12)
//   high=XX   — set pump OFF threshold in cm (e.g. high=17)
//
// WIRING:
//   Arduino Uno A4  → Level converter SDA_LV → MS5837 SDA
//   Arduino Uno A5  → Level converter SCL_LV → MS5837 SCL
//   Arduino Uno 3.3V → Level converter LV + MS5837 VCC
//   Arduino Uno 5V  → Level converter HV
//   Arduino Pin 7   → 220Ω → MOSFET Gate
//   MOSFET Source   → GND (shared with 12V supply GND)
//   MOSFET Drain    → Contactor Coil(+), flyback diode anode, toggle leg 2
//   1N4007 Cathode  → 12V rail
//   All GNDs tied together

#include <Wire.h>
#include "MS5837.h"

MS5837 sensor;

const int PUMP_PIN = 7;

// Setpoints in cm
float setpointLow  = 12.0;  // pump turns ON below this depth
float setpointHigh = 17.0;  // pump turns OFF above this depth

bool pumpOn      = false;
bool loopRunning = false;  // starts paused — type 'on' to begin

// Run time tracking
unsigned long pumpStartTime  = 0;
unsigned long totalPumpRunMs = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);  // pump off at startup

  // Init sensor — halt until found
  while (!sensor.init()) {
    Serial.println("MS5837 not found. Check wiring.");
    delay(1000);
  }
  sensor.setModel(MS5837::MS5837_02BA);
  sensor.setFluidDensity(997);  // freshwater kg/m3

  Serial.println("=================================");
  Serial.println("  PetrChu Pump Control Test");
  Serial.println("=================================");
  Serial.println("Commands:");
  Serial.println("  on       — start auto control");
  Serial.println("  off      — pause, pump forced off");
  Serial.println("  low=XX   — set ON threshold (cm)");
  Serial.println("  high=XX  — set OFF threshold (cm)");
  Serial.println("---------------------------------");
  Serial.println("Status: PAUSED — type 'on' to start");
  printSetpoints();
}

void loop() {
  // Always read sensor
  sensor.read();
  float depthCm = sensor.depth() * 100.0;

  if (loopRunning) {
    // Hysteresis control
    if (!pumpOn && depthCm < setpointLow) {
      setPump(true);
    } else if (pumpOn && depthCm > setpointHigh) {
      setPump(false);
    }

    // Print status
    Serial.print("[AUTO] Depth: ");
    Serial.print(depthCm, 1);
    Serial.print(" cm | Pump: ");
    Serial.print(pumpOn ? "ON" : "OFF");
    Serial.print(" | Runtime: ");
    Serial.print(getPumpRuntime());
    Serial.print("s | Low: ");
    Serial.print(setpointLow, 1);
    Serial.print(" cm | High: ");
    Serial.print(setpointHigh, 1);
    Serial.println(" cm");

  } else {
    // Paused — still print depth so you can see sensor is working
    Serial.print("[PAUSED] Depth: ");
    Serial.print(depthCm, 1);
    Serial.println(" cm");
  }

  handleSerial();
  delay(300);
}

// ── Pump control ──────────────────────────────────────
void setPump(bool state) {
  if (state && !pumpOn) {
    pumpOn = true;
    pumpStartTime = millis();
    digitalWrite(PUMP_PIN, HIGH);
    Serial.println(">>> PUMP ON");
  } else if (!state && pumpOn) {
    pumpOn = false;
    totalPumpRunMs += millis() - pumpStartTime;
    digitalWrite(PUMP_PIN, LOW);
    Serial.println(">>> PUMP OFF");
  }
}

// Returns total pump runtime in seconds this session
unsigned long getPumpRuntime() {
  unsigned long total = totalPumpRunMs;
  if (pumpOn) total += millis() - pumpStartTime;
  return total / 1000;
}

// ── Serial input handler ───────────────────────────────
void handleSerial() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  input.toLowerCase();

  if (input == "on") {
    loopRunning = true;
    Serial.println(">>> AUTO CONTROL STARTED");

  } else if (input == "off") {
    loopRunning = false;
    setPump(false);  // force pump off when pausing
    Serial.println(">>> PAUSED — pump forced off");

  } else if (input.startsWith("low=")) {
    float val = input.substring(4).toFloat();
    if (val > 0 && val < setpointHigh) {
      setpointLow = val;
      Serial.print(">>> Low threshold set to: ");
      Serial.print(setpointLow, 1);
      Serial.println(" cm");
    } else {
      Serial.println("! Invalid: low must be > 0 and < high threshold");
    }

  } else if (input.startsWith("high=")) {
    float val = input.substring(5).toFloat();
    if (val > setpointLow) {
      setpointHigh = val;
      Serial.print(">>> High threshold set to: ");
      Serial.print(setpointHigh, 1);
      Serial.println(" cm");
    } else {
      Serial.println("! Invalid: high must be > low threshold");
    }

  } else {
    Serial.println("! Unknown command. Use: on, off, low=XX, high=XX");
  }

  printSetpoints();
}

void printSetpoints() {
  Serial.print("    Setpoints — Low: ");
  Serial.print(setpointLow, 1);
  Serial.print(" cm | High: ");
  Serial.print(setpointHigh, 1);
  Serial.println(" cm");
}
