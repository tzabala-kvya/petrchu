// ============================================================================
// PetrChu - E-stop + ARM switch bench test (Arduino Mega)
// ----------------------------------------------------------------------------
// Verifies the safety-input wiring before integrating into TestStandFirmware.
// Prints state-change events for E-stop (interrupt-driven) and ARM (polled).
// Red LED on d52 mirrors E-stop state. Green LED on d50 mirrors ARM state.
//
// WHAT THIS TESTS:
//   1. E-stop fails safe (button OR cut wire reads as ACTIVE).
//   2. Interrupt fires on the press edge with sub-millisecond latency.
//   3. ARM switch is reliably read by polling.
//   4. Both switches debounce cleanly (no chatter on serial output).
//
// WIRING - E-STOP (use the Normally Closed contact on the mushroom button):
//
//                            Mega
//                          +------+
//                d18 ----+ +      |
//                        |        |
//                  [E-STOP NC]    |
//                        |        |
//                        +---- GND|
//                          +------+
//
//   d18 -> one NC terminal -> other NC terminal -> Mega GND
//   No external resistor (INPUT_PULLUP enables the internal ~30 kohm).
//   Normal (button up): NC closed -> d18 = LOW.
//   Pressed OR wire cut: d18 = HIGH (pull-up wins). This is FAIL-SAFE.
//
// WIRING - ARM switch (any SPST toggle, NC orientation = fail-safe-to-disarmed):
//
//                d37 ---- [ARM SWITCH] ---- GND
//
//   Closed (ARMED): d37 = LOW.   Open (DISARMED) or wire cut: d37 = HIGH.
//
// WIRING - Status LEDs (each in series with a 220-330 ohm resistor):
//
//                d52 ---[220R]---|>|--- GND     (RED LED, mirrors E-stop)
//                d50 ---[220R]---|>|--- GND     (GREEN LED, mirrors ARM)
//
// SERIAL: 115200 baud. Prints one CSV row per state change:
//   # format: t_ms, event, estop, arm, isr_count
//
//   estop: 0 = safe, 1 = ACTIVE.
//   arm:   0 = disarmed, 1 = ARMED.
//   isr_count: total E-stop ISR firings since boot (sanity check).
//
// TROUBLESHOOTING:
//   - E-stop "chattering" (multiple events per press): bouncy contact. Add a
//     0.1 uF capacitor across the switch contacts, or increase DEBOUNCE_MS.
//   - E-stop reads ACTIVE even when button is up: check the NC vs NO terminals
//     on the switch body. Many mushroom buttons have both pairs.
//   - ARM reads HIGH always: wire is open somewhere. Continuity-check from
//     d37 to GND with the switch closed.
//   - ISR not firing: confirm d18 is wired (not d2 or d3). On the Mega, d18 is
//     INT5; the attachInterrupt() call must reference digitalPinToInterrupt().
// ============================================================================
#include <Arduino.h>

// ---------- PIN ASSIGNMENTS ----------
const uint8_t PIN_ESTOP    = 18;   // INT5 on Mega - external interrupt capable
const uint8_t PIN_ARM      = 37;
const uint8_t PIN_LED_RED  = 52;   // mirrors E-stop
const uint8_t PIN_LED_GRN  = 50;   // mirrors ARM

// ---------- CONFIG ----------
const unsigned long DEBOUNCE_MS = 30;
const unsigned long POLL_PERIOD_MS = 5;   // ARM polling period

// ---------- STATE ----------
// E-stop (interrupt-driven)
volatile bool          g_estop_raw      = false;  // set by ISR
volatile unsigned long g_estop_isr_ms   = 0;      // last ISR timestamp
volatile unsigned long g_estop_isr_count = 0;     // total ISR fires

static bool            g_estop_debounced = false;
static unsigned long   g_estop_last_edge_ms = 0;

// ARM (polled)
static bool            g_arm_debounced  = false;
static uint8_t         g_arm_last_raw   = HIGH;
static unsigned long   g_arm_edge_ms    = 0;
static unsigned long   g_last_poll_ms   = 0;

// ---------- ISR ----------
// Runs whenever d18 changes state in either direction. Stays tiny:
// captures the raw level + timestamp + bumps a counter. All debouncing
// and printing happens in loop() where it is safe.
void estopISR() {
  g_estop_raw       = (digitalRead(PIN_ESTOP) == HIGH);
  g_estop_isr_ms    = millis();
  g_estop_isr_count++;
}

// ---------- HELPERS ----------
static void mirrorLeds() {
  digitalWrite(PIN_LED_RED, g_estop_debounced ? HIGH : LOW);
  digitalWrite(PIN_LED_GRN, g_arm_debounced   ? HIGH : LOW);
}

static void emitEvent(const __FlashStringHelper* tag) {
  // Snapshot the volatile counter atomically.
  noInterrupts();
  const unsigned long n = g_estop_isr_count;
  interrupts();

  Serial.print(millis());                Serial.print(',');
  Serial.print(tag);                     Serial.print(',');
  Serial.print(g_estop_debounced ? 1:0); Serial.print(',');
  Serial.print(g_arm_debounced   ? 1:0); Serial.print(',');
  Serial.println(n);
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_ARM,   INPUT_PULLUP);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GRN, OUTPUT);

  // Seed initial states from current pin levels.
  g_estop_raw       = (digitalRead(PIN_ESTOP) == HIGH);
  g_estop_debounced = g_estop_raw;
  g_arm_last_raw    = digitalRead(PIN_ARM);
  g_arm_debounced   = (g_arm_last_raw == LOW);

  attachInterrupt(digitalPinToInterrupt(PIN_ESTOP), estopISR, CHANGE);

  Serial.println(F("# estop + arm test ready"));
  Serial.print  (F("# pins: ESTOP=d")); Serial.print(PIN_ESTOP);
  Serial.print  (F(" (INT)  ARM=d"));   Serial.print(PIN_ARM);
  Serial.print  (F("  LED_RED=d"));     Serial.print(PIN_LED_RED);
  Serial.print  (F("  LED_GRN=d"));     Serial.println(PIN_LED_GRN);
  Serial.println(F("# format: t_ms, event, estop, arm, isr_count"));
  emitEvent(F("boot"));
  mirrorLeds();
}

// ---------- LOOP ----------
void loop() {
  const unsigned long now = millis();

  // --- E-stop debounce (acts on the ISR-latched raw value) ---
  // ISR fires immediately; we wait DEBOUNCE_MS of stability before
  // accepting the change as a real event.
  noInterrupts();
  const bool          raw      = g_estop_raw;
  const unsigned long edge_ms  = g_estop_isr_ms;
  interrupts();

  if (raw != g_estop_debounced) {
    if (edge_ms != g_estop_last_edge_ms) {
      g_estop_last_edge_ms = edge_ms;
    }
    if ((now - g_estop_last_edge_ms) >= DEBOUNCE_MS) {
      g_estop_debounced = raw;
      emitEvent(raw ? F("estop_ACTIVE") : F("estop_clear"));
      mirrorLeds();
    }
  }

  // --- ARM polling debounce ---
  if (now - g_last_poll_ms >= POLL_PERIOD_MS) {
    g_last_poll_ms = now;
    const uint8_t raw_arm = digitalRead(PIN_ARM);
    if (raw_arm != g_arm_last_raw) {
      g_arm_last_raw = raw_arm;
      g_arm_edge_ms  = now;
    } else if ((now - g_arm_edge_ms) >= DEBOUNCE_MS) {
      const bool armed_now = (raw_arm == LOW);
      if (armed_now != g_arm_debounced) {
        g_arm_debounced = armed_now;
        emitEvent(armed_now ? F("arm_ARMED") : F("arm_disarmed"));
        mirrorLeds();
      }
    }
  }
}
