// ============================================================================
// PetrChu — A3144 single Hall-effect RPM test (Arduino Uno)
// ----------------------------------------------------------------------------
// The A3144 is a UNIPOLAR digital Hall switch with an open-collector output:
// the output transistor sinks to GND when a magnet of the correct polarity
// (typically south face) gets close enough, and floats otherwise. Because it
// is open-collector, a pull-up is REQUIRED — we use the AVR's internal pull-up
// (INPUT_PULLUP), so no external resistor is needed for short wire runs. For
// long leads or noisy environments, add an external 10 kohm pull-up to 5V.
//
// Unlike the 3-Hall BLDC setup in hall_test_uno, this sketch uses a SINGLE
// A3144 reading magnets on a rotating target (a flywheel, shaft, fan blade,
// etc.). One pulse is generated per magnet that passes the sensor face, so:
//
//       pulses per mechanical rev = MAGNETS_PER_REV
//
// WIRING:
//
//        +5V ----o VCC (pin 1)
//                |
//                o OUT (pin 3) ----> D2  (INT0, with internal pull-up)
//                |
//        GND ----o GND (pin 2)
//
//   Pinout (flat face toward you, leads down): 1=VCC, 2=GND, 3=OUT.
//   The A3144 tolerates 4.5..24 V on VCC; 5 V from the Uno is fine.
//
// HOW THIS SKETCH MEASURES RPM:
//   On every FALLING edge (magnet enters the sensor's active region) the ISR
//   captures micros() and stores the gap to the previous edge as `lastPeriod`.
//   The main loop converts that period into RPM:
//
//       rpm = 60e6 / (lastPeriod_us * MAGNETS_PER_REV)
//
//   This is far more responsive at low RPM than counting pulses in a fixed
//   window: at 60 RPM with 1 magnet you'd see one pulse per second, so a
//   200 ms counting window would alternate between 0 and 300 RPM. Period
//   measurement gives an instant reading from each pulse.
//
//   To reject contact bounce / EMI bursts we ignore any edge that arrives
//   sooner than MIN_PERIOD_US after the previous one (an upper RPM clamp).
//   And if no pulse arrives within IDLE_TIMEOUT_MS, RPM is reported as 0
//   instead of leaving the last value frozen.
//
// CONFIG NOTES:
//   - Set MAGNETS_PER_REV to the number of magnets passing the sensor in one
//     mechanical revolution. To verify: zero `pulses`, hand-spin exactly one
//     revolution, read `pulses` — it should equal MAGNETS_PER_REV.
//   - MIN_PERIOD_US sets the maximum measurable RPM. Default 500 us caps at
//     120000 RPM with 1 magnet (60e6 / 500), well above mechanical limits.
//     Lower it only if you genuinely spin that fast; raising it helps reject
//     bouncy/jittery edges on dirty signals.
//   - IDLE_TIMEOUT_MS is how long we wait after the last pulse before
//     declaring zero. Set this above the period of your slowest expected
//     RPM, e.g. 60 RPM with 1 magnet = 1000 ms period, so use 2000+.
//   - Magnet polarity matters: A3144 is UNIPOLAR. If the sensor never
//     triggers, flip the magnet so the opposite face points at the chip.
//   - Serial Monitor: set Baud Rate to 115200.
// ============================================================================
#include <Arduino.h>

// ---------- USER CONFIG ----------
const uint8_t  PIN_HALL          = 2;     // must be D2 (INT0) or D3 (INT1) on Uno
const uint16_t MAGNETS_PER_REV   = 1;     // pulses per mechanical revolution

const unsigned long MIN_PERIOD_US     = 500;     // debounce / max-RPM clamp
const unsigned long IDLE_TIMEOUT_MS   = 2000;    // no pulse for this long -> RPM=0
const unsigned long PRINT_INTERVAL_MS = 200;

// ---------- STATE ----------
volatile unsigned long pulses        = 0;
volatile unsigned long lastEdgeUs    = 0;
volatile unsigned long lastPeriodUs  = 0;

unsigned long lastPulses     = 0;
unsigned long lastPrintMs    = 0;

// ---------- ISR ----------
void onHallEdge() {
  const unsigned long now = micros();
  const unsigned long dt  = now - lastEdgeUs;   // unsigned wrap is well-defined
  if (dt < MIN_PERIOD_US) return;               // glitch / bounce — drop it
  lastPeriodUs = dt;
  lastEdgeUs   = now;
  pulses++;
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(PIN_HALL, INPUT_PULLUP);
  delay(20);  // let the pull-up settle before arming the interrupt

  lastEdgeUs = micros();
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), onHallEdge, FALLING);

  Serial.println(F("# a3144 single-hall rpm test"));
  Serial.print  (F("# magnets/rev=")); Serial.print(MAGNETS_PER_REV);
  Serial.print  (F(", min_period_us=")); Serial.print(MIN_PERIOD_US);
  Serial.print  (F(", idle_timeout_ms=")); Serial.println(IDLE_TIMEOUT_MS);
  Serial.println(F("# format: t_ms, pulses, dPulses, period_us, rpm"));
  lastPrintMs = millis();
}

// ---------- LOOP ----------
void loop() {
  const unsigned long now = millis();
  if (now - lastPrintMs < PRINT_INTERVAL_MS) return;

  noInterrupts();
  const unsigned long p          = pulses;
  const unsigned long periodUs   = lastPeriodUs;
  const unsigned long lastEdgeMs = lastEdgeUs / 1000UL;
  interrupts();

  const unsigned long dPulses = p - lastPulses;

  float rpm = 0.0f;
  const bool idle = (now - lastEdgeMs) > IDLE_TIMEOUT_MS;
  if (!idle && periodUs > 0 && MAGNETS_PER_REV > 0) {
    rpm = 60000000.0f / ((float)periodUs * (float)MAGNETS_PER_REV);
  }

  Serial.print(now);             Serial.print(',');
  Serial.print(p);               Serial.print(',');
  Serial.print(dPulses);         Serial.print(',');
  Serial.print(idle ? 0UL : periodUs); Serial.print(',');
  Serial.println(rpm, 1);

  lastPulses  = p;
  lastPrintMs = now;
}
