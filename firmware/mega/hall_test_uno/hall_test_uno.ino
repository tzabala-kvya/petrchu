// ============================================================================
// PetrChu — 3-phase BLDC Hall sensor test (Arduino Uno)
// ----------------------------------------------------------------------------
// The motor has three Hall sensors (HU, HV, HW) plus 5V and GND. Per the
// datasheet, the 5 encoder wires are:
//   RED    -> +5V
//   BLACK  -> GND
//   HU  (Yellow)   -> D2  (PCINT18) 
//   HV  (Green)    -> D3  (PCINT19)
//   HW  (Blue)     -> D4  (PCINT20)
//
// All three pins live on Port D so they share the PCINT2 vector — one ISR
// handles edges on any of them, no external INT0/INT1 needed.
//
// HOW HALL DECODING WORKS:
//   The three Halls produce a 3-bit state. As the rotor turns, the state
//   advances through 6 valid Gray-coded values (001, 011, 010, 110, 100, 101)
//   per ELECTRICAL revolution. A motor with N pole-pairs has N electrical
//   revolutions per MECHANICAL revolution, so:
//       transitions per mechanical rev = 6 * N
//   Resolution is therefore much coarser than a quadrature encoder (typical
//   BLDCs land at 24-100 counts/mech-rev). Fine for RPM and rough position;
//   not fine for precision positioning.
//
// HOW THIS SKETCH DECODES:
//   At startup we build a 64-entry transition lookup table indexed by
//   (prev_state << 3) | curr_state. Entries are +1 for a forward step,
//   -1 for a reverse step, or 0 for an invalid transition (skipped state
//   or glitch). The PCINT2 ISR reads the new state on every edge and adds
//   the table entry to `count`.
//
// CONFIG NOTES:
//   - POLE_PAIRS: set this to enable RPM. To measure: zero count, hand-spin
//     exactly one mechanical revolution, read count, divide by 6.
//   - If `invalid` transitions dominate while you're spinning, this motor's
//     Hall sequence differs from the assumed one. Easiest fix: swap two of
//     the PIN_HU / PIN_HV / PIN_HW assignments below until counts climb
//     cleanly and `invalid` stays near zero.
//   - Sign: if count goes the "wrong way" for your positive rotation, swap
//     any two pin assignments (or negate count in the consumer).
//   - Serial Monitor: Set Baud Rate to 115200
// ============================================================================
#include <Arduino.h>

// ---------- USER CONFIG ----------
const uint8_t  PIN_HU      = 2;   // PCINT18
const uint8_t  PIN_HV      = 3;   // PCINT19
const uint8_t  PIN_HW      = 4;   // PCINT20
const uint16_t POLE_PAIRS  = 4;   // 0 = unknown -> RPM not computed

const unsigned long PRINT_INTERVAL_MS = 200;

// ---------- STATE ----------
volatile long          count               = 0;
volatile unsigned long invalidTransitions  = 0;
volatile uint8_t       lastState           = 0;

int8_t hallTable[64];  // (prev<<3)|curr -> +1, -1, or 0

long          lastCount   = 0;
unsigned long lastInvalid = 0;
unsigned long lastPrintMs = 0;

// ---------- HELPERS ----------
static inline uint8_t readHallState() {
  uint8_t s = 0;
  if (digitalRead(PIN_HU)) s |= 0x1;
  if (digitalRead(PIN_HV)) s |= 0x2;
  if (digitalRead(PIN_HW)) s |= 0x4;
  return s;
}

static void buildHallTable() {
  // standard 6-step BLDC Hall sequence: 001 -> 011 -> 010 -> 110 -> 100 -> 101
  static const uint8_t fwd[6] = { 0b001, 0b011, 0b010, 0b110, 0b100, 0b101 };
  for (uint8_t i = 0; i < 64; ++i) hallTable[i] = 0;
  for (uint8_t i = 0; i < 6; ++i) {
    uint8_t a = fwd[i];
    uint8_t b = fwd[(i + 1) % 6];
    hallTable[(a << 3) | b] = +1;   // forward
    hallTable[(b << 3) | a] = -1;   // reverse
  }
}

// ---------- ISR ----------
// PCINT2 covers Port D (D0..D7). We unmask only D2/D3/D4 below.
ISR(PCINT2_vect) {
  uint8_t s = readHallState();
  if (s != lastState) {
    int8_t delta = hallTable[((uint16_t)lastState << 3) | s];
    if (delta != 0) count += delta;
    else            invalidTransitions++;
    lastState = s;
  }
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(PIN_HU, INPUT_PULLUP);
  pinMode(PIN_HV, INPUT_PULLUP);
  pinMode(PIN_HW, INPUT_PULLUP);
  delay(20);  // settle

  buildHallTable();
  lastState = readHallState();

  // Enable Pin Change Interrupts on D2, D3, D4 (PCINT18, PCINT19, PCINT20).
  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20);

  Serial.println(F("# hall test ready"));
  Serial.println(F("# format: t_ms, state, count, dCount, rpm, dInvalid"));
  Serial.println(F("# (rpm = 0 until POLE_PAIRS is set)"));
  Serial.print  (F("# initial state: "));
  Serial.println(lastState);
  lastPrintMs = millis();
}

// ---------- LOOP ----------
void loop() {
  const unsigned long now = millis();
  if (now - lastPrintMs < PRINT_INTERVAL_MS) return;

  // atomic snapshot
  noInterrupts();
  const long          c    = count;
  const uint8_t       st   = lastState;
  const unsigned long inv  = invalidTransitions;
  interrupts();

  const long          dCount = c   - lastCount;
  const unsigned long dInv   = inv - lastInvalid;

  float rpm = 0.0f;
  if (POLE_PAIRS > 0) {
    // 6 transitions per electrical rev * POLE_PAIRS per mechanical rev
    rpm = (float)dCount * 60000.0f
          / ((float)PRINT_INTERVAL_MS * 6.0f * (float)POLE_PAIRS);
  }

  Serial.print(now);    Serial.print(',');
  Serial.print(st);     Serial.print(',');
  Serial.print(c);      Serial.print(',');
  Serial.print(dCount); Serial.print(',');
  Serial.print(rpm, 1); Serial.print(',');
  Serial.println(dInv);

  lastCount   = c;
  lastInvalid = inv;
  lastPrintMs = now;
}
