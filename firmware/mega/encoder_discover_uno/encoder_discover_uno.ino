// ============================================================================
// PetrChu — encoder discovery sketch (Arduino Uno)
// ----------------------------------------------------------------------------
// Goal: figure out what this 5-wire encoder actually IS before deciding how to
// decode it. The motor is a separate 3-phase BLDC; these 5 wires are signal
// only (Vcc, GND, and 3 unknown signals).
//
// Wiring:
//   encoder Vcc   -> Arduino 5V   (verify the encoder accepts 5V — many do;
//                                  some BLDC Hall boards want 3.3V)
//   encoder GND   -> Arduino GND
//   wire1 (sig 1) -> D2
//   wire2 (sig 2) -> D4
//   wire3 (sig 3) -> D3
//
// What this sketch does:
//   - Polls all three signal lines as fast as loop() runs.
//   - On any state change, prints: t_ms, current 3-bit state (HHL etc),
//     per-channel cumulative edge counts.
//   - Every 1 s, prints a summary line with edges-per-second on each channel.
//   - Self-throttles printing if the serial buffer is filling up; counters
//     keep updating regardless.
//
// ----------------------------------------------------------------------------
// HOW TO USE:
//   1. Wire the 5 leads per the diagram above (same signal pins as
//      encoder_test_uno, so no rewiring if you swap sketches). Encoder Vcc
//      to 5V, encoder GND to GND.
//   2. Upload, then open Serial Monitor at 115200 baud.
//   3. Before spinning, check the "# initial state" line. With INPUT_PULLUP
//      and nothing actively driving the lines you'd see HHH; with the
//      encoder powered you'll see whatever state it's resting in.
//   4. Spin the shaft SLOWLY by hand — aim for one full revolution over
//      ~5 seconds the first time. Slow enough to read individual state
//      lines as they scroll.
//   5. Watch the state column (the HLx / LHx etc).
//
// ----------------------------------------------------------------------------
// HOW TO INTERPRET (spin shaft slowly by hand, ~1 rev per several seconds):
//
//   QUADRATURE  (Vcc, GND, A, B, Z):
//     - State on bits 1+2 cycles: LL -> LH -> HH -> HL -> LL  (forward)
//                              or LL -> HL -> HH -> LH -> LL  (reverse)
//     - Channel 3 (Z) is mostly steady, toggles ~twice per shaft rev
//       (one rising + one falling per index pulse).
//     - Per-rev edge counts: ch1 ~= ch2 (both = 2*CPR), ch3 = 2.
//
//   THREE HALL SENSORS  (Vcc, GND, H1, H2, H3) — common on BLDC motors:
//     - All three channels toggle at similar rates (none rare).
//     - State (1,2,3 bits) cycles through 6 Gray-code values, e.g.:
//         001 -> 011 -> 010 -> 110 -> 100 -> 101 -> 001 ...
//       (specific pattern depends on which Hall is wired to which pin.)
//     - 6 state transitions per electrical rev. A BLDC with N pole-pairs
//       has N electrical revs per mechanical rev, so a hand-turned full rev
//       gives 6*N transitions on each channel (often 30-100+ per rev).
//
//   DIFFERENTIAL SINGLE CHANNEL + INDEX  (Vcc, GND, A+, A-, Z):
//     - Channels 1 and 2 are ALWAYS opposite (one HIGH iff the other LOW).
//       The state column will only ever show patterns like LHx and HLx,
//       never LLx or HHx.
//     - Channel 3 toggles rarely (= once per rev, like Z above).
//
//   STUCK / DEAD WIRE:
//     - Any channel that never changes after pulling/releasing INPUT_PULLUP
//       is either not connected, in differential mode (always opposite),
//       or the encoder needs different supply voltage / open-collector.
//
// ----------------------------------------------------------------------------
// QUICK DECISION GUIDE:
//   - ch3 mostly steady, ~2 toggles per revolution; ch1 and ch2 toggle a lot
//       at equal rates                              -> QUADRATURE with index.
//       Note the cycle direction of bits (1,2):
//         LL->LH->HH->HL  is one direction
//         LL->HL->HH->LH  is the other.
//   - All three channels toggle at similar rates;
//     state cycles through 6 distinct values, never 000 or 111
//                                                   -> 3 HALL SENSORS.
//       Cycle order (e.g. 001->011->010->110->100->101) gives commutation seq.
//   - First two state characters always opposite (only LHx and HLx ever seen)
//                                                   -> DIFFERENTIAL A + index.
//   - One channel never moves no matter how you spin
//                                                   -> wire not connected,
//       wrong supply voltage (try 3.3V), or open-collector needing a different
//       pull-up arrangement.
//
// Once the type is known, paste the observed pattern back to update
// encoder_test_uno.ino with the matching decoder.
//
// Note: polling + Serial.print is fine for hand-spin discovery. For powered
// motor speeds you'd want ISRs and no Serial in the hot path.
// ============================================================================
#include <Arduino.h>

const uint8_t PIN_1 = 2;
const uint8_t PIN_2 = 4;
const uint8_t PIN_3 = 3;

unsigned long count1 = 0;
unsigned long count2 = 0;
unsigned long count3 = 0;

unsigned long lastSummaryMs = 0;
unsigned long lastSummaryC1 = 0;
unsigned long lastSummaryC2 = 0;
unsigned long lastSummaryC3 = 0;

uint8_t lastState = 0;  // bit0 = pin1, bit1 = pin2, bit2 = pin3

static inline uint8_t readState() {
  uint8_t s = 0;
  if (digitalRead(PIN_1)) s |= 0x1;
  if (digitalRead(PIN_2)) s |= 0x2;
  if (digitalRead(PIN_3)) s |= 0x4;
  return s;
}

static void printStateBits(uint8_t s) {
  Serial.print((s & 0x1) ? 'H' : 'L');
  Serial.print((s & 0x2) ? 'H' : 'L');
  Serial.print((s & 0x4) ? 'H' : 'L');
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(PIN_1, INPUT_PULLUP);
  pinMode(PIN_2, INPUT_PULLUP);
  pinMode(PIN_3, INPUT_PULLUP);

  delay(50);  // let pull-ups settle
  lastState = readState();

  Serial.println(F("# encoder discovery — spin shaft slowly by hand"));
  Serial.println(F("# columns:  t_ms  state(p1,p2,p3)  totals(c1 c2 c3)"));
  Serial.print  (F("# initial state: "));
  printStateBits(lastState);
  Serial.println();

  lastSummaryMs = millis();
}

void loop() {
  uint8_t s = readState();

  if (s != lastState) {
    uint8_t changed = s ^ lastState;
    if (changed & 0x1) count1++;
    if (changed & 0x2) count2++;
    if (changed & 0x4) count3++;

    // self-throttle prints so we don't choke the serial buffer at speed;
    // counts keep updating regardless.
    if (Serial.availableForWrite() > 32) {
      Serial.print(millis()); Serial.print('\t');
      printStateBits(s);      Serial.print('\t');
      Serial.print(count1);   Serial.print(' ');
      Serial.print(count2);   Serial.print(' ');
      Serial.println(count3);
    }

    lastState = s;
  }

  unsigned long now = millis();
  if (now - lastSummaryMs >= 1000) {
    unsigned long d1 = count1 - lastSummaryC1;
    unsigned long d2 = count2 - lastSummaryC2;
    unsigned long d3 = count3 - lastSummaryC3;
    Serial.print(F("# 1s edges: ch1="));
    Serial.print(d1);
    Serial.print(F("  ch2="));
    Serial.print(d2);
    Serial.print(F("  ch3="));
    Serial.print(d3);
    Serial.print(F("   totals: "));
    Serial.print(count1); Serial.print('/');
    Serial.print(count2); Serial.print('/');
    Serial.println(count3);
    lastSummaryC1 = count1;
    lastSummaryC2 = count2;
    lastSummaryC3 = count3;
    lastSummaryMs = now;
  }
}
