// ============================================================================
// PetrChu — MOSFET serial-controlled test, low-side N-channel switch (Mega 2560)
// ----------------------------------------------------------------------------
// Drives a logic-level N-channel MOSFET as a low-side switch under control of
// SERIAL COMMANDS instead of a pushbutton. Type a command in the Serial
// Monitor (newline-terminated) to switch the gate:
//
//   on        — gate held high (full enhancement), load at full current
//   off       — gate held low, load fully disconnected
//   ?         — print current state
//
// Commands are case-insensitive and tolerant of surrounding whitespace and a
// trailing CR (so "ON\r\n" from a Windows terminal works). The Mega's onboard
// LED (D13) mirrors the gate output as a visual cue. Serial echoes every
// accepted command and every state change so you can correlate scope traces or
// multimeter readings with the commands you sent.
//
// MOSFET ASSUMPTIONS — this sketch assumes a LOGIC-LEVEL N-channel part
// (e.g. IRLZ44N, IRL540, IRLB8721, AOI518). Standard-level MOSFETs like the
// IRF540 need ~10 V Vgs to fully enhance and will run hot at 5 V gate drive —
// the part will still switch, but Rds(on) will be far above datasheet spec
// and the device may dissipate enough power to need a heatsink even at modest
// load currents. If you only have a standard-level part on the bench, use a
// gate driver IC or a small NPN level-shifter; do not just push it harder.
//
// WHY THE GATE PULL-DOWN — MOSFET gates are pure capacitance. If the Mega pin
// floats (boot, reset, sketch upload, wire popped loose), residual gate charge
// can leave the MOSFET partly-on, dissipating power and possibly damaging the
// load. A 10k pull-down from gate to GND drains that charge in microseconds
// and forces a defined OFF state any time the Mega isn't actively driving the
// gate high.
//
// WHY THE GATE SERIES RESISTOR — at the instant the Mega pin transitions, the
// MOSFET gate looks like a short to whatever's on the cap. A 220 ohm series
// resistor limits the inrush current to ~25 mA (5 V / 220), keeping the Mega
// pin inside its 40 mA absolute-max rating. It also damps any L-C ringing
// between pin trace and gate cap, which on a breadboard can otherwise cause
// the MOSFET to oscillate during the switching edge.
//
// FLYBACK DIODE — only required for INDUCTIVE loads (relay coils, solenoids,
// motor windings, the air-motor servo on the test stand). For a pure
// resistive load (light bulb, resistor, LED bank), skip it. For inductive
// loads use a 1N4007 or similar across the load with cathode (band) toward
// the supply (+) side. Without it, turn-off inductive kickback spikes the
// drain and will eventually punch through the MOSFET's avalanche rating.
//
// WIRING (low-side N-channel switch):
//
//                              +V_LOAD (external supply +)
//                                  |
//                                 [LOAD]            (with flyback diode if inductive:
//                                  |                 1N4007 across load, cathode->+V)
//                                  | drain
//                            +-----+
//                            |     |
//   Mega D9 --[220 ohm]------+ G   |   (N-channel logic-level MOSFET, TO-220)
//                            |     |
//                            +--+--+
//                               |  source
//                              GND  (common with Mega GND AND load supply GND)
//                               |
//   Mega D9 --[10 kOhm]-------- GND   (gate pull-down — connect at the gate
//                                      side of the 220 R, i.e. directly on
//                                      the MOSFET gate pin)
//
//   - Mega GND and the external load supply GND MUST be tied together. The
//     MOSFET source is the reference point for Vgs; if grounds aren't common,
//     the gate sees an undefined voltage relative to source and switching
//     becomes unpredictable.
//   - For a first sanity check, you can use a single LED + 120 ohm resistor
//     as the "load" — it'll be visibly bright in the ON state and dark in OFF.
//
// PIN CHOICE — D9 is PWM-capable on the Mega (timer2). This sketch only does
// digital on/off, but the PWM-capable pin is kept so you can extend it later
// without re-wiring. Avoid pins reserved in TestStandFirmware if you intend to
// run this on the same board (see pin_map_poc).
//
//   - Serial Monitor: set Baud Rate to 115200, line ending to "Newline".
// ============================================================================
#include <Arduino.h>

// ---------- USER CONFIG ----------
const uint8_t  PIN_MOSFET     = 47;            // Drives gate via 220 R.
const uint8_t  PIN_LED_MIRROR = LED_BUILTIN;  // D13 — visual state cue.

const unsigned long HEARTBEAT_MS = 2000;      // periodic "still alive" print

// ---------- SERIAL RX ----------
const uint8_t RX_BUF_SIZE = 32;
static char          g_rx_buf[RX_BUF_SIZE];
static uint8_t       g_rx_len         = 0;
static bool          g_mosfet_on      = false;
static unsigned long g_last_heartbeat = 0;

// ---------- HELPERS ----------
static void applyOutput(bool on) {
  g_mosfet_on = on;
  digitalWrite(PIN_MOSFET,     on ? HIGH : LOW);
  digitalWrite(PIN_LED_MIRROR, on ? HIGH : LOW);
  Serial.print(F("# state -> "));
  Serial.println(on ? F("ON") : F("OFF"));
}

// Compare the RX buffer (already NUL-terminated) against a command word,
// case-insensitively. Returns true on an exact match.
static bool cmdEquals(const char* cmd) {
  uint8_t i = 0;
  for (; cmd[i] != '\0'; i++) {
    if (i >= g_rx_len) return false;
    char a = g_rx_buf[i];
    if (a >= 'A' && a <= 'Z') a += 32;   // to lower
    if (a != cmd[i]) return false;
  }
  return i == g_rx_len;   // both ended together
}

// Act on a fully-received line (trailing CR/whitespace already stripped).
static void handleLine() {
  if (g_rx_len == 0) return;             // blank line — ignore

  if      (cmdEquals("on"))  applyOutput(true);
  else if (cmdEquals("off")) applyOutput(false);
  else if (cmdEquals("?")) {
    Serial.print(F("# current state: "));
    Serial.println(g_mosfet_on ? F("ON") : F("OFF"));
  } else {
    Serial.print(F("# unknown command: '"));
    Serial.print(g_rx_buf);
    Serial.println(F("'  (expected: on | off | ?)"));
  }
}

// Read available serial bytes into the line buffer. On newline, trim a trailing
// CR plus any trailing spaces/tabs, NUL-terminate, and dispatch.
static void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n') {
      // strip trailing CR / whitespace
      while (g_rx_len > 0 &&
             (g_rx_buf[g_rx_len - 1] == '\r' ||
              g_rx_buf[g_rx_len - 1] == ' '  ||
              g_rx_buf[g_rx_len - 1] == '\t')) {
        g_rx_len--;
      }
      g_rx_buf[g_rx_len] = '\0';
      handleLine();
      g_rx_len = 0;
      continue;
    }

    if (g_rx_len < RX_BUF_SIZE - 1) {
      g_rx_buf[g_rx_len++] = c;
    } else {
      // overflow — drop the line, warn, and resync on next newline
      g_rx_len = 0;
      Serial.println(F("# line too long — discarded"));
    }
  }
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(PIN_MOSFET, OUTPUT);
  digitalWrite(PIN_MOSFET, LOW);     // boot in OFF — gate held low

  pinMode(PIN_LED_MIRROR, OUTPUT);
  digitalWrite(PIN_LED_MIRROR, LOW);

  Serial.println(F("# mosfet serial test ready"));
  Serial.print  (F("# gate pin = D")); Serial.println(PIN_MOSFET);
  Serial.println(F("# commands: on | off | ?   (newline-terminated)"));
  Serial.println(F("# initial state: OFF"));
}

// ---------- LOOP ----------
void loop() {
  pollSerial();

  const unsigned long now = millis();
  if (now - g_last_heartbeat >= HEARTBEAT_MS) {
    g_last_heartbeat = now;
    Serial.print(F("# heartbeat — state="));
    Serial.println(g_mosfet_on ? F("ON") : F("OFF"));
  }
}
