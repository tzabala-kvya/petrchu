/*
  prop_valve_test_uno.ino
  -----------------------------------------------------------------------------
  Bench test for HSH-Flo 2-way 0-10V proportional motorized ball valve (9-24V).
  Drives the valve via a PWM-to-voltage converter and reads its position
  feedback (open-collector PWM on the white wire).

  Arduino can't output true analog above 5V, so:
     Arduino PWM (0-5V) -> PWM-to-V converter -> 0-10V -> valve GREEN (control in)

  =============================================================================
  DO-FIRST CHECKLIST (do these BEFORE connecting the valve)
  =============================================================================
  [ ] 1. CALIBRATE THE CONVERTER TO 10V.
         The converter board has a 5V/10V range + trimpot. With the Arduino
         driving 100% duty (send "100" over serial), measure VOUT with a meter
         and adjust until it reads ~10.0V. If it's stuck on the 5V range the
         valve only sees half its travel. DO THIS WITH THE VALVE DISCONNECTED.

  [ ] 2. COMMON GROUND. Tie all grounds to ONE point:
            Arduino GND  +  converter GND (in AND out)  +  12V supply (-)
         The Uno is USB-powered at 5V logic. If its ground doesn't reference
         the 12V rail, the PWM the converter sees is floating garbage.

  [ ] 3. Confirm 12V supply polarity before powering the valve.

  =============================================================================
  WIRING
  =============================================================================

    12V SUPPLY (+) ----+---------------------------> valve RD (red, +12V)
                       |
                       +---------------------------> converter VIN

    12V SUPPLY (-) ----+--- STAR GROUND ------------> valve BK (black, -)
                       |        |
                       |        +------------------> converter GND (in + out)
                       |        |
                       |        +------------------> Arduino GND
                       |
    ARDUINO D9 (PWM) ------------------------------> converter PWM IN
    converter VOUT (0-10V) ------------------------> valve GR (green, 0-10V IN)

    POSITION FEEDBACK (open-collector PWM):
       valve WT (white) --+--[ 10k ]-- Arduino 5V   (external pull-up, preferred)
                          |
                          +---------- Arduino D2     (digital read)
       (No 10k on hand? Set FB_USE_INTERNAL_PULLUP true and skip the resistor.
        Internal pull-up is weak so the duty reading may be slightly off, but
        it confirms the signal is present.)

    valve YW (yellow, error/fault output): not wired here. It's another OC
    output you could read the same way as white if you want fault detection.

  =============================================================================
  HOW TO USE
  =============================================================================
   - Open Serial Monitor at 115200 baud, line ending = Newline.
   - Type a number 0-100 and hit enter to command valve position (%).
       0   = fully closed (0V command)
       100 = fully open   (10V command)
   - The sketch prints: commanded %, commanded voltage, and measured feedback
     duty %. (Feedback duty -> true position mapping needs calibration; see
     note in readFeedback() below.)

  NOTE on Arduino folder rule: the .ino must live in a folder of the SAME name
  (prop_valve_test_uno/prop_valve_test_uno.ino) or the toolchain won't open it.
*/

// ---------------- Pin assignments ----------------
const uint8_t PIN_PWM_OUT  = 9;   // PWM to converter (Uno PWM pins: 3,5,6,9,10,11)
const uint8_t PIN_FEEDBACK = 2;   // reads valve white-wire OC PWM

// Set true to use the Uno's internal pull-up instead of an external 10k.
// External pull-up to 5V is more accurate; internal is fine for a first check.
const bool FB_USE_INTERNAL_PULLUP = false;

// ---------------- Config ----------------
const float VALVE_FULL_SCALE_V = 10.0;   // converter output at 100% duty (after calibration)
const unsigned long PRINT_PERIOD_MS = 500;
const unsigned long PULSEIN_TIMEOUT_US = 50000UL; // 50ms; covers feedback down to ~20Hz

// Feedback duty endpoints from bench calibration (this specific valve).
// position% = (fb_duty - FB_DUTY_CLOSED) / (FB_DUTY_OPEN - FB_DUTY_CLOSED) * 100
const float FB_DUTY_CLOSED = 4.9;
const float FB_DUTY_OPEN   = 93.5;

// ---------------- State ----------------
int commandedPercent = 0;        // 0..100
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);

  pinMode(PIN_PWM_OUT, OUTPUT);
  if (FB_USE_INTERNAL_PULLUP) {
    pinMode(PIN_FEEDBACK, INPUT_PULLUP);
  } else {
    pinMode(PIN_FEEDBACK, INPUT);   // external 10k pull-up to 5V provides the high level
  }

  setValve(0);  // safe state on boot: commanded closed

  Serial.println(F("Proportional valve test ready."));
  Serial.println(F("Type 0-100 to command position (%). Start at 0."));
  Serial.println(F("Reminder: calibrate converter to 10V at 100% duty BEFORE wiring the valve."));
}

void loop() {
  // --- read a commanded percent from serial ---
  if (Serial.available()) {
    int val = Serial.parseInt();
    // parseInt returns 0 on junk; ignore obviously-empty reads
    if (val >= 0 && val <= 100) {
      commandedPercent = val;
      setValve(commandedPercent);
      Serial.print(F("-> commanded "));
      Serial.print(commandedPercent);
      Serial.println(F("%"));
    } else {
      Serial.println(F("ignored: enter 0-100"));
    }
    // flush any trailing chars (e.g. newline)
    while (Serial.available()) Serial.read();
  }

  // --- periodic status print ---
  if (millis() - lastPrint >= PRINT_PERIOD_MS) {
    lastPrint = millis();
    float cmdV = (commandedPercent / 100.0) * VALVE_FULL_SCALE_V;
    float fbDuty = readFeedbackDuty();   // -1 if no valid signal

    Serial.print(F("cmd="));
    Serial.print(commandedPercent);
    Serial.print(F("%  Vcmd="));
    Serial.print(cmdV, 2);
    Serial.print(F("V  fb="));
    if (fbDuty < 0) {
      Serial.println(F("--- (no signal / not wired)"));
    } else {
      Serial.print(fbDuty, 1);
      Serial.print(F("% duty  pos="));
      Serial.print(dutyToPosition(fbDuty), 1);
      Serial.println(F("%"));
    }
  }
}

// Convert raw feedback duty to valve position % using the bench-calibrated
// endpoints. Clamped to [0,100] so slight over/undershoot doesn't print weird.
float dutyToPosition(float duty) {
  float pos = (duty - FB_DUTY_CLOSED) * 100.0 / (FB_DUTY_OPEN - FB_DUTY_CLOSED);
  if (pos < 0.0)   pos = 0.0;
  if (pos > 100.0) pos = 100.0;
  return pos;
}

// Map 0-100% to PWM 0-255 and write it.
void setValve(int percent) {
  percent = constrain(percent, 0, 100);
  int pwm = map(percent, 0, 100, 0, 255);
  analogWrite(PIN_PWM_OUT, pwm);
}

// Measure duty cycle of the open-collector feedback PWM.
// Returns duty in % (0-100), or -1.0 if no transitions were seen.
//
// CALIBRATION NOTE: this returns raw duty %, NOT valve position %. The valve's
// datasheet may encode position differently (e.g. duty offset, inverted, or a
// fixed carrier). To map duty -> real position: command several known
// positions (0/25/50/75/100), record the duty here, and fit the relationship.
float readFeedbackDuty() {
  unsigned long highT = pulseIn(PIN_FEEDBACK, HIGH, PULSEIN_TIMEOUT_US);
  unsigned long lowT  = pulseIn(PIN_FEEDBACK, LOW,  PULSEIN_TIMEOUT_US);

  if (highT == 0 && lowT == 0) return -1.0;     // no edges: line static or unwired
  if (highT == 0) return 0.0;                   // stuck low  = 0% duty
  if (lowT == 0)  return 100.0;                 // stuck high = 100% duty

  unsigned long period = highT + lowT;
  return (100.0 * highT) / period;
}
