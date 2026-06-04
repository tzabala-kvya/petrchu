// ============================================================================
// PetrChu - MS5837 calibration + EEPROM tool (Arduino Mega)
// ----------------------------------------------------------------------------
// One-time bench utility that captures the local atmospheric pressure with
// the MS5837 sensor dry (mounted but not immersed) and stores the result in
// EEPROM. The main firmware reads that offset at boot and subtracts it from
// every live MS5837 reading to convert raw pressure -> water depth above the
// sensor.
//
// EEPROM LAYOUT (matches firmware/mega/draft/config_block_draft.h):
//   addr 0  : magic uint16 = 0xCA1B  (marker that EEPROM has been written)
//   addr 2  : atm offset   float     (millibar, what the dry sensor reads)
//   addr 6  : reserved (load-side ACS712 zero, written by separate util)
//   addr 10 : V_setpoint default int16 centivolts (written by Pi via main FW)
//
// PROCEDURE (do this once per rig):
//   1. Wire the MS5837:
//        VCC   -> Mega 3.3V    (NOT 5V - the chip is 3.3V max)
//        GND   -> Mega GND
//        SDA   -> Mega d20 (SDA)
//        SCL   -> Mega d21 (SCL)
//      Pull-ups: many MS5837 breakouts include 10k pull-ups on SDA/SCL. If
//      yours does not, add 4.7k from each line to 3.3V.
//   2. Mount the sensor in the lower tank where it will live during operation,
//      but leave the tank EMPTY so the sensor is reading air.
//   3. Flash this sketch to the Mega.
//   4. Open Serial Monitor at 115200 baud. Set line ending to "Newline".
//   5. Type 'live' and press Enter. You should see pressure readings around
//      1013 mbar (sea level) +/- ~30 mbar for your altitude. If the number
//      looks crazy (0, 6000, NaN), recheck wiring before continuing.
//   6. Type 'cal' and press Enter. The sketch averages CAL_SAMPLES readings
//      over CAL_SAMPLE_MS milliseconds, writes the result + magic to EEPROM,
//      and reads it back to confirm.
//   7. Type 'read' to confirm the EEPROM contents on next power-up.
//   8. Power-cycle the Mega and type 'read' again to verify the offset
//      persisted across reboot.
//
// SERIAL COMMANDS:
//   live   - stream pressure / temp every 500 ms (Ctrl-A or any char to stop)
//   cal    - run the cal: average + write + verify
//   read   - dump EEPROM magic + atm offset
//   clear  - wipe EEPROM addrs 0..5 (re-arms 'cal' on a fresh slate)
//   help   - reprint the command list
//
// SAFETY: this sketch does not drive any outputs. It is read-only on the
// sensor and writes only to its own EEPROM slots. Safe to run anytime.
//
// LIBRARY: install BlueRobotics MS5837 via Arduino Library Manager
//   (Tools -> Manage Libraries -> search "MS5837" -> Blue Robotics MS5837).
//   NOTE: this sketch is configured for the MS5837-02BA. The library does NOT
//   autodetect the part - it defaults to the -30BA and will mis-scale readings
//   for our -02BA unless setModel(MS5837::MS5837_02BA) is called (done in
//   initSensor() below). If you ever swap to a -30BA, change that line.
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <MS5837.h>

// ---------- CONFIG ----------
const int      EEPROM_MAGIC_ADDR = 0;       // 2 bytes
const int      EEPROM_ATM_ADDR   = 2;       // 4 bytes float
const uint16_t EEPROM_MAGIC      = 0xCA1B;

// Calibration sampling. 200 samples * 50 ms = 10 s of averaging - enough to
// beat down sensor noise (typically ~0.2 mbar 1-sigma at 1 Hz).
const uint16_t CAL_SAMPLES   = 200;
const uint16_t CAL_SAMPLE_MS = 50;

// Live readout cadence
const unsigned long LIVE_PRINT_MS = 500;

// Fluid density for depth conversion. 997 kg/m^3 = freshwater at ~25 C.
const float FLUID_DENSITY_KG_M3 = 997.0f;

// Standard gravity, used to convert a pressure head (Pa) to a column height.
const float GRAVITY_M_S2 = 9.80665f;

// Fallback atmospheric pressure when no EEPROM cal is stored (standard atm).
const float ATM_DEFAULT_MBAR = 1013.25f;

// ---------- STATE ----------
MS5837 sensor;
static bool g_sensor_ok = false;
static bool g_live_mode = false;
static unsigned long g_last_live_ms = 0;

// ---------- EEPROM helpers ----------
static bool eepromHasValidCal() {
  uint16_t magic = 0;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  return (magic == EEPROM_MAGIC);
}

static float eepromReadAtm() {
  float v = NAN;
  EEPROM.get(EEPROM_ATM_ADDR, v);
  return v;
}

static void eepromWriteAtm(float atm_mbar) {
  EEPROM.put(EEPROM_ATM_ADDR,   atm_mbar);
  EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
}

static void eepromClear() {
  for (int i = 0; i < 6; i++) EEPROM.update(i, 0xFF);
}

// ---------- Sensor helpers ----------
static bool initSensor() {
  Wire.begin();
  if (!sensor.init()) return false;
  // IMPORTANT: the Blue Robotics MS5837 library does NOT autodetect the model.
  // It defaults to MS5837_30BA and applies 30BA conversion math. Our sensor is
  // the -02BA, so we must set the model explicitly or pressure()/depth() will
  // be wrong (different scaling/bit-shifts in the second-order temp comp).
  sensor.setModel(MS5837::MS5837_02BA);
  // Set fluid density for depth math.
  sensor.setFluidDensity(FLUID_DENSITY_KG_M3);
  return true;
}

static float readPressureMbar() {
  sensor.read();
  return sensor.pressure();   // mbar
}

static float readTemperatureC() {
  return sensor.temperature();
}

// Calibration-aware depth: subtracts the stored EEPROM atmospheric offset
// (captured by 'cal' with the sensor dry) instead of the library's hardcoded
// 101300 Pa standard atmosphere. Falls back to ATM_DEFAULT_MBAR if no cal is
// stored, so this still produces a reasonable number on an uncalibrated rig.
//   head_Pa = (p_mbar - atm_mbar) * 100      [mbar -> Pa]
//   depth_cm = head_Pa / (rho * g) * 100     [m -> cm]
static float depthCmCalibrated(float p_mbar) {
  const float atm_mbar = eepromHasValidCal() ? eepromReadAtm() : ATM_DEFAULT_MBAR;
  const float head_pa  = (p_mbar - atm_mbar) * 100.0f;
  return head_pa / (FLUID_DENSITY_KG_M3 * GRAVITY_M_S2) * 100.0f;
}

// ---------- Commands ----------
static void cmdRead() {
  Serial.println(F("# --- EEPROM contents ---"));
  uint16_t magic = 0;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  Serial.print(F("# magic @ addr ")); Serial.print(EEPROM_MAGIC_ADDR);
  Serial.print(F(" = 0x"));            Serial.print(magic, HEX);
  Serial.print(F("  ("));
  Serial.print(magic == EEPROM_MAGIC ? F("VALID") : F("UNINITIALIZED"));
  Serial.println(F(")"));

  if (magic == EEPROM_MAGIC) {
    float atm = eepromReadAtm();
    Serial.print(F("# atm offset @ addr ")); Serial.print(EEPROM_ATM_ADDR);
    Serial.print(F(" = "));                  Serial.print(atm, 3);
    Serial.println(F(" mbar"));
  } else {
    Serial.println(F("# no cal stored. Run 'cal' with sensor dry."));
  }
}

static void cmdCal() {
  if (!g_sensor_ok) {
    Serial.println(F("# ERROR: sensor not initialized. Check wiring + power."));
    return;
  }

  Serial.print(F("# starting cal: averaging "));
  Serial.print(CAL_SAMPLES);
  Serial.print(F(" samples over ~"));
  Serial.print((unsigned long)CAL_SAMPLES * CAL_SAMPLE_MS / 1000UL);
  Serial.println(F(" seconds. Keep the sensor DRY and STILL."));

  double sum = 0.0;
  float  pmin =  1e9f;
  float  pmax = -1e9f;

  for (uint16_t i = 0; i < CAL_SAMPLES; i++) {
    const float p = readPressureMbar();
    sum += p;
    if (p < pmin) pmin = p;
    if (p > pmax) pmax = p;

    // Progress dots every 10 samples
    if (i % 10 == 0) { Serial.print('.'); }
    delay(CAL_SAMPLE_MS);
  }
  Serial.println();

  const float mean   = (float)(sum / (double)CAL_SAMPLES);
  const float spread = pmax - pmin;

  Serial.print(F("# mean = "));   Serial.print(mean, 3);   Serial.println(F(" mbar"));
  Serial.print(F("# min  = "));   Serial.print(pmin, 3);   Serial.println(F(" mbar"));
  Serial.print(F("# max  = "));   Serial.print(pmax, 3);   Serial.println(F(" mbar"));
  Serial.print(F("# spread = ")); Serial.print(spread, 3); Serial.println(F(" mbar"));

  // Sanity: real-world atm at sea level ~1013 mbar; high altitude down to ~700.
  // If we see anything wildly outside 600..1100, the sensor probably isn't
  // sitting in air (immersed? blocked?). Refuse to write.
  if (mean < 600.0f || mean > 1100.0f) {
    Serial.println(F("# ERROR: mean outside 600..1100 mbar range."));
    Serial.println(F("#        Sensor probably not in free air. EEPROM NOT written."));
    return;
  }
  if (spread > 2.0f) {
    Serial.println(F("# WARN: spread > 2 mbar across samples - sensor may be in"));
    Serial.println(F("#       moving air or vibrating. Writing anyway, but consider"));
    Serial.println(F("#       redoing the cal in a still environment."));
  }

  eepromWriteAtm(mean);

  // Verify by reading back
  Serial.println(F("# verifying EEPROM..."));
  if (!eepromHasValidCal()) {
    Serial.println(F("# ERROR: magic readback failed. EEPROM hardware issue?"));
    return;
  }
  const float readback = eepromReadAtm();
  Serial.print(F("# readback = ")); Serial.print(readback, 3); Serial.println(F(" mbar"));
  if (fabs(readback - mean) > 0.01f) {
    Serial.println(F("# ERROR: readback mismatch."));
    return;
  }
  Serial.println(F("# CAL COMPLETE. Power-cycle the Mega and run 'read' to confirm persistence."));
}

static void cmdLive() {
  g_live_mode = true;
  g_last_live_ms = 0;   // print first row immediately
  Serial.println(F("# live mode ON. Send any character to stop."));
  Serial.print(F("# depth_cm uses "));
  if (eepromHasValidCal()) {
    Serial.print(F("STORED cal offset "));
    Serial.print(eepromReadAtm(), 2);
    Serial.println(F(" mbar"));
  } else {
    Serial.print(F("DEFAULT atm "));
    Serial.print(ATM_DEFAULT_MBAR, 2);
    Serial.println(F(" mbar (no cal stored - run 'cal')"));
  }
  Serial.println(F("# format: t_ms, p_mbar, t_C, depth_cm"));
}

static void cmdHelp() {
  Serial.println(F("# commands: live  cal  read  clear  help"));
}

static void cmdClear() {
  eepromClear();
  Serial.println(F("# EEPROM addrs 0..5 wiped (set to 0xFF). 'cal' will start fresh."));
}

// ---------- Serial parser ----------
static void serviceSerial() {
  if (!Serial.available()) return;

  if (g_live_mode) {
    while (Serial.available()) Serial.read();  // drain
    g_live_mode = false;
    Serial.println(F("# live mode OFF."));
    return;
  }

  String s = Serial.readStringUntil('\n');
  s.trim();
  s.toLowerCase();
  if (s.length() == 0) return;

  if      (s == "live")  cmdLive();
  else if (s == "cal")   cmdCal();
  else if (s == "read")  cmdRead();
  else if (s == "clear") cmdClear();
  else if (s == "help")  cmdHelp();
  else {
    Serial.print(F("# unknown command: '")); Serial.print(s); Serial.println(F("'"));
    cmdHelp();
  }
}

// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println(F("# MS5837 cal + EEPROM tool ready"));
  Serial.println(F("# pins: SDA=d20, SCL=d21, VCC=3.3V (not 5V!), GND=GND"));

  g_sensor_ok = initSensor();
  if (!g_sensor_ok) {
    Serial.println(F("# ERROR: MS5837 init failed."));
    Serial.println(F("#   - check 3.3V supply (NOT 5V)"));
    Serial.println(F("#   - check SDA/SCL on d20/d21"));
    Serial.println(F("#   - check I2C pull-ups (4.7k to 3.3V if breakout has none)"));
    Serial.println(F("# Commands 'read' and 'clear' still work without the sensor."));
  } else {
    Serial.println(F("# sensor OK"));
  }

  cmdRead();
  cmdHelp();
}

void loop() {
  serviceSerial();

  if (g_live_mode && g_sensor_ok && (millis() - g_last_live_ms >= LIVE_PRINT_MS)) {
    g_last_live_ms = millis();
    const float p = readPressureMbar();
    const float t = readTemperatureC();
    const float d_cm = depthCmCalibrated(p);  // uses stored EEPROM atm offset

    Serial.print(millis()); Serial.print(',');
    Serial.print(p, 3);     Serial.print(',');
    Serial.print(t, 2);     Serial.print(',');
    Serial.println(d_cm, 2);
  }
}
