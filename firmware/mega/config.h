// ============================================
// PetrChu — Configuration & Pin Assignments
// Arduino Mega 2560 Primary Controller
// ============================================
// 
// All pin assignments, sensor calibration values, PID tuning 
// constants, and system thresholds live here. Change them in 
// ONE place instead of hunting through the main sketch.
//
// Last updated: YYYY-MM-DD
// Updated by:   [name]
// ============================================

#ifndef CONFIG_H
#define CONFIG_H

// ----- Pin Assignments: Digital Outputs -----
// MOSFET-driven loads (active HIGH through IRLB8721 drivers)
const int PUMP_RELAY_PIN       = 0;   // TODO: assign actual pin
const int SOLENOID_VALVE_1_PIN = 0;   // TODO: assign actual pin
const int SOLENOID_VALVE_2_PIN = 0;   // TODO: assign actual pin
const int STATUS_LED_PIN       = 13;  // onboard LED for heartbeat
const int BUZZER_PIN           = 0;   // TODO: assign actual pin

// ----- Pin Assignments: Digital Inputs -----
const int ARM_SWITCH_PIN       = 0;   // TODO: key switch, active HIGH
const int ESTOP_STATUS_PIN     = 0;   // TODO: reads E-stop circuit state
const int FLOW_SENSOR_PIN      = 0;   // TODO: YF-S201 pulse input (needs interrupt-capable pin)
const int RPM_HALL_PIN         = 0;   // TODO: Hall effect RPM sensor (interrupt-capable)

// ----- Pin Assignments: Analog Inputs -----
const int PRESSURE_XDUCER_PIN = A0;   // TODO: pressure transducer analog output
const int ALTERNATOR_V_PIN    = A1;   // TODO: voltage divider from alternator output
const int CURRENT_SENSE_PIN   = A2;   // TODO: SCT-013 with burden resistor + bias

// ----- Pin Assignments: I2C (shared bus on A4/A5) -----
// MS5837-02BA depth sensor — address 0x76 (fixed)
// 1602A LCD via I2C backpack — address 0x27 (typical, check with I2C scanner)
// BSS138-based level shifter between 5V Mega and 3.3V sensor

// ----- Sensor Calibration -----
const float VOLTS_PER_COUNT       = 5.0 / 1023.0;
const float PRESSURE_OFFSET_V     = 0.5;    // TODO: measure actual zero-pressure output
const float PRESSURE_SCALE_PSI    = 30.0;   // TODO: verify sensor full-scale range

// ----- PID Tuning -----
const float PID_KP = 2.0;   // TODO: tune during PoC testing
const float PID_KI = 0.5;
const float PID_KD = 0.1;

// ----- Control Thresholds -----
const float PUMP_ON_PRESSURE_PSI  = 10.0;   // TODO: set actual threshold
const float PUMP_OFF_PRESSURE_PSI = 14.0;   // TODO: set actual threshold (hysteresis band)
const unsigned long WATCHDOG_TIMEOUT_MS = 5000;  // max time without valid sensor reading

// ----- System Constants -----
const unsigned long TELEMETRY_INTERVAL_MS = 500;   // UART reporting rate
const unsigned long SENSOR_SAMPLE_INTERVAL_MS = 50; // sensor polling rate

#endif // CONFIG_H
