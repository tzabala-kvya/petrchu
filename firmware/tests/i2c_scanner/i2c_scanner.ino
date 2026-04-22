// ============================================
// PetrChu — I2C Bus Scanner
// ============================================
//
// Scans the I2C bus and reports all devices found.
// Use this to verify:
//   - MS5837 pressure sensor at address 0x76
//   - 1602A LCD backpack at address 0x27 (typical)
//   - Level shifter is working (no devices = wiring issue)
//
// Wiring:
//   SDA = D20 (through BSS138 level shifter if 3.3V device)
//   SCL = D21 (through BSS138 level shifter if 3.3V device)
//
// Run this FIRST when debugging any I2C issue.
// ============================================

#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(115200);
  
  Serial.println(F(""));
  Serial.println(F("PetrChu I2C Scanner"));
  Serial.println(F("Scanning..."));
}

void loop() {
  int devicesFound = 0;
  
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print(F("  Device found at 0x"));
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Identify known devices
      if (address == 0x76) Serial.print(F("  ← MS5837 pressure sensor"));
      if (address == 0x27) Serial.print(F("  ← LCD backpack (typical)"));
      if (address == 0x3F) Serial.print(F("  ← LCD backpack (alternate)"));
      
      Serial.println();
      devicesFound++;
    }
  }
  
  if (devicesFound == 0) {
    Serial.println(F("  No devices found!"));
    Serial.println(F("  Check: SDA/SCL wiring, level shifter, 3.3V power to sensor"));
  } else {
    Serial.print(F("  Total: "));
    Serial.print(devicesFound);
    Serial.println(F(" device(s)"));
  }
  
  Serial.println(F(""));
  delay(5000);  // Scan every 5 seconds
}
