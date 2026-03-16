# Troubleshooting

Known issues and how to fix them.

## Sensor Issues

### MS5837 not responding on I2C
- Check wiring: SDA → A4, SCL → A5 (through BSS138 level shifter)
- Run I2C scanner sketch (`firmware/tests/i2c_scanner.ino`) — should see address 0x76
- If nothing shows up: check level shifter wiring, verify 3.3V supply to sensor
- Long cable runs (>1m): lower pull-up resistors to ~1kΩ, use CAT5e twisted pairs

### Pressure reading drifts or is wrong after restart
- The `zero` command must be issued while the sensor is at surface/ambient pressure
- Auto-zeroing at boot was deliberately removed — if Arduino restarts while sensor is submerged, the zero reference would be wrong
- Fix: bring sensor to surface, open Serial Monitor, send `zero`

## Electrical Issues

### MOSFET not switching load
- Verify you're using IRLB8721, NOT IRF510N — the IRF510N won't fully turn on at 5V gate
- Check gate-to-source pulldown resistor (10kΩ) is present
- Measure gate voltage with multimeter: should be ~5V when Arduino pin is HIGH

### E-stop doesn't kill the system
- E-stop must be wired in series with the relay coil circuit — it's a hardware interlock, not a software input
- If wired correctly: pressing E-stop physically breaks the coil circuit, relay drops out regardless of Arduino state
- Check: with E-stop pressed, measure continuity across E-stop terminals — should be open

## Software Issues

### Serial Monitor shows garbage
- Baud rate mismatch: firmware uses 115200
- Make sure only one program is connected to the COM port at a time

### Can't upload to Arduino
- Close Serial Monitor and any Python scripts using the COM port
- Try a different USB cable
- Check Device Manager for the correct COM port
