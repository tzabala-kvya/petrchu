# Setup Guide

How to build and run the PetrChu controls system from scratch.

## Hardware Required
<!-- TODO: reference BOM in hardware/bom/ -->

## Wiring
<!-- TODO: reference wiring diagrams in hardware/wiring/ -->

## Software Setup

### Arduino Mega 2560
1. Install Arduino IDE 2.x
2. Install required libraries:
   - BlueRobotics MS5837 library
   - (others TBD)
3. Open `firmware/mega/mega.ino`
4. Select Board: **Arduino Mega or Mega 2560**
5. Select the correct COM port
6. Upload

### Raspberry Pi 5
1. Flash Raspberry Pi OS (64-bit)
2. Install Python dependencies:
   ```bash
   pip install pyserial matplotlib
   ```
3. Connect Pi to Mega via USB (through ADUM3160 isolator)
4. Run: `python3 firmware/pi/supervisor.py`

## First Power-On Checklist
<!-- TODO: write power-on procedure -->
- [ ] Verify 12V rail voltage with multimeter
- [ ] Confirm E-stop kills coil circuit (test with relay click)
- [ ] Confirm arm switch enables relay (key ON → relay energizes)
- [ ] Check Serial Monitor at 115200 baud for boot messages
