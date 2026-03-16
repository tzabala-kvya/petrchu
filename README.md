# PetrChu

**Hybrid Mechanical Energy Storage System — Compressed Air + Pumped Hydro**

UCI MAE 151A/B Senior Capstone Project | 2025–2026

---

## What Is This?

PetrChu is a benchtop-scale hybrid energy storage system that combines **compressed air energy storage (CAES)** and **pumped hydro storage** into a single platform. The system uses dual prime movers — a reciprocating air piston engine and a jet-nozzle impulse water turbine — connected through overrunning clutches to a shared alternator.

The goal: demonstrate that mechanical energy storage is viable without chemical batteries.

## System Overview

<!-- TODO: Add a system block diagram image here -->
<!-- ![System Block Diagram](media/images/system-block-diagram.png) -->

- **CAES subsystem:** Compressed air reservoir → reciprocating piston engine → alternator
- **Pumped hydro subsystem:** Elevated water reservoir → jet nozzle → impulse turbine → alternator
- **Shared powertrain:** Overrunning clutches allow either prime mover to drive the alternator independently or simultaneously
- **Controls:** Arduino Mega 2560 (primary controller) + Raspberry Pi 5 (supervisor/logger) implementing a 10-mode state machine with hardware safety interlocks

## Repository Structure

```
petrchu/
├── firmware/          # All microcontroller and supervisor code
│   ├── mega/          # Arduino Mega 2560 main firmware
│   ├── pi/            # Raspberry Pi 5 Python supervisor scripts
│   └── tests/         # Test sketches for individual sensors/actuators
│
├── hardware/          # Physical design documentation
│   ├── schematics/    # Circuit diagrams and electrical schematics
│   ├── wiring/        # Wiring diagrams, pinout tables
│   ├── panel-layouts/ # Control panel and demo panel SVG layouts
│   └── bom/           # Bill of materials (CSV + spreadsheets)
│
├── docs/              # Project documentation
│   ├── design-decisions/  # Why we chose X over Y
│   └── test-plans/        # PoC and integration test procedures
│
├── data/              # Test data and calibration records
│   ├── calibration/
│   └── test-results/
│
└── media/             # Photos, diagrams, images for docs
    └── images/
```

## Key Hardware

| Component | Role |
|-----------|------|
| Arduino Mega 2560 | Primary controller — state machine, PID, sensor I/O |
| Raspberry Pi 5 | Supervisor, data logging, HDMI dashboard |
| MS5837-02BA | Depth/pressure sensor (I2C) |
| IRLB8721 N-ch MOSFET | Logic-level switching for valves, relays, pump |
| SCT-013 30A/1V | Non-invasive current transformer for alternator output |
| YF-S201 | Flow sensor (pulse output) |
| DC Contactor | Main power switching with hardware interlock |

## Key Design Principles

- **Hardware-level safety interlocks** — E-stop and arm switch are in the electrical circuit, not just firmware inputs. The system is safe even if the Arduino crashes.
- **Galvanic isolation** — 12V control domain and generated power domain are electrically separated.
- **No chemical batteries** — the entire point is demonstrating mechanical-only storage.

## Getting Started

<!-- TODO: Fill this in as the build progresses -->

### Prerequisites
- Arduino IDE 2.x (or PlatformIO)
- Python 3.x on the Raspberry Pi

### Required Arduino Libraries
Install these through the Arduino IDE Library Manager (Sketch → Include Library → Manage Libraries), 
or clone from GitHub into your Arduino/libraries/ folder:

| Library | Source | Used For |
|---------|--------|----------|
| BlueRobotics MS5837 | [GitHub](https://github.com/bluerobotics/BlueRobotics_MS5837_Library) | MS5837-02BA water pressure sensor (I2C) |

### Upload Firmware
```bash
# Open firmware/mega/mega.ino in Arduino IDE
# Select Board: Arduino Mega 2560
# Select correct COM port
# Upload
```

### Run Pi Supervisor
```bash
cd firmware/pi/
python3 supervisor.py
```

## Design Decisions

See [`docs/design-decisions/`](docs/design-decisions/) for detailed write-ups on major choices, including:
- Why IRLB8721 over IRF510N (logic-level MOSFET compatibility)
- Why hardware interlocks instead of firmware-only safety
- Why Serial-triggered pressure zeroing instead of auto-zero at boot
- Sensor selection and I2C bus architecture

## Team

| Name | Role |
|------|------|
| Tristan | Controls & Electrical |
| Brennan | CAES Subsystem |
| Philip | Pumped Hydro Subsystem |
| Jasmine | Alternator |
| Derek | Alternator |
| Ethan | Clutching System|

**Sponsor:** Abdelrahman Elmaradny

## License

<!-- Choose one: -->
<!-- MIT License — see [LICENSE](LICENSE) -->
<!-- Or remove this section if keeping it private -->
