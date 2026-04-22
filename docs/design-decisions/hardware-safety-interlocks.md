# Decision: Hardware E-Stop Interlock Independent of Firmware

## Context
The system needs an emergency stop that guarantees all actuators (solenoids, contactors) are de-energized when the operator hits the E-stop button. This is a life-safety function — it must work even if the Arduino has crashed, is stuck in an infinite loop, or has a firmware bug.

## Options Considered

| Option | Pros | Cons |
|--------|------|------|
| Software-only E-stop (Arduino reads button, sets pins LOW) | Simple wiring, flexible logic | Firmware crash = E-stop doesn't work. Single point of failure in the most critical safety path. |
| Hardware relay in series with actuator 12V rail, coil controlled by E-stop button | Works regardless of firmware state. Cannot be bypassed by software. Meets NFPA 79 §9.2.5.3. | Adds a relay and wiring. Slightly more complex electrical design. |
| Redundant hardware + software (hardware relay AND firmware detection) | Two independent layers. Hardware guarantees shutdown; firmware logs the event and manages recovery. | Most complex, but complexity is in the right place (safety). |

## Decision
**Redundant hardware + software (Option 3).** The E-stop button is wired in series with the 12V relay coil circuit. Pressing E-stop physically breaks the coil circuit, de-energizing the relay. The relay's normally-open contacts open, cutting 12V to ALL actuator loads. This happens in <10ms (relay dropout time) regardless of what the Arduino is doing.

Separately, the Arduino reads the E-stop status on a digital input pin and transitions the state machine to E-STOP mode for logging and controlled recovery. But this is a secondary layer — the hardware interlock is the primary safety mechanism.

## Consequences
- The E-stop circuit must use a relay rated for the total actuator load current
- Recovery from E-stop requires physical button release AND arm switch cycle (seal-in circuit topology prevents automatic restart)
- The Arduino cannot override, delay, or suppress the E-stop — by design
- All solenoid valves must be normally-closed (spring return) so they fail shut when power is cut
- All contactors must be normally-open so they drop out when power is cut
