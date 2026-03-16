# Decision: IRLB8721 over IRF510N for MOSFET Switching

## Context
We need N-channel MOSFETs to switch 12V inductive loads (solenoid valves, relay coils, pump motor) from the Arduino Mega's 5V digital outputs. The MOSFET must fully turn on with a 5V gate signal — no external gate driver.

## Options Considered

| Option | Pros | Cons |
|--------|------|------|
| IRF510N | Cheap, widely available | V_GS(th) is 2–4V but needs 10V gate drive for full saturation. At 5V gate, it's barely on — high R_DS(on), gets hot, unreliable switching. |
| IRLB8721 | Logic-level: fully saturated at 4.5V gate. R_DS(on) = 8.7mΩ at V_GS=4.5V. Cheap ($0.80). | Slightly lower max V_DS (30V vs 100V), but we only need 12V. |
| IRL540N | Also logic-level, higher current rating. | Overkill for our current levels, larger package. |

## Decision
**IRLB8721.** It's specifically designed for logic-level (5V) gate drive, which is exactly what the Arduino Mega outputs. At 5V gate, it's fully on with milliohm-level resistance — essentially a closed switch. The IRF510N is *not* a logic-level FET despite the low threshold voltage spec, and would cause unreliable operation and heat buildup.

## Consequences
- All MOSFET driver circuits use IRLB8721 with 10kΩ gate-to-source pulldown and 1N4007 flyback diode across each inductive load (cathode to +12V rail).
- If we ever move to a 3.3V controller, we'd need to re-evaluate — the IRLB8721 wants at least 4.5V gate for full performance.
