# Decision: Serial-Triggered Pressure Zeroing Instead of Auto-Zero at Boot

## Context
The MS5837-02BA pressure sensor measures absolute pressure relative to a factory calibration. For accurate depth/head measurements, we need to subtract the ambient atmospheric pressure as a zero reference. The question is when and how to capture this reference.

## Options Considered

| Option | Pros | Cons |
|--------|------|------|
| Auto-zero at boot (capture ambient pressure in setup()) | Simple, automatic, no operator action needed | **Unsafe:** If the Arduino resets while the sensor is submerged (brownout, watchdog, manual reset), the zero reference captures the submerged pressure instead of atmospheric. All subsequent readings are wrong with no indication of error. |
| Serial-triggered zeroing (operator sends 'zero' command) | Operator explicitly confirms sensor is at surface/ambient before zeroing. Cannot be corrupted by unexpected reboot. | Requires operator action. Slightly more complex firmware. |
| Factory calibration only (no field zeroing) | Simplest code | Atmospheric pressure varies by ~30 mbar day-to-day. Uncompensated drift degrades accuracy. |

## Decision
**Serial-triggered zeroing.** The operator positions the sensor at the surface (atmospheric reference), opens Serial Monitor, and sends the `zero` command. The firmware captures the current reading as the zero offset and stores it in RAM. This value is used to subtract from all subsequent readings.

If the Arduino reboots unexpectedly, the zero offset resets to 0 (no offset), and readings revert to absolute pressure. This is a safe failure mode — the readings are still physically meaningful, just not depth-referenced. The operator can re-zero when convenient.

## Consequences
- The `zero` command is added to the serial command interface
- Boot sequence does NOT auto-zero — it explicitly warns "Pressure zero not set — send 'zero' at surface"
- Documentation must instruct the operator to zero before testing
- A future enhancement could store the zero offset in EEPROM, but for PoC, RAM is sufficient
