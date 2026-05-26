// ============================================================================
// control_helpers_draft.h
// ----------------------------------------------------------------------------
// DRAFT control primitives for the V-regulation rewrite of TestStandFirmware.ino.
//
// Three helpers, each in its own section:
//   1. PID                — slew-rate-limited PI with anti-windup + bumpless init
//   2. Overvoltage handler — asymmetric closure (closes faster than opens)
//   3. Engagement check    — OWB lock/freewheel detection per drivetrain side
//
// >>> REVIEW: drafts, untested on hardware.
// >>> REVIEW: anti-windup uses conditional integration. Back-calculation is an
// >>>         alternative — review and pick before committing.
// >>> REVIEW: all timing uses millis(), so loop dt resolution is ~1 ms. At
// >>>         LOOP_PERIOD_MS=10 that's fine. If the loop ever runs faster
// >>>         (e.g., 1 ms), switch to micros().
// ============================================================================

#ifndef CONTROL_HELPERS_DRAFT_H
#define CONTROL_HELPERS_DRAFT_H

#include <Arduino.h>
#include "config_block_draft.h"

// ============================================================================
// 1. PID — slew-rate-limited PI with anti-windup
// ============================================================================
// Used by:
//   - Air-side V regulation during S_DISCHARGE_AIR and DISCHARGE_BOTH (LOCKED)
//   - Water-side V regulation during S_DISCHARGE_WATER and DISCHARGE_BOTH
//
// Anti-windup strategy: conditional integration. If the output would saturate
// AND the error sign would deepen the saturation, don't accumulate.
//   >>> REVIEW: alternative is back-calculation (subtract clamped overshoot
//   >>>         from integral). Conditional is simpler and adequate at 100 Hz.
//
// Slew limiting:
//   - Caps how fast the commanded servo angle can change per second.
//   - Same limit applied symmetrically (open and close).
//   - To force fast-closure (e.g., overvoltage), the caller overrides
//     prev_output directly — see ov_check() in section 2.

struct PIDState {
    // Tuning — set once at init, do not change at runtime
    float kp;
    float ki;
    float kd;                       // typically 0; V_alt noise blows D up
    float out_min;
    float out_max;
    float slew_max_per_s;           // max output change per second

    // Running state
    float    integral;
    float    prev_error;
    float    prev_output;
    uint32_t last_ms;               // millis() of last pid_compute() call
};

// Initialize a PIDState. Must be called before first pid_compute().
static inline void pid_init(PIDState &p,
                            float kp, float ki, float kd,
                            float out_min, float out_max,
                            float slew_max_per_s) {
    p.kp = kp;
    p.ki = ki;
    p.kd = kd;
    p.out_min = out_min;
    p.out_max = out_max;
    p.slew_max_per_s = slew_max_per_s;
    p.integral = 0;
    p.prev_error = 0;
    p.prev_output = out_min;
    p.last_ms = 0;                  // sentinel: first compute skips dt-dependent terms
}

// Reset running state only — keeps tuning intact.
// Use when entering a state where the loop should start fresh (e.g., re-arming).
static inline void pid_reset(PIDState &p) {
    p.integral = 0;
    p.prev_error = 0;
    p.last_ms = 0;
    // prev_output intentionally left alone — caller can set it for bumpless transfer
}

// Bumpless transfer: enter the loop with a chosen initial output so the
// first computed step is small. Used when adding a side mid-test
// (e.g., DISCHARGE_BOTH SPINUP → LOCKED transition).
static inline void pid_bumpless_init(PIDState &p, float initial_output) {
    pid_reset(p);
    p.prev_output = constrain(initial_output, p.out_min, p.out_max);
}

// Compute one PID step.
//   setpoint    — desired value (V setpoint in volts)
//   measurement — current value (V_alt in volts)
//   now_ms      — current millis()
//
// Returns the new clamped, slew-limited output (servo angle in degrees).
// First call after init/reset returns P-term only (dt=0 disables I and D).
static float pid_compute(PIDState &p,
                          float setpoint,
                          float measurement,
                          uint32_t now_ms) {
    float err = setpoint - measurement;

    // --- Compute dt ---
    // last_ms == 0 means this is the first call after init/reset — skip I and D
    // by setting dt_s = 0.
    float dt_s = 0.0f;
    if (p.last_ms != 0) {
        dt_s = (float)(now_ms - p.last_ms) / 1000.0f;
        // Guard against pathological dt (clock weirdness, long loop hangs).
        // >>> REVIEW: 0.5s cap is arbitrary; normal LOOP_PERIOD_MS=10 → dt=0.010.
        // >>>         Anything >100 ms is already a yellow flag.
        if (dt_s > 0.5f) dt_s = 0.5f;
    }
    p.last_ms = now_ms;

    // --- P term ---
    float p_term = p.kp * err;

    // --- I term (candidate — committed only if not winding up) ---
    float i_candidate = p.integral + p.ki * err * dt_s;

    // --- D term (off by default; left in for future tuning) ---
    float d_term = 0.0f;
    if (p.kd != 0.0f && dt_s > 0.0f) {
        d_term = p.kd * (err - p.prev_error) / dt_s;
    }

    float u_raw = p_term + i_candidate + d_term;

    // --- Output clamp ---
    float u_clamped = u_raw;
    bool saturated_high = false;
    bool saturated_low  = false;
    if (u_clamped > p.out_max) { u_clamped = p.out_max; saturated_high = true; }
    if (u_clamped < p.out_min) { u_clamped = p.out_min; saturated_low  = true; }

    // --- Anti-windup (conditional integration) ---
    // Only accept the integral update if not saturated, OR if the error sign
    // points back toward valid range.
    bool windup_blocked =
        (saturated_high && err > 0.0f) ||
        (saturated_low  && err < 0.0f);
    if (!windup_blocked) {
        p.integral = i_candidate;
    }

    // --- Slew rate limit ---
    if (dt_s > 0.0f) {
        float max_step = p.slew_max_per_s * dt_s;
        float delta = u_clamped - p.prev_output;
        if (delta >  max_step) u_clamped = p.prev_output + max_step;
        if (delta < -max_step) u_clamped = p.prev_output - max_step;
    }

    p.prev_error  = err;
    p.prev_output = u_clamped;
    return u_clamped;
}

// Feed-forward initial servo angle for bumpless transfer.
// Used when adding a side mid-test: rather than starting at out_min and letting
// the integral wind up to the right angle, predict an approximate angle from
// the current voltage target and let PID trim from there.
//
// >>> REVIEW: this is a very rough linear model — pressure → flow → torque → V
// >>>         is genuinely nonlinear. A measured lookup table per side would
// >>>         be more accurate. Placeholder: half-range scaling.
static inline float servo_angle_feedforward(float v_target,
                                            float v_setpoint_nominal,
                                            float out_min,
                                            float out_max) {
    if (v_setpoint_nominal <= 0.0f) return out_min;
    float frac = v_target / v_setpoint_nominal;
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    // Half-range FF — PID trims from this midpoint.
    return out_min + frac * (out_max - out_min) * 0.5f;
}


// ============================================================================
// 2. OVERVOLTAGE HANDLER
// ============================================================================
// Runs every main loop tick. Two thresholds:
//   - SOFT (sustained): V > V_OVERVOLTAGE_SOFT_V for V_OVERVOLTAGE_SOFT_DURATION_MS
//   - HARD (instantaneous): V > V_OVERVOLTAGE_HARD_V — no debounce, latches fault
//
// When tripped, BOTH servos go to SERVO_ANGLE_CLOSED_DEG immediately, regardless
// of PID slew limits. The caller is responsible for:
//   1. Calling ov_check() once per loop with the current V_alt
//   2. If it returns true: bypass normal PID output, command both servos closed,
//      and set PID prev_output = SERVO_ANGLE_CLOSED_DEG so PID re-engages from
//      the closed angle when V returns to range (no slew-limited slow reopen).
//   3. If hard_trip true: latch F_OVERVOLTAGE fault flag
//
// Hardware crowbar at ~16V is the last-resort backup. Soft + hard thresholds
// must remain below the crowbar so firmware acts first (crowbars don't like
// repeat triggers).

struct OvervoltageState {
    uint32_t soft_first_ms;  // millis() when V first crossed SOFT (0 if not over)
    bool     latched;         // hard-trip latches until ov_reset()
};

static inline void ov_init(OvervoltageState &ov) {
    ov.soft_first_ms = 0;
    ov.latched = false;
}

// Clear the overvoltage state. Call when operator confirms reset, after the
// upstream fault clearing pathway (clear_faults_if_allowed()) succeeds.
static inline void ov_reset(OvervoltageState &ov) {
    ov.soft_first_ms = 0;
    ov.latched = false;
}

// Returns true if overvoltage protection should override normal control.
//   out_hard_trip is set to true on hard-threshold breach (caller latches fault).
static inline bool ov_check(OvervoltageState &ov,
                            float v_alt,
                            uint32_t now_ms,
                            bool &out_hard_trip) {
    out_hard_trip = false;

    // --- Hard threshold: instant action ---
    if (v_alt > V_OVERVOLTAGE_HARD_V) {
        ov.latched = true;
        out_hard_trip = true;
        return true;
    }

    // --- Soft threshold: timed ---
    if (v_alt > V_OVERVOLTAGE_SOFT_V) {
        if (ov.soft_first_ms == 0) {
            ov.soft_first_ms = now_ms;  // start the debounce timer
        }
        if ((now_ms - ov.soft_first_ms) > V_OVERVOLTAGE_SOFT_DURATION_MS) {
            return true;  // soft trip active (no fault latch yet — see >>> below)
        }
        // Still counting up — not yet active, but don't reset timer.
        return ov.latched;
    }

    // --- V back in normal range ---
    ov.soft_first_ms = 0;
    return ov.latched;  // stay latched until ov_reset()
}

// >>> REVIEW: should a sustained soft trip also latch the fault flag, or is
// >>>         "fast-close servos and let V settle" the right behavior without
// >>>         requiring an operator reset?
// >>>         Argument FOR latching: any OV event indicates a control issue
// >>>         the operator should investigate before continuing.
// >>>         Argument AGAINST: load disconnect events are recoverable —
// >>>         servos close, V drops, controller re-engages on next loop.
// >>>         Current implementation: soft trip is recoverable; hard trip latches.


// ============================================================================
// 3. ENGAGEMENT CHECK — OWB lock/freewheel detection per side
// ============================================================================
// One instance per drivetrain side (air, water). Tracks whether the OWB is
// currently locked (input pinion driving alternator bevel) or freewheeling.
//
// Decision logic:
//   pinion_locked_rpm = alt_rpm × BEVEL_TEETH_RATIO
//     = what the pinion would spin at if its OWB were locked
//
//   engage    when input_rpm  ≥ pinion_locked_rpm × ENGAGE_LOCK_FRAC
//   disengage when input_rpm  <  pinion_locked_rpm × ENGAGE_DISENGAGE_FRAC
//
// Hysteresis (lock_frac > disengage_frac) prevents flapping at the boundary.
//
// Mismatch check (separate function): input significantly faster than alt-locked
// = OWB would slam on engagement. State machine must close that side's servo
// before re-engagement is allowed.

struct EngagementState {
    bool     engaged;
    uint32_t state_change_ms;   // when current state was entered (telemetry / debounce)
};

static inline void eng_init(EngagementState &eng) {
    eng.engaged = false;
    eng.state_change_ms = 0;
}

// Update engagement state from current RPM readings.
// Call once per main loop with the latest filtered RPMs.
//
//   input_rpm           — hall reading at this side's gearbox output
//   alt_rpm             — hall reading at alternator shaft
//   alt_to_pinion_ratio — BEVEL_TEETH_RATIO (alt_rpm × this = pinion-locked RPM)
//   now_ms              — millis() for state-change timestamp
//
// Returns current engagement state (true = locked).
static inline bool eng_update(EngagementState &eng,
                              float input_rpm,
                              float alt_rpm,
                              float alt_to_pinion_ratio,
                              uint32_t now_ms) {
    // If alt is essentially stopped, engagement is meaningless — treat as not engaged.
    // >>> REVIEW: 10 RPM floor is arbitrary.
    // >>>         LOOP_PERIOD_MS=10 with 2 PPR gives ~6 RPM resolution at the noise floor.
    // >>>         Pick based on actual measured noise floor of the hall RPM channel.
    if (alt_rpm < 10.0f) {
        if (eng.engaged) {
            eng.engaged = false;
            eng.state_change_ms = now_ms;
        }
        return false;
    }

    float locked_rpm = alt_rpm * alt_to_pinion_ratio;

    if (eng.engaged) {
        // Currently locked — check for drop into freewheel
        if (input_rpm < locked_rpm * ENGAGE_DISENGAGE_FRAC) {
            eng.engaged = false;
            eng.state_change_ms = now_ms;
        }
    } else {
        // Currently freewheeling — check for catch-up to lock
        if (input_rpm >= locked_rpm * ENGAGE_LOCK_FRAC) {
            eng.engaged = true;
            eng.state_change_ms = now_ms;
        }
    }
    return eng.engaged;
}

// Mismatch detector — input significantly faster than alt-locked.
// Used as a guard before LOCKED phase entry: if true, close that side's servo
// before allowing engagement to prevent OWB slam.
static inline bool eng_mismatch(float input_rpm,
                                float alt_rpm,
                                float alt_to_pinion_ratio) {
    if (alt_rpm < 10.0f) return false;
    return input_rpm > (alt_rpm * alt_to_pinion_ratio * ENGAGE_MISMATCH_FRAC);
}

// Convenience: time-since-last-engagement-change (for telemetry / dwell checks).
static inline uint32_t eng_time_in_state_ms(const EngagementState &eng, uint32_t now_ms) {
    if (eng.state_change_ms == 0) return 0;
    return now_ms - eng.state_change_ms;
}

#endif // CONTROL_HELPERS_DRAFT_H
