#include "enginesim/EngineSim.hpp"

namespace enginesim {

namespace {
constexpr uint32_t kMaxDtMs = 100; // stall clamp, same idea as ERS
} // namespace

EngineSim::EngineSim(EngineSimConfig config) : config_(config) {
    rpm_ = config_.idleRpm;
}

uint16_t EngineSim::targetRpm(const link2::VehicleState& state) const {
    // Map throttle 0..100 across idle..max. Throttle is already the
    // post-gearbox/post-ERS commanded value from board #1, so the engine
    // note tracks the actual motor, not the raw stick.
    const int32_t span = config_.maxRpm - config_.idleRpm;
    const int32_t t = state.throttlePercent < 0 ? 0 : state.throttlePercent;
    return static_cast<uint16_t>(config_.idleRpm + span * t / 100);
}

void EngineSim::update(uint32_t nowMs, const link2::VehicleState& state) {
    if (!seeded_) {
        seeded_ = true;
        lastMs_ = nowMs;
    }
    uint32_t dtMs = nowMs - lastMs_;
    lastMs_ = nowMs;
    if (dtMs > kMaxDtMs) {
        dtMs = kMaxDtMs;
    }

    // --- Ignition state machine (driven by the sound authority:
    // armed || showcase, with the showcase branch gated on !failsafe and
    // !lowBattery -- see ignitionAuthority() in the header). ---
    if (!ignitionAuthority(state)) {
        ignition_ = Ignition::Off;
    } else if (ignition_ == Ignition::Off) {
        ignition_ = Ignition::Cranking;
        crankStartMs_ = nowMs;
    } else if (ignition_ == Ignition::Cranking && (nowMs - crankStartMs_) >= config_.crankMs) {
        ignition_ = Ignition::Running;
        rpm_ = config_.idleRpm;
    }

    // --- Gear-shift blip detection (only while Running, never on the first
    // state seen or across a failsafe recovery: a gear delta then is not a
    // real shift). ---
    if (everSeenState_ && ignition_ == Ignition::Running && !state.failsafe) {
        if (state.gear > lastGear_) {
            blipRpm_ = -static_cast<int16_t>(config_.shiftBlipRpm); // upshift dip
            blipStartMs_ = nowMs;
            blipActive_ = true;
        } else if (state.gear < lastGear_) {
            blipRpm_ = static_cast<int16_t>(config_.shiftBlipRpm); // downshift blip
            blipStartMs_ = nowMs;
            blipActive_ = true;
        }
    }

    // --- Overrun crackle window: fast throttle drop from high rpm. ---
    if (everSeenState_ && ignition_ == Ignition::Running) {
        const int drop = static_cast<int>(lastThrottle_) - static_cast<int>(state.throttlePercent);
        const bool wasHigh = rpm_ >= config_.idleRpm +
                                        (config_.maxRpm - config_.idleRpm) *
                                            config_.overrunHighRpmPct / 100;
        if (drop >= config_.overrunThrottleDrop && wasHigh) {
            overrunStartMs_ = nowMs;
            overrunActive_ = true;
        }
    }

    lastThrottle_ = state.throttlePercent < 0 ? 0 : static_cast<uint8_t>(state.throttlePercent);
    lastGear_ = state.gear;
    everSeenState_ = true;

    // --- RPM inertia toward the target for the current ignition state. ---
    int32_t target;
    if (ignition_ == Ignition::Off) {
        target = 0;
    } else if (ignition_ == Ignition::Cranking) {
        target = config_.crankRpm;
    } else {
        target = targetRpm(state);
    }

    const int32_t gap = target - rpm_;
    const uint16_t rate = (gap >= 0) ? config_.revUpPerMille : config_.revDownPerMille;
    rpm_ += inertiaStep(gap, rate, dtMs);
    // Guard against overshoot leaving a residual that never settles.
    if ((gap >= 0 && rpm_ > target) || (gap < 0 && rpm_ < target)) {
        rpm_ = target;
    }
    if (rpm_ < 0) {
        rpm_ = 0;
    }

    // --- Compose the audible rpm: base + idle wobble + active blip. ---
    // Engine Off = not spinning: report 0 immediately (the synth is silent
    // when Off regardless, and this keeps engineRpm honest rather than
    // trailing a slow inertial decay of a dead engine).
    if (ignition_ == Ignition::Off) {
        out_.engineRpm = 0;
        out_.throttlePercent = lastThrottle_;
        out_.ignition = ignition_;
        out_.limiterActive = false;
        out_.overrunActive = false;
        out_.ersWhine = state.ersDeploying;
        return;
    }

    int32_t audible = rpm_;

    if (ignition_ == Ignition::Running) {
        // Idle wobble: triangle in [-amp, +amp], strongest at idle, fading as
        // throttle opens.
        wobblePhase_ = static_cast<uint16_t>((wobblePhase_ + dtMs) % 400);
        const int32_t tri = (wobblePhase_ < 200) ? (wobblePhase_ - 100) : (300 - wobblePhase_);
        const int32_t idleness = 100 - (state.throttlePercent < 0 ? 0 : state.throttlePercent);
        audible += tri * config_.idleWobbleRpm * idleness / (100 * 100);
    }

    out_.limiterActive = false;
    // Wrap-safe blip window: active while elapsed < blipMs (exclusive at the
    // end, matching the old `nowMs < start + blipMs`). Clear the flag on expiry
    // so the elapsed value can never wrap back below blipMs and phantom-reopen.
    if (blipActive_) {
        if (static_cast<uint32_t>(nowMs - blipStartMs_) < config_.blipMs) {
            audible += blipRpm_;
        } else {
            blipActive_ = false;
        }
    }
    if (ignition_ == Ignition::Running && state.throttlePercent >= 95 &&
        rpm_ >= config_.maxRpm - config_.limiterBandRpm) {
        out_.limiterActive = true;
    }

    if (audible < 0) {
        audible = 0;
    }
    out_.engineRpm = static_cast<uint16_t>(audible);
    out_.throttlePercent = lastThrottle_;
    out_.ignition = ignition_;
    // Wrap-safe overrun window: active while elapsed < overrunMs (exclusive at
    // the end, matching the old `nowMs < start + overrunMs`). Clear on expiry so
    // elapsed cannot wrap back below overrunMs and phantom-reopen.
    if (overrunActive_ && static_cast<uint32_t>(nowMs - overrunStartMs_) >= config_.overrunMs) {
        overrunActive_ = false;
    }
    out_.overrunActive = overrunActive_ && ignition_ == Ignition::Running;
    out_.ersWhine = state.ersDeploying;
}

} // namespace enginesim
