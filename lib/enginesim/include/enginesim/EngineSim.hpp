#pragma once

#include <cstdint>

#include "link2/Link2Frame.hpp"

namespace enginesim {

// Ignition / running state, driven by the `armed` flag from board #1.
//   Off      : disarmed -> silence.
//   Cranking : armed 0->1 -> a brief starter-whir + fire-up before idle.
//   Running  : normal idle..redline behavior.
// A failsafe (effective armed == false via the monitor) drops back to Off.
enum class Ignition : uint8_t { Off, Cranking, Running };

struct EngineSimConfig {
    uint16_t idleRpm = 3500;    // engine rpm at idle (not wheel rpm)
    uint16_t maxRpm = 15000;    // redline
    uint16_t crankRpm = 1800;   // starter cranking pitch, below idle
    uint32_t crankMs = 600;     // cranking duration before Running

    // Asymmetric inertia, per-mille of the remaining gap closed per tick-ms:
    // rev-up is fast, rev-down slower (engine braking is gentler than the
    // pull). Applied as `rpm += (target - rpm) * rate * dtMs / scale`.
    uint16_t revUpPerMille = 6;   // ~0.5s idle->max at full throttle
    uint16_t revDownPerMille = 3; // ~1.2s max->idle on lift

    // Idle wobble amplitude (rpm) and step, a small triangle so idle isn't
    // dead-flat.
    uint16_t idleWobbleRpm = 120;

    // Gear-shift blip: on an upshift the perceived rpm dips, on a downshift
    // it blips up, for blipMs. Magnitude in rpm.
    uint16_t shiftBlipRpm = 1400;
    uint32_t blipMs = 130;

    // Rev limiter: within this rpm of maxRpm at full throttle, cut ignition
    // in bursts (the iconic F1 buzz). Cut cadence handled in the synth via
    // the limiter flag; here we just detect it.
    uint16_t limiterBandRpm = 250;

    // Overrun crackle window: after a fast throttle drop from high rpm,
    // crackle is enabled for this long (the synth adds gated noise bursts).
    uint16_t overrunThrottleDrop = 40; // percentage-point drop in one tick
    uint16_t overrunHighRpmPct = 60;   // only above this % of maxRpm
    uint32_t overrunMs = 900;

    constexpr bool valid() const {
        return idleRpm > crankRpm && maxRpm > idleRpm &&
               revUpPerMille > 0 && revDownPerMille > 0 &&
               idleWobbleRpm < idleRpm && limiterBandRpm < (maxRpm - idleRpm) &&
               overrunHighRpmPct <= 100;
    }
};

// Output of one tick -- everything the synth + lights need. Pure data.
struct EngineState {
    uint16_t engineRpm = 0; // idle..max while Running, crankRpm while Cranking, 0 when Off
    uint8_t throttlePercent = 0; // pass-through of the commanded throttle (0..100)
    Ignition ignition = Ignition::Off;
    bool limiterActive = false; // pinned at redline under full throttle
    bool overrunActive = false; // in the post-lift crackle window
    bool ersWhine = false;      // ersDeploying pass-through (synth gates the whine)
};

// Virtual engine. Pure logic; time supplied by the caller. Feed it the
// EFFECTIVE VehicleState (post-staleness) each control tick.
class EngineSim {
public:
    explicit EngineSim(EngineSimConfig config = EngineSimConfig{});

    void update(uint32_t nowMs, const link2::VehicleState& state);

    const EngineState& engine() const { return out_; }

private:
    uint16_t targetRpm(const link2::VehicleState& state) const;

    EngineSimConfig config_;
    EngineState out_;

    bool seeded_ = false;
    uint32_t lastMs_ = 0;

    Ignition ignition_ = Ignition::Off;
    uint32_t crankStartMs_ = 0;
    int32_t rpm_ = 0; // internal high-resolution rpm (integer)

    uint8_t lastThrottle_ = 0;
    uint8_t lastGear_ = 1;
    bool everSeenState_ = false;

    // Event windows are tracked as (start timestamp + explicit active flag),
    // tested with wrap-safe unsigned elapsed arithmetic. The boolean is the
    // authoritative "inactive" state: timestamp 0 is a valid event start (boot
    // time) and must not double as a sentinel.
    uint32_t blipStartMs_ = 0;
    bool blipActive_ = false;
    int16_t blipRpm_ = 0; // signed offset applied while blipping

    uint32_t overrunStartMs_ = 0;
    bool overrunActive_ = false;
    uint16_t wobblePhase_ = 0;
};

} // namespace enginesim
