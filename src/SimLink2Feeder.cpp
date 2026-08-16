#ifdef W17_SIM_LINK2_FEEDER

#include "SimLink2Feeder.hpp"

#include <Arduino.h>

#include "BuildConfig.hpp"
#include "link2/Link2Codec.hpp"

// ~14s looping bench script. Demonstrates: startup (armed), idle, throttle
// sweeps + gear shifts, ERS deploy (whine) and harvest (rain light), braking
// (brake light), steering (indicators), a 1s link dropout (engine to
// silence + hazard blink), then recovery.

namespace {

constexpr uint32_t kCycleMs = 14000;

int8_t triangle(uint32_t t, uint32_t periodMs, int8_t peak) {
    const uint32_t phase = t % periodMs;
    const uint32_t half = periodMs / 2;
    const uint32_t up = phase < half ? (phase * 100 / half) : ((periodMs - phase) * 100 / half);
    return static_cast<int8_t>(static_cast<int32_t>(up) * peak / 100);
}

const char* buildState(uint32_t t, link2::VehicleState& s) {
    s = link2::VehicleState{};
    s.armed = true;
    s.failsafe = false;
    s.gear = 1;
    s.driveMode = 1;
    s.batteryMv = 7800;

    if (t < 2000) {
        s.throttlePercent = 0; // idle after startup
        return "IDLE";
    }
    if (t < 6000) {
        s.throttlePercent = triangle(t - 2000, 2000, 100);
        s.gear = static_cast<uint8_t>(1 + (t - 2000) / 1000); // climb gears
        s.drsOpen = s.throttlePercent > 70; // flap open on the "straights"
        return "DRIVING";
    }
    if (t < 8000) {
        // ERS mode: deploy boost (whine) then it recharges (harvest).
        s.driveMode = 2;
        s.gear = 3;
        s.throttlePercent = 90;
        s.ersDeploying = true;
        s.ersPercent = static_cast<uint8_t>(80 - (t - 6000) / 40); // falling = deploy
        return "ERS_DEPLOY";
    }
    if (t < 9500) {
        s.driveMode = 2;
        s.gear = 2;
        s.throttlePercent = 0;
        s.braking = true;                                        // brake light
        s.ersPercent = static_cast<uint8_t>(30 + (t - 8000) / 30); // rising = harvest
        return "BRAKE_HARVEST";
    }
    if (t < 11000) {
        s.throttlePercent = 40;
        s.gear = 3;
        s.steeringPercent = triangle(t - 9500, 1500, 90); // indicators sweep
        return "CORNERING";
    }
    if (t < 12000) {
        // Scripted dropout: emit NOTHING -> monitor goes stale -> hazard.
        return "DROPOUT";
    }
    s.throttlePercent = 30;
    s.gear = 2;
    return "RECOVERED";
}

} // namespace

namespace simfeeder {

size_t tick(uint32_t nowMs, uint8_t* out) {
    static uint32_t lastFrameMs = 0;
    static const char* lastPhase = nullptr;

    const uint32_t t = nowMs % kCycleMs;
    link2::VehicleState s;
    const char* phase = buildState(t, s);

    if (phase != lastPhase) {
        lastPhase = phase;
        W17_UART0_PRINTF("[sim] phase: %s\n", phase);
    }

    // During DROPOUT emit nothing (the whole point -- exercise staleness).
    const bool dropout = (t >= 11000 && t < 12000);
    if (dropout) {
        return 0;
    }

    if (nowMs - lastFrameMs < 50) { // 20 Hz
        return 0;
    }
    lastFrameMs = nowMs;
    return link2::encodeFrame(s, out);
}

} // namespace simfeeder

#endif // W17_SIM_LINK2_FEEDER
