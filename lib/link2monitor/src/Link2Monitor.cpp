#include "link2monitor/Link2Monitor.hpp"

namespace link2monitor {

Link2Monitor::Link2Monitor(Link2MonitorConfig config) : config_(config) {
    // Boot state = never-connected local failsafe (failsafe flag set by the
    // VehicleState default).
    effective_.failsafe = true;
}

void Link2Monitor::feedByte(uint8_t b, uint32_t nowMs) {
    const link2::Link2FrameAssembler::FeedResult result = assembler_.feedByte(b);
    if (result == link2::Link2FrameAssembler::FeedResult::FrameReady) {
        lastGood_ = assembler_.lastState();
        everReceived_ = true;
        lastFrameMs_ = nowMs;
    }
    recompute(nowMs);
}

void Link2Monitor::poll(uint32_t nowMs) { recompute(nowMs); }

void Link2Monitor::recompute(uint32_t nowMs) {
    if (!everReceived_) {
        status_ = LinkStatus::NeverConnected;
        effective_ = link2::VehicleState{}; // all-safe defaults, failsafe = true
        return;
    }

    // >= is inclusive: exactly stalenessMs since the last frame is stale.
    const bool stale = (nowMs - lastFrameMs_) >= config_.stalenessMs;
    if (!stale) {
        status_ = LinkStatus::Up;
        effective_ = lastGood_;
        return;
    }

    // Lost: per-field failsafe projection over the last good frame.
    status_ = LinkStatus::Lost;
    effective_ = lastGood_;
    // Commands -> safe.
    effective_.throttlePercent = 0;
    effective_.steeringPercent = 0;
    effective_.braking = false;
    effective_.reverse = false;
    effective_.drsOpen = false;
    effective_.armed = false;
    effective_.ersDeploying = false;
    effective_.failsafe = true;
    // modeFlags bits are STATE (mode indications), not config: a stale
    // showcase/pairing indication must not outlive the link. Nothing keys
    // off them today (both reserved, always 0 from current board #1); the
    // safe projection exists now so future consumers inherit it instead of
    // re-deciding it under pressure.
    effective_.showcase = false;
    effective_.awaitingController = false;
    // Motion telemetry must not persist -- a stale rpm would drive the engine
    // sound and speed readout.
    effective_.rpm = 0;
    // Held last-known (garnish / latched judgments / operator config):
    // batteryMv, lowBattery, gear, ersPercent, driveMode, soundProfile,
    // volume -- left as copied from lastGood_. The v2 sound pair is
    // CONFIGURATION, not state: silencing on link loss comes from the
    // failsafe flag above through the engine state machine and always wins
    // over volume, so holding these avoids a voice/volume glitch on
    // recovery. (NeverConnected still resets them to the wire defaults via
    // VehicleState{}.)
}

} // namespace link2monitor
