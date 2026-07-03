#pragma once

#include <cstdint>

#include "link2/Link2Codec.hpp"

namespace link2monitor {

// Where the link is, as a first-class signal (lights need the
// NeverConnected vs Lost distinction; tests read better than inferring it
// from field values).
enum class LinkStatus : uint8_t {
    NeverConnected, // no valid frame has ever arrived
    Up,             // a valid frame arrived within the staleness window
    Lost,           // was Up, then no valid frame for >= staleness window
};

struct Link2MonitorConfig {
    // Protocol mandate (docs/link2_protocol.md): no CRC-valid frame for this
    // long => local failsafe. 500 ms = 10 missed frames at the nominal 20 Hz.
    uint32_t stalenessMs = 500;

    constexpr bool valid() const { return stalenessMs > 0; }
};

// Wraps link2::Link2FrameAssembler: feed raw UART bytes, get back the
// EFFECTIVE vehicle state -- i.e. the last good frame while the link is Up,
// but a safe per-field failsafe projection once the link goes stale.
//
// Per-field staleness (the key design decision): commands are zeroed and
// failsafe is asserted, motion telemetry (rpm) is zeroed so a stale value
// cannot drive sound/lights, but latched/qualified judgments from board #1
// (lowBattery) and slowly-meaningful fields (battery mV, gear, ersPercent,
// driveMode) hold their last-known value.
class Link2Monitor {
public:
    explicit Link2Monitor(Link2MonitorConfig config = Link2MonitorConfig{});

    // Feed one raw byte. nowMs stamps a successfully decoded frame.
    void feedByte(uint8_t b, uint32_t nowMs);

    // Recompute status from the clock (call every tick even when no bytes
    // arrived -- that is how a silent link becomes Lost).
    void poll(uint32_t nowMs);

    LinkStatus status() const { return status_; }

    // The effective state to drive sound + lights from. Reflects the
    // per-field staleness table once the link is not Up.
    const link2::VehicleState& state() const { return effective_; }

private:
    void recompute(uint32_t nowMs);

    Link2MonitorConfig config_;
    link2::Link2FrameAssembler assembler_;
    link2::VehicleState lastGood_{}; // last successfully decoded frame
    link2::VehicleState effective_{}; // what state() returns
    LinkStatus status_ = LinkStatus::NeverConnected;
    bool everReceived_ = false;
    uint32_t lastFrameMs_ = 0;
};

} // namespace link2monitor
