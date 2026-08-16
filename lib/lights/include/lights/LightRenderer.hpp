#pragma once

#include <cstdint>

#include "enginesim/EngineSim.hpp"
#include "link2/Link2Frame.hpp"
#include "link2monitor/Link2Monitor.hpp"

namespace lights {

inline constexpr uint8_t kNumPixels = 30; // WS2812B strip length

struct Rgb {
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    bool operator==(const Rgb& o) const { return r == o.r && g == o.g && b == o.b; }
};

// A contiguous run of pixels [start, start+len). len 0 = segment absent.
struct Segment {
    uint8_t start = 0;
    uint8_t len = 0;
};

struct LightConfig {
    // Strip layout (bench-tune to the physical build). Overlaps are allowed;
    // the compositor's priority order decides who wins a shared pixel.
    Segment brake{0, 6};       // rear brake bar (its 2 edge pixels double as the DRS tell)
    Segment rainLight{6, 2};   // F1 rain light (flashes while ERS harvesting)
    Segment halo{8, 14};       // halo ring
    Segment leftIndicator{22, 4};
    Segment rightIndicator{26, 4};

    // Global brightness cap (0..255 scale applied after gamma). Worst case is
    // the all-amber hazard blink, not normal driving; keep this modest.
    uint8_t maxBrightness = 110; // ~43%

    // Turn-indicator steering thresholds (normalized -100..100 from the
    // frame's steeringPercent) with hysteresis + minimum-on so a flick still
    // completes one blink.
    int8_t indicatorOnPercent = 40;
    int8_t indicatorOffPercent = 20;

    // Blink periods (ms). Derived from a free-running clock so segments stay
    // phase-locked and re-triggering doesn't reset phase.
    uint16_t indicatorPeriodMs = 660; // ~1.5 Hz
    uint16_t hazardPeriodMs = 500;    // 2 Hz
    uint16_t rainPeriodMs = 250;      // ~4 Hz (rapid)
    uint16_t lowBatteryPeriodMs = 1600;

    // How recently ersPercent must have risen to count as "harvesting".
    uint16_t harvestWindowMs = 400;

    // Ignition-on animation (vision decision 16), keyed off the enginesim
    // ignition state machine -- the renderer never re-derives ignition from
    // `armed`. The look ("fire-up in Petronas teal"):
    //   Cranking -> a single bright-cyan "starter" comet with a two-pixel
    //     trail sweeps the halo, one full lap per ignitionSweepPeriodMs
    //     (free-running phase, like the blinks; the default 600 ms crank
    //     gives two laps).
    //   Cranking->Running -> the whole halo flashes bright cyan and
    //     crossfades linearly into the normal armed teal over
    //     ignitionFlashMs (the "engine catches" moment).
    // Deliberately a BASE-layer effect: low-battery and the failsafe hazard
    // still overwrite it, brake/indicators/rain are untouched segments. The
    // teal-family palette can never be confused with amber hazard/indicator
    // or red brake/low-battery signals.
    uint16_t ignitionSweepPeriodMs = 300;
    uint16_t ignitionFlashMs = 350;

    // NeverConnected grace window: how long the calm "waiting for board #1"
    // breathe may run (measured from the first NeverConnected render) before
    // an empty link escalates to the hazard pattern. Both boards power from
    // the same UBEC rail (docs/link2_protocol.md), so a healthy boot delivers
    // the first frame within a couple of seconds; once this window expires
    // with no frame EVER received, the link is a broken/unplugged harness and
    // must look like one, not breathe calmly forever (audit defect 9).
    uint32_t neverConnectedGraceMs = 5000;

    // Worst-case current budget: every LED at the brightness cap, all three
    // primaries. WS2812 ~ 20mA/channel at full; scale by cap. Kept well
    // under the 5A rail; hazard (single amber color) is the real worst case.
    static constexpr uint32_t kBudgetMilliamps = 900;

    constexpr bool valid() const {
        // Estimate worst case: all pixels amber (R+G) at the cap.
        const uint32_t perLedMa = (2u * 20u * maxBrightness) / 255u;
        return indicatorOnPercent > indicatorOffPercent && maxBrightness > 0 &&
               indicatorPeriodMs > 0 && hazardPeriodMs > 0 && rainPeriodMs > 0 &&
               // >= 2 so the pulse half-period (period / 2) is never 0 -- the
               // renderer divides the triangle by it (LightRenderer.cpp).
               lowBatteryPeriodMs >= 2 &&
               // Sweep period is a divisor in the comet position math; the
               // flash is a moment, not a mode (bounded so it can never sit
               // on the halo masking the true armed color for long).
               ignitionSweepPeriodMs > 0 && ignitionFlashMs > 0 &&
               ignitionFlashMs <= 2000 &&
               // Lower bound: must outlast normal same-rail power-up skew so a
               // healthy boot never flashes hazard. Upper bound: a genuinely
               // dead link must be signaled while someone is still looking at
               // the car (giftee operator model).
               neverConnectedGraceMs >= 1000 && neverConnectedGraceMs <= 30000 &&
               (perLedMa * kNumPixels) <= kBudgetMilliamps;
    }
};

// Pure compositor: (effective VehicleState, LinkStatus, Ignition, nowMs) ->
// pixels[N]. Stateful only for indicator hysteresis, harvest edge detection,
// the never-connected grace seed and the ignition flash window; time is
// caller-supplied so blink phase, self-cancel, grace escalation and the
// fire-up crossfade are deterministic in tests.
//
// Priority (low to high, later overrides): base (halo incl. the ignition-on
// animation + dim tail) -> DRS-open tell (steady green on the rear bar's two
// edge pixels) -> functional (brake, indicators, rain; the brake overwrite is
// what keeps the DRS tell from ever masking the brake light) -> alert
// (low-battery halo pulse) -> FAILSAFE hazard (all amber, overrides
// everything).
class LightRenderer {
public:
    explicit LightRenderer(LightConfig config = LightConfig{});

    // `ignition` comes from the enginesim state machine (core-1-local, same
    // as this renderer -- no cross-core traffic).
    void render(const link2::VehicleState& state, link2monitor::LinkStatus link,
                enginesim::Ignition ignition, uint32_t nowMs, Rgb outPixels[kNumPixels]);

private:
    void fill(Rgb* px, const Segment& seg, Rgb color);
    bool blinkOn(uint32_t nowMs, uint16_t periodMs) const;

    LightConfig config_;

    // Indicator hysteresis state (left/right latched by steering).
    bool leftOn_ = false;
    bool rightOn_ = false;

    // Harvest detection: remember the last ersPercent and when it last rose.
    uint8_t lastErsPercent_ = 0;
    uint32_t lastHarvestMs_ = 0;
    bool harvestSeeded_ = false;

    // Ignition-on animation: previous ignition value (fire-up transition
    // detection) and the flash window (wrap-safe start+flag pattern, same as
    // enginesim's event windows; cleared on expiry or when ignition leaves
    // Running, so a disarm mid-flash cancels it).
    enginesim::Ignition lastIgnition_ = enginesim::Ignition::Off;
    uint32_t flashStartMs_ = 0;
    bool flashActive_ = false;

    // NeverConnected grace: timestamp of the first render seen while no frame
    // has ever arrived (wrap-safe start+flag pattern, same as enginesim's
    // event windows). Once the link has been Up this state is unreachable
    // again, so the seed never needs resetting.
    bool graceSeeded_ = false;
    uint32_t graceStartMs_ = 0;
};

} // namespace lights
