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

// ---- Gamma-2.2 LUT (the ONE copy) ------------------------------------------
//
// WS2812 duty is linear in the byte written, human brightness perception is
// not, so every composited channel goes through gamma 2.2 on its way to the
// strip: duty = round(255 * (value/255)^2.2).
//
// It lives in the header, constexpr, because BOTH the renderer and
// LightConfig::valid()'s current budget need it and a second copy of a curve
// is a second curve. `pow` is not constexpr (and __builtin_pow is rejected in
// constant expressions by clang), so the exponent is evaluated with a small
// compile-time ln/exp pair; test_lights pins all 256 entries against the
// libm reference the previous runtime table used, so "same numbers" is a
// checked fact, not a hope. Nothing here runs at boot: no static
// initializer, no float in any render path.
namespace gamma_detail {

constexpr double kLn2 = 0.69314718055994530941723212145818;

// ln(x) for x > 0. Range-reduce to [1,2), then the atanh series
// ln(m) = 2*(z + z^3/3 + z^5/5 + ...), z = (m-1)/(m+1), |z| <= 1/3.
constexpr double ln(double x) {
    int k = 0;
    while (x < 1.0) {
        x *= 2.0;
        ++k;
    }
    while (x >= 2.0) {
        x /= 2.0;
        --k;
    }
    const double z = (x - 1.0) / (x + 1.0);
    const double z2 = z * z;
    double term = z;
    double sum = z;
    for (int n = 3; n <= 61; n += 2) {
        term *= z2;
        sum += term / n;
    }
    return 2.0 * sum - k * kLn2;
}

// exp(u) for u >= 0 (Taylor; no cancellation on the positive side).
constexpr double expPos(double u) {
    double term = 1.0;
    double sum = 1.0;
    for (int n = 1; n <= 60; ++n) {
        term *= u / n;
        sum += term;
    }
    return sum;
}

// q^2.2 for q in [0,1], as 1/exp(-2.2*ln q) so only expPos is ever used.
constexpr double pow22(double q) {
    if (q <= 0.0) {
        return 0.0;
    }
    return 1.0 / expPos(-2.2 * ln(q));
}

struct Lut {
    uint8_t v[256] = {};
};

constexpr Lut makeLut() {
    Lut t{};
    for (int i = 0; i < 256; ++i) {
        const double g = 255.0 * pow22(static_cast<double>(i) / 255.0);
        const int val = static_cast<int>(g + 0.5);
        t.v[i] = static_cast<uint8_t>(val < 0 ? 0 : (val > 255 ? 255 : val));
    }
    return t;
}

} // namespace gamma_detail

inline constexpr gamma_detail::Lut kGamma = gamma_detail::makeLut();
static_assert(kGamma.v[0] == 0 && kGamma.v[255] == 255, "gamma LUT endpoints");

// The rendered PWM duty of ONE channel: the brightness cap is applied to the
// perceptual value FIRST, then gamma. This is the order the renderer actually
// uses (LightRenderer.cpp applyBrightnessAndGamma) and the reason the cap is
// so much darker electrically than its percentage suggests -- see
// LightConfig::maxBrightness.
constexpr uint8_t renderedDuty(uint8_t channel, uint8_t maxBrightness) {
    return kGamma.v[static_cast<uint16_t>(channel) * maxBrightness / 255];
}

// The floor a DESIGNED-VISIBLE state must clear after cap + gamma: the dim
// disarmed halo, the dim red tail, the showcase breathe's floor and the
// never-connected breathe's peak are all states whose whole job is to be seen
// while quiet, so each must render at least this duty on its brightest
// channel. 6/255 is ~2.4% electrical duty and ~15% of the 40/255 the default
// cap allows at full scale (renderedDuty(255, 110) == 40).
//
// It is a floor, not a look: whether 6/255 actually reads in daylight is a
// bench judgement (A2 NOT EXECUTED / Phase B BLOCKED -- [bench-TBD],
// learning-manual/open_questions.md #55). Deliberately NOT enforced inside
// valid(): the palette is compile-time constant, so the check belongs at the
// palette's definition site (static_asserts in LightRenderer.cpp) and in
// test_lights, where it is checked through the real renderer.
inline constexpr uint8_t kMinVisibleDuty = 6;

struct LightConfig {
    // Strip layout (bench-tune to the physical build). Overlaps are allowed;
    // the compositor's priority order decides who wins a shared pixel.
    Segment brake{0, 6};       // rear brake bar (its 2 edge pixels double as the DRS tell)
    Segment rainLight{6, 2};   // F1 rain light (flashes while ERS harvesting)
    Segment halo{8, 14};       // halo ring
    Segment leftIndicator{22, 4};
    Segment rightIndicator{26, 4};

    // Global brightness cap, 0..255, applied to each channel BEFORE the gamma
    // LUT (LightRenderer.cpp applyBrightnessAndGamma). ORDER MATTERS AND IT IS
    // NOT THE FLATTERING ONE: capping the perceptual value and THEN gamma-ing
    // it means 110 is ~43% PERCEPTUAL but only ~16% ELECTRICAL --
    // renderedDuty(255, 110) == 40/255. Everything the strip can show
    // therefore lives in duty 0..40, which is why the quiet palette entries
    // are set against kMinVisibleDuty rather than "looking dim" on paper.
    // (This is a deliberate owner decision -- cap-then-gamma is the
    // perceptual choice the manual rationalises,
    // learning-manual/code_explained/soundlight_fw/04_lights_and_light_hal.md:424-431,
    // ruled OD-12 Q1(a) on 2026-09-03; the header used to claim the opposite
    // order, review finding sl:safety-1.)
    // Worst case is the all-amber hazard blink, not normal driving; keep this
    // modest.
    uint8_t maxBrightness = 110; // ~43% perceptual, 40/255 duty at full scale

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

    // Worst-case current budget: every LED lit to the cap on TWO full
    // primaries (the "all-amber hazard" allowance). WS2812 ~ 20 mA/channel at
    // full duty; a channel's current is proportional to the duty it is
    // actually DRIVEN at, which is renderedDuty(255, cap) -- not the cap
    // itself. Kept well under the 5 A rail.
    static constexpr uint32_t kBudgetMilliamps = 900;

    constexpr bool valid() const {
        // Estimate worst case: all pixels amber (R+G) at the cap, measured in
        // POST-GAMMA duty. The pre-gamma model this replaced (2*20*cap/255)
        // over-counted the default cap by ~5x -- 510 mA claimed against 180 mA
        // real -- which made the budget check nearly vacuous (sl:safety-1).
        //
        // Still a true upper bound for any palette: gamma is convex and
        // increasing, so among colors whose pre-gamma channel sum is at most
        // 2*255 (pinned by static_assert at the palette's definition site in
        // LightRenderer.cpp) the largest possible sum of rendered duties is
        // the two-full-primaries corner, exactly what is modelled here. The
        // 3-primary rain-light white is the one documented exception; at 2
        // pixels it is ~7 mA over the model, against ~700 mA of headroom.
        const uint32_t perLedMa = (2u * 20u * renderedDuty(255, maxBrightness)) / 255u;
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
// animation + dim tail; in a showcase boot the disarmed halo is the D6
// teal breathe -- BASE LAYER ONLY, so every layer after it outranks the
// show) -> DRS-open tell (steady green on the rear bar's two edge pixels)
// -> functional (brake, indicators, rain; the brake overwrite is what keeps
// the DRS tell from ever masking the brake light) -> alert (low-battery
// halo pulse -- D5: it owns the halo over the showcase breathe) -> FAILSAFE
// hazard (all amber, overrides everything).
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
