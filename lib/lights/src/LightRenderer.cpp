#include "lights/LightRenderer.hpp"

namespace lights {

namespace {

// Petronas teal, F1 palette.
constexpr Rgb kTeal{0, 130, 120};
constexpr Rgb kDimWhite{40, 40, 46};
constexpr Rgb kDimRed{40, 0, 0};
constexpr Rgb kBrightRed{255, 0, 0};
constexpr Rgb kAmber{255, 90, 0};
constexpr Rgb kWhite{255, 255, 255};
constexpr Rgb kOff{0, 0, 0};

// Ignition-on animation palette (vision 16): the whole fire-up sequence stays
// in the Petronas teal family -- bright cyan comet/flash settling into the
// armed teal -- so it can never be mistaken for amber hazard/indicators or
// red brake/low-battery.
constexpr Rgb kIgnitionCyan{0, 255, 230};  // comet head + fire-up flash
constexpr Rgb kIgnitionTrail{0, 90, 80};   // comet trail, one pixel behind
constexpr Rgb kIgnitionTrail2{0, 45, 40};  // fading tail, two pixels behind

// DRS-open tell (vision 16): pure green, the TV-graphics / sim-racing "DRS
// open" color -- the only green anywhere in this palette, so it reads
// unambiguously.
constexpr Rgb kDrsGreen{0, 255, 0};

// The static power budget in LightConfig::valid() models the worst case as
// every LED at TWO FULL primaries (the "all-amber hazard" allowance:
// 2 * 20 mA scaled by the cap). That model stays a true upper bound only
// while no color that can cover the 14-pixel halo exceeds a channel sum of
// 2 * 255 -- pin it here so a palette tweak cannot silently invalidate the
// budget arithmetic. (The 3-primary rain-light white predates this rule; at
// 2 pixels it is covered by the halo colors sitting under the model bound.)
constexpr int channelSum(Rgb c) {
    return static_cast<int>(c.r) + static_cast<int>(c.g) + static_cast<int>(c.b);
}
constexpr int kBudgetModelChannelSum = 2 * 255; // two full primaries per LED
static_assert(channelSum(kIgnitionCyan) <= kBudgetModelChannelSum,
              "halo-wide ignition color exceeds the two-primary budget model");
static_assert(channelSum(kIgnitionTrail) <= kBudgetModelChannelSum, "trail exceeds budget model");
static_assert(channelSum(kIgnitionTrail2) <= kBudgetModelChannelSum, "trail exceeds budget model");
static_assert(channelSum(kTeal) <= kBudgetModelChannelSum, "halo teal exceeds budget model");
static_assert(channelSum(kDrsGreen) <= kBudgetModelChannelSum, "DRS green exceeds budget model");

// Gamma-2.2 LUT (WS2812 look linear-perceptual). Built once.
struct GammaLut {
    uint8_t v[256];
    GammaLut() {
        for (int i = 0; i < 256; ++i) {
            double g = 255.0 * __builtin_pow(i / 255.0, 2.2);
            int val = static_cast<int>(g + 0.5);
            v[i] = static_cast<uint8_t>(val < 0 ? 0 : (val > 255 ? 255 : val));
        }
    }
};
const GammaLut kGamma;

Rgb applyBrightnessAndGamma(Rgb c, uint8_t maxBrightness) {
    // Scale by the cap, then gamma-correct.
    auto ch = [&](uint8_t x) {
        uint16_t scaled = static_cast<uint16_t>(x) * maxBrightness / 255;
        return kGamma.v[scaled];
    };
    return Rgb{ch(c.r), ch(c.g), ch(c.b)};
}

// Channel-wise linear crossfade `from` -> `to` at num/den (num < den, den > 0
// by LightConfig::valid()). Integer math; both endpoints are exact.
Rgb blendToward(Rgb from, Rgb to, uint32_t num, uint32_t den) {
    auto ch = [&](uint8_t f, uint8_t t) {
        const int32_t d = static_cast<int32_t>(t) - static_cast<int32_t>(f);
        return static_cast<uint8_t>(static_cast<int32_t>(f) +
                                    d * static_cast<int32_t>(num) / static_cast<int32_t>(den));
    };
    return Rgb{ch(from.r, to.r), ch(from.g, to.g), ch(from.b, to.b)};
}

} // namespace

LightRenderer::LightRenderer(LightConfig config) : config_(config) {}

void LightRenderer::fill(Rgb* px, const Segment& seg, Rgb color) {
    for (uint8_t i = 0; i < seg.len; ++i) {
        const uint8_t idx = seg.start + i;
        if (idx < kNumPixels) {
            px[idx] = color;
        }
    }
}

bool LightRenderer::blinkOn(uint32_t nowMs, uint16_t periodMs) const {
    // Free-running square wave: on for the first half of each period.
    return (nowMs % periodMs) < (periodMs / 2u);
}

void LightRenderer::render(const link2::VehicleState& state, link2monitor::LinkStatus link,
                           enginesim::Ignition ignition, uint32_t nowMs,
                           Rgb outPixels[kNumPixels]) {
    Rgb px[kNumPixels];
    for (uint8_t i = 0; i < kNumPixels; ++i) {
        px[i] = kOff;
    }

    const bool neverConnected = link == link2monitor::LinkStatus::NeverConnected;

    // --- Never connected: a distinct calm "waiting" breathe, but only inside
    // a bounded grace window measured from the first NeverConnected render.
    // A wire cut AFTER the first good frame reads as Lost (hazard below)
    // within the 500 ms staleness rule -- but a wire cut or never-plugged
    // harness BEFORE any frame keeps the monitor in NeverConnected forever,
    // so once the grace expires with still no frame ever received we
    // escalate to the same hazard pattern a Lost link shows (audit defect 9).
    // Inside the window the sound side is already failsafe-equivalent (the
    // monitor's effective state is all-safe defaults, engine Off/silent);
    // only the hazard BLINK is deferred, so a normal same-rail power-up
    // breathes calmly instead of flashing hazard while board #1 boots. ---
    bool neverConnectedExpired = false;
    if (neverConnected) {
        if (!graceSeeded_) {
            graceSeeded_ = true;
            graceStartMs_ = nowMs;
        }
        neverConnectedExpired =
            static_cast<uint32_t>(nowMs - graceStartMs_) >= config_.neverConnectedGraceMs;
        if (!neverConnectedExpired) {
            const uint32_t phase = nowMs % 2000;
            const uint32_t tri = phase < 1000 ? phase : (2000 - phase); // 0..1000..0
            const uint8_t lvl = static_cast<uint8_t>(tri * 255 / 1000);
            Rgb breathe{static_cast<uint8_t>(lvl / 6), static_cast<uint8_t>(lvl / 3),
                        static_cast<uint8_t>(lvl / 3)};
            fill(px, config_.halo, breathe);
            for (uint8_t i = 0; i < kNumPixels; ++i) {
                outPixels[i] = applyBrightnessAndGamma(px[i], config_.maxBrightness);
            }
            return;
        }
    }

    const bool localFailsafe =
        state.failsafe || link == link2monitor::LinkStatus::Lost || neverConnectedExpired;

    // --- FAILSAFE hazard: all amber blink, overrides everything. ---
    if (localFailsafe) {
        Rgb c = blinkOn(nowMs, config_.hazardPeriodMs) ? kAmber : kOff;
        for (uint8_t i = 0; i < kNumPixels; ++i) {
            outPixels[i] = applyBrightnessAndGamma(c, config_.maxBrightness);
        }
        return;
    }

    // --- Ignition-on animation bookkeeping (vision 16). The fire-up flash
    // triggers on entering Running; detection lives in the normal path only,
    // which is safe because enginesim can neither reach nor stay in Running
    // under failsafe (armed is forced false), so no transition can hide
    // behind the early returns above. ---
    if (ignition == enginesim::Ignition::Running &&
        lastIgnition_ != enginesim::Ignition::Running) {
        flashStartMs_ = nowMs;
        flashActive_ = true;
    }
    lastIgnition_ = ignition;
    if (ignition != enginesim::Ignition::Running) {
        flashActive_ = false; // disarm/failsafe mid-flash cancels it
    }

    // --- Base layer: dim red tail + halo. The halo carries the ignition-on
    // animation: Cranking = starter comet sweep; first ignitionFlashMs of
    // Running = bright-cyan flash crossfading into the armed teal; otherwise
    // teal armed / dim white disarmed. Alerts still overwrite all of it. ---
    fill(px, config_.brake, kDimRed);
    if (ignition == enginesim::Ignition::Cranking && config_.halo.len > 0) {
        // Free-running comet: the head walks the halo once per period, with a
        // two-pixel trail behind it (wrapping inside the halo segment).
        const uint32_t phase = nowMs % config_.ignitionSweepPeriodMs;
        const uint8_t head =
            static_cast<uint8_t>(phase * config_.halo.len / config_.ignitionSweepPeriodMs);
        auto haloPx = [&](uint8_t offset, Rgb c) {
            const uint8_t idx = static_cast<uint8_t>(
                config_.halo.start + (offset % config_.halo.len));
            if (idx < kNumPixels) {
                px[idx] = c;
            }
        };
        haloPx(head, kIgnitionCyan);
        haloPx(static_cast<uint8_t>(head + config_.halo.len - 1), kIgnitionTrail);
        haloPx(static_cast<uint8_t>(head + config_.halo.len - 2), kIgnitionTrail2);
    } else {
        Rgb haloColor = state.armed ? kTeal : kDimWhite;
        if (flashActive_) {
            // Wrap-safe window; expiry clears the flag so elapsed can never
            // wrap back below the duration and phantom-reopen.
            const uint32_t elapsed = nowMs - flashStartMs_;
            if (elapsed >= config_.ignitionFlashMs) {
                flashActive_ = false;
            } else {
                haloColor = blendToward(kIgnitionCyan, kTeal, elapsed, config_.ignitionFlashMs);
            }
        }
        fill(px, config_.halo, haloColor);
    }

    // --- DRS-open tell (vision 16): the OUTERMOST pixels of the rear brake
    // bar glow steady green while board #1's arbitrated drsOpen bit is set.
    // Mapping rationale: the DRS flap lives in the rear wing, so the tell
    // belongs on the rear bar; the two edge pixels read as the flap "opening
    // outward" while the bar's middle keeps the dim tail; steady-not-blinking
    // keeps it subtle next to the blinking signals. Drawn BEFORE the brake
    // layer, so a braking car shows a full bright-red bar -- the tell can
    // never mask the brake light (and the hazard early-return above already
    // outranks everything). ---
    if (state.drsOpen && config_.brake.len > 0) {
        const uint8_t first = config_.brake.start;
        const uint8_t last = static_cast<uint8_t>(config_.brake.start + config_.brake.len - 1);
        if (first < kNumPixels) {
            px[first] = kDrsGreen;
        }
        if (last < kNumPixels) {
            px[last] = kDrsGreen;
        }
    }

    // --- Low-battery: slow red pulse on the halo (alert layer). ---
    if (state.lowBattery) {
        const uint32_t phase = nowMs % config_.lowBatteryPeriodMs;
        const uint32_t half = config_.lowBatteryPeriodMs / 2u;
        const uint32_t tri = phase < half ? phase : (config_.lowBatteryPeriodMs - phase);
        const uint8_t lvl = static_cast<uint8_t>(tri * 255 / half);
        fill(px, config_.halo, Rgb{lvl, 0, 0});
    }

    // --- Functional layer: brake. ---
    if (state.braking) {
        fill(px, config_.brake, kBrightRed);
    }

    // --- Rain light: flash while ERS is HARVESTING (ersPercent rising in
    // ERS mode) -- the real-F1 mapping, derived locally since the frame has
    // no explicit harvest flag. ---
    if (harvestSeeded_ && state.driveMode == 2 && state.ersPercent > lastErsPercent_) {
        lastHarvestMs_ = nowMs;
    }
    lastErsPercent_ = state.ersPercent;
    harvestSeeded_ = true;
    const bool harvesting = (nowMs - lastHarvestMs_) < config_.harvestWindowMs && lastHarvestMs_ != 0;
    if (harvesting) {
        Rgb c = blinkOn(nowMs, config_.rainPeriodMs) ? kWhite : kOff;
        fill(px, config_.rainLight, c);
    }

    // --- Indicators: steering-threshold with hysteresis + min-on (one full
    // blink cycle guaranteed via free-running phase). ---
    const int8_t steer = state.steeringPercent;
    if (steer >= config_.indicatorOnPercent) {
        rightOn_ = true;
        leftOn_ = false;
    } else if (steer <= -config_.indicatorOnPercent) {
        leftOn_ = true;
        rightOn_ = false;
    } else if (steer > -config_.indicatorOffPercent && steer < config_.indicatorOffPercent) {
        leftOn_ = false;
        rightOn_ = false;
    }
    const bool indBlink = blinkOn(nowMs, config_.indicatorPeriodMs);
    if (leftOn_ && indBlink) {
        fill(px, config_.leftIndicator, kAmber);
    }
    if (rightOn_ && indBlink) {
        fill(px, config_.rightIndicator, kAmber);
    }

    for (uint8_t i = 0; i < kNumPixels; ++i) {
        outPixels[i] = applyBrightnessAndGamma(px[i], config_.maxBrightness);
    }
}

} // namespace lights
