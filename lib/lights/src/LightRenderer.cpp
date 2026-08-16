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
                           uint32_t nowMs, Rgb outPixels[kNumPixels]) {
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

    // --- Base layer: halo (teal armed / dim white disarmed) + dim red tail. ---
    fill(px, config_.brake, kDimRed);
    fill(px, config_.halo, state.armed ? kTeal : kDimWhite);

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
