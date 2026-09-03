#include "lights/LightRenderer.hpp"

namespace lights {

namespace {

// Petronas teal, F1 palette.
//
// THE QUIET ENTRIES ARE SET AGAINST THE CAP, NOT BY EYE. The brightness cap
// is applied before gamma (LightConfig::maxBrightness), so a full channel
// renders at 40/255 and the old kDimWhite/kDimRed value of 40 rendered at
// PWM 1 -- invisible, which is what sl:safety-1 caught. Each quiet entry is
// now the SMALLEST value whose brightest channel clears kMinVisibleDuty at
// the default cap, keeping the original hue ratios (dim white was 40:40:46,
// now 91:91:105). The static_asserts at the bottom of this namespace hold
// the line; test_lights checks it through the real renderer.
constexpr Rgb kTeal{0, 130, 120};
constexpr Rgb kDimWhite{91, 91, 105};
constexpr Rgb kDimRed{105, 0, 0};
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

// Showcase base halo (owner decision D6): a SLOW pure-teal breathe with a
// brightness floor, rendered with the dim red tail lit by the normal base
// layer. WHY it must be distinct from the NeverConnected grace breathe:
// the two states mean OPPOSITE things about board #1. Grace = PRE-FIRST-
// FRAME -- "no frame has ever arrived, board #1 may be booting or the
// harness may be dead" -- a bounded benefit-of-the-doubt window (<= 5 s,
// then hazard; audit defect 9). Showcase = FRAMES PRESENT with modeFlags
// bit0 set -- board #1 is alive, talking, and explicitly authorizing the
// demo (the monitor zeroes the bit in NeverConnected and Lost, so this
// look can only ever render on a live link). If the two looks could be
// confused, a dead harness would read as a healthy shelf demo -- the exact
// calm-pretty-lights-during-a-fault trap the design exists to exclude.
// Distinct on four axes, in the order the eye can actually use them AFTER
// cap + gamma (the honest ranking, sl:safety-1):
//   - floor: the showcase breathe never dips dark (duty 6..9 across the
//     cycle) vs the grace breathe touching black once every 2 s. This is
//     the loudest axis: one of them goes out, the other never does.
//   - context: dim red tail LIT (base layer) vs grace's halo-only frame.
//   - period: 3 s vs the grace's 2 s.
//   - color family: pure teal (r == 0) vs the grace breathe's cyan-white
//     tinge (r nonzero when lit). Weakest of the four -- the tinge is
//     1/255 of red against 6/255 of teal at the grace peak, so treat it as
//     a tie-breaker in a photo, not something a person reads across a room.
// Also distinct from solid armed teal (steady, duty 9) and dim-white
// disarmed (steady, gray). Amber/red stay reserved for faults/alerts.
constexpr uint32_t kShowcaseBreathePeriodMs = 3000;
// The dip of the breathe. Raised from 102 (40 % of scale, which rendered at
// PWM 1 -- the "floor" was black) to the smallest level whose teal clears
// kMinVisibleDuty. The cost of keeping cap-then-gamma (OD-12 Q1(a)) is a
// SHALLOW breathe: the whole usable range at this cap is duty 0..40 and the
// show now swings 6..9. Whether that reads as breathing in daylight is a
// bench judgement ([bench-TBD], open_questions.md #55); the alternative --
// dipping into invisibility -- is not a judgement call, it is a defect.
constexpr uint32_t kShowcaseBreatheFloor = 206; // ~81% of full scale
constexpr Rgb showcaseBreathe(uint32_t nowMs) {
    const uint32_t phase = nowMs % kShowcaseBreathePeriodMs;
    const uint32_t half = kShowcaseBreathePeriodMs / 2u;
    const uint32_t tri = phase < half ? phase : (kShowcaseBreathePeriodMs - phase);
    const uint32_t lvl =
        kShowcaseBreatheFloor + tri * (255u - kShowcaseBreatheFloor) / half;
    return Rgb{0, static_cast<uint8_t>(kTeal.g * lvl / 255u),
               static_cast<uint8_t>(kTeal.b * lvl / 255u)};
}

// NeverConnected grace breathe: the calm "waiting for board #1" look. It
// breathes from BLACK (deliberate -- the distinctness axis above) up to this
// peak, which is the smallest cyan-white that clears kMinVisibleDuty on its
// brightest channel. The 1:2:2 red:green:blue ratio is the original
// lvl/6 : lvl/3 : lvl/3 tinge, preserved.
constexpr uint32_t kGraceBreathePeriodMs = 2000;
constexpr Rgb kGraceBreathePeak{55, 110, 110};
constexpr Rgb graceBreathe(uint32_t nowMs) {
    const uint32_t phase = nowMs % kGraceBreathePeriodMs;
    const uint32_t half = kGraceBreathePeriodMs / 2u;
    const uint32_t tri = phase < half ? phase : (kGraceBreathePeriodMs - phase);
    const uint32_t lvl = tri * 255u / half; // 0..255..0
    return Rgb{static_cast<uint8_t>(kGraceBreathePeak.r * lvl / 255u),
               static_cast<uint8_t>(kGraceBreathePeak.g * lvl / 255u),
               static_cast<uint8_t>(kGraceBreathePeak.b * lvl / 255u)};
}

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
// Also halo-covering (disarmed dim-white halo, NeverConnected grace peak) --
// raised by sl:safety-1's minimum-visible-duty fix, so pin them here too
// rather than trusting that a floor raised for visibility stayed under the
// ceiling raised for current.
static_assert(channelSum(kDimWhite) <= kBudgetModelChannelSum,
              "disarmed halo exceeds budget model");
static_assert(channelSum(kGraceBreathePeak) <= kBudgetModelChannelSum,
              "grace breathe peak exceeds budget model");
// The showcase breathe peaks at exactly kTeal (lvl 255) and only ever scales
// it down -- pin the peak so the budget claim survives a formula tweak.
static_assert(channelSum(showcaseBreathe(kShowcaseBreathePeriodMs / 2)) ==
                  channelSum(kTeal),
              "showcase breathe peak must be the plain halo teal");
static_assert(channelSum(showcaseBreathe(0)) <= channelSum(kTeal),
              "showcase breathe floor exceeds its own peak");

// Cap BEFORE gamma -- see LightConfig::maxBrightness for why the order is
// what it is and what it costs. The curve itself is the constexpr LUT in the
// header (one copy, shared with the power budget in valid()).
Rgb applyBrightnessAndGamma(Rgb c, uint8_t maxBrightness) {
    auto ch = [&](uint8_t x) { return renderedDuty(x, maxBrightness); };
    return Rgb{ch(c.r), ch(c.g), ch(c.b)};
}

// ---- The minimum-visible contract, checked where the palette is defined ----
//
// A state the design intends someone to SEE must render at least
// kMinVisibleDuty on its brightest channel at the default cap. These
// static_asserts are the compile-time half (a palette edit that drops a quiet
// state back to PWM 1 fails the build); test_lights drives the same claim
// through the real renderer, states rather than colors.
constexpr uint8_t maxRenderedDuty(Rgb c) {
    const uint8_t cap = LightConfig{}.maxBrightness;
    const uint8_t r = renderedDuty(c.r, cap);
    const uint8_t g = renderedDuty(c.g, cap);
    const uint8_t b = renderedDuty(c.b, cap);
    const uint8_t rg = r > g ? r : g;
    return rg > b ? rg : b;
}
static_assert(maxRenderedDuty(kDimWhite) >= kMinVisibleDuty, "disarmed halo renders invisible");
static_assert(maxRenderedDuty(kDimRed) >= kMinVisibleDuty, "dim red tail renders invisible");
static_assert(maxRenderedDuty(kTeal) >= kMinVisibleDuty, "armed halo renders invisible");
static_assert(maxRenderedDuty(kAmber) >= kMinVisibleDuty, "hazard/indicator amber invisible");
static_assert(maxRenderedDuty(kBrightRed) >= kMinVisibleDuty, "brake light renders invisible");
static_assert(maxRenderedDuty(kWhite) >= kMinVisibleDuty, "rain light renders invisible");
static_assert(maxRenderedDuty(kDrsGreen) >= kMinVisibleDuty, "DRS tell renders invisible");
static_assert(maxRenderedDuty(kIgnitionCyan) >= kMinVisibleDuty, "starter comet head invisible");
static_assert(maxRenderedDuty(showcaseBreathe(0)) >= kMinVisibleDuty,
              "showcase breathe dips out of sight");
static_assert(maxRenderedDuty(graceBreathe(kGraceBreathePeriodMs / 2)) >= kMinVisibleDuty,
              "never-connected breathe peaks out of sight");
// DELIBERATE EXCLUSIONS, so their absence above is not read as an oversight:
// the two comet TRAIL pixels (duty 4 and 1) are a fading gradient behind a
// duty-40 head, not states that stand alone; the low-battery and hazard
// pulses are checked at their PEAK (kBrightRed / kAmber above) because a
// blink whose dark half is dark is the point.
static_assert(maxRenderedDuty(kIgnitionTrail) < maxRenderedDuty(kIgnitionCyan),
              "the comet trail must stay dimmer than its head");
static_assert(maxRenderedDuty(kIgnitionTrail2) <= maxRenderedDuty(kIgnitionTrail),
              "the comet tail must stay dimmer than the trail");
// The quiet states must also stay READABLE against their loud counterparts:
// a dim tail that is nearly as bright as the brake light is a brake light
// that never seems to come on.
static_assert(maxRenderedDuty(kDimRed) * 3 <= maxRenderedDuty(kBrightRed),
              "dim tail too close to the brake light");
static_assert(maxRenderedDuty(kDimWhite) < maxRenderedDuty(kTeal),
              "disarmed halo must read dimmer than the armed halo");

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
            fill(px, config_.halo, graceBreathe(nowMs));
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
        // Base halo. armed teal outranks the showcase breathe on purpose: a
        // truthful board #1 never sends armed+showcase together (the
        // showcase boot pins its arm input false), but if a frame carried
        // both anyway the car must LOOK armed -- honesty beats aesthetics.
        // The breathe itself renders only with the link Up: the monitor
        // zeroes the showcase bit in both NeverConnected and Lost, so the
        // grace/hazard paths above never see it (see showcaseBreathe's
        // distinctness comment).
        Rgb haloColor =
            state.armed ? kTeal : (state.showcase ? showcaseBreathe(nowMs) : kDimWhite);
        if (flashActive_) {
            // Wrap-safe window; expiry clears the flag so elapsed can never
            // wrap back below the duration and phantom-reopen.
            const uint32_t elapsed = nowMs - flashStartMs_;
            if (elapsed >= config_.ignitionFlashMs) {
                flashActive_ = false;
            } else {
                // "Engine catches" flash crossfades into whatever the base
                // look is -- armed teal in a drive boot (byte-identical to
                // before), the breathe in a showcase boot (the catch
                // settles into the show without a color snap).
                haloColor =
                    blendToward(kIgnitionCyan, haloColor, elapsed, config_.ignitionFlashMs);
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

    // --- Indicators: steering-threshold with hysteresis + minimum-on.
    //
    // The minimum-on half was documented and NOT implemented until
    // correctness-1: the blink is a free-running square wave, so a 240 ms
    // flick that happened to land in a dark half-cycle latched on and
    // self-cancelled again without ever lighting a pixel. The latch now
    // survives one full indicatorPeriodMs from its rising edge. The blink
    // stays phase-locked to nowMs (not to the latch), so that window always
    // contains indicatorPeriodMs/2 of lit time (330 ms at the default), in
    // at most two separate runs -- it does NOT guarantee one contiguous
    // half-cycle. The shortest guaranteed CONTINUOUS flash is
    // indicatorPeriodMs/4 (165 ms at the default; worst case is a latch
    // rising edge at phase indicatorPeriodMs/4, splitting the lit time into
    // two 165 ms runs either side of a 330 ms dark gap). test_lights pins
    // both the 330 ms total and the 165 ms worst-case contiguous run over
    // every start phase.
    //
    // Deliberately self-cancel only. Steering hard the other way still swaps
    // sides immediately -- that is a new gesture, not an expired one, and
    // both indicators blinking at once would read as hazard. ---
    const int8_t steer = state.steeringPercent;
    const bool wasLeftOn = leftOn_;
    const bool wasRightOn = rightOn_;
    if (steer >= config_.indicatorOnPercent) {
        rightOn_ = true;
        leftOn_ = false;
    } else if (steer <= -config_.indicatorOnPercent) {
        leftOn_ = true;
        rightOn_ = false;
    } else if (steer > -config_.indicatorOffPercent && steer < config_.indicatorOffPercent) {
        // Wrap-safe elapsed compare, same convention as the other timed
        // effects in this file.
        if (static_cast<uint32_t>(nowMs - indicatorStartMs_) >= config_.indicatorPeriodMs) {
            leftOn_ = false;
            rightOn_ = false;
        }
    }
    if ((leftOn_ && !wasLeftOn) || (rightOn_ && !wasRightOn)) {
        indicatorStartMs_ = nowMs;
    }
    const bool indBlink = blinkOn(nowMs, config_.indicatorPeriodMs);
    if (leftOn_ && indBlink) {
        fill(px, config_.leftIndicator, kAmber);
    }
    if (rightOn_ && indBlink) {
        fill(px, config_.rightIndicator, kAmber);
    }

    // --- ALERT layer: the low-battery halo pulse (D5). Composited AFTER the
    // functional layer, which is the order LightRenderer.hpp has always
    // declared (base -> DRS -> brake/rain/indicators -> low-battery ->
    // hazard) and the reverse of where this block used to sit
    // (correctness-4).
    //
    // Moot with the shipped segments -- the halo does not overlap brake,
    // rain or the indicators -- so nothing renders differently today. Fixed
    // in the CODE rather than by relaxing the header comment because the
    // segments are an explicit bench tunable: the day someone widens the
    // halo over a functional segment, "head home now" is the cue that must
    // not be chopped up by a blinker. The failsafe hazard still outranks it
    // (early return above). ---
    if (state.lowBattery) {
        const uint32_t phase = nowMs % config_.lowBatteryPeriodMs;
        const uint32_t half = config_.lowBatteryPeriodMs / 2u;
        const uint32_t tri = phase < half ? phase : (config_.lowBatteryPeriodMs - phase);
        const uint8_t lvl = static_cast<uint8_t>(tri * 255 / half);
        fill(px, config_.halo, Rgb{lvl, 0, 0});
    }

    for (uint8_t i = 0; i < kNumPixels; ++i) {
        outPixels[i] = applyBrightnessAndGamma(px[i], config_.maxBrightness);
    }
}

} // namespace lights
