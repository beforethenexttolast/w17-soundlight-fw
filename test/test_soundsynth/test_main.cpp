#include <unity.h>

#include <cmath>
#include <cstdint>

#include "soundsynth/EngineSynth.hpp"

using soundsynth::clampToInt16;
using soundsynth::EngineSynth;
using soundsynth::EngineSynthConfig;
using soundsynth::kSampleRateHz;
using soundsynth::detail::smoothingStepForDelta;

namespace {

// Counts upward zero crossings (L channel) over a rendered block -> ~ the
// fundamental frequency of a single-oscillator signal.
int countUpwardZeroCrossings(const int16_t* buf, size_t frames) {
    int count = 0;
    for (size_t f = 1; f < frames; ++f) {
        if (buf[2 * (f - 1)] <= 0 && buf[2 * f] > 0) {
            count++;
        }
    }
    return count;
}

// A valid() config that routes ALL output through the noise multiplication
// (partials + whine zeroed), so the widened multiply is the only contributor.
// With noiseAmpMax at the headroom ceiling this is the worst case for the
// formerly-overflowing signed int32 product, yet peakSum()==noiseAmpMax so
// valid() still holds -- it is a legal public config, not a weakened one.
EngineSynthConfig allNoiseConfig(int16_t noiseAmpMax) {
    EngineSynthConfig cfg;
    for (int i = 0; i < soundsynth::kMaxPartials; ++i) cfg.partialAmp[i] = 0;
    cfg.noiseAmpMax = noiseAmpMax;
    cfg.whineAmp = 0;
    return cfg;
}

// Scan an interleaved L/R block, OR-ing in whether either int16 rail was hit.
void scanRails(const int16_t* buf, size_t samples, bool& sawPos, bool& sawNeg) {
    for (size_t i = 0; i < samples; ++i) {
        if (buf[i] == 32767) sawPos = true;
        if (buf[i] == -32768) sawNeg = true;
    }
}

} // namespace

void setUp() {}
void tearDown() {}

void test_config_valid_and_headroom() {
    TEST_ASSERT_TRUE(EngineSynthConfig{}.valid());
    // Default partial+noise+whine sum must fit under the clip headroom.
    TEST_ASSERT_TRUE(EngineSynthConfig{}.peakSum() <= EngineSynthConfig::kHeadroomPeak);

    EngineSynthConfig loud;
    loud.partialAmp[0] = 30000; // blow the headroom
    TEST_ASSERT_FALSE(loud.valid());
}

void test_pitch_matches_firing_frequency() {
    // Fundamental-only config so zero-crossings equal the fundamental (a
    // multi-partial stack ripples and would over-count).
    EngineSynthConfig cfg;
    for (int i = 0; i < soundsynth::kMaxPartials; ++i) cfg.partialAmp[i] = 0;
    cfg.partialAmp[0] = 200;
    cfg.noiseAmpMax = 0;
    cfg.whineAmp = 0;
    EngineSynth synth(cfg);

    // 12000 rpm, 5 firings/rev => f = 12000/60*5 = 1000 Hz.
    synth.setParams(12000, 255, false, false, false);

    // Render 1 second; skip the first block so the smoother has settled.
    const size_t oneSec = kSampleRateHz;
    static int16_t buf[kSampleRateHz * 2];
    synth.render(buf, oneSec); // warm-up + measured in one go is fine at const rpm

    const int crossings = countUpwardZeroCrossings(buf, oneSec);
    // Smoothing ramps rpm from 0 at the start, so allow generous tolerance;
    // the steady-state fundamental is 1000 Hz.
    TEST_ASSERT_INT_WITHIN(120, 1000, crossings);
}

void test_volume_scales_amplitude() {
    EngineSynth synth;
    static int16_t loud[512 * 2];
    static int16_t quiet[512 * 2];

    synth.setParams(9000, 255, false, false, false);
    for (int i = 0; i < 40; ++i) synth.render(loud, 512); // settle
    synth.render(loud, 512);
    int32_t loudPeak = 0;
    for (int i = 0; i < 512; ++i) {
        int32_t a = loud[2 * i] < 0 ? -loud[2 * i] : loud[2 * i];
        if (a > loudPeak) loudPeak = a;
    }

    EngineSynth synth2;
    synth2.setParams(9000, 40, false, false, false);
    for (int i = 0; i < 40; ++i) synth2.render(quiet, 512);
    synth2.render(quiet, 512);
    int32_t quietPeak = 0;
    for (int i = 0; i < 512; ++i) {
        int32_t a = quiet[2 * i] < 0 ? -quiet[2 * i] : quiet[2 * i];
        if (a > quietPeak) quietPeak = a;
    }

    TEST_ASSERT_TRUE(quietPeak < loudPeak);
}

void test_zero_volume_is_silent() {
    EngineSynth synth;
    synth.setParams(9000, 0, false, false, false);
    static int16_t buf[512 * 2];
    for (int i = 0; i < 100; ++i) synth.render(buf, 512); // let the smoother reach 0
    synth.render(buf, 512);
    for (int i = 0; i < 512 * 2; ++i) {
        TEST_ASSERT_EQUAL_INT16(0, buf[i]);
    }
}

// Direct test of the production clamp helper (clampToInt16). The render path
// calls this exact function (see test_render_path_saturates_via_clamp), so
// these boundaries pin the real int16 clip policy. Expected values are fixed
// literals / standard limits -- never computed by another clamp. This fails if
// the clamp is removed, reversed, or given the wrong limits.
void test_clamp_to_int16_boundaries() {
    // Around zero and small in-range values.
    TEST_ASSERT_EQUAL_INT16(0, clampToInt16(0));
    TEST_ASSERT_EQUAL_INT16(1, clampToInt16(1));
    TEST_ASSERT_EQUAL_INT16(-1, clampToInt16(-1));

    // Representative in-range values (well inside both rails).
    TEST_ASSERT_EQUAL_INT16(12345, clampToInt16(12345));
    TEST_ASSERT_EQUAL_INT16(-12345, clampToInt16(-12345));

    // Positive rail: pass through up to 32767, saturate strictly above it.
    TEST_ASSERT_EQUAL_INT16(32766, clampToInt16(32766));
    TEST_ASSERT_EQUAL_INT16(32767, clampToInt16(32767));
    TEST_ASSERT_EQUAL_INT16(32767, clampToInt16(32768));
    TEST_ASSERT_EQUAL_INT16(32767, clampToInt16(32769));
    TEST_ASSERT_EQUAL_INT16(32767, clampToInt16(2000000000));
    TEST_ASSERT_EQUAL_INT16(32767, clampToInt16(INT32_MAX));

    // Negative rail: pass through down to -32768, saturate strictly below it.
    // Note -32768 is a LEGAL sample (asymmetry vs +32767) and must not clamp.
    TEST_ASSERT_EQUAL_INT16(-32767, clampToInt16(-32767));
    TEST_ASSERT_EQUAL_INT16(-32768, clampToInt16(-32768));
    TEST_ASSERT_EQUAL_INT16(-32768, clampToInt16(-32769));
    TEST_ASSERT_EQUAL_INT16(-32768, clampToInt16(-32770));
    TEST_ASSERT_EQUAL_INT16(-32768, clampToInt16(-2000000000));
    TEST_ASSERT_EQUAL_INT16(-32768, clampToInt16(INT32_MIN));

    // Prove the rails are intentionally asymmetric: |min| = |max| + 1.
    TEST_ASSERT_EQUAL_INT16(32767, clampToInt16(INT32_MAX));
    TEST_ASSERT_EQUAL_INT16(-32768, clampToInt16(INT32_MIN));
    TEST_ASSERT_NOT_EQUAL(-clampToInt16(INT32_MAX), clampToInt16(INT32_MIN));
}

// Proves the production render path actually routes through clampToInt16. The
// default (production) config never clips (see test_headroom_...), so drive a
// still-valid() config whose overrun crackle burst (3x noiseAmpMax) legally
// exceeds the int16 rails, and assert the real renderer saturates to the exact
// clamp limits on BOTH signs. peakSum() stays within the headroom budget so
// valid() is not weakened; the burst is genuine production DSP that the
// steady-state headroom budget does not cover. No overflow: the widest noise
// product is 32768 * (3 * 14000) ~= 1.38e9 < INT32_MAX.
void test_render_path_saturates_via_clamp() {
    EngineSynthConfig cfg;
    for (int i = 0; i < soundsynth::kMaxPartials; ++i) cfg.partialAmp[i] = 0;
    cfg.partialAmp[0] = 8000;
    cfg.noiseAmpMax = 14000;
    cfg.whineAmp = 0;
    TEST_ASSERT_TRUE(cfg.valid()); // not weakened; peakSum == 22000 <= headroom
    TEST_ASSERT_TRUE(cfg.peakSum() <= EngineSynthConfig::kHeadroomPeak);

    EngineSynth synth(cfg, 0x1234u);
    synth.setParams(15000, 255, /*whine=*/false, /*limiter=*/false, /*overrun=*/true);
    static int16_t buf[256 * 2];
    bool sawPosRail = false;
    bool sawNegRail = false;
    for (int block = 0; block < 60 && !(sawPosRail && sawNegRail); ++block) {
        synth.render(buf, 256);
        for (int i = 0; i < 256 * 2; ++i) {
            // A sample only reaches exactly a rail because the clamp pinned an
            // out-of-range accumulator there -- proving render() calls it.
            if (buf[i] == 32767) sawPosRail = true;
            if (buf[i] == -32768) sawNegRail = true;
        }
    }
    TEST_ASSERT_TRUE(sawPosRail); // positive saturation observed via render()
    TEST_ASSERT_TRUE(sawNegRail); // negative saturation observed via render()
}

// Replaces the former vacuous "never clips" test (which asserted an int16 lay
// within int16 range -- always true). The real contract at MAX production
// settings is a headroom one: the default config is deliberately scaled so the
// mixed output stays under the documented kHeadroomPeak budget and never
// reaches the int16 rails. The bound is the independent design constant, not a
// value recomputed from the renderer. Would fail if partial/noise/whine amps
// were bumped past the headroom budget.
void test_headroom_holds_below_rail_at_max_settings() {
    EngineSynth synth; // default (production) config
    synth.setParams(15000, 255, /*whine=*/true, /*limiter=*/false, /*overrun=*/true);
    static int16_t buf[1024 * 2];
    int32_t peak = 0;
    for (int block = 0; block < 200; ++block) {
        synth.render(buf, 1024);
        for (int i = 0; i < 1024 * 2; ++i) {
            const int32_t a = buf[i] < 0 ? -static_cast<int32_t>(buf[i])
                                         : static_cast<int32_t>(buf[i]);
            if (a > peak) peak = a;
        }
    }
    TEST_ASSERT_TRUE(peak > 0);                                   // signal present
    TEST_ASSERT_TRUE(peak <= EngineSynthConfig::kHeadroomPeak);   // within budget
    TEST_ASSERT_TRUE(peak < 32767);                              // never touches the rail
}

void test_stereo_channels_identical() {
    EngineSynth synth;
    synth.setParams(8000, 200, true, false, false);
    static int16_t buf[256 * 2];
    for (int i = 0; i < 20; ++i) synth.render(buf, 256);
    synth.render(buf, 256);
    for (int f = 0; f < 256; ++f) {
        TEST_ASSERT_EQUAL_INT16(buf[2 * f], buf[2 * f + 1]); // mono duplicated to L+R
    }
}

void test_limiter_gates_some_samples_to_zero() {
    EngineSynth synth;
    synth.setParams(15000, 255, false, /*limiter=*/true, false);
    static int16_t buf[kSampleRateHz / 2 * 2]; // 0.5s
    for (int i = 0; i < 20; ++i) synth.render(buf, 512); // settle
    synth.render(buf, kSampleRateHz / 2);
    int zeros = 0;
    for (size_t f = 0; f < kSampleRateHz / 2; ++f) {
        if (buf[2 * f] == 0) zeros++;
    }
    // The ~18 Hz ignition cut zeroes roughly half the samples in bursts.
    TEST_ASSERT_TRUE(zeros > 1000);
}

void test_deterministic_given_seed() {
    EngineSynth a(EngineSynthConfig{}, 0xABCDu);
    EngineSynth b(EngineSynthConfig{}, 0xABCDu);
    a.setParams(10000, 200, true, false, true);
    b.setParams(10000, 200, true, false, true);
    static int16_t ba[256 * 2];
    static int16_t bb[256 * 2];
    for (int i = 0; i < 10; ++i) {
        a.render(ba, 256);
        b.render(bb, 256);
    }
    for (int i = 0; i < 256 * 2; ++i) {
        TEST_ASSERT_EQUAL_INT16(ba[i], bb[i]);
    }
}

void test_packed_params_roundtrip() {
    const uint32_t p = soundsynth::packParams(12345, 200, true, false, true);
    EngineSynth synth;
    synth.applyPackedParams(p);
    // Render advances using the unpacked params; just assert it produces
    // non-silent output at a high rpm/volume (params were applied).
    static int16_t buf[256 * 2];
    for (int i = 0; i < 40; ++i) synth.render(buf, 256);
    synth.render(buf, 256);
    int32_t peak = 0;
    for (int i = 0; i < 256 * 2; ++i) {
        int32_t v = buf[i] < 0 ? -buf[i] : buf[i];
        if (v > peak) peak = v;
    }
    TEST_ASSERT_TRUE(peak > 0);
}

// --- Arithmetic-safety batch: the noise multiplication is now widened to
// int64 before the >> 15. These drive valid() extreme configs through the REAL
// render() path (never re-deriving the synthesis formula) to prove the
// formerly-overflowing product now completes, saturates via the production
// clamp, and stays deterministic. Strongest when paired with the UBSan harness
// (run out-of-repo): here they also stand alone as behavioral regressions. ---

// (1) Valid all-noise config at the headroom ceiling, rendered at ~production
// rpm with the overrun crackle active (noiseAmp = 3 * noiseAmpMax = 90000, well
// past the pre-fix safe int32 limit). Both rails must be reached and two
// same-seed instances must agree byte-for-byte.
void test_valid_overrun_extreme_saturates_and_deterministic() {
    EngineSynthConfig cfg = allNoiseConfig(EngineSynthConfig::kHeadroomPeak); // 30000
    TEST_ASSERT_TRUE(cfg.valid());
    TEST_ASSERT_EQUAL_INT32(EngineSynthConfig::kHeadroomPeak, cfg.peakSum());

    EngineSynth a(cfg, 0xF00Du);
    EngineSynth b(cfg, 0xF00Du);
    a.setParams(15000, 255, /*whine=*/false, /*limiter=*/false, /*overrun=*/true);
    b.setParams(15000, 255, /*whine=*/false, /*limiter=*/false, /*overrun=*/true);

    static int16_t ba[512 * 2];
    static int16_t bb[512 * 2];
    bool sawPos = false, sawNeg = false, identical = true;
    for (int block = 0; block < 80; ++block) {
        a.render(ba, 512);
        b.render(bb, 512);
        scanRails(ba, 512 * 2, sawPos, sawNeg);
        for (int i = 0; i < 512 * 2; ++i) {
            if (ba[i] != bb[i]) identical = false;
        }
    }
    TEST_ASSERT_TRUE(sawPos);     // +32767 reached through render()'s clamp
    TEST_ASSERT_TRUE(sawNeg);     // -32768 reached through render()'s clamp
    TEST_ASSERT_TRUE(identical);  // same seed + inputs => byte-identical
}

// (2) Same valid all-noise config, overrun OFF, commanding the max uint16 rpm
// (65535) at full volume. This exercises the rpm-scaled noiseAmp path
// (noiseAmp = noiseAmpMax * rpm / 15000), which for a headroom-ceiling config
// climbs to ~130k -- the widest valid noiseAmp. NOTE: production EngineSim does
// not emit rpm 65535; this deliberately tests the public EngineSynth
// input/config contract, not the EngineSim range.
void test_valid_high_rpm_extreme_saturates_and_deterministic() {
    EngineSynthConfig cfg = allNoiseConfig(EngineSynthConfig::kHeadroomPeak); // 30000
    TEST_ASSERT_TRUE(cfg.valid());

    EngineSynth a(cfg, 0x5A5Au);
    EngineSynth b(cfg, 0x5A5Au);
    a.setParams(65535, 255, /*whine=*/false, /*limiter=*/false, /*overrun=*/false);
    b.setParams(65535, 255, /*whine=*/false, /*limiter=*/false, /*overrun=*/false);

    static int16_t ba[1024 * 2];
    static int16_t bb[1024 * 2];
    bool sawPos = false, sawNeg = false, identical = true;
    // ~9 s of audio: ample for the ~23 ms smoother to climb into the high-rpm
    // regime and for the noise sequence to walk the formerly-overflowing path.
    for (int block = 0; block < 200; ++block) {
        a.render(ba, 1024);
        b.render(bb, 1024);
        scanRails(ba, 1024 * 2, sawPos, sawNeg);
        for (int i = 0; i < 1024 * 2; ++i) {
            if (ba[i] != bb[i]) identical = false;
        }
    }
    TEST_ASSERT_TRUE(sawPos);
    TEST_ASSERT_TRUE(sawNeg);
    TEST_ASSERT_TRUE(identical);
}

// (3) The audited safe/first-overflow boundaries for each amplification path.
// Pre-fix these straddled the int32 limit (21846 and 15015 overflowed); post-
// fix all four are valid() and render to completion. Assertions are behavioral
// (both rails observed + same-seed determinism), never a tautological
// "within int16 range" check.
namespace {
struct BoundaryCase {
    int16_t noiseAmpMax;
    bool overrun;
    uint16_t rpm;
};
void runBoundaryCase(const BoundaryCase& c) {
    EngineSynthConfig cfg = allNoiseConfig(c.noiseAmpMax);
    TEST_ASSERT_TRUE(cfg.valid());

    EngineSynth a(cfg, 0x2468u);
    EngineSynth b(cfg, 0x2468u);
    a.setParams(c.rpm, 255, /*whine=*/false, /*limiter=*/false, c.overrun);
    b.setParams(c.rpm, 255, /*whine=*/false, /*limiter=*/false, c.overrun);

    static int16_t ba[1024 * 2];
    static int16_t bb[1024 * 2];
    bool sawPos = false, sawNeg = false, identical = true;
    for (int block = 0; block < 200; ++block) {
        a.render(ba, 1024);
        b.render(bb, 1024);
        scanRails(ba, 1024 * 2, sawPos, sawNeg);
        for (int i = 0; i < 1024 * 2; ++i) {
            if (ba[i] != bb[i]) identical = false;
        }
    }
    TEST_ASSERT_TRUE(sawPos);
    TEST_ASSERT_TRUE(sawNeg);
    TEST_ASSERT_TRUE(identical);
}
} // namespace

void test_boundary_configs_render_safely_and_deterministic() {
    const BoundaryCase cases[] = {
        {21845, /*overrun=*/true, 15000},  // overrun path: last safe pre-fix
        {21846, /*overrun=*/true, 15000},  // overrun path: first pre-fix overflow
        {15014, /*overrun=*/false, 65535}, // high-rpm path: last safe pre-fix
        {15015, /*overrun=*/false, 65535}, // high-rpm path: first pre-fix overflow
    };
    for (const auto& c : cases) {
        runBoundaryCase(c);
    }
}

// --- Param-smoothing batch (SS-1). The render-path low-pass on rpm/volume
// used a bare arithmetic (target - smooth) >> 6, which stalls 63 units short
// for a positive residual of 1..63 (that shifts to 0) while a negative
// residual of -1..-63 shifts to -1 and still converges. The result was that
// steady-state loudness depended on command history. detail::smoothingStepForDelta
// is the production policy: same 1/64 step for every nonzero shifted result,
// but a minimum one-unit step so a nonzero gap always closes. These tests call
// the EXACT production helper (never a re-implemented formula). ---

// (1) Step-boundary policy at the audited residual boundaries. Expected values
// are fixed literals. Confirms: every old nonzero-step result is preserved
// (including negative non-multiple rounding), positive residuals 1..63 now
// step +1 instead of 0, zero stays zero, and no step overshoots its residual.
void test_smoothing_step_boundary_policy() {
    // Negative side: unchanged from the bare arithmetic shift.
    TEST_ASSERT_EQUAL_INT32(-2, smoothingStepForDelta(-128));
    TEST_ASSERT_EQUAL_INT32(-2, smoothingStepForDelta(-127));
    TEST_ASSERT_EQUAL_INT32(-2, smoothingStepForDelta(-65));
    TEST_ASSERT_EQUAL_INT32(-1, smoothingStepForDelta(-64));
    TEST_ASSERT_EQUAL_INT32(-1, smoothingStepForDelta(-63));
    TEST_ASSERT_EQUAL_INT32(-1, smoothingStepForDelta(-1));
    // Zero: no movement.
    TEST_ASSERT_EQUAL_INT32(0, smoothingStepForDelta(0));
    // Positive side: 1..63 now yield +1 (was 0); 64+ unchanged.
    TEST_ASSERT_EQUAL_INT32(1, smoothingStepForDelta(1));
    TEST_ASSERT_EQUAL_INT32(1, smoothingStepForDelta(63));
    TEST_ASSERT_EQUAL_INT32(1, smoothingStepForDelta(64));
    TEST_ASSERT_EQUAL_INT32(1, smoothingStepForDelta(65));
    TEST_ASSERT_EQUAL_INT32(1, smoothingStepForDelta(127));
    TEST_ASSERT_EQUAL_INT32(2, smoothingStepForDelta(128));

    // No step ever overshoots its own residual (|step| <= |delta|, same sign).
    for (int32_t d = -300; d <= 300; ++d) {
        const int32_t step = smoothingStepForDelta(d);
        if (d == 0) {
            TEST_ASSERT_EQUAL_INT32(0, step);
        } else {
            TEST_ASSERT_TRUE((step > 0) == (d > 0)); // same sign, always moves
            const int32_t ad0 = d < 0 ? -d : d;
            const int32_t as0 = step < 0 ? -step : step;
            TEST_ASSERT_TRUE(as0 <= ad0); // never past the target
        }
    }
}

// (2) Exact convergence: driving the production helper repeatedly from a start
// to a target reaches the target exactly, never overshoots, and stays put.
namespace {
void assertConvergesExactly(int32_t start, int32_t target) {
    int32_t v = start;
    const bool fromBelow = start < target;
    // 1/64 + min-1-unit closes any gap in the value domain well within this.
    for (int i = 0; i < 100000; ++i) {
        const int32_t next = v + smoothingStepForDelta(target - v);
        if (fromBelow) {
            TEST_ASSERT_TRUE(next <= target); // no overshoot from below
        } else {
            TEST_ASSERT_TRUE(next >= target); // no overshoot from above
        }
        v = next;
        if (v == target) break;
    }
    TEST_ASSERT_EQUAL_INT32(target, v); // exact arrival within the bound
    // Once at the target the step is zero: it holds.
    TEST_ASSERT_EQUAL_INT32(0, smoothingStepForDelta(target - v));
    TEST_ASSERT_EQUAL_INT32(target, v + smoothingStepForDelta(target - v));
}
} // namespace

void test_smoothing_converges_exactly() {
    assertConvergesExactly(0, 1);
    assertConvergesExactly(0, 63);   // formerly stalled at 0
    assertConvergesExactly(0, 70);   // formerly stalled at 7
    assertConvergesExactly(0, 90);   // formerly stalled at 27
    assertConvergesExactly(0, 255);  // formerly stalled at 192
    assertConvergesExactly(255, 90); // from above: already converged pre-fix
    assertConvergesExactly(255, 0);  // mute path: already converged pre-fix
    assertConvergesExactly(90, 27);  // from above
}

// (3) Renderer-level history-independence regression. Two synths with identical
// config, seed, rpm and flags, rendered the same number of samples, differ only
// in the volume they were commanded during warm-up. After both are commanded
// the same final volume and allowed to converge, their output must be byte-
// identical (steady-state loudness is history-independent). Because volume
// affects neither the oscillator phase nor the noise LFSR, and both synths see
// identical rpm/flags for identical sample counts, phase and noise progression
// stay aligned -- so equal output is exact, not approximate.
//
// On the OLD implementation this FAILS: synth A (warmed up commanding 90 from 0)
// stalls at smoothed volume 27, while synth B (warmed up commanding 255) drops
// from 192 to exactly 90 -- so their final blocks diverge.
void test_renderer_volume_history_independent() {
    EngineSynthConfig cfg; // default (production) config
    EngineSynth a(cfg, 0x1234u);
    EngineSynth b(cfg, 0x1234u);

    static int16_t ba[512 * 2];
    static int16_t bb[512 * 2];

    // Warm-up: identical rpm/flags, DIFFERENT commanded volume.
    a.setParams(9000, 90, false, false, false);
    b.setParams(9000, 255, false, false, false);
    for (int i = 0; i < 16; ++i) { // 16 * 512 = 8192 samples
        a.render(ba, 512);
        b.render(bb, 512);
    }

    // Command the SAME final volume on both and let them converge.
    a.setParams(9000, 90, false, false, false);
    b.setParams(9000, 90, false, false, false);
    for (int i = 0; i < 16; ++i) {
        a.render(ba, 512);
        b.render(bb, 512);
    }

    // Compare a fresh block: must be byte-identical and non-silent.
    a.render(ba, 512);
    b.render(bb, 512);
    bool nonSilent = false;
    for (int i = 0; i < 512 * 2; ++i) {
        TEST_ASSERT_EQUAL_INT16(ba[i], bb[i]);
        if (ba[i] != 0) nonSilent = true;
    }
    TEST_ASSERT_TRUE(nonSilent);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_config_valid_and_headroom);
    RUN_TEST(test_pitch_matches_firing_frequency);
    RUN_TEST(test_volume_scales_amplitude);
    RUN_TEST(test_zero_volume_is_silent);
    RUN_TEST(test_clamp_to_int16_boundaries);
    RUN_TEST(test_render_path_saturates_via_clamp);
    RUN_TEST(test_headroom_holds_below_rail_at_max_settings);
    RUN_TEST(test_stereo_channels_identical);
    RUN_TEST(test_limiter_gates_some_samples_to_zero);
    RUN_TEST(test_deterministic_given_seed);
    RUN_TEST(test_packed_params_roundtrip);
    RUN_TEST(test_valid_overrun_extreme_saturates_and_deterministic);
    RUN_TEST(test_valid_high_rpm_extreme_saturates_and_deterministic);
    RUN_TEST(test_boundary_configs_render_safely_and_deterministic);
    RUN_TEST(test_smoothing_step_boundary_policy);
    RUN_TEST(test_smoothing_converges_exactly);
    RUN_TEST(test_renderer_volume_history_independent);
    return UNITY_END();
}
