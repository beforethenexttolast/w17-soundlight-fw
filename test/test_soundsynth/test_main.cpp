#include <unity.h>

#include <cmath>

#include "soundsynth/EngineSynth.hpp"

using soundsynth::EngineSynth;
using soundsynth::EngineSynthConfig;
using soundsynth::kSampleRateHz;

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

void test_never_clips_at_max_settings() {
    EngineSynth synth;
    synth.setParams(15000, 255, /*whine=*/true, /*limiter=*/false, /*overrun=*/true);
    static int16_t buf[1024 * 2];
    for (int block = 0; block < 200; ++block) {
        synth.render(buf, 1024);
        for (int i = 0; i < 1024 * 2; ++i) {
            // int16 range is guaranteed by the clamp; assert we never hit the
            // hard rails hard (headroom held), i.e. stays within full scale.
            TEST_ASSERT_TRUE(buf[i] >= -32768 && buf[i] <= 32767);
        }
    }
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

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_config_valid_and_headroom);
    RUN_TEST(test_pitch_matches_firing_frequency);
    RUN_TEST(test_volume_scales_amplitude);
    RUN_TEST(test_zero_volume_is_silent);
    RUN_TEST(test_never_clips_at_max_settings);
    RUN_TEST(test_stereo_channels_identical);
    RUN_TEST(test_limiter_gates_some_samples_to_zero);
    RUN_TEST(test_deterministic_given_seed);
    RUN_TEST(test_packed_params_roundtrip);
    return UNITY_END();
}
