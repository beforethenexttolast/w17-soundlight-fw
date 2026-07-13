#pragma once

#include <cstdint>

#include "soundsynth/ISampleSource.hpp"

namespace soundsynth {

inline constexpr uint32_t kSampleRateHz = 22050;
inline constexpr int kMaxPartials = 8;

// The single authoritative int16 clip policy for the render path. render()
// mixes into a 32-bit accumulator and calls this as the final saturating
// narrowing to the int16 sample transport. Kept as a public constexpr helper
// (no Arduino/ESP32 dependency, no allocation) so tests exercise the *exact*
// production clamp through the normal library include path rather than a copy.
// The limits are intentionally asymmetric: a two's-complement int16 covers
// [-32768, 32767], so -32768 is a legal sample and must survive unclamped.
constexpr int16_t clampToInt16(int32_t sample) {
    if (sample > 32767) sample = 32767;
    if (sample < -32768) sample = -32768;
    return static_cast<int16_t>(sample);
}

namespace detail {

// Per-sample smoothing step for the render-path low-pass on rpm/volume.
// Applies 1/64 of the remaining gap (arithmetic >> 6), but with a minimum
// one-unit step whenever the gap is nonzero, so a smoothed value converges
// *exactly* instead of stalling. Rationale for the shift-not-divide form:
// an arithmetic >> 6 rounds a positive residual of 1..63 to 0 (stall) but a
// negative residual of -1..-63 to -1 (already converges). We keep the exact
// existing behavior for every nonzero shifted step -- including negative
// non-multiple rounding, which `/ 64` would change -- and only patch the
// positive 1..63 zero-step case up to +1. This is deliberately asymmetric:
// approach-from-above and all larger steps are untouched.
constexpr int32_t smoothingStepForDelta(int32_t delta) {
    const int32_t shifted = delta >> 6;
    if (shifted != 0 || delta == 0) {
        return shifted;
    }
    return delta > 0 ? 1 : -1;
}

} // namespace detail

// Packed synth parameters, written by the control core and read by the audio
// core through a single std::atomic<uint32_t> (see EngineSynth::packParams /
// applyPackedParams). One 32-bit word => torn-free lock-free hand-off.
//   bits  0..15  engineRpm      (0..65535)
//   bits 16..23  volume         (0..255, 0 = silent)
//   bit     24   ersWhine
//   bit     25   limiterActive  (redline ignition-cut buzz)
//   bit     26   overrunActive  (lift-off crackle window)
//   bits 27..31  reserved
inline constexpr uint32_t packParams(uint16_t engineRpm, uint8_t volume, bool ersWhine,
                                     bool limiter, bool overrun) {
    return static_cast<uint32_t>(engineRpm) | (static_cast<uint32_t>(volume) << 16) |
           (static_cast<uint32_t>(ersWhine ? 1u : 0u) << 24) |
           (static_cast<uint32_t>(limiter ? 1u : 0u) << 25) |
           (static_cast<uint32_t>(overrun ? 1u : 0u) << 26);
}

struct EngineSynthConfig {
    // Firings per crank revolution sets the fundamental: f = engineRpm *
    // firingsPerRev / 60 / ... actually f_fire = rpm/60 * firingsPerRev.
    // 5 = V10 flavor. Chosen with the rpm range so the fundamental lands in
    // a small 3W speaker's usable band (~300 Hz+): at 3500..15000 rpm the
    // fundamental is ~290..1250 Hz, harmonics reach ~6 kHz.
    uint8_t firingsPerRev = 5;

    // Partial amplitudes (absolute int16 weights, summed then scaled by
    // volume). Weighted toward harmonics 2-4, not a fundamental-heavy 1/n
    // sawtooth, because the fundamental barely reproduces on the speaker.
    // Index 0 = fundamental. Sized to use most of the int16 headroom at full
    // volume; sum + noise + whine must stay under kHeadroomPeak (valid()).
    int16_t partialAmp[kMaxPartials] = {2200, 4400, 5600, 4000, 2600, 1400, 0, 0};

    int16_t noiseAmpMax = 1600; // throttle-correlated noise ceiling
    int16_t whineAmp = 2800;    // ERS whine partial amplitude
    // ERS whine pitch as a multiple of the firing frequency (crank-coupled
    // MGU-K feel), in eighths (24 = 3.0x).
    uint8_t whinePitchEighths = 24;

    // Attack/release for the whine gate, in samples (~ one control tick).
    uint16_t whineRampSamples = 512;

    // Rev-limiter ignition-cut cadence: gate volume off/on at this rate.
    uint16_t limiterCutHz = 18;

    // Master headroom: every partial + noise + whine is scaled so the peak
    // theoretical sum can't clip int16.
    static constexpr int16_t kHeadroomPeak = 30000;

    constexpr int32_t peakSum() const {
        int32_t s = 0;
        for (int i = 0; i < kMaxPartials; ++i) {
            s += partialAmp[i] < 0 ? -partialAmp[i] : partialAmp[i];
        }
        s += noiseAmpMax + whineAmp;
        return s;
    }

    constexpr bool valid() const {
        return firingsPerRev >= 1 && firingsPerRev <= 16 && whinePitchEighths > 0 &&
               whineRampSamples > 0 && limiterCutHz > 0 && noiseAmpMax >= 0 && whineAmp >= 0 &&
               peakSum() > 0 && peakSum() <= kHeadroomPeak;
    }
};

// Procedural engine synth. All state below is AUDIO-TASK-ONLY except the
// packed param word, which crosses cores via the caller's atomic. Pure
// integer DSP; deterministic given a seed, so render() is unit-testable.
class EngineSynth : public ISampleSource {
public:
    explicit EngineSynth(EngineSynthConfig config = EngineSynthConfig{}, uint32_t noiseSeed = 0x1234u);

    // Called from the control core (packed word read via applyPackedParams in
    // render()). Provided as a direct setter too for native tests.
    void setParams(uint16_t engineRpm, uint8_t volume, bool ersWhine, bool limiter, bool overrun);
    void applyPackedParams(uint32_t packed);

    size_t render(int16_t* out, size_t frameCount) override;

private:
    // Phase is a full uint32 accumulator (wraps naturally at 2^32 = one
    // cycle); the top 8 bits index a 256-entry signed sine table.
    int16_t sineLookup(uint32_t phase) const;
    uint16_t nextNoise();

    EngineSynthConfig config_;

    // Target params (set by control core) and smoothed params (audio core)
    // to avoid zipper/click on 50 Hz steps.
    uint32_t targetRpm_ = 0;
    uint16_t targetVolume_ = 0;
    bool ersWhine_ = false;
    bool limiter_ = false;
    bool overrun_ = false;

    int32_t smoothRpm_ = 0;     // fixed-point-ish, plain rpm
    int32_t smoothVolume_ = 0;  // 0..255

    uint32_t partialPhase_[kMaxPartials] = {};
    uint32_t whinePhase_ = 0;
    int32_t whineEnv_ = 0; // 0..whineRampSamples, ramps the whine in/out
    uint32_t limiterPhase_ = 0;

    uint32_t noiseState_;
    uint32_t sampleCounter_ = 0;
};

} // namespace soundsynth
