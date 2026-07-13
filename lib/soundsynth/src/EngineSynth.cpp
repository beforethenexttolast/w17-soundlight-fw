#include "soundsynth/EngineSynth.hpp"

namespace soundsynth {

namespace {

// 256-entry signed sine table, amplitude +/-256, built once at startup. A
// small table + phase-accumulator is the standard integer wavetable trick;
// 256 entries is plenty at these frequencies (interpolation unnecessary).
struct SineTable {
    int16_t v[256];
    SineTable() {
        // Float is allowed here: one-time table setup, not the render path.
        for (int i = 0; i < 256; ++i) {
            const double a = (2.0 * 3.14159265358979323846 * i) / 256.0;
            double s = 0.0;
            // Cheap sine via a few terms is overkill; use the standard lib at
            // init only. (Kept dependency-free-ish; <cmath> is host-safe.)
            // Taylor-free: use the identity through successive doubling is
            // messy -- just call sin via a small polynomial approximation.
            // Simplicity: 5th-order minimax-ish over the phase.
            // For init cost we just do a plain series good enough for a table.
            double x = a - 3.14159265358979323846; // center at 0 for the series
            double x2 = x * x;
            // -sin(x+pi) = sin(x); approximate sin(x) on [-pi,pi]
            s = x * (1.0 - x2 / 6.0 * (1.0 - x2 / 20.0 * (1.0 - x2 / 42.0)));
            s = -s;
            int val = static_cast<int>(s * 256.0);
            if (val > 256) val = 256;
            if (val < -256) val = -256;
            v[i] = static_cast<int16_t>(val);
        }
    }
};

const SineTable kSine;

// Phase increment per sample for a given frequency (in milli-Hz to keep
// integer precision at low rpm). inc = freqMilliHz * 2^32 / (1000 * fs).
uint32_t phaseIncForMilliHz(uint32_t freqMilliHz) {
    // 2^32 / (1000 * 22050) computed as a scaled constant to avoid 64-bit
    // divide per call: (2^32) / 22050000 ~= 194.98. Use 64-bit mul then div.
    return static_cast<uint32_t>((static_cast<uint64_t>(freqMilliHz) * 4294967296ull) /
                                 (1000ull * kSampleRateHz));
}

} // namespace

EngineSynth::EngineSynth(EngineSynthConfig config, uint32_t noiseSeed)
    : config_(config), noiseState_(noiseSeed ? noiseSeed : 1u) {}

void EngineSynth::setParams(uint16_t engineRpm, uint8_t volume, bool ersWhine, bool limiter,
                            bool overrun) {
    targetRpm_ = engineRpm;
    targetVolume_ = volume;
    ersWhine_ = ersWhine;
    limiter_ = limiter;
    overrun_ = overrun;
}

void EngineSynth::applyPackedParams(uint32_t p) {
    setParams(static_cast<uint16_t>(p & 0xFFFF), static_cast<uint8_t>((p >> 16) & 0xFF),
              (p >> 24) & 1u, (p >> 25) & 1u, (p >> 26) & 1u);
}

int16_t EngineSynth::sineLookup(uint32_t phase) const { return kSine.v[phase >> 24]; }

uint16_t EngineSynth::nextNoise() {
    // xorshift32: deterministic given the seed, so render() is reproducible.
    uint32_t x = noiseState_;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    noiseState_ = x;
    return static_cast<uint16_t>(x & 0xFFFF);
}

size_t EngineSynth::render(int16_t* out, size_t frameCount) {
    for (size_t f = 0; f < frameCount; ++f) {
        // --- Per-sample param smoothing (kills zipper on 50 Hz steps). ---
        // Move ~1/1024 of the gap each sample: ~23 ms time constant.
        smoothRpm_ += (static_cast<int32_t>(targetRpm_) - smoothRpm_) >> 6;
        smoothVolume_ += (static_cast<int32_t>(targetVolume_) - smoothVolume_) >> 6;

        const uint32_t rpm = smoothRpm_ < 0 ? 0 : static_cast<uint32_t>(smoothRpm_);
        const int32_t vol = smoothVolume_ < 0 ? 0 : smoothVolume_;

        // Firing fundamental in milli-Hz: rpm/60 * firingsPerRev, *1000.
        // = rpm * firingsPerRev * 1000 / 60.
        const uint32_t fundMilliHz =
            static_cast<uint32_t>(rpm) * config_.firingsPerRev * 1000u / 60u;

        int32_t sample = 0;

        // --- Harmonic partial stack. ---
        for (int p = 0; p < kMaxPartials; ++p) {
            if (config_.partialAmp[p] == 0) {
                continue;
            }
            const uint32_t inc = phaseIncForMilliHz(fundMilliHz * (p + 1));
            partialPhase_[p] += inc;
            // sine table is +/-256; amp is the relative weight.
            sample += (sineLookup(partialPhase_[p]) * config_.partialAmp[p]) >> 8;
        }

        // --- Throttle-correlated noise (louder with rpm). During the
        // overrun window, randomly gated louder bursts give the lift-off
        // crackle/pop. ---
        int32_t noiseAmp = config_.noiseAmpMax * static_cast<int32_t>(rpm) / 15000;
        if (overrun_ && (nextNoise() & 0x3) == 0) {
            noiseAmp = config_.noiseAmpMax * 3; // crackle burst
        }
        // Widen to int64 before the multiply: for valid extreme configs
        // (e.g. all-noise, noiseAmpMax up to kHeadroomPeak) the rpm- or
        // overrun-scaled noiseAmp can push (int16-centered sample) * noiseAmp
        // past int32 and signed-overflow. The arithmetic >> 15 and the int32
        // result are unchanged -- the shifted magnitude always fits int32.
        const int32_t noise = static_cast<int32_t>(
            (static_cast<int64_t>(static_cast<int32_t>(nextNoise()) - 32768) * noiseAmp) >> 15);
        sample += noise;

        // --- ERS whine: pitch tracks the firing freq, gated with a ramp. ---
        const int32_t whineTarget = ersWhine_ ? config_.whineRampSamples : 0;
        if (whineEnv_ < whineTarget) {
            whineEnv_++;
        } else if (whineEnv_ > whineTarget) {
            whineEnv_--;
        }
        if (whineEnv_ > 0) {
            const uint32_t whineMilliHz = fundMilliHz * config_.whinePitchEighths / 8u;
            whinePhase_ += phaseIncForMilliHz(whineMilliHz);
            const int32_t w = (sineLookup(whinePhase_) * config_.whineAmp) >> 8;
            sample += w * whineEnv_ / config_.whineRampSamples;
        }

        // --- Master volume (0..255). ---
        sample = sample * vol / 255;

        // --- Rev-limiter ignition cut: gate to zero at limiterCutHz. ---
        if (limiter_) {
            limiterPhase_ += phaseIncForMilliHz(config_.limiterCutHz * 1000u);
            if ((limiterPhase_ >> 31) != 0) { // upper half of the cycle: cut
                sample = 0;
            }
        }

        // Single authoritative saturating narrow to int16 (see clampToInt16).
        const int16_t s16 = clampToInt16(sample);
        out[2 * f] = s16;     // L
        out[2 * f + 1] = s16; // R (duplicated: mono engine, stereo transport)
        sampleCounter_++;
    }
    return frameCount;
}

} // namespace soundsynth
