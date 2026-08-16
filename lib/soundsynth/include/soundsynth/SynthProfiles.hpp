#pragma once

#include "link2/Link2Frame.hpp" // soundProfile wire values
#include "soundsynth/EngineSynth.hpp"

// Named engine-voice profiles (vision decision 15: V10 default, selectable
// profiles desired). The selection mechanism is DECIDED (owner, 2026-08-16):
// the link2 v2 `soundProfile` payload byte, persisted only on board #1
// (tuning-console key sound.profile) and consumed here at runtime --
// normalizeSoundProfile on the control core, two bits of the packed synth
// word across cores, EngineSynth::setVoiceProfile on the audio task. This
// board deliberately still has no NVS and no build flag for voices; the
// wire byte is the ONLY selector, and voiceForProfile() below is its
// mapping. kDefault stays the V10: the boot voice until the first frame,
// and the meaning of wire value 0.
namespace soundsynth {
namespace profiles {

// Field-for-field voice equality (partial stack included; every voice
// parameter in EngineSynthConfig). constexpr so "the shipping default IS the
// V10" is a compile-time fact, not a convention.
constexpr bool sameVoice(const EngineSynthConfig& a, const EngineSynthConfig& b) {
    for (int i = 0; i < kMaxPartials; ++i) {
        if (a.partialAmp[i] != b.partialAmp[i]) {
            return false;
        }
    }
    return a.firingsPerRev == b.firingsPerRev && a.noiseAmpMax == b.noiseAmpMax &&
           a.whineAmp == b.whineAmp && a.whinePitchEighths == b.whinePitchEighths &&
           a.whineRampSamples == b.whineRampSamples && a.limiterCutHz == b.limiterCutHz;
}

// V10 (the shipping voice since day one): exactly the EngineSynthConfig
// defaults -- 5 firings/rev, harmonic-weighted partial stack, moderate
// crank-coupled whine. Named here so "V10" is a profile like any other
// rather than an anonymous set of defaults.
constexpr EngineSynthConfig v10() { return EngineSynthConfig{}; }

// V6 turbo-hybrid (2014+ power-unit flavor). Same rpm domain as the V10 (the
// rpm range lives in EngineSimConfig, not here); the voice differs:
//   - firingsPerRev 3: a V6 four-stroke fires 3 times per crank rev, so the
//     firing fundamental drops to 3/5 of the V10's at any rpm (idle 175 Hz,
//     redline 750 Hz) -- the flatter, deeper V6 drone.
//   - Partial stack shifted further up-harmonic: the lower fundamental sits
//     below the small 3 W speaker's usable band (~300 Hz), so the fundamental
//     is de-emphasized and harmonics 2-6 (350 Hz..4.5 kHz at idle..redline)
//     carry the note; a 7th partial adds the turbo-era rasp.
//   - Whine louder (4200 vs 2800) and higher-pitched relative to the firing
//     frequency (5.0x vs 3.0x): the ever-present MGU-K/turbo whine IS the
//     hybrid era's signature; 5.0x keeps it ~0.9-3.8 kHz across the band.
//   - Noise ceiling slightly up (1800 vs 1600): induction/wastegate breath.
//   - Whine ramp and limiter-cut cadence unchanged: those are chassis feel,
//     not voice.
// Headroom: partials sum 20000 + noise 1800 + whine 4200 = 26000 <= 30000.
constexpr EngineSynthConfig v6TurboHybrid() {
    EngineSynthConfig c;
    c.firingsPerRev = 3;
    c.partialAmp[0] = 1400;
    c.partialAmp[1] = 4600;
    c.partialAmp[2] = 5200;
    c.partialAmp[3] = 3800;
    c.partialAmp[4] = 2600;
    c.partialAmp[5] = 1600;
    c.partialAmp[6] = 800;
    c.partialAmp[7] = 0;
    c.noiseAmpMax = 1800;
    c.whineAmp = 4200;
    c.whinePitchEighths = 40; // 5.0x firing frequency
    return c;
}

// The compile-time default: the BOOT voice (what src/main.cpp constructs the
// synth with, heard until the first link2 frame selects a profile) and the
// voice wire value 0 means. Stays the V10 by owner decision 15.
inline constexpr EngineSynthConfig kDefault = v10();

// The wire mapping: link2 v2 `soundProfile` byte -> named voice. Total over
// uint8_t, per the protocol's fallback rule (docs/link2_protocol.md payload
// row 11): every reserved value >= link2::kSoundProfileCount selects the V10
// -- a voice fallback, never a rejection, so a build that predates a future
// voice keeps making sound. Callers normally pass an already-normalized
// value (audiodecision::normalizeSoundProfile); totality here is defense in
// depth, not the primary gate.
constexpr EngineSynthConfig voiceForProfile(uint8_t wireProfile) {
    return wireProfile == link2::kSoundProfileV6Hybrid ? v6TurboHybrid() : v10();
}

// Definition-site guarantees (house rule: config valid() + static_assert):
// both named voices are valid, genuinely distinct, the default is pinned to
// the historical V10 parameter set, and the wire mapping honors both the
// defined values and the reserved-value fallback.
static_assert(v10().valid(), "V10 profile invalid");
static_assert(v6TurboHybrid().valid(), "V6 turbo-hybrid profile invalid");
static_assert(v6TurboHybrid().firingsPerRev == 3, "V6 fires 3 times per rev");
static_assert(sameVoice(v10(), EngineSynthConfig{}),
              "the V10 profile must stay byte-for-byte the historical default voice");
static_assert(!sameVoice(v10(), v6TurboHybrid()), "profiles must actually differ");
static_assert(sameVoice(kDefault, v10()),
              "the boot/default voice stays the V10 (owner decision 15; wire value 0)");
static_assert(sameVoice(voiceForProfile(link2::kSoundProfileV10), v10()),
              "wire 0 must select the V10");
static_assert(sameVoice(voiceForProfile(link2::kSoundProfileV6Hybrid), v6TurboHybrid()),
              "wire 1 must select the V6 turbo-hybrid");
static_assert(sameVoice(voiceForProfile(link2::kSoundProfileCount), v10()),
              "the first reserved wire value must fall back to the V10");
static_assert(sameVoice(voiceForProfile(0xFF), v10()),
              "every reserved wire value must fall back to the V10");

} // namespace profiles
} // namespace soundsynth
