#pragma once

#include <cstddef>
#include <cstdint>

namespace soundsynth {

// The audio-render seam. EngineSynth is the procedural implementation; a PCM
// sample player can drop in behind this exact interface later if the synth
// disappoints on the real speaker (the user's "hybrid: synth now, samples
// later" choice). render() runs on the core-0 audio task and must be
// allocation- and lock-free.
class ISampleSource {
public:
    virtual ~ISampleSource() = default;

    // Fill `out` with `frameCount` interleaved stereo int16 frames (2 samples
    // each: L then R). Returns frames written (== frameCount). The engine
    // sound is mono; L and R carry the same value (see the protocol/HAL note
    // on why we transmit duplicated stereo).
    virtual size_t render(int16_t* out, size_t frameCount) = 0;
};

} // namespace soundsynth
