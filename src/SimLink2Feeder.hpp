#pragma once

// Wokwi/bench-only link2 feeder (validation Stage 2): plays a scripted drive
// through link2::encodeFrame, so the board runs sound + lights standalone
// with no board #1 attached. Exercises the real assembler + monitor +
// staleness path (including a scripted dropout). Compiled ONLY in
// [env:esp32dev_sim]; the whole module vanishes from the real firmware.

#ifdef W17_SIM_LINK2_FEEDER

#include <cstddef>
#include <cstdint>

namespace simfeeder {

// Call every loop pass. When it is time to emit a frame (~20Hz, and not
// during the scripted dropout), fills `out` (>= link2::kFrameLen bytes) with
// one encoded frame and returns its length; otherwise returns 0. Prints
// phase transitions on Serial0.
size_t tick(uint32_t nowMs, uint8_t* out);

} // namespace simfeeder

#endif // W17_SIM_LINK2_FEEDER
