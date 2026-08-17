#include "enginesim/ShowScript.hpp"

namespace enginesim {

namespace {

// One xorshift32 step (Marsaglia's 13/17/5 triple -- the same generator
// family the synth's noise LFSR uses; deterministic, integer-only, period
// 2^32-1 over nonzero states).
constexpr uint32_t xorshift32(uint32_t x) {
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return x;
}

} // namespace

uint8_t showScriptThrottleAt(uint32_t nowMs) {
    const uint32_t slot = nowMs / kShowSlotMs;
    const uint32_t tInSlot = nowMs % kShowSlotMs;

    // Per-slot stream: seed mixed with the slot index via the golden-ratio
    // constant so adjacent slots decorrelate, then guarded nonzero (xorshift
    // fixes 0) and whitened one step before use.
    uint32_t r = kShowScriptSeed ^ (slot * 0x9E3779B9u);
    if (r == 0) {
        r = kShowScriptSeed;
    }
    r = xorshift32(r);

    // Occasional, not constant: most slots stay pure idle (the engine's own
    // wobble is the soundtrack there -- D8 "mostly 0").
    if ((r % 8u) >= kShowBlipSlotsIn8) {
        return 0;
    }

    r = xorshift32(r);
    const uint32_t peak =
        kShowMinBlipPct + (r % static_cast<uint32_t>(kShowMaxBlipPct - kShowMinBlipPct + 1));
    r = xorshift32(r);
    const uint32_t durationMs = kShowBlipMinMs + (r % (kShowBlipMaxMs - kShowBlipMinMs + 1u));
    r = xorshift32(r);
    // Blip start anywhere that keeps a full margin of silence on both slot
    // edges (>= kShowBlipMarginMs each side; static_assert guarantees room).
    const uint32_t latestStart = kShowSlotMs - durationMs - kShowBlipMarginMs;
    const uint32_t startMs = kShowBlipMarginMs + (r % (latestStart - kShowBlipMarginMs + 1u));

    if (tInSlot < startMs || tInSlot >= startMs + durationMs) {
        return 0;
    }

    // Integer triangle 0 -> peak -> 0 across the blip: gentle up, gentle
    // down. Worst per-ms slope is peak / (duration/2) <= 30/200 -- well
    // under 1 point/ms, so even tick-quantized sampling can only ever see
    // single-digit per-tick steps (the overrun's 40-point cliff is
    // unreachable twice over; the static_asserts carry the formal claim).
    const uint32_t x = tInSlot - startMs;
    const uint32_t half = durationMs / 2u;
    const uint32_t rise = (x <= half) ? x : (durationMs - x);
    return static_cast<uint8_t>(peak * rise / half);
}

} // namespace enginesim
