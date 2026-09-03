#!/usr/bin/env bash
#
# Delivery-image shape guard (review finding sl:safety-5, cluster "every
# shipped artefact's contents are asserted by prose, not CI").
#
# WHAT IT PROVES. The delivery firmware ([env:esp32dev]) must not contain the
# bench sim feeder. That feeder plays a scripted link2 drive whose frames say
# "armed", so a sim image left on the finished car looks and sounds alive
# whatever board #1 is doing. platformio.ini and SimLink2Feeder.hpp both say
# "the whole module vanishes from the real firmware"; until this script, that
# was a claim in a comment. Now it is an assertion over the linked ELF.
#
# WHY THE POSITIVE CONTROL MATTERS. "grep found nothing" is also what a wrong
# nm, a stripped binary, a renamed namespace or a missing build produces. So
# the same symbol MUST be found in the sim image ([env:esp32dev_sim]). If the
# control fails, the run is "could not check" (exit 2), never a pass -- the
# one failure mode a guard like this actually has.
#
# It NEVER builds, flashes, or writes anything; it only reads two ELFs.
#
# Usage:
#   tools/delivery_shape_check.sh [--delivery ELF] [--sim ELF] [--nm PATH] [-q]
#
#   --delivery ELF  the image that ships. Default .pio/build/esp32dev/firmware.elf
#   --sim ELF       the positive control.  Default .pio/build/esp32dev_sim/firmware.elf
#   --nm PATH       symbol lister to use. Default: xtensa-esp32-elf-nm from PATH,
#                   else the PlatformIO toolchain copy, else plain nm (GNU nm
#                   reads foreign-architecture ELFs fine).
#
# Exit codes (deliberately distinct: "the wrong thing shipped" and "the check
# did not run" are different failures and CI must tell them apart):
#   0  the delivery image is clean AND the control found the marker
#   1  FAIL: the delivery image carries the sim feeder
#   2  COULD NOT CHECK: missing ELF, no usable nm, or the control came up empty
#   3  usage error

set -u

# The marker: the sim feeder's only externally visible function. Matched as a
# substring so it survives C++ mangling on any toolchain.
MARKER='simfeeder'

DELIVERY_ELF='.pio/build/esp32dev/firmware.elf'
SIM_ELF='.pio/build/esp32dev_sim/firmware.elf'
NM=''
QUIET=0

say() { [ "$QUIET" -eq 1 ] || echo "$@"; }

while [ $# -gt 0 ]; do
    case "$1" in
        --delivery) shift
                    if [ $# -eq 0 ]; then echo "error: --delivery needs an ELF path" >&2; exit 3; fi
                    DELIVERY_ELF="$1"; shift ;;
        --sim)      shift
                    if [ $# -eq 0 ]; then echo "error: --sim needs an ELF path" >&2; exit 3; fi
                    SIM_ELF="$1"; shift ;;
        --nm)       shift
                    if [ $# -eq 0 ]; then echo "error: --nm needs a PATH" >&2; exit 3; fi
                    NM="$1"; shift ;;
        -q|--quiet) QUIET=1; shift ;;
        -h|--help)  sed -n '2,36p' "$0"; exit 0 ;;
        *)          echo "error: unknown argument '$1'" >&2; exit 3 ;;
    esac
done

# --- Locate a symbol lister -------------------------------------------------
if [ -z "$NM" ]; then
    if command -v xtensa-esp32-elf-nm >/dev/null 2>&1; then
        NM="$(command -v xtensa-esp32-elf-nm)"
    elif [ -x "$HOME/.platformio/packages/toolchain-xtensa-esp32/bin/xtensa-esp32-elf-nm" ]; then
        NM="$HOME/.platformio/packages/toolchain-xtensa-esp32/bin/xtensa-esp32-elf-nm"
    elif command -v nm >/dev/null 2>&1; then
        NM="$(command -v nm)"
    else
        echo "delivery shape check: COULD NOT CHECK -- no nm found (looked for" \
             "xtensa-esp32-elf-nm, the PlatformIO toolchain copy, and plain nm)." >&2
        exit 2
    fi
fi

for elf in "$DELIVERY_ELF" "$SIM_ELF"; do
    if [ ! -r "$elf" ]; then
        echo "delivery shape check: COULD NOT CHECK -- '$elf' is missing or unreadable." \
             "Build both environments first: pio run -e esp32dev && pio run -e esp32dev_sim" >&2
        exit 2
    fi
done

symbols_for() {
    # A binary nm cannot read is a could-not-check, not a pass; the caller
    # distinguishes empty output from a non-zero exit.
    "$NM" "$1" 2>/dev/null
}

# --- Positive control FIRST: an unusable checker must never look like a pass --
sim_syms="$(symbols_for "$SIM_ELF")"
if [ -z "$sim_syms" ]; then
    echo "delivery shape check: COULD NOT CHECK -- '$NM' listed no symbols at all in" \
         "'$SIM_ELF'. Wrong nm for this architecture, or a stripped binary." >&2
    exit 2
fi
if ! printf '%s\n' "$sim_syms" | grep -q "$MARKER"; then
    echo "delivery shape check: COULD NOT CHECK -- the positive control failed:" \
         "'$MARKER' is absent from the SIM image '$SIM_ELF' too. The marker was renamed," \
         "the feeder was dropped, or the sim build is stale. This run proves nothing" \
         "about the delivery image." >&2
    exit 2
fi
say "delivery shape check: positive control OK -- '$MARKER' present in $SIM_ELF."

# --- The assertion ----------------------------------------------------------
# Same could-not-check guard as the sim positive control above: an unreadable
# or zero-byte/truncated DELIVERY_ELF must not silently read as "no marker
# found" -> PASS. Capture the full symbol table first and require it be
# non-empty before searching it for the marker.
delivery_syms="$(symbols_for "$DELIVERY_ELF")"
if [ -z "$delivery_syms" ]; then
    echo "delivery shape check: COULD NOT CHECK -- '$NM' listed no symbols at all in" \
         "'$DELIVERY_ELF'. Wrong nm for this architecture, a stripped binary, or a" \
         "zero-byte/truncated build artifact." >&2
    exit 2
fi
delivery_hits="$(printf '%s\n' "$delivery_syms" | grep "$MARKER" || true)"
if [ -n "$delivery_hits" ]; then
    echo "::error title=SIM FEEDER IN THE DELIVERY IMAGE::'$MARKER' symbols are linked into" \
         "$DELIVERY_ELF. That build plays scripted 'armed' link2 frames with no board #1" \
         "attached and must never reach the car. Symbols:"
    printf '%s\n' "$delivery_hits"
    exit 1
fi

say "delivery shape check: PASS -- no '$MARKER' symbols in $DELIVERY_ELF."

# --- Secondary, advisory: the sim banner strings ----------------------------
# Cheap second axis in case the symbol ever gets inlined away. Advisory only:
# `strings` is not guaranteed to be installed, and its absence must not turn a
# real pass into a failure.
if command -v strings >/dev/null 2>&1; then
    if strings "$DELIVERY_ELF" | grep -q 'SIMULATION IMAGE'; then
        echo "::error title=SIM BANNER IN THE DELIVERY IMAGE::the sim boot banner string is" \
             "present in $DELIVERY_ELF."
        exit 1
    fi
    say "delivery shape check: no sim banner strings in $DELIVERY_ELF."
else
    say "delivery shape check: note -- 'strings' unavailable, skipped the banner check" \
        "(the symbol check above is the load-bearing one)."
fi

exit 0
