#pragma once

// Compile-time gate for application-owned UART0 (Serial) diagnostics
// (historical finding SL-3). This is a build-feature knob only -- no runtime
// setting, no NVS, no strap, no command. Both firmware TUs (src/main.cpp and
// src/SimLink2Feeder.cpp) share this gate, hence a small shared header.
//
//   0 => diagnostics compiled OUT (the delivery firmware: no Serial.begin, no
//        application Serial.print*, no diagnostic strings in the binary).
//   1 => diagnostics retained (simulation / tuning / diagnostic / dev builds
//        where UART0 output is useful).
//
// This gate covers APPLICATION UART0 only. It does not touch UART2 / Serial2
// (link2 input), framework/ROM/bootloader output, or flashing behavior.
//
// A defensive default of 0 keeps an unconfigured build quiet, but every real
// firmware environment defines the value explicitly (see platformio.ini).

#ifndef W17_UART0_DIAGNOSTICS
#define W17_UART0_DIAGNOSTICS 0
#endif

#if W17_UART0_DIAGNOSTICS != 0 && W17_UART0_DIAGNOSTICS != 1
#error "W17_UART0_DIAGNOSTICS must be 0 or 1"
#endif

// Application UART0 diagnostic print. When disabled it expands to a no-op:
// arguments are NOT evaluated and no string literals or Serial reference remain
// at the call site. This wrapper carries no control flow -- callers keep their
// own failure handling (e.g. vTaskDelete) as separate statements.
#if W17_UART0_DIAGNOSTICS
#define W17_UART0_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define W17_UART0_PRINTF(...) \
    do {                      \
    } while (0)
#endif
