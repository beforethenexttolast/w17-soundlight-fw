#include <unity.h>

#include <cstdint>
#include <cstring>

// Tests for the production audio-startup sequencing in lib/audiostartup: the
// install -> pins -> DMA clear -> task order, best-effort uninstall on any
// post-install failure, and the distinct result per failed step (SLR-3). These
// drive the SAME audiostartup::startAudio() the firmware runs, through a fake
// Ops that records call counts and order -- no ESP-IDF, no hardware. The fake
// only *reports* success/failure; it never reimplements the sequencing.

#include "audiostartup/AudioStartup.hpp"

using audiostartup::AudioStartupResult;
using audiostartup::startAudio;

namespace {

// Fake operations adapter. Each op returns its configured success flag and logs
// its letter to `order` so tests can assert both counts and sequence. A fresh
// instance per test => no shared state, no ordering dependency.
struct FakeOps {
    // Configure which step fails (all succeed by default).
    bool installOk = true;
    bool pinsOk = true;
    bool dmaOk = true;
    bool taskOk = true;
    // Model a cleanup that itself hit an error. uninstallDriver() is void by
    // contract, so this cannot reach startAudio's result -- the flag exists
    // only to prove that.
    bool uninstallErrored = false;

    int installCalls = 0;
    int pinCalls = 0;
    int dmaCalls = 0;
    int taskCalls = 0;
    int uninstallCalls = 0;

    char order[16] = {};
    int orderLen = 0;
    void note(char c) {
        if (orderLen < 15) order[orderLen++] = c;
    }

    bool installDriver() {
        ++installCalls;
        note('I');
        return installOk;
    }
    bool configurePins() {
        ++pinCalls;
        note('P');
        return pinsOk;
    }
    bool clearDma() {
        ++dmaCalls;
        note('D');
        return dmaOk;
    }
    bool createTask() {
        ++taskCalls;
        note('T');
        return taskOk;
    }
    void uninstallDriver() {
        ++uninstallCalls;
        note('U');
        // Even when cleanup "errors", nothing is returned and nothing changes.
        (void)uninstallErrored;
    }
};

} // namespace

void setUp() {}
void tearDown() {}

// ---- Success path ---------------------------------------------------------

// All four steps succeed exactly once, in order, and no cleanup runs.
void test_success_path_calls_each_once_in_order_no_cleanup() {
    FakeOps ops;
    const AudioStartupResult r = startAudio(ops);

    TEST_ASSERT_EQUAL(AudioStartupResult::Ready, r);
    TEST_ASSERT_EQUAL_INT(1, ops.installCalls);
    TEST_ASSERT_EQUAL_INT(1, ops.pinCalls);
    TEST_ASSERT_EQUAL_INT(1, ops.dmaCalls);
    TEST_ASSERT_EQUAL_INT(1, ops.taskCalls);
    TEST_ASSERT_EQUAL_INT(0, ops.uninstallCalls);
    TEST_ASSERT_EQUAL_STRING("IPDT", ops.order);
}

// ---- Driver-install failure ----------------------------------------------

// Install fails: no later step runs, and nothing is uninstalled (a driver that
// never installed must not be torn down).
void test_driver_install_failure_stops_and_skips_cleanup() {
    FakeOps ops;
    ops.installOk = false;
    const AudioStartupResult r = startAudio(ops);

    TEST_ASSERT_EQUAL(AudioStartupResult::DriverInstallFailed, r);
    TEST_ASSERT_EQUAL_INT(1, ops.installCalls);
    TEST_ASSERT_EQUAL_INT(0, ops.pinCalls);
    TEST_ASSERT_EQUAL_INT(0, ops.dmaCalls);
    TEST_ASSERT_EQUAL_INT(0, ops.taskCalls);
    TEST_ASSERT_EQUAL_INT(0, ops.uninstallCalls);
    TEST_ASSERT_EQUAL_STRING("I", ops.order);
}

// ---- Pin-configuration failure -------------------------------------------

// Install succeeds, pins fail: DMA clear and task creation are skipped, and the
// installed driver is uninstalled exactly once.
void test_pin_failure_cleans_up_once_and_stops() {
    FakeOps ops;
    ops.pinsOk = false;
    const AudioStartupResult r = startAudio(ops);

    TEST_ASSERT_EQUAL(AudioStartupResult::PinConfigurationFailed, r);
    TEST_ASSERT_EQUAL_INT(1, ops.installCalls);
    TEST_ASSERT_EQUAL_INT(1, ops.pinCalls);
    TEST_ASSERT_EQUAL_INT(0, ops.dmaCalls);
    TEST_ASSERT_EQUAL_INT(0, ops.taskCalls);
    TEST_ASSERT_EQUAL_INT(1, ops.uninstallCalls);
    TEST_ASSERT_EQUAL_STRING("IPU", ops.order);
}

// ---- DMA-clear failure ----------------------------------------------------

// Install + pins succeed, DMA clear fails: task creation is skipped and the
// driver is uninstalled exactly once.
void test_dma_clear_failure_cleans_up_once_and_stops() {
    FakeOps ops;
    ops.dmaOk = false;
    const AudioStartupResult r = startAudio(ops);

    TEST_ASSERT_EQUAL(AudioStartupResult::DmaClearFailed, r);
    TEST_ASSERT_EQUAL_INT(1, ops.installCalls);
    TEST_ASSERT_EQUAL_INT(1, ops.pinCalls);
    TEST_ASSERT_EQUAL_INT(1, ops.dmaCalls);
    TEST_ASSERT_EQUAL_INT(0, ops.taskCalls);
    TEST_ASSERT_EQUAL_INT(1, ops.uninstallCalls);
    TEST_ASSERT_EQUAL_STRING("IPDU", ops.order);
}

// ---- Task-creation failure -----------------------------------------------

// First three steps succeed, task creation fails: the driver is uninstalled
// exactly once (no task exists to own it).
void test_task_creation_failure_cleans_up_once() {
    FakeOps ops;
    ops.taskOk = false;
    const AudioStartupResult r = startAudio(ops);

    TEST_ASSERT_EQUAL(AudioStartupResult::TaskCreationFailed, r);
    TEST_ASSERT_EQUAL_INT(1, ops.installCalls);
    TEST_ASSERT_EQUAL_INT(1, ops.pinCalls);
    TEST_ASSERT_EQUAL_INT(1, ops.dmaCalls);
    TEST_ASSERT_EQUAL_INT(1, ops.taskCalls);
    TEST_ASSERT_EQUAL_INT(1, ops.uninstallCalls);
    TEST_ASSERT_EQUAL_STRING("IPDTU", ops.order);
}

// ---- Cleanup-failure independence ----------------------------------------

// A cleanup that itself errors does not change the primary (pin) failure result
// and does not trigger a second uninstall.
void test_cleanup_failure_does_not_change_primary_result() {
    FakeOps ops;
    ops.dmaOk = false;
    ops.uninstallErrored = true;
    const AudioStartupResult r = startAudio(ops);

    TEST_ASSERT_EQUAL(AudioStartupResult::DmaClearFailed, r);
    TEST_ASSERT_EQUAL_INT(1, ops.uninstallCalls);
    TEST_ASSERT_EQUAL_STRING("IPDU", ops.order);
}

// ---- Only the first mandatory failure is acted on -------------------------

// If an earlier step fails, later steps never run even if they would also have
// failed -- the reported cause is the first failure, and cleanup happens once.
void test_first_failure_wins_no_calls_after_it() {
    FakeOps ops;
    ops.pinsOk = false;
    ops.dmaOk = false;  // would also fail, but must never be reached
    ops.taskOk = false; // ditto
    const AudioStartupResult r = startAudio(ops);

    TEST_ASSERT_EQUAL(AudioStartupResult::PinConfigurationFailed, r);
    TEST_ASSERT_EQUAL_INT(0, ops.dmaCalls);
    TEST_ASSERT_EQUAL_INT(0, ops.taskCalls);
    TEST_ASSERT_EQUAL_INT(1, ops.uninstallCalls);
    TEST_ASSERT_EQUAL_STRING("IPU", ops.order);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_success_path_calls_each_once_in_order_no_cleanup);
    RUN_TEST(test_driver_install_failure_stops_and_skips_cleanup);
    RUN_TEST(test_pin_failure_cleans_up_once_and_stops);
    RUN_TEST(test_dma_clear_failure_cleans_up_once_and_stops);
    RUN_TEST(test_task_creation_failure_cleans_up_once);
    RUN_TEST(test_cleanup_failure_does_not_change_primary_result);
    RUN_TEST(test_first_failure_wins_no_calls_after_it);
    return UNITY_END();
}
