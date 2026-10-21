/* Host test of App/imu_watchdog.c: when the BNO055 is started again. */
#include "imu_watchdog.h"
#include "app_config.h"
#include <assert.h>
#include <stdio.h>

/* The control loop: a read every 10 ms for ms, all good or all failed. Returns the time a re-init
   fell due, 0 if none did. */
static uint32_t reads(imu_watchdog_t *w, bool ok, uint32_t *now, uint32_t ms)
{
    for (uint32_t t = 0U; t < ms; t += 10U) {
        ImuWatchdog_OnRead(w, ok, *now);
        if (ImuWatchdog_ReinitDue(w, *now)) return *now;
        *now += 10U;
    }
    return 0U;
}

int main(void)
{
    imu_watchdog_t w;
    uint32_t now = 1000U;

    /* A good sensor: no re-init, a mode check once a second. */
    ImuWatchdog_Init(&w, true, now);
    assert(reads(&w, true, &now, 5000U) == 0U);
    unsigned checks = 0U;
    for (uint32_t t = 0U; t < 5000U; t += 10U, now += 10U) checks += ImuWatchdog_CheckDue(&w, now) ? 1U : 0U;
    assert(checks == 5U);

    /* A single failed read, or a few, is not a lost sensor. */
    assert(reads(&w, false, &now, (IMU_REINIT_AFTER_FAILS - 1U) * 10U) == 0U);
    assert(reads(&w, true, &now, 10U) == 0U);
    assert(reads(&w, false, &now, (IMU_REINIT_AFTER_FAILS - 1U) * 10U) == 0U);
    assert(reads(&w, true, &now, 10U) == 0U);

    /* IMU_REINIT_AFTER_FAILS in a row: a re-init at once. */
    uint32_t start = now;
    uint32_t due = reads(&w, false, &now, 2000U);
    assert(due == start + (IMU_REINIT_AFTER_FAILS - 1U) * 10U && w.cause == IMU_CAUSE_READS);
    /* No mode check while a re-init is wanted. */
    assert(!ImuWatchdog_CheckDue(&w, now + 5000U));

    /* It fails: tried again after the back-off, doubling, capped. */
    uint32_t expected = IMU_REINIT_FIRST_MS;
    for (int i = 0; i < 8; i++) {
        ImuWatchdog_OnReinit(&w, false, now);
        assert(!ImuWatchdog_ReinitDue(&w, now + expected - 1U));
        assert(ImuWatchdog_ReinitDue(&w, now + expected));
        now += expected;
        expected = expected * 2U > IMU_REINIT_MAX_MS ? IMU_REINIT_MAX_MS : expected * 2U;
    }
    assert(expected == IMU_REINIT_MAX_MS);

    /* It succeeds: back to normal, and the next loss starts from the first back-off again. */
    ImuWatchdog_OnReinit(&w, true, now);
    assert(!ImuWatchdog_ReinitDue(&w, now) && w.recovered == 1U && w.attempts == 9U);
    assert(reads(&w, true, &now, 2000U) == 0U);
    reads(&w, false, &now, 2000U);
    ImuWatchdog_OnReinit(&w, false, now);
    assert(ImuWatchdog_ReinitDue(&w, now + IMU_REINIT_FIRST_MS));
    ImuWatchdog_OnReinit(&w, true, now + IMU_REINIT_FIRST_MS);
    now += IMU_REINIT_FIRST_MS;

    /* Reads all good but the sensor reset itself (CONFIGMODE, zeros): the mode check catches it. */
    assert(reads(&w, true, &now, IMU_MODE_CHECK_MS) == 0U);
    while (!ImuWatchdog_CheckDue(&w, now)) now += 10U;
    ImuWatchdog_OnModeCheck(&w, false, now);
    assert(!ImuWatchdog_ReinitDue(&w, now));
    now += IMU_MODE_CHECK_MS;
    assert(ImuWatchdog_CheckDue(&w, now));
    ImuWatchdog_OnModeCheck(&w, true, now);
    assert(ImuWatchdog_ReinitDue(&w, now) && w.cause == IMU_CAUSE_MODE);
    ImuWatchdog_OnReinit(&w, true, now);

    /* An init that failed at boot: tried again after the first back-off, not at once. */
    now = 500U;
    ImuWatchdog_Init(&w, false, now);
    assert(!ImuWatchdog_ReinitDue(&w, now) && w.cause == IMU_CAUSE_INIT);
    assert(ImuWatchdog_ReinitDue(&w, now + IMU_REINIT_FIRST_MS));

    /* Across the tick counter's wrap. */
    now = 0xFFFFFFFFU - 500U;
    ImuWatchdog_Init(&w, false, now);
    assert(!ImuWatchdog_ReinitDue(&w, now + IMU_REINIT_FIRST_MS - 1U));
    assert(ImuWatchdog_ReinitDue(&w, now + IMU_REINIT_FIRST_MS));

    printf("imu_watchdog_test: PASS\n");
    return 0;
}
