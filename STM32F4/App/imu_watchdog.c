#include "imu_watchdog.h"
#include "app_config.h"
#include <string.h>

static void want(imu_watchdog_t *w, imu_cause_t cause, uint32_t due_ms)
{
    if (w->need) return;
    w->need = true;
    w->cause = (uint8_t)cause;
    w->due_ms = due_ms;
    w->backoff_ms = IMU_REINIT_FIRST_MS;
}

void ImuWatchdog_Init(imu_watchdog_t *w, bool init_ok, uint32_t now_ms)
{
    memset(w, 0, sizeof(*w));
    w->last_check_ms = now_ms;
    if (!init_ok) want(w, IMU_CAUSE_INIT, now_ms + IMU_REINIT_FIRST_MS);   /* boot just tried */
}

void ImuWatchdog_OnRead(imu_watchdog_t *w, bool ok, uint32_t now_ms)
{
    if (ok) { w->fails = 0U; return; }
    if (w->fails < 0xFFFFU) w->fails++;
    if (w->fails >= IMU_REINIT_AFTER_FAILS) want(w, IMU_CAUSE_READS, now_ms);
}

bool ImuWatchdog_CheckDue(imu_watchdog_t *w, uint32_t now_ms)
{
    if (w->need || (uint32_t)(now_ms - w->last_check_ms) < IMU_MODE_CHECK_MS) return false;
    w->last_check_ms = now_ms;
    return true;
}

void ImuWatchdog_OnModeCheck(imu_watchdog_t *w, bool sensor_reset, uint32_t now_ms)
{
    if (sensor_reset) want(w, IMU_CAUSE_MODE, now_ms);
}

bool ImuWatchdog_ReinitDue(const imu_watchdog_t *w, uint32_t now_ms)
{
    return w->need && (int32_t)(now_ms - w->due_ms) >= 0;
}

void ImuWatchdog_OnReinit(imu_watchdog_t *w, bool ok, uint32_t now_ms)
{
    if (w->attempts < 0xFFFFU) w->attempts++;
    if (ok) {
        if (w->recovered < 0xFFFFU) w->recovered++;
        w->need = false;
        w->fails = 0U;
        w->last_check_ms = now_ms;
        return;
    }
    w->due_ms = now_ms + w->backoff_ms;
    w->backoff_ms = w->backoff_ms >= IMU_REINIT_MAX_MS / 2U ? IMU_REINIT_MAX_MS : w->backoff_ms * 2U;
}
