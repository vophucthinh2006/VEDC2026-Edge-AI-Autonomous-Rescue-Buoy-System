#pragma once

#include <stdbool.h>
#include <stdint.h>

/* When the main loop starts the BNO055 again (IMU_Init). It used to be started once, at boot: an
 * init that failed there, or a sensor lost later, left the boat disarmed until the next reset.
 *
 *   the init failed (at boot or on a retry)          again after a back-off
 *   IMU_REINIT_AFTER_FAILS reads in a row failed      at once (the sensor or the bus went away)
 *   the mode check found it out of its fusion mode    at once: it reset itself (a brown-out when the
 *                                                     motors pull) and reads zeros in CONFIGMODE,
 *                                                     which look like good data
 *
 * A re-init blocks for ~0.7 to 2 s, so failed attempts back off from IMU_REINIT_FIRST_MS, doubling,
 * up to IMU_REINIT_MAX_MS. The mode is checked every IMU_MODE_CHECK_MS while the sensor is up.
 * Pure C, no HAL: host test in Tests/. Read live over ST-LINK as imu_watchdog. */

typedef enum {
    IMU_CAUSE_NONE = 0,
    IMU_CAUSE_INIT = 1,        /* the init failed */
    IMU_CAUSE_READS = 2,       /* reads failed in a row */
    IMU_CAUSE_MODE = 3         /* out of its fusion mode: the sensor reset itself */
} imu_cause_t;

typedef struct {
    uint16_t fails;            /* failed reads in a row */
    bool need;                 /* a re-init is wanted */
    uint32_t due_ms;           /* not before this */
    uint32_t backoff_ms;       /* wait after the next failed attempt */
    uint32_t last_check_ms;    /* last mode check */
    uint16_t attempts;         /* re-inits since boot */
    uint16_t recovered;        /* of which succeeded */
    uint8_t cause;             /* imu_cause_t of the last need */
} imu_watchdog_t;

void ImuWatchdog_Init(imu_watchdog_t *w, bool init_ok, uint32_t now_ms);
/* Every control tick: whether the IMU read succeeded. */
void ImuWatchdog_OnRead(imu_watchdog_t *w, bool ok, uint32_t now_ms);
/* True when a mode check is due now (and counts it as done). */
bool ImuWatchdog_CheckDue(imu_watchdog_t *w, uint32_t now_ms);
/* The result of that check: the sensor answered out of its fusion mode. */
void ImuWatchdog_OnModeCheck(imu_watchdog_t *w, bool sensor_reset, uint32_t now_ms);
bool ImuWatchdog_ReinitDue(const imu_watchdog_t *w, uint32_t now_ms);
void ImuWatchdog_OnReinit(imu_watchdog_t *w, bool ok, uint32_t now_ms);
