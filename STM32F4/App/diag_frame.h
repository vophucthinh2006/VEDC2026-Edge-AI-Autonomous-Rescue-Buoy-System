#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* The boat's diagnostics for the dashboard: why it stands or drives, what the Pi is doing, the pack's
 * voltage and current, how often the I2C bus and the BNO055 were recovered. The beacon is full (120
 * bytes), so they go in a frame of their own every LORA_DIAG_PERIOD_MS, which the station does not
 * answer (App/lora_beacon.c):
 *
 *     dg=PHAO-01,n=obstacle_too_close,s=P,w=2/5,a=0,d=0,u=1184,j=-35,k=0:0
 *
 *   n  the reason of the Pi's last NAV command (Rasp_Pi/modules/navigation.py, safety.py), "-" unknown,
 *      pi_silent when no $HBT came for DIAG_PI_SILENT_MS (the Pi off, main.py stopped, the UART lost)
 *   s  the Pi's mission: I idle, P patrol, F follow a person, R home (shore lost), B home (battery
 *      low), H holding at home, - unknown
 *   w  waypoint the Pi is going to / waypoints in the route, 0/0 without a route
 *   a  arm_block (App/control.h): 0 armed, 1 no fresh IMU, 2 overturned
 *   d  drive_block (App/control.h): 0 driving, 1 disarmed, 2 the Pi gives no NAV AUTO, 3 IMU stale,
 *      4 STOP from the dashboard
 *   u  pack voltage x100, j current x100 (A), both left out while the INA226 is not found
 *   k  I2C bus recoveries : BNO055 re-inits since boot
 *   f  the Pi's detector frames a second x10 (its $HBT), left out while the detector is off or the Pi silent
 *
 * Plain C, no HAL: host test in Tests/. */

#define DIAG_REASON_MAX  24U

typedef struct {
    char reason[DIAG_REASON_MAX];   /* the Pi's NAV reason, [a-z_] only; "" unknown */
    char mission;                   /* I P F R B H, 0 unknown */
    uint16_t wp;                    /* waypoint being gone to, 1-based, 0 none */
    uint16_t wp_total;
    uint8_t arm_block;
    uint8_t drive_block;
    bool has_battery;
    float volts;
    float amps;
    uint16_t i2c_recoveries;
    uint16_t imu_reinits;
    bool has_fps;                   /* false: field f left out */
    uint16_t fps_x10;               /* the Pi's detector frames a second x10 */
} diag_t;

#define DIAG_PI_SILENT_MS  2000U   /* $HBT comes every 100 ms */

/* Copies text into reason, keeping [a-z0-9_] and cutting at DIAG_REASON_MAX - 1 characters. */
void Diag_SetReason(diag_t *diag, const char *text);

/* What the Pi said in its last $HBT: its NAV reason, mission and waypoint. With the Pi silent
 * (heard false) none of it holds any more: reason pi_silent, mission and waypoint unknown. */
void Diag_SetPi(diag_t *diag, bool heard, const char *reason, char mission, uint16_t wp, uint16_t wp_total);

/* The frame, 0 when it does not fit in size (then buf holds ""). */
size_t DiagFrame_Build(char *buf, size_t size, const char *buoy_id, const diag_t *diag);
