#include "battery.h"
#include "app_config.h"
#include "ina226.h"
#include <string.h>

volatile battery_debug_t battery_debug;

#define FILTER_TAU_S     5.0f          /* the charge follows the pack, not each throttle step */
#define RETRY_INIT_MS    5000U         /* a module plugged in late is found again */

#if INA226_ENABLED
static ina226_t ina;
static I2C_HandleTypeDef *bus;
static uint32_t last_read_ms;
static uint32_t last_init_ms;
static bool filtered;
#endif

/* Resting voltage of a LiPo cell against charge, from the usual discharge curve. */
static const float cell_volts[] = { 3.27f, 3.61f, 3.69f, 3.71f, 3.73f, 3.77f, 3.79f, 3.84f, 3.92f, 4.02f, 4.20f };

#if INA226_ENABLED
/* INA226_ADDRESS first, then every address the A0/A1 straps give (0x40..0x4F): modules are sold
   strapped differently. The BNO055 (0x28/0x29) is outside that range. */
static bool find_ina226(void)
{
    if (INA226_Init(&ina, bus, INA226_ADDRESS, INA226_SHUNT_OHM, INA226_MAX_A)) {
        battery_debug.address = INA226_ADDRESS;
        return true;
    }
    for (uint8_t address = 0x40U; address <= 0x4FU; address++) {
        if (address != INA226_ADDRESS && INA226_Init(&ina, bus, address, INA226_SHUNT_OHM, INA226_MAX_A)) {
            battery_debug.address = address;
            return true;
        }
    }
    battery_debug.address = 0U;
    return false;
}
#endif

int Battery_CellPercent(float volts_per_cell)
{
    const int steps = (int)(sizeof(cell_volts) / sizeof(cell_volts[0])) - 1;
    if (volts_per_cell <= cell_volts[0]) return 0;
    if (volts_per_cell >= cell_volts[steps]) return 100;
    for (int i = 0; i < steps; i++) {
        if (volts_per_cell < cell_volts[i + 1]) {
            float share = (volts_per_cell - cell_volts[i]) / (cell_volts[i + 1] - cell_volts[i]);
            return (int)(10.0f * ((float)i + share) + 0.5f);
        }
    }
    return 100;
}

void Battery_Init(I2C_HandleTypeDef *hi2c)
{
    memset((void *)&battery_debug, 0, sizeof(battery_debug));
    battery_debug.percent = -1;
#if INA226_ENABLED
    bus = hi2c;
    last_init_ms = HAL_GetTick();
    battery_debug.ok = find_ina226() ? 1U : 0U;
#else
    (void)hi2c;
#endif
}

void Battery_Tick(uint32_t now_ms)
{
#if INA226_ENABLED
    if (!battery_debug.ok) {
        if ((uint32_t)(now_ms - last_init_ms) < RETRY_INIT_MS) return;
        last_init_ms = now_ms;
        battery_debug.ok = find_ina226() ? 1U : 0U;
        return;
    }
    if ((uint32_t)(now_ms - last_read_ms) < BATTERY_READ_PERIOD_MS) return;
    float dt = (float)(uint32_t)(now_ms - last_read_ms) / 1000.0f;
    last_read_ms = now_ms;
    float volts, amps;
    if (!INA226_Read(&ina, &volts, &amps)) {
        if (battery_debug.fails < 0xFFFFU) battery_debug.fails++;
        if (battery_debug.fails % 10U == 0U) battery_debug.ok = 0U;   /* gone: look for it again */
        return;
    }
    battery_debug.reads++;
    if (!filtered) {
        battery_debug.volts = volts;
        battery_debug.amps = amps;
        filtered = true;
    } else {
        float k = dt / (FILTER_TAU_S + dt);
        battery_debug.volts += k * (volts - battery_debug.volts);
        battery_debug.amps += k * (amps - battery_debug.amps);
    }
    /* Below 1 V per cell the INA226 is reading something other than the pack. */
    float per_cell = battery_debug.volts / (float)BATTERY_CELLS;
    battery_debug.percent = per_cell < 1.0f ? -1 : (int8_t)Battery_CellPercent(per_cell);
#else
    (void)now_ms;
#endif
}

float Battery_Volts(void)
{
    return battery_debug.ok ? battery_debug.volts : 0.0f;
}

int Battery_Percent(void)
{
    return battery_debug.ok ? battery_debug.percent : -1;
}
