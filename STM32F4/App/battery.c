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
static bool searching;         /* looking for the module, one address a call */
static uint8_t probe;          /* how many addresses this search has tried */
static uint8_t first_address = INA226_ADDRESS;   /* where it answered last, tried first */
#endif

/* Resting voltage of a LiPo cell against charge, from the usual discharge curve. */
static const float cell_volts[] = { 3.27f, 3.61f, 3.69f, 3.71f, 3.73f, 3.77f, 3.79f, 3.84f, 3.92f, 4.02f, 4.20f };

#if INA226_ENABLED
/* The address the last one answered on (INA226_ADDRESS at first), then every address the A0/A1
   straps give (0x40..0x4F): modules are sold strapped differently. The BNO055 (0x28/0x29) is outside
   that range. One address a call: the whole search in one call took ~0.4 s of the main loop when
   the bus was stuck (25 ms of HAL busy wait each), and stopped the IMU, the LoRa and the telemetry. */
static void search_step(void)
{
    uint8_t address = probe == 0U ? first_address : (uint8_t)(0x40U + probe - 1U);
    probe++;
    if ((probe == 1U || address != first_address) && INA226_Init(&ina, bus, address, INA226_SHUNT_OHM, INA226_MAX_A)) {
        battery_debug.address = address;
        battery_debug.ok = 1U;
        first_address = address;
        searching = false;
        return;
    }
    if (probe > 16U) {
        battery_debug.address = 0U;
        searching = false;
        last_init_ms = HAL_GetTick();
    }
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
    searching = true;   /* from the first Battery_Tick on */
    probe = 0U;
#else
    (void)hi2c;
#endif
}

void Battery_Tick(uint32_t now_ms)
{
#if INA226_ENABLED
    if (!battery_debug.ok) {
        if (!searching) {
            if ((uint32_t)(now_ms - last_init_ms) < RETRY_INIT_MS) return;
            searching = true;
            probe = 0U;
        }
        search_step();
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
