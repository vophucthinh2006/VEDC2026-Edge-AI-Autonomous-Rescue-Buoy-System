#pragma once

#include <stdint.h>

/* Bench test for the rudder servos, the ESCs and the buzzer, driven from a
   debugger (OpenOCD mww / CubeMonitor) while the control loop is off.
   Write the targets first, then the matching *_run_ms: each group falls back
   to its safe value once its timer runs out. Unless HW_TEST_ESC_ARM_AT_BOOT
   is set, the ESC outputs stay silent until esc_enable is written to 1. */
typedef struct {
    uint16_t servo_us[4];   /* 0x00: camera, front rudder, right rudder, left rudder */
    uint16_t esc_us[3];     /* 0x08: front (PWM, 1500 = stop), right, left (DShot) */
    uint16_t buzzer_ms;     /* 0x0E: beep length, counts down */
    uint32_t servo_run_ms;  /* 0x10: servos return to their trimmed centre at 0 */
    uint32_t esc_run_ms;    /* 0x14: ESCs return to stop at 0, capped */
    uint32_t esc_enable;    /* 0x18: 1 starts the ESC PWM at the stop value */
    uint32_t sweep_half_ms; /* 0x1C: non-zero swings the three rudders right/left
                               around their centres, this long each way; the
                               camera stays put. Overrides servo_us. */
} hw_test_t;

extern volatile hw_test_t hw_test;

void HwTest_Init(void);
void HwTest_Tick(uint32_t now_ms);
