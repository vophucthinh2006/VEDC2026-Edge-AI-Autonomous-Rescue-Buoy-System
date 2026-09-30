#pragma once

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* Output map, from the Pinout (Hy) sheet:
   rear ESCs  DShot300 on TIM2 (PA5 left CH1, PA3 right CH4), forward only
   front ESC  PWM on TIM5_CH2 (PA1), bidirectional around ESC_FRONT_NEUTRAL_US
   servos     PWM on TIM3: PA6 camera, PA7 front, PB0 left, PB1 right rudder */

typedef enum { SERVO_CAMERA, SERVO_FRONT, SERVO_RIGHT, SERVO_LEFT, SERVO_COUNT } servo_id_t;
typedef enum { ESC_FRONT, ESC_RIGHT, ESC_LEFT, ESC_COUNT } esc_id_t;

/* What the boat should do, independent of pulse widths. */
typedef struct {
    float rear_left;    /* 0..1 */
    float rear_right;   /* 0..1 */
    float front;        /* -1..1, positive drives the boat forward */
    float steer;        /* -1..1, positive turns the bow right */
} actuator_cmd_t;

/* Last command and the pulses it became, for CubeMonitor. With
   ACTUATORS_ENABLED at 0 the ESC pulses are computed here but the pins stay
   at stop; the rudders still move. */
typedef struct {
    actuator_cmd_t cmd;
    uint16_t esc_us[ESC_COUNT];
    uint16_t servo_us[SERVO_COUNT];
} actuator_debug_t;

extern volatile actuator_debug_t actuator_debug;
extern const uint16_t actuator_servo_center[SERVO_COUNT];

/* Servos to their trimmed centres. ESC outputs stay silent until
   Actuators_StartEscs(). */
void Actuators_Init(void);
/* Start the stop signal (DShot 0 rear, neutral front) so the ESCs arm. */
void Actuators_StartEscs(void);
bool Actuators_EscsStarted(void);

/* Ramped command: magnitudes rise at ESC_RAMP_PER_S and fall at once, and the
   front motor rests at neutral for ESC_FRONT_REVERSE_DELAY_MS before it turns
   the other way. */
void Actuators_Apply(const actuator_cmd_t *cmd, uint32_t now_ms);
/* Everything to stop and rudders to centre, immediately. */
void Actuators_Stop(void);

/* Raw pulses for the bench test, no ramp and no ACTUATORS_ENABLED gate.
   Rear ESCs take 1000..2000 us, mapped onto DShot 0 / 48..2047. */
void Actuators_WriteServo(servo_id_t id, uint32_t us);
void Actuators_WriteEsc(esc_id_t id, uint32_t us);
