#pragma once

#include <stdbool.h>
#include <stdint.h>

/* The victim's confirm button on PE4 (CubeMX label ESTOP): "supplies taken". Not an E-stop: it
   stops nothing. Each debounced press adds one to presses, which goes to the Pi at the end of $SYS;
   a count, not an event, so a lost packet loses no press. Pure C, no HAL: host tests in Tests/. */
typedef struct {
    bool stable;            /* debounced state, true = pressed */
    bool raw;               /* last raw reading */
    uint32_t raw_since_ms;  /* when raw last changed */
    uint16_t presses;       /* released -> pressed transitions since boot, wraps at 65535 */
} confirm_button_t;

void ConfirmButton_Init(confirm_button_t *button, uint32_t now_ms);
/* pressed: the raw pin reading. A change counts once it has held CONFIRM_DEBOUNCE_MS. */
void ConfirmButton_Update(confirm_button_t *button, bool pressed, uint32_t now_ms);
