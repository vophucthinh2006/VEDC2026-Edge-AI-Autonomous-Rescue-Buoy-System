#include "confirm_button.h"
#include "app_config.h"

void ConfirmButton_Init(confirm_button_t *button, uint32_t now_ms) {
    button->stable = button->raw = false;
    button->raw_since_ms = now_ms;
    button->presses = 0U;
}

void ConfirmButton_Update(confirm_button_t *button, bool pressed, uint32_t now_ms) {
    if (pressed != button->raw) {
        button->raw = pressed;
        button->raw_since_ms = now_ms;
        return;
    }
    if (pressed != button->stable && (uint32_t)(now_ms - button->raw_since_ms) >= CONFIRM_DEBOUNCE_MS) {
        button->stable = pressed;
        if (pressed) button->presses++;
    }
}
