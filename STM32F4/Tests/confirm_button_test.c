#include "confirm_button.h"
#include "app_config.h"
#include <assert.h>
#include <stdio.h>

static void hold(confirm_button_t *b, bool pressed, uint32_t *now, uint32_t ms) {
    for (uint32_t t = 0; t < ms; t += 10U) { ConfirmButton_Update(b, pressed, *now); *now += 10U; }
}

int main(void) {
    confirm_button_t b;
    uint32_t now = 1000U;
    ConfirmButton_Init(&b, now);

    /* A clean press counts once, however long it is held. */
    hold(&b, true, &now, 500U);
    assert(b.presses == 1U && b.stable);
    hold(&b, false, &now, 200U);
    assert(b.presses == 1U && !b.stable);

    /* Contact bounce shorter than the debounce time counts nothing. */
    for (int i = 0; i < 5; i++) { hold(&b, true, &now, 20U); hold(&b, false, &now, 20U); }
    assert(b.presses == 1U);

    /* A bouncy press that then holds counts once. */
    hold(&b, true, &now, 20U); hold(&b, false, &now, 10U); hold(&b, true, &now, 300U);
    assert(b.presses == 2U);
    hold(&b, false, &now, 200U);

    /* Two separate presses count twice. */
    hold(&b, true, &now, 200U); hold(&b, false, &now, 200U);
    assert(b.presses == 3U);

    printf("confirm_button_test: PASS\n");
    return 0;
}
