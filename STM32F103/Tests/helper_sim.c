/* The helper board on a PC (Linux), for the simulator: the firmware's own Pi link and decisions
 * (Modules/PiLink, App/helper_logic.c, App/lora_frame.c) with stdin and stdout in place of the
 * UART and the pins. sim/tools/helper_board_sim.py runs it behind a pseudo-terminal.
 *
 *   stdin    what the Pi sends ("$CAM,..." lines), and lines starting with '!' from the simulator:
 *              !estop 0|1            the button
 *              !battery <mV> <mA>    what the INA226 would read
 *   stdout   "$AUX,..." lines for the Pi, and what the board does, one line per change:
 *              E servo <us>   E relays 0|1   E pi 0|1   E estop 0|1   E beacon <LoRa line>
 *
 *   gcc -std=c11 -D_POSIX_C_SOURCE=200809L -IApp -IModules/PiLink Tests/helper_sim.c \
 *       App/helper_logic.c App/lora_frame.c Modules/PiLink/pi_link.c -o helper_sim
 */
#include "app_config.h"
#include "helper_logic.h"
#include "lora_frame.h"
#include "pi_link.h"
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static uint32_t now_ms(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint32_t)((uint64_t)t.tv_sec * 1000U + (uint64_t)t.tv_nsec / 1000000U);
}

int main(void) {
    pi_link_t pi;
    helper_logic_t logic;
    helper_inputs_t in = { .ina_ok = true, .bus_mv = 12400U, .current_ma = 1500, .lora_ok = true };
    PiLink_Init(&pi);
    HelperLogic_Init(&logic);
    setvbuf(stdout, NULL, _IOLBF, 0);

    char line[256];
    size_t length = 0U;
    uint32_t start = now_ms(), last_fast = start, last_report = start, next_beacon = start + 1000U, sequence = 0U;
    int servo = -1, relays = -1, pi_ok = -1, estop = -1;
    struct pollfd fd = { .fd = STDIN_FILENO, .events = POLLIN };

    for (;;) {
        if (poll(&fd, 1, 2) > 0) {
            char buffer[256];
            ssize_t n = read(STDIN_FILENO, buffer, sizeof(buffer));
            if (n <= 0) return 0;                         /* the simulator closed the pipe */
            for (ssize_t i = 0; i < n; i++) {
                if (buffer[i] != '\n') {
                    if (length < sizeof(line) - 1U) line[length++] = buffer[i];
                    continue;
                }
                line[length] = '\0';
                if (line[0] == '!') {
                    int a = 0, b = 0;
                    if (sscanf(line, "!estop %d", &a) == 1) in.estop_pressed = a != 0;
                    else if (sscanf(line, "!battery %d %d", &a, &b) == 2) { in.bus_mv = (uint16_t)a; in.current_ma = b; }
                } else {
                    uint32_t t = now_ms();
                    for (size_t k = 0U; k < length; k++) PiLink_FeedByte(&pi, (uint8_t)line[k], t);
                    PiLink_FeedByte(&pi, '\n', t);
                }
                length = 0U;
            }
        }

        uint32_t t = now_ms();
        if ((uint32_t)(t - last_fast) >= HELPER_FAST_PERIOD_MS) {
            last_fast = t;
            HelperLogic_Fast(&logic, &pi, &in, t);
            if (HelperLogic_TakeBeaconRequest(&logic)) next_beacon = t;
            const helper_outputs_t *out = &logic.out;
            if (out->servo_us != servo) printf("E servo %d\n", servo = out->servo_us);
            if (out->relays_on != relays) printf("E relays %d\n", relays = out->relays_on);
            if (out->pi_ok != pi_ok) printf("E pi %d\n", pi_ok = out->pi_ok);
            if (out->estop != estop) printf("E estop %d\n", estop = out->estop);
        }
        if ((uint32_t)(t - last_report) >= PI_REPORT_PERIOD_MS) {
            last_report = t;
            char report[64];
            if (HelperLogic_Report(&logic, &in, report, sizeof(report)) > 0U) fputs(report, stdout);
        }
        if ((int32_t)(t - next_beacon) >= 0) {
            next_beacon = t + LORA_BEACON_PERIOD_MS;
            char frame[LORA_FRAME_SIZE];
            if (HelperLogic_Beacon(&logic, pi.status, pi.status_seen, pi.last_status_ms, t, sequence++, frame, sizeof(frame)) > 0U)
                printf("E beacon %s\n", frame);
        }
    }
}
