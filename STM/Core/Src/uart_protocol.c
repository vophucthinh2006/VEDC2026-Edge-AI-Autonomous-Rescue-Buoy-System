#include "uart_protocol.h"
#include "app_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINE 192U

static uint8_t xor_checksum(const char *text) { uint8_t cs = 0U; while (*text) cs ^= (uint8_t)*text++; return cs; }
static bool sequence_is_newer(uint16_t received, uint16_t previous) { return (uint16_t)(received - previous) < 0x8000U; }

void Protocol_Init(pi_command_t *command) {
    memset(command, 0, sizeof(*command));
    command->mode = NAV_STOP;
}

static void handle_packet(pi_command_t *command, char *payload, uint32_t now_ms) {
    char *fields[8] = {0}; uint8_t count = 0U;
    char *token = strtok(payload, ",");
    while (token && count < 8U) { fields[count++] = token; token = strtok(NULL, ","); }
    if (count < 2U) return;
    if (strcmp(fields[0], "NAV") == 0 && count >= 6U) {
        uint16_t seq = (uint16_t)strtoul(fields[1], NULL, 10);
        if (command->last_nav_ms != 0U && !sequence_is_newer(seq, command->sequence)) return;
        nav_mode_t mode = strcmp(fields[2], "AUTO") == 0 ? NAV_AUTO : (strcmp(fields[2], "MANUAL") == 0 ? NAV_MANUAL : NAV_STOP);
        float speed = strtof(fields[3], NULL), heading = strtof(fields[4], NULL);
        uint16_t ttl = (uint16_t)strtoul(fields[5], NULL, 10);
        if (speed < 0.0f || speed > 2.0f || heading < 0.0f || heading >= 360.0f || ttl == 0U || ttl > 1000U) return;
        command->mode = mode; command->speed_mps = speed; command->heading_deg = heading; command->ttl_ms = ttl; command->sequence = seq; command->last_nav_ms = now_ms;
    } else if (strcmp(fields[0], "HBT") == 0 && count >= 3U) {
        command->last_hbt_ms = now_ms; command->pi_link_ok = true;
    } else if (strcmp(fields[0], "SOS") == 0 && count >= 3U) {
        command->sos_on = strtoul(fields[2], NULL, 10) != 0U;
    } else if (strcmp(fields[0], "TXD") == 0 && count >= 6U) {
        /* Validation/ack is local. Attach an SX1278 driver here once its wiring is fixed. */
        command->txd_sequence = (uint16_t)strtoul(fields[1], NULL, 10);
        command->txd_pending = true;
    }
}

void Protocol_FeedByte(pi_command_t *command, uint8_t byte, uint32_t now_ms) {
    static char line[MAX_LINE]; static uint16_t length = 0U;
    if (byte == '\r') return;
    if (byte != '\n') {
        if (length < MAX_LINE - 1U) line[length++] = (char)byte;
        else length = 0U;
        return;
    }
    line[length] = '\0'; length = 0U;
    if (line[0] != '$') return;
    char *star = strchr(line, '*');
    if (!star || strlen(star + 1U) != 2U) return;
    *star = '\0';
    char *end = NULL; unsigned long received = strtoul(star + 1U, &end, 16);
    if (!end || *end != '\0' || received > 0xFFU || xor_checksum(line + 1U) != (uint8_t)received) return;
    handle_packet(command, line + 1U, now_ms);
}

bool Protocol_Send(UART_HandleTypeDef *huart, const char *command, const char *payload) {
    char body[MAX_LINE]; char line[MAX_LINE];
    int body_size = snprintf(body, sizeof(body), "%s,%s", command, payload);
    if (body_size < 0 || body_size >= (int)sizeof(body)) return false;
    int line_size = snprintf(line, sizeof(line), "$%s*%02X\n", body, xor_checksum(body));
    if (line_size < 0 || line_size >= (int)sizeof(line)) return false;
    return HAL_UART_Transmit(huart, (uint8_t *)line, (uint16_t)line_size, 20U) == HAL_OK;
}
