#pragma once

/* The two lights on the relays, for the dark: RELAY1 PC3 (bit 0) and RELAY2 PC2 (bit 1).
 * Only the admin switches them, from the dashboard; the command reaches the boat in the shore
 * station's ACK (App/shore_link.h) and nothing on the boat turns them on by itself. Off at boot
 * until the first ACK; kept as they are while the station is silent. */

#include <stdint.h>

void Lights_Init(void);
void Lights_Set(uint8_t mask);
uint8_t Lights_Get(void);          /* what the relays are driven to, for the beacon's L= */
