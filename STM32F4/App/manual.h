#pragma once

#include "actuators.h"
#include "ibus.h"
#include <stdbool.h>

/* Car-style driving from an FS-i6 in stick mode 2:
   right stick up     rear motors forward (front one too, by MANUAL_FRONT_FWD_GAIN)
   right stick down   rear motors off, front motor in reverse
   right stick sides  all three rudders
   left stick up      power limit: full right stick gives this much
   Releasing the right stick stops the boat. */
void Manual_Mix(const ibus_state_t *rc, actuator_cmd_t *cmd);
bool Manual_SpeedCentred(const ibus_state_t *rc);
