#pragma once

/* Helper board of the Pixhawk boat (STM32F103C8, Blue Pill). The autopilot drives the boat; this
   board does what hangs off the Raspberry Pi: camera servo, alarm relays, E-stop button, battery
   monitor, LoRa beacon to the shore station, telemetry to the FS-i6 handset. */

/* Raspberry Pi link (USART1). */
#define PI_LINK_TIMEOUT_MS         2000U   /* no HBT for this long: the Pi is taken to be down */
#define PI_REPORT_PERIOD_MS        500U    /* AUX packet to the Pi */
#define PI_STATUS_STALE_MS         3000U   /* POS older than this is not sent on: the beacon says NO_FIX */

/* Camera pan servo (TIM2_CH3, PA2), 1 tick = 1 us. Pulse = centre + dir * us_per_deg * angle.
   Flip DIR if the camera turns away from the person. Without a fresh CAM packet it looks ahead. */
#define CAM_CENTER_US              1500
#define CAM_US_PER_10DEG           55      /* 5.5 us per degree: 1000..2000 us over about 180 deg */
#define CAM_PAN_DIR                (-1)
#define CAM_PAN_LIMIT_DDEG         800     /* tenths of a degree */
#define CAM_TIMEOUT_MS             2000U

/* Alarm relays (PA1 RELAY1, PA0 RELAY2). The 5 V relay module pulls in on a low input and the
   lamp is wired to the normally-closed contact: pin low = relay in = lamp off, pin high = relay
   released = lamp on. Both switch on when the Pi's count of people rises, for ALARM_HOLD_MS. */
#define RELAY_ON_LEVEL             GPIO_PIN_SET
#define ALARM_HOLD_MS              30000U

/* E-stop button (PB9, internal pull-up): pressed pulls the pin to ground. The board cannot stop
   the motors itself; it tells the Pi, which puts the autopilot in HOLD. A cut wire reads as
   released: this is a request button, not a safety chain. */
#define ESTOP_PRESSED_LEVEL        GPIO_PIN_RESET
#define ESTOP_DEBOUNCE_TICKS       3U      /* x 10 ms */

/* INA226 battery monitor (I2C1), values from the pinout sheet: 5 mOhm shunt, 0.5 mA per count. */
#define INA226_ENABLED             1
#define INA226_ADDRESS             0x44U
#define INA226_CALIBRATION         2048U
#define INA226_CURRENT_LSB_UA      500U
#define INA226_PERIOD_MS           200U
#define INA226_RETRY_MS            5000U   /* a missing chip is looked for again this often */

/* LoRa beacon (SPI1, RA-02). The radio settings MUST match the station
   (esp32-lora-station/main/main.c) and the STM32F4 boat (STM32F4/App/app_config.h). Fit the
   433 MHz antenna before enabling: transmitting without one can damage the PA. */
#define LORA_ENABLED               1
#define LORA_BUOY_ID               "PHAO-01"
#define LORA_FREQUENCY_HZ          433000000UL
#define LORA_SF                    9U
#define LORA_BW_HZ                 125000UL
#define LORA_CR                    5U
#define LORA_SYNC_WORD             0xF3U
#define LORA_TX_POWER_DBM          17U
#define LORA_BEACON_PERIOD_MS      5000U
