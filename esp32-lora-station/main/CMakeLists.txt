idf_component_register(
    SRCS "main.c" "nmea_gps.c" "lora_payload.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_uart esp_wifi esp_event esp_netif esp_http_client mbedtls nvs_flash sx127x
)
