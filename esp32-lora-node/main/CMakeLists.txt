idf_component_register(
    SRCS "main.c" "nmea_gps.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_uart esp_driver_gpio esp_timer sx127x
)
