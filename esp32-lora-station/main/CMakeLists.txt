idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    EMBED_TXTFILES "dashboard.html"
    REQUIRES esp_wifi esp_event esp_netif esp_http_server nvs_flash sx127x
)
