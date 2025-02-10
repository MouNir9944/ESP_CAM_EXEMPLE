idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_camera nvs_flash esp_system esp_netif esp_wifi esp_event driver esp_http_server 
)
