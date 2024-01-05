idf_component_register(
        SRCS "communication.c"
        INCLUDE_DIRS "."
        REQUIRES esp_wifi nvs_flash)