idf_component_register(
        SRCS "communication.c"
        INCLUDE_DIRS "." "../../include"
        REQUIRES esp_wifi nvs_flash
)