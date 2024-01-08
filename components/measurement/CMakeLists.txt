idf_component_register(
        SRCS "sensor_reader.c"
        INCLUDE_DIRS "." "../../include"
        REQUIRES driver esp_adc)