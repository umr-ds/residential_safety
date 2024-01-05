idf_component_register(
        SRCS "sensor_reader.c"
        INCLUDE_DIRS "." "../../include"
        REQUIRES esp_driver_i2c esp_adc)