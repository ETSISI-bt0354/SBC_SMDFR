# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES esp_driver_i2c esp_driver_uart esp_wifi esp_timer nvs_flash)

set(COMPONENT_SRCS "main.c ../components/BME280/bme280.c")
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()
