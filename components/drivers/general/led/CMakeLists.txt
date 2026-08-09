idf_component_register(SRCS "led_esp32.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES crazyflie platform
                       PRIV_REQUIRES esp_driver_gpio)