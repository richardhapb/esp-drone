idf_component_register(SRCS "buzzdeck.c" "piezo.c"
                       INCLUDE_DIRS "." "include"
                        REQUIRES crazyflie platform config
                        PRIV_REQUIRES esp_driver_ledc)