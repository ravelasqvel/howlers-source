idf_component_register(SRCS "compare_filters_esp32.c"
                            "robot_model.c"
                            "ekf.c"
                            "ukf.c"
                            "pf.c"
                    INCLUDE_DIRS "."
                    REQUIRES linearalgebra esp_timer)
