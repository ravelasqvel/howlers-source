file(GLOB SRCS_AS5600  "../../../components/sensors/as5600/v1_striker1/*.c")

file(GLOB SRCS_PLATFORM  "../../../components/platform_hal/autotuning/*.c")
file(GLOB SRCS_BLDC_PWM  "../../../components/actuators/bldc_pwm/autotuning/*.c")
file(GLOB SRCS_PID_LIB  "../../../components/control/pid_lib/autotuning_rnn/*.c")
file(GLOB SRCS_SENSOR_FUSION "../../../components/control/sensor_fusion/autotuning/*.c")

idf_component_register(SRCS "firmware_main.c" "control_main.c" "mov_calculation.c" "telemetry.c" ${SRCS_AS5600} ${SRCS_PLATFORM} ${SRCS_BLDC_PWM} ${SRCS_PID_LIB} ${SRCS_SENSOR_FUSION}
                       INCLUDE_DIRS "." "../../../components/sensors/as5600/v1_striker1/include" "../../../components/platform_hal/autotuning/include"
                       "../../../components/actuators/bldc_pwm/autotuning/include" "../../../components/control/pid_lib/autotuning_rnn/include" "../../../components/control/sensor_fusion/autotuning/include"
                        "./include"
                        
                        REQUIRES driver freertos esp_timer nvs_flash esp_adc esp_wifi esp_netif
                    )

