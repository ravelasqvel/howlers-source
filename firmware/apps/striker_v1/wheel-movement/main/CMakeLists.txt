file(GLOB SRCS_AS5600  "../../../../components/sensors/as5600/v1_striker1/*.c")
file(GLOB SRCS_TM151  "../../../../components/sensors/tm151/v1_striker1/*.c")
file(GLOB SRCS_VL53L1X  "../../../../components/sensors/vl53l1x/v1_striker1/*.c")

file(GLOB SRCS_PLATFORM  "../../../../components/platform_hal/v1_striker1/*.c")
file(GLOB SRCS_BLDC_PWM  "../../../../components/actuators/bldc_pwm/v1_striker1/*.c")
file(GLOB SRCS_PID_LIB  "../../../../components/control/pid_lib/v1_striker1/*.c")
file(GLOB SRCS_SENSOR_FUSION "../../../../components/control/sensor_fusion/v1_striker1/*.c")



idf_component_register(
        SRCS "main.c" ${SRCS_AS5600} ${SRCS_TM151} ${SRCS_VL53L1X} ${SRCS_PLATFORM} ${SRCS_BLDC_PWM} ${SRCS_PID_LIB} ${SRCS_SENSOR_FUSION}

        INCLUDE_DIRS "." "../../../../components/sensors/as5600/v1_striker1/include" "../../../../components/sensors/tm151/v1_striker1/include" "../../../../components/sensors/vl53l1x/v1_striker1/include"
                    "../../../../components/platform_hal/v1_striker1/include" "../../../../components/actuators/bldc_pwm/v1_striker1/include" "../../../../components/control/pid_lib/v1_striker1/include" "../../../../components/control/sensor_fusion/v1_striker1/include"
)


