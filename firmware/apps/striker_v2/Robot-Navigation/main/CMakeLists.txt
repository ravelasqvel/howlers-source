
file(GLOB MAIN_SRCS "main.c" "tasks.c" "functions.c" "*.c")
file(GLOB PLATFORM_SRCS "../../../../components/platform_hal/v2_striker2/*.c")
file(GLOB AS5600_SRCS "../../../../components/sensors/as5600/v2_striker2/*.c")
file(GLOB BLDC_PWM_SRCS "../../../../components/actuators/bldc_pwm/v2_striker2/*.c")
file(GLOB CONTROL_SRCS "../../../../components/control/pid_lib/v2_striker2/*.c")
file(GLOB BNO055_SRCS "../../../../components/sensors/bno055/v2_striker2/*.c")
file(GLOB LED_SRCS "../../../../components/actuators/led_status/v2_striker2/*.c")
file(GLOB UART_CONSOLE_SRCS "../../../../components/comms/uart_console/v2_striker2/*.c")
file(GLOB VL53L1X_SRCS "../../../../components/sensors/vl53l1x/v2_striker2/*.c")

idf_component_register(
    SRCS
        ${MAIN_SRCS}
        ${PLATFORM_SRCS}
        ${AS5600_SRCS}
        ${BLDC_PWM_SRCS}
        ${CONTROL_SRCS}
        ${BNO055_SRCS}
        ${LED_SRCS}
        ${UART_CONSOLE_SRCS}
        ${VL53L1X_SRCS}
    INCLUDE_DIRS
        "."
        "../include"
        "../../../../components/platform_hal/v2_striker2"
        "../../../../components/sensors/as5600/v2_striker2"
        "../../../../components/actuators/bldc_pwm/v2_striker2"
        "../../../../components/control/pid_lib/v2_striker2"
        "../../../../components/sensors/bno055/v2_striker2"
        "../../../../components/comms/uart_console/v2_striker2"
        "../../../../components/sensors/vl53l1x/v2_striker2" 

    
)
