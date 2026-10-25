
file(GLOB MAIN_SRCS "main.c" "tasks.c" "functions.c" "*.c")
file(GLOB PLATFORM_SRCS "../components/platform/*.c")
file(GLOB AS5600_SRCS "../components/drivers/as5600/*.c")
file(GLOB BLDC_PWM_SRCS "../components/drivers/bldc_pwm/*.c")
file(GLOB CONTROL_SRCS "../components/control/*.c")
file(GLOB BNO055_SRCS "../components/drivers/bno055/*.c")
file(GLOB LED_SRCS "../components/drivers/led/*.c")
file(GLOB UART_CONSOLE_SRCS "../components/drivers/uart_console/*.c")
file(GLOB VL53L1X_SRCS "../components/drivers/vl53l1x/*.c")

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
        "../components/platform"
        "../components/drivers/as5600"
        "../components/drivers/bldc_pwm"
        "../components/control"
        "../components/drivers/bno055"
        "../components/drivers/uart_console"
        "../components/drivers/vl53l1x" 

    
)