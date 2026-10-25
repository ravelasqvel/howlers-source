/**
 * \file        main.c
 * \brief       APDS9960 RGB Color Sensor Example
 * \details
 * 
 * 
 * \author      Striker2
 * \version     0.0.1
 * \date        08/04/20245
 * \copyright   Unlicensed
 */
#include <stdio.h>
#include "apds9960_lib.h"

#define I2C_MASTER_SCL_GPIO 7       /*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_GPIO 6       /*!< GPIO number for I2C master data  */
#define I2C_MASTER_NUM 1            /*!< I2C port number for master dev */

// Sensor configuration
#define APDS9960_GAIN 16               /*!< Gain (1, 4, 16, or 64) */
#define APDS9960_INTEGRATION_TIME 350  /*!< Integration time in ms (2.78 ms to 712 ms) */

/**
 * @brief Main application entry point.
 * 
 * This function initializes the APDS9960 sensor, configures it with default
 * gain and integration time, and continuously reads and prints RGBC values.
 */
void app_main(void)
{
    printf("APDS9960 RGB Color Sensor Example\n");
    printf("Initializing...\n");

    // Declare the sensor handle
    APDS9960_t gAPDS9960;

    // Initialize the sensor
    APDS9960_Init(&gAPDS9960, I2C_MASTER_NUM, I2C_MASTER_SCL_GPIO, I2C_MASTER_SDA_GPIO);

    // Configure the sensor with default values
    APDS9960_Configure(&gAPDS9960, APDS9960_GAIN, APDS9960_INTEGRATION_TIME);
    printf("APDS9960 sensor configured with gain %dx and integration time %d ms\n", APDS9960_GAIN, APDS9960_INTEGRATION_TIME);

    // Main loop to read RGBC data
    while (1) {
        uint16_t red, green, blue, clear;

        // Read RGBC values from the sensor
        APDS9960_Read_Rgbc(&gAPDS9960, &red, &green, &blue, &clear);

        // Print the read values
        printf("Red: %d, Green: %d, Blue: %d, Clear: %d\n", red, green, blue, clear);

        // Wait 1 second before the next reading
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
