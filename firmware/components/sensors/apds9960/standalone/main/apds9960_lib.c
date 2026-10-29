/**
 * \file        apds9960_lib.c
 * \brief       APDS9960 library
 * \details
 * 
 * 
 * \author      Striker2
 * \version     0.0.1
 * \date        02/04/20245
 * \copyright   Unlicensed
 */

#include "apds9960_lib.h"
#include <stdlib.h>

/**
 * @brief      Initializes the APDS9960 sensor.
 * 
 * @param[in]  handle   Pointer to an APDS9960 handle structure.
 * @param[in]  i2c_num  I2C port number.
 * @param[in]  gpio_scl GPIO pin for the I2C clock line.
 * @param[in]  gpio_sda GPIO pin for the I2C data line.
 * 
 * @note       This function initializes the I2C communication for the APDS9960 sensor.
 *             If the initialization fails, an error message is printed.
 */
void APDS9960_Init(APDS9960_t *handle, i2c_port_t i2c_num, uint8_t gpio_scl, uint8_t gpio_sda)
{
        if(!i2c_init(&handle->i2c_handle, i2c_num, gpio_scl, gpio_sda, I2C_MASTER_FREQ_HZ, APDS9960_ADDRESS)) {
                printf("I2C initialization failed\n");
                return;
        }
}

/**
 * @brief      Deinitializes the APDS9960 sensor.
 * 
 * @param[in]  handle   Pointer to an APDS9960 handle structure.
 * 
 * @note       This function releases the I2C resources used by the APDS9960 sensor.
 */
void APDS9960_Deinit(APDS9960_t *handle)
{
        i2c_deinit(&handle->i2c_handle);
}

/**
 * @brief      Reads the RGBC (Red, Green, Blue, Clear) data from the APDS9960 sensor.
 * 
 * @param[in]  handle   Pointer to an APDS9960 handle structure.
 * @param[out] red      Pointer to store the red channel value.
 * @param[out] green    Pointer to store the green channel value.
 * @param[out] blue     Pointer to store the blue channel value.
 * @param[out] clear    Pointer to store the clear channel value.
 * 
 * @note       This function reads 8 bytes from the sensor's RGBC data registers and
 *             combines them into 16-bit values for each channel.
 */
void APDS9960_Read_Rgbc(APDS9960_t *handle, uint16_t *red, uint16_t *green, uint16_t *blue, uint16_t *clear)
{
        uint8_t data[8]; // Buffer to store the register data

        // Read the registers for CDATAL (Clear), RDATAL (Red), GDATAL (Green), and BDATAL (Blue)
        i2c_read_reg(&handle->i2c_handle, APDS9960_REG_CDATAL, data, 8);

        // Combine the 8-bit values into 16-bit integers
        *clear = (data[1] << 8) | data[0];  // Clear
        *red = (data[3] << 8) | data[2];    // Red
        *green = (data[5] << 8) | data[4];  // Green
        *blue = (data[7] << 8) | data[6];   // Blue
}

/**
 * @brief      Configures the APDS9960 sensor with user-defined gain and integration time.
 * 
 * @param[in]  handle   Pointer to an APDS9960 handle structure.
 * @param[in]  gain     Gain value (1, 4, 16, or 64).
 * @param[in]  integration_time_ms Integration time in milliseconds (2.78 ms to 712 ms).
 * 
 * @note       This function enables the sensor, sets the integration time and gain
 *             based on user input, and writes the configuration values to the appropriate registers.
 */
void APDS9960_Configure(APDS9960_t *handle, uint8_t gain, float integration_time_ms)
{
    uint8_t value;

    // Enable the sensor (PON and AEN)
    value = 0x03;
    i2c_write_reg(&handle->i2c_handle, 0x80, &value, 1);

    // Configure integration time

    value = (uint8_t)(256 - (integration_time_ms / 2.78)); // Convert ms to register value
    i2c_write_reg(&handle->i2c_handle, 0x81, &value, 1);

    // Configure gain
    switch (gain) {
        case 1:
            value = 0x00; // 1x gain
            break;
        case 4:
            value = 0x01; // 4x gain
            break;
        case 16:
            value = 0x02; // 16x gain
            break;
        case 64:
            value = 0x03; // 64x gain
            break;
        default:
            printf("Error: Invalid gain value (1, 4, 16, or 64).\n");
            return;
    }
    i2c_write_reg(&handle->i2c_handle, 0x8F, &value, 1);

    printf("APDS9960 configured with %.2f ms integration time and %dx gain.\n", integration_time_ms, gain);
}