/**
 * \file        apds9960_lib.h
 * \brief       APDS9960 library
 * \details
 * 
 * 
 * \author      Striker2
 * \version     0.0.1
 * \date        02/04/20245
 * \copyright   Unlicensed
 */

 #ifndef __APDS9960_H__
 #define __APDS9960_H__
 
 #include <stdint.h>
 #include <stdlib.h>
 #include <string.h>
 #include <stdio.h>
 
 #include "apds9960_defs.h"
 #include "platform_esp32s3.h"
 
 #ifdef __cplusplus
 extern "C"{
 #endif
 
 /**
   * @brief apds9960 handle structure definition
   */
typedef struct apds9960_handle_s
{
    i2c_t i2c_handle;
    APDS9960_reg_t reg;
}APDS9960_t;


 
/**
 * @brief      Initializes the APDS9960 sensor.
 * 
 * @param[in]  handle   Pointer to an APDS9960 handle structure.
 * @param[in]  i2c_num  I2C port number.
 * @param[in]  gpio_scl GPIO pin for the I2C clock line.
 * @param[in]  gpio_sda GPIO pin for the I2C data line.
 */
void APDS9960_Init(APDS9960_t *handle, i2c_port_t i2c_num, uint8_t gpio_scl, uint8_t gpio_sda);

 
/**
 * @brief      Deinitializes the APDS9960 sensor.
 * 
 * @param[in]  handle   Pointer to an APDS9960 handle structure.
 */
void APDS9960_Deinit(APDS9960_t *handle);
 
/**
 * @brief      Reads the RGBC (Red, Green, Blue, Clear) data from the APDS9960 sensor.
 * 
 * @param[in]  handle   Pointer to an APDS9960 handle structure.
 * @param[out] red      Pointer to store the red channel value.
 * @param[out] green    Pointer to store the green channel value.
 * @param[out] blue     Pointer to store the blue channel value.
 * @param[out] clear    Pointer to store the clear channel value.
 */
void APDS9960_Read_Rgbc(APDS9960_t *handle, uint16_t *red, uint16_t *green, uint16_t *blue, uint16_t *clear);
 
/**
 * @brief      Configures the APDS9960 sensor with user-defined gain and integration time.
 * 
 * @param[in]  handle   Pointer to an APDS9960 handle structure.
 * @param[in]  gain     Gain value (1, 4, 16, or 64).
 * @param[in]  integration_time_ms Integration time in milliseconds (2.78 ms to 712 ms).
 */
void APDS9960_Configure(APDS9960_t *handle, uint8_t gain, float integration_time_ms);


 
 
 
 
 
 #ifdef __cplusplus
  }
 #endif
 
 #endif // __APDS9960_H__
 