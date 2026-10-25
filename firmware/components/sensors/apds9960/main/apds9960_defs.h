/**
 * \file        apds9960_defs.h
 * \brief
 * \details
 * \author      Striker2
 * \version     0.0.1
 * \date        02/04/2025
 * \copyright   Unlicensed
 */
 
 #include <stdint.h>
 #include <stdlib.h>
 #include <string.h>
 #include <stdio.h>
/**
  * @brief chip information definition
  */
 #define CHIP_NAME                 "Broadcom APDS9960"        /**< chip name */
 #define MANUFACTURER_NAME         "Broadcom"                 /**< manufacturer name */
 #define SUPPLY_VOLTAGE_MIN        2.4f                       /**< chip min supply voltage */
 #define SUPPLY_VOLTAGE_MAX        3.6f                       /**< chip max supply voltage */
 #define MAX_CURRENT               100.0f                     /**< chip max current */
 #define TEMPERATURE_MIN           -40.0f                     /**< chip min operating temperature */
 #define TEMPERATURE_MAX           85.0f                      /**< chip max operating temperature */
 #define DRIVER_VERSION            1000                       /**< driver version */

/**
 * @brief frequency to use for i2c master
 */
#define I2C_MASTER_FREQ_HZ        400000                      /**< i2c master clock frequency */
  

/**
 * @brief i2c address definition
 */
#define APDS9960_ADDRESS        0x39        /**< i2c address */


/**
 * @brief register address definition
 * 
 */
 typedef enum
{
    APDS9960_REG_ENABLE    =    0x80,       /**< enable states and interrupts register */
    APDS9960_REG_ATIME     =    0x81,      /**< adc integration time register */
    APDS9960_REG_WTIME     =    0x83,      /**< wait time register */
    APDS9960_REG_AILTL     =    0x84,        /**< als interrupt low threshold low byte register */
    APDS9960_REG_AILTH     =    0x85,       /**< als interrupt low threshold high byte register */
    APDS9960_REG_AIHTL     =    0x86,       /**< als interrupt high threshold low byte register */
    APDS9960_REG_AIHTH     =    0x87,       /**< als interrupt high threshold high byte register */
    APDS9960_REG_PERS      =    0x8C,       /**< interrupt persistence filters register */
    APDS9960_REG_CONFIG1   =    0x8D,       /**< configuration register one register */
    APDS9960_REG_CONTROL   =    0x8F,       /**< gain control register */
    APDS9960_REG_CONFIG2   =    0x90,       /**< configuration register two register */
    APDS9960_REG_ID        =    0x92,       /**< device id register */
    APDS9960_REG_STATUS    =    0x93,       /**< device status register */
    APDS9960_REG_CDATAL    =    0x94,       /**< low byte of clear channel data register */
    APDS9960_REG_CDATAH    =    0x95,       /**< high byte of clear channel data register */
    APDS9960_REG_RDATAL    =    0x96,       /**< low byte of red channel data register */
    APDS9960_REG_RDATAH    =    0x97,       /**< high byte of red channel data register */
    APDS9960_REG_GDATAL    =    0x98,       /**< low byte of green channel data register */
    APDS9960_REG_GDATAH    =    0x99,       /**< high byte of green channel data register */
    APDS9960_REG_BDATAL    =    0x9A,       /**< low byte of blue channel data register */
    APDS9960_REG_BDATAH    =    0x9B,       /**< high byte of blue channel data register */
    APDS9960_REG_PDATA     =    0x9C,       /**< low byte of proximity data register */
    APDS9960_REG_CONFIG3   =    0x9F,       /**< configuration register three register */
    APDS9960_REG_IFORCE    =    0xE4,       /**< force interrupt register */
    APDS9960_REG_PICLEAR   =    0xE5,       /**< proximity interrupt clear register */
    APDS9960_REG_CICLEAR   =    0xE6,       /**< als clear channel interrupt clear register */
    APDS9960_REG_AICLEAR   =    0xE7        /**< all non-gesture interrupts clear register */

} APDS9960_reg_t;




 
 /**
  * @brief apds9960 bool enumeration definition
  */
 typedef enum
 {
     APDS9960_BOOL_FALSE = 0x00,        /**< false */
     APDS9960_BOOL_TRUE  = 0x01,        /**< true */
 } APDS9960_bool_t;

  /**
  * @brief apds9960 conf enumeration definition
  */
 typedef enum
 {
     APDS9960_CONF_GESTURE_ENABLE             = 6,        /**< gesture enable */
     APDS9960_CONF_PROXIMITY_INTERRUPT_ENABLE = 5,        /**< proximity interrupt enable */
     APDS9960_CONF_ALS_INTERRUPT_ENABLE       = 4,        /**< als interrupt enable */
     APDS9960_CONF_WAIT_ENABLE                = 3,        /**< wait enable */
     APDS9960_CONF_PROXIMITY_DETECT_ENABLE    = 2,        /**< proximity detect enable */
     APDS9960_CONF_ALS_ENABLE                 = 1,        /**< als enable */
     APDS9960_CONF_POWER_ON                   = 0,        /**< power on */
 } APDS9960_conf_t;

/**
  * @brief apds9960 als interrupt cycle enumeration definition
  */
 typedef enum
 {
     APDS9960_ALS_INTERRUPT_CYCLE_EVERY = 0,        /**< every als cycle */
     APDS9960_ALS_INTERRUPT_CYCLE_ANY   = 1,        /**< any als value outside of threshold range */
     APDS9960_ALS_INTERRUPT_CYCLE_2     = 2,        /**< 2 consecutive als values out of range */
     APDS9960_ALS_INTERRUPT_CYCLE_3     = 3,        /**< 3 consecutive als values out of range */
     APDS9960_ALS_INTERRUPT_CYCLE_5     = 4,        /**< 5 consecutive als values out of range */
     APDS9960_ALS_INTERRUPT_CYCLE_10    = 5,        /**< 10 consecutive als values out of range */
     APDS9960_ALS_INTERRUPT_CYCLE_15    = 6,        /**< 15 consecutive als values out of range */
     APDS9960_ALS_INTERRUPT_CYCLE_20    = 7,        /**< 20 consecutive als values out of range */
     APDS9960_ALS_INTERRUPT_CYCLE_25    = 8,        /**< 25 consecutive als values out of range */
     APDS9960_ALS_INTERRUPT_CYCLE_30    = 9,        /**< 30 consecutive als values out of range */
     APDS9960_ALS_INTERRUPT_CYCLE_35    = 10,       /**< 35 consecutive als values out of range */
     APDS9960_ALS_INTERRUPT_CYCLE_40    = 11,       /**< 40 consecutive als values out of range */
     APDS9960_ALS_INTERRUPT_CYCLE_45    = 12,       /**< 45 consecutive als values out of range */
     APDS9960_ALS_INTERRUPT_CYCLE_50    = 13,       /**< 50 consecutive als values out of range */
     APDS9960_ALS_INTERRUPT_CYCLE_55    = 14,       /**< 55 consecutive als values out of range */
     APDS9960_ALS_INTERRUPT_CYCLE_60    = 15,       /**< 60 consecutive als values out of range */
 } APDS9960_als_interrupt_cycle_t;

  /**
  * @brief apds9960 led current enumeration definition
  */
 typedef enum
 {
     APDS9960_LED_CURRENT_100_MA  = 0x00,        /**< 100 mA */
     APDS9960_LED_CURRENT_50_MA   = 0x01,        /**< 50 mA */
     APDS9960_LED_CURRENT_25_MA   = 0x02,        /**< 25 mA */
     APDS9960_LED_CURRENT_12P5_MA = 0x03,        /**< 12.5 mA */
 } APDS9960_led_current_t;

 /**
  * @brief apds9960 als and color gain enumeration definition
  */
 typedef enum
 {
     APDS9960_ALS_COLOR_GAIN_1X  = 0x00,        /**< 1x */
     APDS9960_ALS_COLOR_GAIN_4X  = 0x01,        /**< 4x */
     APDS9960_ALS_COLOR_GAIN_16X = 0x02,        /**< 16x */
     APDS9960_ALS_COLOR_GAIN_64X = 0x03,        /**< 64x */
 } APDS9960_als_color_gain_t;

 /**
  * @brief apds9960 saturation interrupt enumeration definition
  */
 typedef enum
 {
     APDS9960_SATURATION_INTERRUPT_CLEAR_PHOTODIODE = 6,        /**< clear photo diode saturation interrupt */
 } APDS9960_saturation_interrupt_t;

 /**
  * @brief apds9960 status enumeration definition
  */
 typedef enum
 {
     APDS9960_STATUS_CPSAT  = 7,        /**< clear photo diode saturation */
     APDS9960_STATUS_PGSAT  = 6,        /**< indicates that an analog saturation event occurred during a previous proximity or gesture cycle */
     APDS9960_STATUS_AINT   = 4,        /**< als interrupt */
     APDS9960_STATUS_AVALID = 0,        /**< als valid */
 } APDS9960_status_t;

 /**
  * @brief apds9960 interrupt status enumeration definition
  */
 typedef enum
 {
     APDS9960_INTERRUPT_STATUS_CPSAT         = 7,        /**< clear photo diode saturation */
     APDS9960_INTERRUPT_STATUS_AINT          = 4,        /**< als interrupt */
     APDS9960_INTERRUPT_STATUS_AVALID        = 0,        /**< als valid */
 } APDS9960_interrupt_status_t;

 /**
  * @defgroup apds9960_link_driver apds9960 link driver function
  * @brief    apds9960 link driver modules
  * @ingroup  apds9960_driver
  * @{
  */
 
 /**
  * @brief     initialize apds9960_t structure
  * @param[in] HANDLE pointer to an apds9960 handle structure
  * @param[in] STRUCTURE apds9960_t
  * @note      none
  */
 #define DRIVER_APDS9960_LINK_INIT(HANDLE, STRUCTURE)        memset(HANDLE, 0, sizeof(STRUCTURE))

/**
  * @brief     link delay_ms function
  * @param[in] HANDLE pointer to an apds9960 handle structure
  * @param[in] FUC pointer to a delay_ms function address
  * @note      none
  */
 #define DRIVER_APDS9960_LINK_DELAY_MS(HANDLE, FUC)          (HANDLE)->delay_ms = FUC
 
 /**
  * @brief     link debug_print function
  * @param[in] HANDLE pointer to an apds9960 handle structure
  * @param[in] FUC pointer to a debug_print function address
  * @note      none
  */
 #define DRIVER_APDS9960_LINK_DEBUG_PRINT(HANDLE, FUC)       (HANDLE)->debug_print = FUC
 
 /**
  * @brief     link receive_callback function
  * @param[in] HANDLE pointer to an apds9960 handle structure
  * @param[in] FUC pointer to a receive_callback function address
  * @note      none
  */
 #define DRIVER_APDS9960_LINK_RECEIVE_CALLBACK(HANDLE, FUC)  (HANDLE)->receive_callback = FUC









 