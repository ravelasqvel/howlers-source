/**
 * \file        uart_console.h
 * \brief
 * \details
 * \author      MaverickST
 * \version     0.0.3
 * \date        05/10/2023
 * \copyright   Unlicensed
 */

#ifndef __UART_CONSOLE_H__
#define __UART_CONSOLE_H__

#include <stdio.h>
#include "esp_log.h"
#include "soc/soc.h"
#include "esp_intr_alloc.h"
#include "driver/uart.h"
#include "hal/uart_types.h"
#include "hal/uart_hal.h"
#include "freertos/queue.h"
// #include "types.h"

#define PATTERN_CHR_NUM    (3)  /*!< Set the number of consecutive and identical characters received 
                                     by receiver which defines a UART pattern*/

#define READ_BUF_SIZE   128
#define UART_BUF_SIZE   (READ_BUF_SIZE * 2)

// static const char* TAG_UART = "uart_console";

/**
 * @brief Structure to handle the UART console.
 * It just will just receive the data from the UART, but it can be extended to send data.
 * 
 */
typedef struct
{
    uint8_t *data;
    uint16_t len;
    uint8_t uart_num;
    QueueHandle_t uart_queue;

}uart_console_t;


/**
 * @brief Initialize the UART console
 * 
 */
void uconsole_init(uart_console_t *uc, uint8_t uart_num);

/**
 * @brief Read the data from the UART console
 * 
 * @param uc 
 */
void uconsole_read_data(uart_console_t *uc);

/**
 * @brief Deinitialize the UART console
 * 
 * @param uc 
 */
void uconsole_deinit(uart_console_t *uc);

/**
 * @brief UART console interrupt handler
 * 
 * @param arg 
 */
void uconsole_intr_handler(void *arg);

#endif // __UART_CONSOLE_H__