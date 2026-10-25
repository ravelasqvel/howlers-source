#include "uart_console.h"

void uconsole_init(uart_console_t *uc, uint8_t uart_num)
{
    uc->data = NULL;
    uc->len = 0;
    uc->uart_num = uart_num;

    ///< Configure the UART
    uart_config_t uart_config = {
        .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(uc->uart_num, UART_BUF_SIZE, UART_BUF_SIZE, 20, &uc->uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(uc->uart_num, &uart_config));

    //Set UART log level
    // esp_log_level_set(TAG_UART, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // //Set uart pattern detect function.
    // uart_enable_pattern_det_baud_intr(uart_num, '+', PATTERN_CHR_NUM, 9, 0, 0);
    // //Reset the pattern queue length to record at most 20 pattern positions.
    // uart_pattern_queue_reset(uart_num, 20);

    uc->data = (uint8_t *)malloc(READ_BUF_SIZE);
    if (uc->data == NULL)
    {
        // ESP_LOGE(TAG_UART, "Failed to allocate memory for UART data buffer");
        return;
    }
}

void uconsole_read_data(uart_console_t *uc)
{
    bzero(uc->data, READ_BUF_SIZE); ///< Clear the buffer, and must be different from NULL
    uart_get_buffered_data_len(uc->uart_num, (size_t*)&uc->len);
    uart_read_bytes(uc->uart_num, uc->data, uc->len, 100 / portTICK_PERIOD_MS);
    uc->data[uc->len] = '\0'; ///< Add the null terminator
}

void uconsole_deinit(uart_console_t *uc)
{
    free(uc->data);
    uc->data = NULL;
    uc->len = 0;

    ESP_ERROR_CHECK(uart_driver_delete(uc->uart_num));
}

void uconsole_intr_handler(void *arg)
{
    uart_console_t *uc = (uart_console_t *)arg;

    uconsole_read_data(uc);
    ESP_LOGI("UART", "Data received: %s", uc->data);
    // gFlag.uc_data = true;
}