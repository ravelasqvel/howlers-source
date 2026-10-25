/**
 * \file        platform_wifi.h
 * \brief       WiFi HAL for ESP32-S3
 * \details     WiFi initialization and connection management
 * 
 * \author      Kevin Jimenez
 * \version     0.0.1
 * \date        14/06/2025
 */

#ifndef __HAL_WIFI_ESP32S3__
#define __HAL_WIFI_ESP32S3__

#include <stdbool.h>
#include <stdint.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif_ip_addr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include <string.h>
#include "functions.h"
#include <lwip/sockets.h>
#include <lwip/netdb.h>



/**
 * @brief Initializes the WiFi setup
 * 
 */
void wifi_prepare(void);
 
/**
 * @brief Initializes the WiFi station mode and connects to the specified access point (AP)
 * @param ssid SSID of the AP
 * @param password Password of the AP
 * @param out_ip Pointer to store the obtained IP address
 * @return True if the connection was successful, false otherwise
 */
bool wifi_sta_init(const char *ssid, const char *password, esp_ip4_addr_t *out_ip);

/**
 * @brief Scans for available WiFi networks and prints their SSIDs
 * @return None
 */
void wifi_scan_and_print_networks(void);




#endif // __HAL_WIFI_ESP32S3__