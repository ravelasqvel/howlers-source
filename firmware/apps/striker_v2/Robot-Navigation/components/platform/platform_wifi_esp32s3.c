// Include necessary libraries for Wi-Fi, logging, events, sockets, etc.
#include "platform_wifi_esp32s3.h"


// Wi-Fi connection status flags
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// Maximum number of connection retries
#define MAX_RETRY          20

// Custom command port (not used in this code)
#define WIFI_CMD_PORT 3333

// Event group to signal when connected or failed
static EventGroupHandle_t s_wifi_event_group;

// Logging tags
static const char *TAG = "wifi_sta";
static const char *TAG_SCAN = "wifi_scan";

// Connection retry counter
static int retry_num = 0;

// Stores obtained IP address
static esp_ip4_addr_t s_ip;

// Handles Wi-Fi and IP events
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
    ESP_LOGI(TAG, "Event received: base=%s, id=%ld", event_base, event_id);

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        // Try to connect to the configured Wi-Fi network
        esp_wifi_connect();

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // Log disconnection reason
        wifi_event_sta_disconnected_t *disconn = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "Disconnected from AP. Reason: %d", disconn->reason);

        // Retry connection if limit not reached
        ESP_LOGW(TAG, "Wi-Fi disconnected! Trying to reconnect...");
        if (retry_num < MAX_RETRY) {
            esp_wifi_connect();
            retry_num++;
            ESP_LOGI(TAG, "Retrying to connect to the AP (%d/%d)", retry_num, MAX_RETRY);
        } else {
            // Set fail bit if all retries fail
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        // Successfully connected and got IP address
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        s_ip = event->ip_info.ip;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&s_ip));
        retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Prepares Wi-Fi module: init NVS, netif, events and Wi-Fi driver
void wifi_prepare(void) {
    static bool initialized = false;
    if (initialized) return;

    // Initialize NVS flash memory
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Erase NVS if needed and re-init
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Init network interface and event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create default Wi-Fi station interface
    esp_netif_create_default_wifi_sta();

    // Initialize Wi-Fi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    initialized = true;
}

// Starts Wi-Fi in station mode and connects to the given SSID
bool wifi_sta_init(const char *ssid, const char *password, esp_ip4_addr_t *out_ip) {
    if (!ssid || strlen(ssid) == 0) {
        ESP_LOGE(TAG, "SSID is null or empty");
        return false;
    }

    // Create event group to handle connection flags
    s_wifi_event_group = xEventGroupCreate();

    // Register event handlers
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    // Set Wi-Fi configuration
    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    // Disable PMF (Protected Management Frames)
    wifi_config.sta.pmf_cfg.capable = false;
    wifi_config.sta.pmf_cfg.required = false;

    // Set station mode and apply configuration
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi init STA finished. Waiting for connection...");

    // Wait until connected or failed
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    bool connected = false;
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP: %s", ssid);
        connected = true;
        if (out_ip) *out_ip = s_ip;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to AP: %s", ssid);
    }

    // Cleanup handlers and event group
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);

    return connected;
}

// Scans available Wi-Fi networks and prints results
void wifi_scan_and_print_networks(void) {
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true
    };

    ESP_LOGI(TAG_SCAN, "Starting Wi-Fi scan...");
    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));

    // Get number of APs found
    uint16_t ap_num = 0;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_num));

    // Retrieve AP records (up to 20)
    wifi_ap_record_t ap_records[20];
    if (ap_num > 20) ap_num = 20;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_num, ap_records));

    ESP_LOGI(TAG_SCAN, "Found networks: %d", ap_num);
    for (int i = 0; i < ap_num; ++i) {
        // Translate auth mode enum to string
        const char *auth_mode = "UNKNOWN";
        switch (ap_records[i].authmode) {
            case WIFI_AUTH_OPEN: auth_mode = "OPEN"; break;
            case WIFI_AUTH_WEP: auth_mode = "WEP"; break;
            case WIFI_AUTH_WPA_PSK: auth_mode = "WPA"; break;
            case WIFI_AUTH_WPA2_PSK: auth_mode = "WPA2"; break;
            case WIFI_AUTH_WPA_WPA2_PSK: auth_mode = "WPA/WPA2"; break;
            case WIFI_AUTH_WPA3_PSK: auth_mode = "WPA3"; break;
            default: break;
        }

        // Log SSID, channel, RSSI, and auth mode
        ESP_LOGI(TAG_SCAN, "[%2d] SSID: %-20s | CH: %2d | RSSI: %3d | Auth: %s",
                 i + 1,
                 (char *)ap_records[i].ssid,
                 ap_records[i].primary,
                 ap_records[i].rssi,
                 auth_mode);
    }
}

// Simple UDP server task that receives and prints messages
void udp_server_task(void *pvParameters) {
    char rx_buffer[UDP_RX_BUF_SIZE];

    // Define listening address and port
    struct sockaddr_in6 dest_addr;
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    struct sockaddr_in listen_addr = {
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_family = AF_INET,
        .sin_port = htons(UDP_PORT)
    };

    // Create UDP socket
    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG_UDP_SERVER, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG_UDP_SERVER, "Socket created");

    // Bind socket to port
    if (bind(sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0) {
        ESP_LOGE(TAG_UDP_SERVER, "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG_UDP_SERVER, "Socket bound, port %d", UDP_PORT);

    // Main loop: wait for incoming UDP packets
    while (1) {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                           (struct sockaddr *)&source_addr, &socklen);

        if (len < 0) {
            ESP_LOGE(TAG_UDP_SERVER, "recvfrom failed: errno %d", errno);
            break;
        } else {
            rx_buffer[len] = 0; // Null-terminate received string
            ESP_LOGI(TAG_UDP_SERVER, "Received %d bytes from %s: '%s'",
                     len, inet_ntoa(source_addr.sin_addr), rx_buffer);
        }
    }

    // Clean up socket and end task
    close(sock);
    vTaskDelete(NULL);
}
