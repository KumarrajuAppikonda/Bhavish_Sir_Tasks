/*
 * ESP32 Wi-Fi SoftAP + FastLED WS2812 (500ms RGB + 2s break) + Direct TCP OTA + Rollback
 * Firmware Version: 1.0.1
 */
#include <Arduino.h>
#include <FastLED.h>
#include <string.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_image_format.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#define FW_VERSION_STR          "1.0.1"
#define COLOR_DELAY_MS          500     /* 500 ms duration for Version 1.0.1 (Visual indicator) */

#define AP_WIFI_SSID            "ESP32_OTA_AP"
#define AP_WIFI_PASS            "12345678"
#define TCP_OTA_PORT            3333
#define TCP_RX_BUF_SIZE         4096
#define OTA_EOF_IDLE_TIMEOUT_S  3

#define WS2812_PIN              5
#define NUM_LEDS                8
CRGB leds[NUM_LEDS];

/* ================= LED TASK ================= */
static void led_strip_task(void *pvParameters)
{
    FastLED.addLeds<WS2812B, WS2812_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(80);

    while (1) {
        /* Red */
        fill_solid(leds, NUM_LEDS, CRGB::Red);
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(COLOR_DELAY_MS));

        /* Green */
        fill_solid(leds, NUM_LEDS, CRGB::Green);
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(COLOR_DELAY_MS));

        /* Blue */
        fill_solid(leds, NUM_LEDS, CRGB::Blue);
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(COLOR_DELAY_MS));

        /* 2 Second Break (OFF) */
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* ================= BOOT & ROLLBACK ================= */
static void check_boot_partition_and_rollback(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running) {
        Serial.printf("[BOOT] Active Partition: %s (offset 0x%08lx)\n",
                      running->label, (unsigned long)running->address);
    }

    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            Serial.println("[BOOT] New firmware self-test passed (v1.0.1). Confirming validity...");
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }
}

/* ================= WI-FI AP ================= */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        Serial.printf("[WIFI] Phone Connected to %s (MAC: " MACSTR ")\n",
                      AP_WIFI_SSID, MAC2STR(event->mac));
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        Serial.printf("[WIFI] Phone Disconnected from %s (MAC: " MACSTR ")\n",
                      AP_WIFI_SSID, MAC2STR(event->mac));
    }
}

static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));
    strlcpy((char *)wifi_config.ap.ssid, AP_WIFI_SSID, sizeof(wifi_config.ap.ssid));
    wifi_config.ap.ssid_len = strlen(AP_WIFI_SSID);
    wifi_config.ap.channel = 1;
    strlcpy((char *)wifi_config.ap.password, AP_WIFI_PASS, sizeof(wifi_config.ap.password));
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.ap.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    Serial.println("========================================");
    Serial.printf("[WIFI] AP Mode Active | Device Name / SSID: %s\n", AP_WIFI_SSID);
    Serial.printf("[WIFI] Password: %s | IP Address: 192.168.4.1\n", AP_WIFI_PASS);
    Serial.println("========================================");
}

/* ================= DIRECT TCP OTA ================= */
static void handle_direct_bin_ota(int client_sock)
{
    uint8_t *rx_buf = (uint8_t *)malloc(TCP_RX_BUF_SIZE);
    if (!rx_buf) return;

    struct timeval tv_wait = { .tv_sec = 120, .tv_usec = 0 };
    setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &tv_wait, sizeof(tv_wait));

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    esp_ota_handle_t update_handle = 0;
    bool ota_started = false;
    uint32_t total_received = 0;

    while (1) {
        int len = recv(client_sock, rx_buf, TCP_RX_BUF_SIZE, 0);

        if (len > 0) {
            if (!ota_started) {
                if (rx_buf[0] != ESP_IMAGE_HEADER_MAGIC) {
                    Serial.println("[OTA] Error: Invalid firmware binary.");
                    break;
                }

                if (!update_partition) {
                    Serial.println("[OTA] Error: No inactive OTA partition found.");
                    break;
                }

                Serial.println("[OTA] Firmware Transfer Started (Flashing new version)...");

                if (esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle) != ESP_OK) {
                    Serial.println("[OTA] Error: esp_ota_begin failed.");
                    break;
                }

                ota_started = true;
                struct timeval tv_eof = { .tv_sec = OTA_EOF_IDLE_TIMEOUT_S, .tv_usec = 0 };
                setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &tv_eof, sizeof(tv_eof));
            }

            if (esp_ota_write(update_handle, (const void *)rx_buf, len) != ESP_OK) {
                Serial.println("[OTA] Error: Flash write failed.");
                esp_ota_abort(update_handle);
                ota_started = false;
                break;
            }

            total_received += (uint32_t)len;
            Serial.printf("[OTA] Received: %lu bytes...\n", (unsigned long)total_received);
        } else {
            if (!ota_started) {
                Serial.println("[TCP] Client disconnected without sending firmware.");
                break;
            }

            if (len == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
                Serial.println("[OTA] Transfer complete. Validating new firmware image...");

                if (esp_ota_end(update_handle) == ESP_OK) {
                    Serial.println("[OTA] Validation Successful. Updating boot partition...");
                    if (esp_ota_set_boot_partition(update_partition) == ESP_OK) {
                        Serial.println("[OTA] SUCCESS! Restarting ESP32 into v1.0.1 firmware...");
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        free(rx_buf);
                        close(client_sock);
                        esp_restart();
                    }
                } else {
                    Serial.println("[OTA] Error: Firmware validation failed!");
                }
            } else {
                esp_ota_abort(update_handle);
            }
            break;
        }
    }
    free(rx_buf);
}

static void tcp_server_task(void *pvParameters)
{
    struct sockaddr_in dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_port = htons(TCP_OTA_PORT);

    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    listen(listen_sock, 1);

    Serial.printf("[TCP] OTA Server (v1.0.1) Listening on port %d...\n", TCP_OTA_PORT);

    while (1) {
        struct sockaddr_storage source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int client_sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (client_sock < 0) continue;

        Serial.println("[TCP] Mobile Client Connected to OTA Server.");
        handle_direct_bin_ota(client_sock);
        close(client_sock);
    }
}

/* ================= SETUP & LOOP ================= */
void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println("\n========================================");
    Serial.printf("[BOOT] ESP32 OTA System Starting | Version: %s\n", FW_VERSION_STR);
    Serial.println("========================================");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    xTaskCreate(led_strip_task, "led_strip_task", 4096, NULL, 5, NULL);
    check_boot_partition_and_rollback();
    wifi_init_softap();
    xTaskCreate(tcp_server_task, "tcp_server_task", 8192, NULL, 5, NULL);
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}