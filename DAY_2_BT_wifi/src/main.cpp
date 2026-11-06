// ==================== src/main.c ====================
/*
 * ESP32 WiFi device  v11  (PlatformIO, framework = espidf)
 *
 * GPIO5 : WS2812 LED
 * GPIO4 : ONE switch (debounced)
 *           hold 4 s    = AP mode (saved hotspot erased)
 *           short press = print full device status on serial monitor
 *
 * AP  ESP_SGLD_HC is ALWAYS ON; hotspot is a station link on top (APSTA).
 *
 * LED priority: red notice blink > GREEN > YELLOW (NVS try) > BLUE (phone) > RED > VIOLET
 *
 * SERIAL COMMANDS (115200): clear | erase | ap | status | reboot | help
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <ctype.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "led_strip.h"

#include "lwip/sockets.h"
#include "esp_sntp.h"

static const char *TAG = "WIFI_TEST";

/* ---------------- user settings ---------------- */
#define AP_SSID                 "ESP_SGLD_HC"
#define AP_PASS                 "configure123"     /* min 8 characters */
#define AP_IP_STR               "192.168.4.1"

#define DEFAULT_HS_SSID         "Sn"               /* written to NVS on first run */
#define DEFAULT_HS_PASS         "12345678"
#define SEED_VERSION            1                  /* change to write the defaults again */

#define LED_GPIO                5
#define LED_COUNT               1
#define LED_BRIGHTNESS_PCT      30
#define LED_BLINK_MS            300

#define LED_PHONE_BLUE_OVER_GREEN 0
#define LED_RETRY_SHOW_YELLOW   0

#define BUTTON_GPIO             4
#define BUTTON_ACTIVE_LEVEL     0                  /* 0 = pressed pulls to GND (pull-up) */
#define BUTTON_HOLD_MS          4000
#define BUTTON_SAMPLE_MS        10
#define BUTTON_DEBOUNCE_MS      50
#define BUTTON_SHORT_PRESS_STATUS 1

#define STA_MAX_ATTEMPTS        3
#define STA_ATTEMPT_TIMEOUT_MS  15000
#define RED_SOLID_MS            2500
#define RED_BLINK_TOTAL_MS      2000
#define RED_BLINK_STEP_MS       250

#define HOTSPOT_RETRY_MS        10000
#define HOTSPOT_RETRY_PHONE_MS  30000

#define STATUS_LOG_EVERY_S      10

#define CMD_UART                UART_NUM_0

/* ---- clock ---- */
#define TIME_SYNC_MODE          0                  /* 0 = internet NTP, 1 = router only, 2 = router then NTP */
#define TIME_TZ                 "IST-5:30"
#define TIME_STAGE_TIMEOUT_MS   10000
#define TIME_RETRY_MS           300000
#define CLOCK_LOG_EVERY_S       1
/* ------------------------------------------------ */

#ifndef OFFER_DNS
#define OFFER_DNS 0x02
#endif

#define NVS_NS "hotspot"

#define COL_VIOLET   148, 0, 211
#define COL_BLUE       0, 0, 255
#define COL_YELLOW   255, 150, 0
#define COL_GREEN      0, 255, 0
#define COL_RED      255, 0, 0
#define COL_OFF        0, 0, 0

#define BIT_BUTTON      BIT0
#define BIT_CREDS       BIT1
#define BIT_STA_GOT_IP  BIT2
#define BIT_STA_FAIL    BIT3
#define BIT_STA_LOST    BIT4
#define BIT_STATUS      BIT5

typedef enum { HS_IDLE, HS_CONNECTING, HS_CONNECTED } hs_state_t;
typedef enum { RES_OK, RES_FAIL, RES_BUTTON } conn_result_t;

static volatile hs_state_t s_hs = HS_IDLE;
static volatile int  s_ap_clients = 0;
static volatile int  s_result = 0;            /* page: 0 none, 1 connecting, 2 ok, 3 fail */
static volatile bool s_hs_failed = false;
static volatile bool s_ui_connecting = false;
static volatile bool s_led_hold = false;
static volatile bool s_blink_on = true;
static EventGroupHandle_t s_evt;
static SemaphoreHandle_t s_led_mtx;
static led_strip_handle_t s_strip;
static esp_netif_t *s_sta_netif = NULL;
static esp_netif_t *s_ap_netif = NULL;

static char s_ssid[33];
static char s_pass[65];

static bool s_saved = false;
static char s_sav_ssid[33];
static char s_sav_pass[65];

static int64_t s_next_check_ms = 0;

static volatile int s_last_reason = 0;
static volatile int s_conn_count = 0;
static volatile int s_lost_count = 0;
static volatile int s_retry_count = 0;
static volatile bool s_btn_pressed = false;
static volatile uint32_t s_btn_glitches = 0;
static volatile uint32_t s_btn_presses = 0;

static httpd_handle_t s_httpd = NULL;
static volatile bool s_dns_run = false;
static volatile bool s_dns_alive = false;

static inline int64_t now_ms(void) { return esp_timer_get_time() / 1000; }
static inline void status_request(void) { xEventGroupSetBits(s_evt, BIT_STATUS); }

/* ====================== helpers ====================== */
static void banner(const char *title)
{
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "  %s", title);
    ESP_LOGI(TAG, "==================================================");
}

static const char *reason_text(int r)
{
    switch (r) {
    case 0:   return "none yet";
    case 2:   return "authentication expired";
    case 8:   return "hotspot disconnected us (switched off?)";
    case 15:  return "4-way handshake timeout (wrong password?)";
    case 200: return "hotspot signal lost - switched off or out of range";
    case 201: return "hotspot not found - is it ON and 2.4 GHz?";
    case 202: return "authentication failed (wrong password?)";
    case 204: return "handshake timeout (wrong password?)";
    case 205: return "connection failed";
    default:  return "other reason";
    }
}

/* ====================== LED ====================== */
static void led_init(void)
{
    s_led_mtx = xSemaphoreCreateMutex();
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = { .invert_out = false },
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags = { .with_dma = false },
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip));
    led_strip_clear(s_strip);
}

static void led_raw(uint8_t r, uint8_t g, uint8_t b)
{
    r = (r * LED_BRIGHTNESS_PCT) / 100;
    g = (g * LED_BRIGHTNESS_PCT) / 100;
    b = (b * LED_BRIGHTNESS_PCT) / 100;
    xSemaphoreTake(s_led_mtx, portMAX_DELAY);
    for (int i = 0; i < LED_COUNT; i++) led_strip_set_pixel(s_strip, i, r, g, b);
    led_strip_refresh(s_strip);
    xSemaphoreGive(s_led_mtx);
}

/* the ONE place that decides the LED colour */
static void led_refresh(void)
{
    if (s_led_hold) return;
    if (s_hs == HS_CONNECTED) {
        if (LED_PHONE_BLUE_OVER_GREEN && s_ap_clients > 0) led_raw(COL_BLUE);
        else                                               led_raw(COL_GREEN);
    } else if (s_ui_connecting) {
        if (s_blink_on) led_raw(COL_YELLOW);
        else            led_raw(COL_OFF);
    } else if (s_ap_clients > 0) {
        led_raw(COL_BLUE);
    } else if (s_hs_failed) {
        led_raw(COL_RED);
    } else {
        led_raw(COL_VIOLET);
    }
}

static void led_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_MS));
        s_blink_on = !s_blink_on;
        if (s_ui_connecting) led_refresh();
    }
}

static const char *led_name(void)
{
    if (s_led_hold)                return "RED (blinking / solid notice)";
    if (s_hs == HS_CONNECTED)      return (LED_PHONE_BLUE_OVER_GREEN && s_ap_clients > 0) ? "BLUE" : "GREEN";
    if (s_ui_connecting)           return "YELLOW (blinking)";
    if (s_ap_clients > 0)          return "BLUE";
    if (s_hs_failed)               return "RED";
    return "VIOLET";
}

static bool wait_button(uint32_t ms)
{
    return (xEventGroupWaitBits(s_evt, BIT_BUTTON, pdFALSE, pdFALSE, pdMS_TO_TICKS(ms)) & BIT_BUTTON) != 0;
}

/* red blink for total_ms; returns true if the switch was held meanwhile */
static bool blink_red_ms(uint32_t total_ms)
{
    bool btn = false;
    s_led_hold = true;
    for (uint32_t t = 0; t < total_ms && !btn; t += 2 * RED_BLINK_STEP_MS) {
        led_raw(COL_RED);
        if (wait_button(RED_BLINK_STEP_MS)) { btn = true; break; }
        led_raw(COL_OFF);
        if (wait_button(RED_BLINK_STEP_MS)) { btn = true; break; }
    }
    s_led_hold = false;
    led_refresh();
    return btn;
}

/* ====================== NVS ====================== */
static void seed_defaults(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    uint8_t ver = 0;
    nvs_get_u8(h, "seed", &ver);
    if (ver != SEED_VERSION) {
        nvs_set_str(h, "ssid", DEFAULT_HS_SSID);
        nvs_set_str(h, "pass", DEFAULT_HS_PASS);
        nvs_set_u8(h, "seed", SEED_VERSION);
        nvs_commit(h);
        ESP_LOGI(TAG, "[NVS] First run: default hotspot '%s' saved to NVS", DEFAULT_HS_SSID);
    }
    nvs_close(h);
}

static void load_creds(void)
{
    s_saved = false;
    s_sav_ssid[0] = 0;
    s_sav_pass[0] = 0;
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "[NVS] No hotspot saved");
        return;
    }
    size_t l1 = sizeof(s_sav_ssid), l2 = sizeof(s_sav_pass);
    if (nvs_get_str(h, "ssid", s_sav_ssid, &l1) == ESP_OK && l1 > 1) {
        if (nvs_get_str(h, "pass", s_sav_pass, &l2) != ESP_OK) s_sav_pass[0] = 0;
        s_saved = true;
        ESP_LOGI(TAG, "[NVS] Saved hotspot found: '%s'", s_sav_ssid);
    } else {
        s_sav_ssid[0] = 0;
        ESP_LOGI(TAG, "[NVS] No hotspot saved");
    }
    nvs_close(h);
}

static void save_creds(const char *ssid, const char *pass)
{
    if (s_saved && strcmp(ssid, s_sav_ssid) == 0 && strcmp(pass, s_sav_pass) == 0) return;
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str(h, "ssid", ssid);
    nvs_set_str(h, "pass", pass);
    nvs_commit(h);
    nvs_close(h);
    strlcpy(s_sav_ssid, ssid, sizeof(s_sav_ssid));
    strlcpy(s_sav_pass, pass, sizeof(s_sav_pass));
    s_saved = true;
    ESP_LOGI(TAG, "[NVS] Hotspot '%s' saved (used automatically at every power-on)", ssid);
}

static void erase_creds(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, "ssid");
        nvs_erase_key(h, "pass");
        nvs_commit(h);
        nvs_close(h);
    }
    if (s_saved) ESP_LOGW(TAG, "[NVS] Saved hotspot '%s' erased", s_sav_ssid);
    s_saved = false;
    s_sav_ssid[0] = 0;
    s_sav_pass[0] = 0;
}

/* ====================== Network time ====================== */
#define TS_GATEWAY "@gateway"
static const char *const s_time_srv[] = {
#if TIME_SYNC_MODE == 0
    "pool.ntp.org", "time.google.com",
#elif TIME_SYNC_MODE == 1
    TS_GATEWAY,
#else
    TS_GATEWAY, "pool.ntp.org", "time.google.com",
#endif
};
#define TIME_SRV_COUNT ((int)(sizeof(s_time_srv) / sizeof(s_time_srv[0])))

static char s_ntp_srv[24];                    /* lwIP keeps the pointer */
static int  s_time_stage = 0;
static volatile bool s_time_pending = false;
static int64_t s_time_deadline_ms = 0;

static bool time_valid(void) { return time(NULL) > 1700000000; }

static void time_log_now(const char *why)
{
    time_t t = time(NULL);
    struct tm tm;
    char b[40];
    localtime_r(&t, &tm);
    strftime(b, sizeof(b), "%Y-%m-%d %H:%M:%S", &tm);
    ESP_LOGI(TAG, "[TIME] %s: %s  (%s)", why, b, TIME_TZ);
}

static void time_init(void)
{
    setenv("TZ", TIME_TZ, 1);
    tzset();
}

static void time_sync_stop(void)
{
    if (esp_sntp_enabled()) esp_sntp_stop();
    s_time_pending = false;
}

static void time_stage_begin(int stage)
{
    s_time_stage = stage;
    s_time_deadline_ms = now_ms() + TIME_STAGE_TIMEOUT_MS;

    const char *srv = s_time_srv[stage];
    if (strcmp(srv, TS_GATEWAY) == 0) {
        esp_netif_ip_info_t ip;
        if (!s_sta_netif || esp_netif_get_ip_info(s_sta_netif, &ip) != ESP_OK || ip.gw.addr == 0) {
            ESP_LOGW(TAG, "[TIME] Router address not known - skipping router time source");
            s_time_deadline_ms = 0;
            return;
        }
        snprintf(s_ntp_srv, sizeof(s_ntp_srv), IPSTR, IP2STR(&ip.gw));
    } else {
        strlcpy(s_ntp_srv, srv, sizeof(s_ntp_srv));
    }

    if (esp_sntp_enabled()) esp_sntp_stop();
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, s_ntp_srv);
    esp_sntp_init();
    ESP_LOGI(TAG, "[TIME] Asking %s%s for the time ...", s_ntp_srv,
             strcmp(srv, TS_GATEWAY) == 0 ? "  (WiFi router)" : "");
}

static void time_sync_start(void)
{
    time_stage_begin(0);
    s_time_pending = !time_valid();
}

static void time_sync_poll(void)
{
    if (!s_time_pending) return;
    if (s_hs != HS_CONNECTED) { s_time_pending = false; return; }
    if (time_valid()) { s_time_pending = false; time_log_now("Clock synced"); return; }
    if (now_ms() < s_time_deadline_ms) return;

    if (s_time_stage + 1 < TIME_SRV_COUNT) {
        if (s_time_stage >= 0) ESP_LOGW(TAG, "[TIME] No answer from %s - trying next source", s_ntp_srv);
        time_stage_begin(s_time_stage + 1);
    } else {
        ESP_LOGW(TAG, "[TIME] No time source answered - trying again in %d s", TIME_RETRY_MS / 1000);
        if (esp_sntp_enabled()) esp_sntp_stop();
        s_time_stage = -1;
        s_time_deadline_ms = now_ms() + TIME_RETRY_MS;
    }
}

/* ====================== FULL DEVICE STATUS ====================== */
static const char *mode_text(void)
{
    if (s_hs == HS_CONNECTED)  return "HOTSPOT CONNECTED";
    if (s_ui_connecting)       return "CONNECTING to the saved hotspot";
    if (s_hs == HS_CONNECTING) return "RETRYING the saved hotspot (silent, LED stays RED)";
    if (s_hs_failed)           return "HOTSPOT NOT CONNECTED (AP mode, retrying)";
    return "AP MODE - waiting for hotspot details";
}

static void print_status(const char *why)
{
    unsigned up = (unsigned)(esp_timer_get_time() / 1000000);
    ESP_LOGI(TAG, "+-------------------- DEVICE STATUS (%s) --------------------", why);
    ESP_LOGI(TAG, "| Uptime        : %u d %02u:%02u:%02u", up / 86400, (up / 3600) % 24, (up / 60) % 60, up % 60);
    ESP_LOGI(TAG, "| MODE          : %s", mode_text());
    ESP_LOGI(TAG, "| LED           : %s", led_name());

    uint8_t pri = 0;
    wifi_second_chan_t sec;
    esp_wifi_get_channel(&pri, &sec);
    ESP_LOGI(TAG, "| AP            : %s  (ON)  password '%s'  channel %u  IP " AP_IP_STR,
             AP_SSID, AP_PASS, (unsigned)pri);
    ESP_LOGI(TAG, "| AP phones     : %d connected   setup page: %s", s_ap_clients,
             s_result == 1 ? "connecting..." : s_result == 2 ? "last result OK" :
             s_result == 3 ? "last result FAILED" : "idle");

    ESP_LOGI(TAG, "| Switch GPIO%d  : %s   presses %u   glitches rejected %u   (debounce %d ms, hold %d s)",
             BUTTON_GPIO, s_btn_pressed ? "PRESSED" : "released",
             (unsigned)s_btn_presses, (unsigned)s_btn_glitches, BUTTON_DEBOUNCE_MS, BUTTON_HOLD_MS / 1000);

    if (s_saved) {
        ESP_LOGI(TAG, "| NVS hotspot   : '%s'  (password length %d)", s_sav_ssid, (int)strlen(s_sav_pass));
    } else {
        ESP_LOGI(TAG, "| NVS hotspot   : none saved");
    }

    const char *hs_txt = (s_hs == HS_CONNECTED) ? "CONNECTED" :
                         (s_hs == HS_CONNECTING) ? "CONNECTING" : "NOT CONNECTED";
    ESP_LOGI(TAG, "| Hotspot link  : %s", hs_txt);
    if (s_hs == HS_CONNECTED) {
        esp_netif_ip_info_t ip;
        if (s_sta_netif && esp_netif_get_ip_info(s_sta_netif, &ip) == ESP_OK) {
            ESP_LOGI(TAG, "|   IP / gateway : " IPSTR " / " IPSTR, IP2STR(&ip.ip), IP2STR(&ip.gw));
        }
        wifi_ap_record_t rec;
        if (esp_wifi_sta_get_ap_info(&rec) == ESP_OK) {
            ESP_LOGI(TAG, "|   Signal       : %d dBm   hotspot channel %u", rec.rssi, (unsigned)rec.primary);
        }
    } else if (s_saved && s_hs == HS_IDLE) {
        long left = (long)((s_next_check_ms - now_ms()) / 1000);
        if (left < 0) left = 0;
        ESP_LOGI(TAG, "|   Next retry   : in %ld s", left);
    }
    ESP_LOGI(TAG, "|   Last problem : %s (code %d)", reason_text(s_last_reason), s_last_reason);
    ESP_LOGI(TAG, "|   Counters     : connected %d x, lost %d x, background retries %d",
             s_conn_count, s_lost_count, s_retry_count);

    if (s_hs != HS_CONNECTED) {
        ESP_LOGI(TAG, "| Clock         : not running (hotspot not connected)");
    } else if (time_valid()) {
        time_t t = time(NULL);
        struct tm tm;
        char b[32];
        localtime_r(&t, &tm);
        strftime(b, sizeof(b), "%Y-%m-%d %H:%M:%S", &tm);
        ESP_LOGI(TAG, "| Clock         : %s  (%s)", b, TIME_TZ);
    } else {
        ESP_LOGI(TAG, "| Clock         : waiting for network time (asking %s)", s_ntp_srv);
    }

    ESP_LOGI(TAG, "| Memory        : free heap %u bytes   lowest ever %u bytes",
             (unsigned)esp_get_free_heap_size(), (unsigned)esp_get_minimum_free_heap_size());
    ESP_LOGI(TAG, "+------------------------------------------------------------------");
}

static void status_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1500));
    print_status("start");
    for (;;) {
        TickType_t w = (STATUS_LOG_EVERY_S > 0) ? pdMS_TO_TICKS(STATUS_LOG_EVERY_S * 1000) : portMAX_DELAY;
        EventBits_t b = xEventGroupWaitBits(s_evt, BIT_STATUS, pdTRUE, pdFALSE, w);
        print_status((b & BIT_STATUS) ? "update" : "periodic");
    }
}

/* ====================== HTML ====================== */
static const char PAGE_HEAD[] =
"<!DOCTYPE html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>WiFi Setup</title>"
"<style>body{font-family:Arial,sans-serif;background:#1b1530;color:#fff;margin:0;padding:20px}"
".card{max-width:380px;margin:40px auto;background:#2a2150;padding:24px;border-radius:12px}"
"h2{margin-top:0}p{color:#d0c8ff;font-size:14px}.err{background:#7a1f33;color:#fff;padding:10px;border-radius:8px}"
"input{width:100%;padding:12px;margin:8px 0 16px;border:0;border-radius:8px;"
"font-size:16px;box-sizing:border-box}button{width:100%;padding:14px;border:0;border-radius:8px;"
"background:#7c4dff;color:#fff;font-size:17px}</style></head><body><div class='card'>"
"<h2>Hotspot Setup</h2>";

static const char BANNER_FAIL[] =
"<p class='err'>Could not connect to that hotspot (3 tries). "
"Check the name and password and that the hotspot is ON (2.4 GHz). "
"The device keeps retrying it in the background.</p>";

static const char PAGE_TAIL[] =
"<p>Enter the hotspot the device should connect to.</p>"
"<form method='POST' action='/save'>"
"<label>Hotspot name (SSID)</label><input name='ssid' maxlength='32' required>"
"<label>Password</label><input name='pass' type='password' maxlength='64'>"
"<button type='submit'>Connect</button></form>"
"<p id='clk' style='text-align:center;font-size:18px;margin-bottom:0'></p>"
"<script>var b=0,t0=0;"
"function sy(){fetch('/time').then(function(r){return r.text();}).then(function(s){"
"if(s.length<19){b=0;return;}var a=s.split(/[- :]/);"
"b=Date.UTC(a[0],a[1]-1,a[2],a[3],a[4],a[5]);t0=Date.now();}).catch(function(){});}"
"function tk(){var e=document.getElementById('clk');"
"if(!b){e.style.display='none';return;}e.style.display='block';"
"e.innerText='Clock: '+new Date(b+Date.now()-t0).toISOString().replace('T',' ').slice(0,19);}"
"sy();setInterval(sy,30000);setInterval(tk,1000);tk();"
"</script></div></body></html>";

static const char PAGE_SAVED[] =
"<!DOCTYPE html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<style>body{font-family:Arial,sans-serif;background:#1b1530;color:#fff;padding:30px;text-align:center}"
"p{color:#d0c8ff}</style></head><body>"
"<h2 id='t'>Connecting...</h2>"
"<p id='m'>The device is trying to connect to your hotspot (up to 3 tries).<br>"
"BLUE = phone joined. GREEN = hotspot connected. RED = not connected. "
"If this page stops responding, check the LED.</p>"
"<script>"
"function p(){fetch('/status').then(function(r){return r.text();}).then(function(s){"
"if(s==='ok'){document.getElementById('t').innerText='Connected!';"
"document.getElementById('m').innerHTML='GREEN light = hotspot connected.<br>The WiFi "
AP_SSID " stays on.';}"
"else if(s==='fail'){location.href='/';}"
"else{setTimeout(p,1500);}}).catch(function(){setTimeout(p,2000);});}"
"setTimeout(p,1500);"
"</script></body></html>";

/* ====================== URL decode / form parse ====================== */
static void url_decode(char *dst, size_t dst_len, const char *src)
{
    size_t o = 0;
    for (size_t i = 0; src[i] && o + 1 < dst_len; i++) {
        if (src[i] == '+') {
            dst[o++] = ' ';
        } else if (src[i] == '%' && isxdigit((unsigned char)src[i + 1]) && isxdigit((unsigned char)src[i + 2])) {
            char hex[3] = { src[i + 1], src[i + 2], 0 };
            dst[o++] = (char)strtol(hex, NULL, 16);
            i += 2;
        } else {
            dst[o++] = src[i];
        }
    }
    dst[o] = 0;
}

static bool form_get(const char *body, const char *key, char *out, size_t out_len)
{
    size_t klen = strlen(key);
    const char *p = body;
    while (p && *p) {
        if (strncmp(p, key, klen) == 0 && p[klen] == '=') {
            p += klen + 1;
            const char *end = strchr(p, '&');
            size_t n = end ? (size_t)(end - p) : strlen(p);
            char *tmp = malloc(n + 1);
            if (!tmp) { out[0] = 0; return false; }
            memcpy(tmp, p, n);
            tmp[n] = 0;
            url_decode(out, out_len, tmp);
            free(tmp);
            return true;
        }
        p = strchr(p, '&');
        if (p) p++;
    }
    out[0] = 0;
    return false;
}

/* ====================== HTTP server ====================== */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Connection", "close");
    if (strcmp(req->uri, "/") == 0) {
        ESP_LOGI(TAG, "[PORTAL] Setup page opened on phone");
        httpd_resp_set_type(req, "text/html");
        httpd_resp_send_chunk(req, PAGE_HEAD, HTTPD_RESP_USE_STRLEN);
        if (s_result == 3) httpd_resp_send_chunk(req, BANNER_FAIL, HTTPD_RESP_USE_STRLEN);
        httpd_resp_send_chunk(req, PAGE_TAIL, HTTPD_RESP_USE_STRLEN);
        return httpd_resp_send_chunk(req, NULL, 0);
    }
    /* captive-portal probe -> redirect to the form */
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://" AP_IP_STR "/");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t status_get_handler(httpd_req_t *req)
{
    const char *txt = "idle";
    if (s_result == 1) txt = "connecting";
    else if (s_result == 2) txt = "ok";
    else if (s_result == 3) txt = "fail";
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_send(req, txt, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t time_get_handler(httpd_req_t *req)
{
    char out[24] = "0";
    if (s_hs == HS_CONNECTED && time_valid()) {
        time_t t = time(NULL);
        struct tm tm;
        localtime_r(&t, &tm);
        strftime(out, sizeof(out), "%Y-%m-%d %H:%M:%S", &tm);
    }
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t save_post_handler(httpd_req_t *req)
{
    char body[512];
    int total = req->content_len;
    if (total <= 0 || total >= (int)sizeof(body)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad request");
        return ESP_FAIL;
    }
    int got = 0;
    while (got < total) {
        int r = httpd_req_recv(req, body + got, total - got);
        if (r <= 0) {
            if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
            return ESP_FAIL;
        }
        got += r;
    }
    body[got] = 0;

    char ssid[33], pass[65];
    form_get(body, "ssid", ssid, sizeof(ssid));
    form_get(body, "pass", pass, sizeof(pass));

    if (strlen(ssid) == 0) {
        httpd_resp_set_status(req, "302 Found");
        httpd_resp_set_hdr(req, "Location", "http://" AP_IP_STR "/");
        httpd_resp_set_hdr(req, "Connection", "close");
        return httpd_resp_send(req, NULL, 0);
    }

    strlcpy(s_ssid, ssid, sizeof(s_ssid));
    strlcpy(s_pass, pass, sizeof(s_pass));
    s_result = 1;
    ESP_LOGI(TAG, "[PORTAL] Hotspot details received -> SSID='%s' (password length %d)",
             s_ssid, (int)strlen(s_pass));

    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_send(req, PAGE_SAVED, HTTPD_RESP_USE_STRLEN);

    xEventGroupSetBits(s_evt, BIT_CREDS);
    return ESP_OK;
}

static void http_start(void)
{
    if (s_httpd) return;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.max_uri_handlers = 8;
    cfg.max_open_sockets = 4;          /* 4 + 3 internal + 1 DNS < 10 lwIP sockets */
    cfg.lru_purge_enable = true;
    cfg.recv_wait_timeout = 5;
    cfg.send_wait_timeout = 5;
    cfg.stack_size = 6144;

    if (httpd_start(&s_httpd, &cfg) == ESP_OK) {
        httpd_uri_t save   = { .uri = "/save",   .method = HTTP_POST, .handler = save_post_handler };
        httpd_uri_t status = { .uri = "/status", .method = HTTP_GET,  .handler = status_get_handler };
        httpd_uri_t clk    = { .uri = "/time",   .method = HTTP_GET,  .handler = time_get_handler };
        httpd_uri_t root   = { .uri = "/*",      .method = HTTP_GET,  .handler = root_get_handler };
        httpd_register_uri_handler(s_httpd, &save);
        httpd_register_uri_handler(s_httpd, &status);
        httpd_register_uri_handler(s_httpd, &clk);
        httpd_register_uri_handler(s_httpd, &root);
    } else {
        s_httpd = NULL;
        ESP_LOGE(TAG, "HTTP server start failed");
    }
}

/* ====================== Captive DNS ====================== */
static void dns_task(void *arg)
{
    s_dns_alive = true;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        s_dns_alive = false;
        vTaskDelete(NULL);
        return;
    }
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(53),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    bind(sock, (struct sockaddr *)&addr, sizeof(addr));
    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint8_t buf[300];
    while (s_dns_run) {
        struct sockaddr_in from;
        socklen_t flen = sizeof(from);
        int len = recvfrom(sock, buf, sizeof(buf) - 16, 0, (struct sockaddr *)&from, &flen);
        if (len < 12) continue;

        int i = 12;
        while (i < len && buf[i] != 0) i += buf[i] + 1;
        i += 1;
        if (i + 4 > len) continue;
        uint16_t qtype = (buf[i] << 8) | buf[i + 1];
        int q_end = i + 4;

        buf[2] = 0x81; buf[3] = 0x80;
        buf[6] = 0; buf[7] = 0;
        buf[8] = 0; buf[9] = 0;
        buf[10] = 0; buf[11] = 0;
        int out = q_end;

        if (qtype == 1) {
            buf[7] = 1;
            buf[out++] = 0xC0; buf[out++] = 0x0C;
            buf[out++] = 0; buf[out++] = 1;
            buf[out++] = 0; buf[out++] = 1;
            buf[out++] = 0; buf[out++] = 0; buf[out++] = 0; buf[out++] = 30;
            buf[out++] = 0; buf[out++] = 4;
            buf[out++] = 192; buf[out++] = 168; buf[out++] = 4; buf[out++] = 1;
        }
        sendto(sock, buf, out, 0, (struct sockaddr *)&from, flen);
    }
    close(sock);
    s_dns_alive = false;
    vTaskDelete(NULL);
}

static void dns_start(void)
{
    if (s_dns_run || s_dns_alive) return;
    s_dns_run = true;
    xTaskCreate(dns_task, "dns", 4096, NULL, 4, NULL);
}

/* ====================== WiFi events ====================== */
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_AP_STACONNECTED: {
            wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
            s_ap_clients++;
            ESP_LOGI(TAG, "[AP] Phone " MACSTR " joined %s  (%d connected)",
                     MAC2STR(e->mac), AP_SSID, s_ap_clients);
            led_refresh();
            break;
        }
        case WIFI_EVENT_AP_STADISCONNECTED: {
            wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)data;
            if (s_ap_clients > 0) s_ap_clients--;
            ESP_LOGI(TAG, "[AP] Phone " MACSTR " left %s  (%d connected)",
                     MAC2STR(e->mac), AP_SSID, s_ap_clients);
            led_refresh();
            break;
        }
        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
            if (s_hs == HS_CONNECTING) {
                s_last_reason = e->reason;
                ESP_LOGW(TAG, "[HOTSPOT] Not connected: %s (code %d)", reason_text(e->reason), e->reason);
                xEventGroupSetBits(s_evt, BIT_STA_FAIL);
            } else if (s_hs == HS_CONNECTED) {
                s_last_reason = e->reason;
                s_lost_count++;
                ESP_LOGW(TAG, "[HOTSPOT] Connection lost: %s (code %d) -> LED RED", reason_text(e->reason), e->reason);
                s_hs = HS_IDLE;
                s_hs_failed = true;
                led_refresh();
                xEventGroupSetBits(s_evt, BIT_STA_LOST);
            }
            break;
        }
        default: break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "[HOTSPOT] Got IP address: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_evt, BIT_STA_GOT_IP);
    }
}

/* ====================== running clock ====================== */
static void clock_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    int n = 0;
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000));
        if (s_hs != HS_CONNECTED) { n = 0; continue; }
        n++;
        if (time_valid()) {
            if (CLOCK_LOG_EVERY_S > 0 && (n % CLOCK_LOG_EVERY_S) == 0) {
                time_t t = time(NULL);
                struct tm tm;
                char b[32];
                localtime_r(&t, &tm);
                strftime(b, sizeof(b), "%Y-%m-%d %H:%M:%S", &tm);
                ESP_LOGI(TAG, "[CLOCK] %s", b);
            }
        } else if ((n % 5) == 0) {
            ESP_LOGI(TAG, "[CLOCK] hotspot connected - waiting for network time");
        }
    }
}

/* ====================== Hotspot link ====================== */
static void hs_drop(void)
{
    time_sync_stop();
    bool was_active = (s_hs != HS_IDLE);
    s_hs = HS_IDLE;
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(was_active ? 300 : 100));
}

static conn_result_t hotspot_try(const char *ssid, const char *pass, int attempts, bool show_yellow)
{
    conn_result_t res = RES_FAIL;

    hs_drop();

    wifi_config_t sta = { 0 };
    strlcpy((char *)sta.sta.ssid, ssid, sizeof(sta.sta.ssid));
    strlcpy((char *)sta.sta.password, pass, sizeof(sta.sta.password));
    sta.sta.threshold.authmode = WIFI_AUTH_OPEN;
    sta.sta.pmf_cfg.capable = true;
    sta.sta.pmf_cfg.required = false;
    esp_wifi_set_config(WIFI_IF_STA, &sta);

    s_blink_on = true;
    s_ui_connecting = show_yellow;

    for (int a = 1; a <= attempts; a++) {
        s_hs = HS_CONNECTING;
        xEventGroupClearBits(s_evt, BIT_STA_GOT_IP | BIT_STA_FAIL | BIT_STA_LOST);
        led_refresh();
        ESP_LOGI(TAG, "[HOTSPOT] Connecting to '%s'  (attempt %d of %d)%s",
                 ssid, a, attempts, show_yellow ? "  -> LED YELLOW blinking" : "");
        esp_err_t ce = esp_wifi_connect();
        if (ce != ESP_OK) {
            ESP_LOGW(TAG, "[HOTSPOT] esp_wifi_connect failed: %s", esp_err_to_name(ce));
            xEventGroupSetBits(s_evt, BIT_STA_FAIL);
        }

        EventBits_t b = xEventGroupWaitBits(s_evt, BIT_BUTTON | BIT_STA_GOT_IP | BIT_STA_FAIL,
                                            pdFALSE, pdFALSE, pdMS_TO_TICKS(STA_ATTEMPT_TIMEOUT_MS));
        if (b & BIT_BUTTON) {
            hs_drop();
            res = RES_BUTTON;
            goto done;
        }
        if (b & BIT_STA_GOT_IP) {
            s_hs = HS_CONNECTED;
            s_hs_failed = false;
            s_conn_count++;
            res = RES_OK;
            goto done;
        }
        if (!(b & BIT_STA_FAIL)) {
            s_last_reason = 200;
            ESP_LOGW(TAG, "[HOTSPOT] No answer from '%s' within %d s", ssid, STA_ATTEMPT_TIMEOUT_MS / 1000);
        }
        hs_drop();
        if (wait_button(300)) {
            res = RES_BUTTON;
            goto done;
        }
    }
    s_hs = HS_IDLE;
    s_hs_failed = true;

done:
    s_ui_connecting = false;
    led_refresh();
    return res;
}

static void on_hotspot_connected(void)
{
    s_result = 2;
    led_refresh();
    banner("HOTSPOT CONNECTED (LED GREEN)");
    ESP_LOGI(TAG, "[HOTSPOT] Link is up. %s is still ON for your phone.", AP_SSID);
    time_sync_start();
    status_request();
}

/* ====================== control handlers ====================== */
static const char *volatile s_ap_reason = "SWITCH HELD";

static void handle_button(void)
{
    char title[64];
    snprintf(title, sizeof(title), "%s -> AP MODE (VIOLET, BLUE when a phone joins)", s_ap_reason);
    banner(title);
    s_ap_reason = "SWITCH HELD";
    hs_drop();
    erase_creds();
    s_hs_failed = false;
    s_ui_connecting = false;
    s_led_hold = false;
    s_result = 0;
    xEventGroupClearBits(s_evt, BIT_BUTTON | BIT_CREDS | BIT_STA_GOT_IP | BIT_STA_FAIL | BIT_STA_LOST);
    led_refresh();
    ESP_LOGI(TAG, "[AP] WiFi name : %s   password : %s   page: http://" AP_IP_STR, AP_SSID, AP_PASS);
    ESP_LOGI(TAG, "[AP] Old hotspot forgotten - connect your phone and enter the NEW hotspot details");
    status_request();
}

static void handle_new_creds(void)
{
    xEventGroupClearBits(s_evt, BIT_CREDS);
    s_result = 1;
    vTaskDelay(pdMS_TO_TICKS(1500));        /* let the "connecting" page reach the phone */

    char ssid[33], pass[65];
    strlcpy(ssid, s_ssid, sizeof(ssid));
    strlcpy(pass, s_pass, sizeof(pass));

    conn_result_t r = hotspot_try(ssid, pass, STA_MAX_ATTEMPTS, false);
    if (r == RES_OK) {
        save_creds(ssid, pass);
        on_hotspot_connected();
    } else if (r == RES_FAIL) {
        save_creds(ssid, pass);
        s_result = 3;
        s_next_check_ms = now_ms() + HOTSPOT_RETRY_PHONE_MS;
        ESP_LOGW(TAG, "[HOTSPOT] Could not connect to '%s' after %d attempts -> LED RED", ssid, STA_MAX_ATTEMPTS);
        status_request();
        s_led_hold = true;
        led_raw(COL_RED);
        bool btn = wait_button(RED_SOLID_MS);
        s_led_hold = false;
        led_refresh();
        if (btn) handle_button();
    } else {
        ESP_LOGW(TAG, "[AP] AP-mode request (switch or serial command) while connecting");
        handle_button();
    }
}

static void boot_connect(void)
{
    banner("POWER-ON: connecting to the saved hotspot (LED YELLOW blinking)");
    conn_result_t r = hotspot_try(s_sav_ssid, s_sav_pass, STA_MAX_ATTEMPTS, true);
    if (r == RES_OK) {
        on_hotspot_connected();
    } else if (r == RES_FAIL) {
        ESP_LOGW(TAG, "[HOTSPOT] '%s' not connected -> LED RED blink %d s, then WiFi (AP) mode, retry every %d s",
                 s_sav_ssid, RED_BLINK_TOTAL_MS / 1000, HOTSPOT_RETRY_MS / 1000);
        s_next_check_ms = now_ms() + HOTSPOT_RETRY_MS;
        status_request();
        if (blink_red_ms(RED_BLINK_TOTAL_MS)) handle_button();
    } else {
        handle_button();
    }
}

static void handle_lost(void)
{
    xEventGroupClearBits(s_evt, BIT_STA_LOST);
    ESP_LOGW(TAG, "[HOTSPOT] '%s' disconnected -> LED RED blink %d s, then WiFi (AP) mode, retry every %d s",
             s_sav_ssid, RED_BLINK_TOTAL_MS / 1000, HOTSPOT_RETRY_MS / 1000);
    status_request();
    bool btn = blink_red_ms(RED_BLINK_TOTAL_MS);
    s_next_check_ms = now_ms();
    if (btn) handle_button();
}

static void hotspot_retry(void)
{
    ESP_LOGI(TAG, "[RETRY] Looking for saved hotspot '%s' ...", s_sav_ssid);
    s_retry_count++;
    conn_result_t r = hotspot_try(s_sav_ssid, s_sav_pass, 1, LED_RETRY_SHOW_YELLOW ? true : false);
    s_next_check_ms = now_ms() + (s_ap_clients > 0 ? HOTSPOT_RETRY_PHONE_MS : HOTSPOT_RETRY_MS);
    if (r == RES_OK) {
        on_hotspot_connected();
    } else if (r == RES_FAIL) {
        ESP_LOGW(TAG, "[RETRY] Hotspot not available -> LED RED. Next try in %d s",
                 (int)((s_next_check_ms - now_ms()) / 1000));
    } else {
        handle_button();
    }
}

/* ====================== serial commands ====================== */
static void cmd_run(const char *cmd)
{
    if (strcmp(cmd, "clear") == 0 || strcmp(cmd, "erase") == 0 || strcmp(cmd, "ap") == 0) {
        ESP_LOGW(TAG, "[CMD] '%s' -> clearing saved hotspot from NVS and going to AP mode", cmd);
        s_ap_reason = "SERIAL COMMAND 'clear'";
        xEventGroupSetBits(s_evt, BIT_BUTTON);
    } else if (strcmp(cmd, "status") == 0) {
        status_request();
    } else if (strcmp(cmd, "reboot") == 0) {
        ESP_LOGW(TAG, "[CMD] Restarting ...");
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    } else if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        ESP_LOGI(TAG, "[CMD] clear  = erase saved hotspot (NVS) + AP mode (LED VIOLET)");
        ESP_LOGI(TAG, "[CMD] status = full device status");
        ESP_LOGI(TAG, "[CMD] reboot = restart the device");
    } else {
        ESP_LOGW(TAG, "[CMD] Unknown command '%s' - type 'help'", cmd);
    }
}

static void cmd_task(void *arg)
{
    uart_driver_install(CMD_UART, 256, 0, 0, NULL, 0);
    char line[32];
    int n = 0;
    uint8_t c;
    for (;;) {
        if (uart_read_bytes(CMD_UART, &c, 1, pdMS_TO_TICKS(100)) != 1) continue;
        if (c == '\r' || c == '\n') {
            if (n > 0) {
                line[n] = 0;
                cmd_run(line);
                n = 0;
            }
        } else if (n < (int)sizeof(line) - 1) {
            line[n++] = (char)tolower(c);
        }
    }
}

/* ====================== switch (debounce + 4 s hold) ====================== */
static void button_task(void *arg)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en   = BUTTON_ACTIVE_LEVEL == 0 ? GPIO_PULLUP_ENABLE  : GPIO_PULLUP_DISABLE,
        .pull_down_en = BUTTON_ACTIVE_LEVEL == 1 ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    bool stable = false;                 /* accepted level: true = pressed */
    bool cand = false;
    uint32_t cand_ms = 0;
    uint32_t held_ms = 0;
    bool hold_fired = false;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(BUTTON_SAMPLE_MS));
        bool raw = (gpio_get_level(BUTTON_GPIO) == BUTTON_ACTIVE_LEVEL);

        if (raw != stable) {
            if (raw != cand) { cand = raw; cand_ms = 0; }
            cand_ms += BUTTON_SAMPLE_MS;
            if (cand_ms >= BUTTON_DEBOUNCE_MS) {
                stable = raw;
                s_btn_pressed = stable;
                cand_ms = 0;
                if (stable) {
                    s_btn_presses++;
                    held_ms = 0;
                    hold_fired = false;
                } else if (!hold_fired && BUTTON_SHORT_PRESS_STATUS) {
                    status_request();            /* short press */
                }
            }
        } else {
            if (cand_ms > 0) {                   /* glitch: level changed and went back */
                s_btn_glitches++;
                cand_ms = 0;
            }
            cand = stable;
        }

        if (stable && !hold_fired) {
            held_ms += BUTTON_SAMPLE_MS;
            if (held_ms >= BUTTON_HOLD_MS) {
                hold_fired = true;
                ESP_LOGW(TAG, "[SWITCH] held %d s", BUTTON_HOLD_MS / 1000);
                s_ap_reason = "SWITCH HELD";
                xEventGroupSetBits(s_evt, BIT_BUTTON);
            }
        }
    }
}

/* ====================== main control task ====================== */
static void control_task(void *arg)
{
    if (s_saved) {
        boot_connect();
    } else {
        banner("POWER-ON: no hotspot saved -> AP MODE (LED VIOLET)");
        s_hs_failed = false;
        led_refresh();
        ESP_LOGI(TAG, "[AP] WiFi name : %s   password : %s   page: http://" AP_IP_STR, AP_SSID, AP_PASS);
        status_request();
    }

    for (;;) {
        EventBits_t b = xEventGroupWaitBits(s_evt, BIT_BUTTON | BIT_CREDS | BIT_STA_LOST,
                                            pdFALSE, pdFALSE, pdMS_TO_TICKS(500));
        if (b & BIT_BUTTON)   { handle_button();    continue; }
        if (b & BIT_CREDS)    { handle_new_creds(); continue; }
        if (b & BIT_STA_LOST) { handle_lost();      continue; }

        time_sync_poll();

        if (s_saved && s_hs == HS_IDLE && s_hs_failed && now_ms() >= s_next_check_ms) {
            hotspot_retry();
        }
    }
}

/* ====================== WiFi init ====================== */
static void wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_ap_netif  = esp_netif_create_default_wifi_ap();
    s_sta_netif = esp_netif_create_default_wifi_sta();

    /* captive portal: DHCP offers the device itself as DNS server */
    esp_netif_dns_info_t dns = { 0 };
    dns.ip.type = ESP_IPADDR_TYPE_V4;
    dns.ip.u_addr.ip4.addr = esp_ip4addr_aton(AP_IP_STR);
    uint8_t offer = OFFER_DNS;
    esp_netif_dhcps_stop(s_ap_netif);
    esp_netif_set_dns_info(s_ap_netif, ESP_NETIF_DNS_MAIN, &dns);
    esp_netif_dhcps_option(s_ap_netif, ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER, &offer, sizeof(offer));
    esp_netif_dhcps_start(s_ap_netif);

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t ap = { 0 };
    strlcpy((char *)ap.ap.ssid, AP_SSID, sizeof(ap.ap.ssid));
    strlcpy((char *)ap.ap.password, AP_PASS, sizeof(ap.ap.password));
    ap.ap.ssid_len = strlen(AP_SSID);
    ap.ap.channel = 1;
    ap.ap.max_connection = 4;
    ap.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_LOGI(TAG, "[AP] %s started (always ON)", AP_SSID);
}

/* ====================== app_main ====================== */
void app_main(void)
{
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);

    s_evt = xEventGroupCreate();
    led_init();
    led_raw(COL_VIOLET);
    time_init();

    seed_defaults();
    load_creds();

    wifi_start();
    http_start();
    dns_start();

    xTaskCreate(led_task,     "led",     2048, NULL, 3, NULL);
    xTaskCreate(button_task,  "button",  3072, NULL, 5, NULL);
    xTaskCreate(cmd_task,     "cmd",     3072, NULL, 3, NULL);
    xTaskCreate(clock_task,   "clock",   3072, NULL, 2, NULL);
    xTaskCreate(status_task,  "status",  4096, NULL, 2, NULL);
    xTaskCreate(control_task, "control", 6144, NULL, 4, NULL);
}