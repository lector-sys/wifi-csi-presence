/*
 * WiFi-CSI presence sensor - plain ESP32, active_sta mode.
 *
 * Station joins the home router, sends small UDP packets to the gateway so the
 * router keeps ACKing; CSI is extracted from those received ACK frames. That
 * traffic-generator trick and the CSI config below are adapted from
 * StevenMHernandez/ESP32-CSI-Tool (active_sta + _components/csi_component.h).
 *
 * Unlike the reference tool we run the DSP on-device (csi_dsp.h) and emit a
 * small JSON telemetry packet over UDP, so no USB cable is needed at runtime.
 */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "csi_dsp.h"

#define TELEMETRY_PORT   CONFIG_CSI_TELEMETRY_PORT
#define COMMAND_PORT     CONFIG_CSI_COMMAND_PORT
#define TRAFFIC_PORT     2223
#define PUBLISH_MS       500
#define CALIBRATION_MS   60000

static const char *TAG = "csi-presence";

static EventGroupHandle_t s_wifi_events;
static const int CONNECTED_BIT = BIT0;

static csi_dsp_t         s_dsp;
static SemaphoreHandle_t s_dsp_lock;
static char              s_gateway[16];
static int64_t           s_cal_until_us;   /* 0 = not calibrating */

/* ------------------------------------------------------------------ NVS */

static float baseline_load(void)
{
    nvs_handle_t h;
    if (nvs_open("csi", NVS_READONLY, &h) != ESP_OK) return 0.0f;
    uint32_t raw = 0;
    esp_err_t err = nvs_get_u32(h, "baseline", &raw);
    nvs_close(h);
    if (err != ESP_OK) return 0.0f;
    float f;
    memcpy(&f, &raw, sizeof(f));
    return f;
}

static void baseline_save(float f)
{
    nvs_handle_t h;
    if (nvs_open("csi", NVS_READWRITE, &h) != ESP_OK) return;
    uint32_t raw;
    memcpy(&raw, &f, sizeof(raw));
    nvs_set_u32(h, "baseline", raw);
    nvs_commit(h);
    nvs_close(h);
}

/* ----------------------------------------------------------------- WiFi */

static void event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_events, CONNECTED_BIT);
        ESP_LOGW(TAG, "disconnected, retrying");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *) data;
        esp_ip4addr_ntoa(&e->ip_info.gw, s_gateway, sizeof(s_gateway));
        ESP_LOGI(TAG, "ip=" IPSTR " gw=%s", IP2STR(&e->ip_info.ip), s_gateway);
        xEventGroupSetBits(s_wifi_events, CONNECTED_BIT);
    }
}

static bool wifi_up(void)
{
    return (xEventGroupGetBits(s_wifi_events) & CONNECTED_BIT) != 0;
}

static void wifi_start(void)
{
    s_wifi_events = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &event_handler, NULL, NULL));

    wifi_config_t wc = {0};
    strlcpy((char *) wc.sta.ssid,     CONFIG_CSI_WIFI_SSID,     sizeof(wc.sta.ssid));
    strlcpy((char *) wc.sta.password, CONFIG_CSI_WIFI_PASSWORD, sizeof(wc.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* Power save must stay off or CSI arrives in irregular bursts. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
}

/* ------------------------------------------------------------------ CSI */

static void csi_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf || info->len < 128) return;
    /* Never block the WiFi task: if the publisher holds the lock, drop the frame. */
    if (xSemaphoreTake(s_dsp_lock, 0) != pdTRUE) return;
    csi_dsp_push(&s_dsp, info->buf, info->len);
    xSemaphoreGive(s_dsp_lock);
}

static void csi_start(void)
{
    /* Same field values as ESP32-CSI-Tool's csi_component.h. LLTF is present on
     * every frame, HT-LTF is not, so csi_dsp.h only reads the first 64 pairs. */
    wifi_csi_config_t c = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = true,
        .ltf_merge_en      = true,
        .channel_filter_en = false,
        .manu_scale        = false,
        .shift             = 0,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&c));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(&csi_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

/* ------------------------------------------------- traffic generator task */

/* CSI is only produced by *received* frames. Sending UDP to the gateway makes
 * the router ACK at roughly the same rate, and those ACKs are our CSI clock.
 * Adapted from ESP32-CSI-Tool _components/sockets_component.h. */
static void traffic_task(void *arg)
{
    const char payload = '1';
    TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_CSI_PACKET_RATE);
    if (period < 1) period = 1;

    for (;;) {
        while (!wifi_up() || s_gateway[0] == '\0') vTaskDelay(pdMS_TO_TICKS(500));

        int fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd < 0) { vTaskDelay(pdMS_TO_TICKS(1000)); continue; }

        struct sockaddr_in to = { .sin_family = AF_INET, .sin_port = htons(TRAFFIC_PORT) };
        inet_aton(s_gateway, &to.sin_addr);

        while (wifi_up()) {
            sendto(fd, &payload, 1, 0, (struct sockaddr *) &to, sizeof(to));
            vTaskDelay(period);
        }
        close(fd);
    }
}

/* ------------------------------------------------------- telemetry task */

static void telemetry_task(void *arg)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in to = { .sin_family = AF_INET, .sin_port = htons(TELEMETRY_PORT) };
    inet_aton(CONFIG_CSI_HOST_IP, &to.sin_addr);

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    char json[320];
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(PUBLISH_MS));

        int         score;
        bool        cal, calibrating;
        const char *pat;
        uint32_t    pkts;
        float       mv;

        xSemaphoreTake(s_dsp_lock, portMAX_DELAY);
        csi_dsp_publish(&s_dsp);
        score = csi_dsp_score(&s_dsp);
        pat   = csi_dsp_pattern_name(csi_dsp_pattern(&s_dsp));
        cal   = s_dsp.calibrated;
        pkts  = s_dsp.packets;
        mv    = s_dsp.last_mv;

        calibrating = (s_cal_until_us != 0);
        if (calibrating && esp_timer_get_time() >= s_cal_until_us) {
            float b = csi_dsp_cal_finish(&s_dsp);
            s_cal_until_us = 0;
            calibrating = false;
            if (b > 0.0f) {
                baseline_save(b);
                ESP_LOGI(TAG, "calibrated, baseline=%.3e", b);
            } else {
                ESP_LOGW(TAG, "calibration failed: too few CSI samples, check packet rate");
            }
        }
        xSemaphoreGive(s_dsp_lock);

        int n = snprintf(json, sizeof(json),
            "{\"location\":\"%s\",\"node\":\"%02x%02x%02x\",\"activity_score\":%d,"
            "\"calibrated\":%s,\"calibrating\":%s,\"pattern\":\"%s\","
            "\"packets\":%lu,\"raw_variance\":%.3e,\"timestamp\":%lld}",
            CONFIG_CSI_LOCATION, mac[3], mac[4], mac[5], score,
            cal ? "true" : "false", calibrating ? "true" : "false", pat,
            (unsigned long) pkts, mv, (long long) (esp_timer_get_time() / 1000));

        if (wifi_up()) sendto(fd, json, n, 0, (struct sockaddr *) &to, sizeof(to));
        printf("%s\n", json);      /* serial fallback for the bridge's serial mode */
    }
}

/* --------------------------------------------------------- command task */

static void start_calibration(void)
{
    xSemaphoreTake(s_dsp_lock, portMAX_DELAY);
    csi_dsp_cal_start(&s_dsp);
    s_cal_until_us = esp_timer_get_time() + (int64_t) CALIBRATION_MS * 1000;
    xSemaphoreGive(s_dsp_lock);
    ESP_LOGI(TAG, "calibrating for %d s - keep the room still", CALIBRATION_MS / 1000);
}

/* Listens for "CAL" from the bridge so calibration can be started from the
 * dashboard instead of a serial console. */
static void command_task(void *arg)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in me = {
        .sin_family      = AF_INET,
        .sin_port        = htons(COMMAND_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (fd < 0 || bind(fd, (struct sockaddr *) &me, sizeof(me)) < 0) {
        ESP_LOGE(TAG, "command socket failed");
        vTaskDelete(NULL);
        return;
    }

    char buf[16];
    for (;;) {
        int n = recvfrom(fd, buf, sizeof(buf) - 1, 0, NULL, NULL);
        if (n <= 0) continue;
        buf[n] = '\0';
        if (strncmp(buf, "CAL", 3) == 0) start_calibration();
    }
}

/* ----------------------------------------------------------------- main */

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    s_dsp_lock = xSemaphoreCreateMutex();
    float baseline = baseline_load();
    csi_dsp_init(&s_dsp, baseline);
    ESP_LOGI(TAG, "stored baseline: %.3e (0 = none, will auto-calibrate)", baseline);

    wifi_start();
    csi_start();

    xTaskCreate(traffic_task,   "traffic",   4096, NULL, 5, NULL);
    xTaskCreate(telemetry_task, "telemetry", 4096, NULL, 4, NULL);
    xTaskCreate(command_task,   "command",   4096, NULL, 4, NULL);

    /* First boot with no baseline: calibrate automatically once connected. */
    if (baseline <= 0.0f) {
        xEventGroupWaitBits(s_wifi_events, CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(5000));   /* let CSI settle before sampling */
        start_calibration();
    }
}
