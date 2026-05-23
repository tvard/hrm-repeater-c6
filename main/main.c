/*
 * HRM Repeater for ESP32-C6
 *
 * Acts as BLE Central to connect to a real Heart Rate Monitor,
 * and simultaneously as BLE Peripheral advertising as "ESP-<HRM_NAME>" so that
 * a PC/phone/watch can read the forwarded heart rate data.
 *
 * BOOT button (GPIO9):
 *   Short press: Cycle through predefined HRM targets
 *   Long press (>2s): Enter dynamic pair mode (connect to any HRM found)
 *
 * Standard BLE Heart Rate Service (0x180D) is used on both sides.
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/dis/ble_svc_dis.h"

static const char *TAG = "HRM_REPEATER";

/* ---- Configuration ---- */
#define BOOT_BUTTON_GPIO   9
#define LED_GPIO           8      /* Onboard LED (ESP32-C6-DevKitM-1) */
#define LONG_PRESS_MS      2000   /* Hold >2s for dynamic pair mode */

/* Predefined HRM targets — add your devices here */
static const char *hrm_targets[] = {
    "H808S",       /* COOSPO */
    "16821-49",    /* MAGENE */
};
#define NUM_HRM_TARGETS (sizeof(hrm_targets) / sizeof(hrm_targets[0]))

/* ---- Pairing state ---- */
static int      current_target_idx = 0;
static bool     dynamic_pair_mode = false;
static char     active_hrm_name[32] = {0};  /* Name of currently connected/target HRM */
static char     device_name[32] = "ESP-HRM"; /* Advertised name (dynamic) */

/* ---- BLE UUIDs ---- */
static const ble_uuid16_t HRM_SVC_UUID  = BLE_UUID16_INIT(0x180D);
static const ble_uuid16_t HRM_CHR_UUID  = BLE_UUID16_INIT(0x2A37); /* Heart Rate Measurement */
static const ble_uuid16_t BODY_LOC_UUID = BLE_UUID16_INIT(0x2A38); /* Body Sensor Location  */

/* ---- State ---- */
static uint16_t hrm_chr_val_handle;           /* GATT attr handle for our HRM characteristic */
static uint16_t peripheral_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t central_conn_handle    = BLE_HS_CONN_HANDLE_NONE;
static uint8_t  latest_hrm[20];               /* Last received HRM measurement */
static uint8_t  latest_hrm_len = 0;
static bool     scanning = false;

/* Forward declarations */
static void start_scan(void);
static void start_advertising(void);
static int  central_gap_event(struct ble_gap_event *event, void *arg);
static int  peripheral_gap_event(struct ble_gap_event *event, void *arg);
static void disconnect_and_rescan(void);
static void update_device_name(const char *hrm_name);

/* ========================================================================
 * NVS — persist last connected sensor - NVS in flash
 * ======================================================================== */

#define NVS_NAMESPACE "hrm_cfg"

static void nvs_save_target(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, "target_idx", current_target_idx);
        nvs_set_u8(h, "dyn_mode", dynamic_pair_mode ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
}

static void nvs_load_target(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        int32_t idx = 0;
        uint8_t dyn = 0;
        nvs_get_i32(h, "target_idx", &idx);
        nvs_get_u8(h, "dyn_mode", &dyn);
        nvs_close(h);

        if (idx >= 0 && idx < (int)NUM_HRM_TARGETS) {
            current_target_idx = idx;
        }
        dynamic_pair_mode = (dyn != 0);
        ESP_LOGI(TAG, "NVS loaded: target[%d]='%s' dynamic=%d",
                 current_target_idx, hrm_targets[current_target_idx], dynamic_pair_mode);
    }
}

/* ---- LED state ---- */
typedef enum {
    LED_UNPAIRED,          /* No sensor connection: fast blink (100ms) */
    LED_SENSOR_ONLY,       /* Sensor connected, no receiver: slow blink (500ms) */
    LED_FULLY_CONNECTED,   /* Sensor + receiver: solid ON */
    LED_ERROR,             /* Lost connection / 0 bpm / max bpm: triple flash */
} led_state_t;

static volatile led_state_t led_state = LED_UNPAIRED;

/* ========================================================================
 * Device Name Management
 * ======================================================================== */

static void update_device_name(const char *hrm_name)
{
    if (hrm_name && hrm_name[0]) {
        snprintf(device_name, sizeof(device_name), "ESP-%s", hrm_name);
    } else {
        snprintf(device_name, sizeof(device_name), "ESP-HRM");
    }
    ble_svc_gap_device_name_set(device_name);
    ESP_LOGI(TAG, "Device name: %s", device_name);
}

/* ========================================================================
 * LED Indicator Task (WS2812 addressable RGB on GPIO8)
 * ======================================================================== */

/* WS2812 timing (RMT based) */
static rmt_channel_handle_t led_rmt_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;

static void led_strip_init(void)
{
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = LED_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000, /* 10 MHz → 100ns per tick */
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &led_rmt_chan));

    /* Use bytes encoder with WS2812 bit timings */
    rmt_bytes_encoder_config_t bytes_cfg = {
        .bit0 = { .duration0 = 3, .level0 = 1, .duration1 = 9, .level1 = 0 }, /* ~300ns H, ~900ns L */
        .bit1 = { .duration0 = 9, .level0 = 1, .duration1 = 3, .level1 = 0 }, /* ~900ns H, ~300ns L */
        .flags.msb_first = true,
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&bytes_cfg, &led_encoder));
    ESP_ERROR_CHECK(rmt_enable(led_rmt_chan));
}

/* Set LED color (RGB order) */
static void led_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t rgb[3] = { r, g, b };
    rmt_transmit_config_t tx_config = { .loop_count = 0 };
    rmt_transmit(led_rmt_chan, led_encoder, rgb, sizeof(rgb), &tx_config);
    rmt_tx_wait_all_done(led_rmt_chan, portMAX_DELAY);
}

static void led_off(void) { led_set_color(0, 0, 0); }

static void led_task(void *param)
{
    led_strip_init();

    while (1) {
        switch (led_state) {
        case LED_UNPAIRED:
            /* Fast blink RED: 100ms on, 100ms off */
            led_set_color(15, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            led_off();
            vTaskDelay(pdMS_TO_TICKS(100));
            break;

        case LED_SENSOR_ONLY:
            /* Slow blink BLUE: 200ms on, 1800ms off (low duty cycle) */
            led_set_color(0, 0, 15);
            vTaskDelay(pdMS_TO_TICKS(200));
            led_off();
            vTaskDelay(pdMS_TO_TICKS(1800));
            break;

        case LED_FULLY_CONNECTED:
            /* Single dim green pulse every 3s — barely visible, very low power */
            led_set_color(0, 8, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
            led_off();
            vTaskDelay(pdMS_TO_TICKS(2800));
            break;

        case LED_ERROR:
            /* Triple flash YELLOW */
            for (int i = 0; i < 3; i++) {
                led_set_color(15, 8, 0);
                vTaskDelay(pdMS_TO_TICKS(80));
                led_off();
                vTaskDelay(pdMS_TO_TICKS(80));
            }
            vTaskDelay(pdMS_TO_TICKS(700));
            break;
        }
    }
}

/* Helper to update LED state based on connection status */
static void update_led_state(void)
{
    if (central_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        led_state = LED_UNPAIRED;
    } else if (peripheral_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        led_state = LED_SENSOR_ONLY;
    } else {
        led_state = LED_FULLY_CONNECTED;
    }
    ESP_LOGD(TAG, "LED state=%d (central=%d, periph=%d)",
             led_state, central_conn_handle, peripheral_conn_handle);
}

/* ========================================================================
 * Button Handling (BOOT button on GPIO9)
 * ======================================================================== */

static void button_task(void *param)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    bool was_pressed = false;
    int64_t press_start = 0;

    while (1) {
        bool pressed = (gpio_get_level(BOOT_BUTTON_GPIO) == 0); /* Active low */

        if (pressed && !was_pressed) {
            /* Button just pressed — record time */
            press_start = esp_timer_get_time();
        } else if (!pressed && was_pressed) {
            /* Button released — determine short vs long press */
            int64_t duration_ms = (esp_timer_get_time() - press_start) / 1000;

            if (duration_ms >= LONG_PRESS_MS) {
                /* Long press → toggle dynamic pair mode */
                dynamic_pair_mode = !dynamic_pair_mode;
                if (dynamic_pair_mode) {
                    ESP_LOGW(TAG, ">>> DYNAMIC PAIR MODE — will connect to any HRM <<<");
                    update_device_name("PAIR");
                } else {
                    ESP_LOGI(TAG, ">>> PRESET MODE — target: '%s' <<<",
                             hrm_targets[current_target_idx]);
                    update_device_name(hrm_targets[current_target_idx]);
                }
                disconnect_and_rescan();
                nvs_save_target();
            } else if (duration_ms > 50) {
                /* Short press → cycle to next predefined target */
                dynamic_pair_mode = false;
                current_target_idx = (current_target_idx + 1) % NUM_HRM_TARGETS;
                ESP_LOGI(TAG, ">>> Switched to target [%d/%d]: '%s' <<<",
                         current_target_idx + 1, NUM_HRM_TARGETS,
                         hrm_targets[current_target_idx]);
                update_device_name(hrm_targets[current_target_idx]);
                disconnect_and_rescan();
                nvs_save_target();
            }
        }

        was_pressed = pressed;
        vTaskDelay(pdMS_TO_TICKS(20)); /* 20ms polling */
    }
}

static void disconnect_and_rescan(void)
{
    /* Disconnect from current HRM if connected */
    if (central_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(central_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        /* The disconnect event handler will restart scanning */
    } else {
        /* Cancel any ongoing scan and restart */
        if (scanning) {
            ble_gap_disc_cancel();
            scanning = false;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
        start_scan();
    }

    /* Restart advertising with new name */
    if (peripheral_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(peripheral_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    } else {
        ble_gap_adv_stop();
        start_advertising();
    }
}

/* ========================================================================
 * GATT Server (Peripheral side) — advertises Heart Rate Service
 * ======================================================================== */

static int
hrm_chr_access(uint16_t conn_handle, uint16_t attr_handle,
               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        if (attr_handle == hrm_chr_val_handle && latest_hrm_len > 0) {
            os_mbuf_append(ctxt->om, latest_hrm, latest_hrm_len);
        } else {
            /* No data yet — return flags=0, bpm=0 */
            uint8_t dummy[2] = {0x00, 0x00};
            os_mbuf_append(ctxt->om, dummy, 2);
        }
    }
    return 0;
}

static int
body_loc_access(uint16_t conn_handle, uint16_t attr_handle,
                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        /* Body Sensor Location: 1 = Chest */
        uint8_t loc = 1;
        os_mbuf_append(ctxt->om, &loc, 1);
    }
    return 0;
}

static const struct ble_gatt_svc_def gatt_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &HRM_SVC_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* Heart Rate Measurement — notify + read */
                .uuid       = &HRM_CHR_UUID.u,
                .access_cb  = hrm_chr_access,
                .val_handle = &hrm_chr_val_handle,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                /* Body Sensor Location — read only */
                .uuid       = &BODY_LOC_UUID.u,
                .access_cb  = body_loc_access,
                .flags      = BLE_GATT_CHR_F_READ,
            },
            { 0 } /* Terminator */
        },
    },
    { 0 } /* Terminator */
};

/* ========================================================================
 * Advertising (Peripheral side)
 * ======================================================================== */

static void start_advertising(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    fields.uuids16 = (ble_uuid16_t[]) { HRM_SVC_UUID };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: rc=%d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;  /* Connectable */
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;  /* General discoverable */

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, peripheral_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising as '%s'...", device_name);
    }
}

static int
peripheral_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            peripheral_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "PC/phone connected (handle=%d)", peripheral_conn_handle);
            update_led_state();
        } else {
            ESP_LOGW(TAG, "Peripheral connection failed, re-advertising...");
            start_advertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "PC/phone disconnected, re-advertising...");
        peripheral_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        update_led_state();
        start_advertising();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "PC/phone subscribed to HR notifications (cur_notify=%d)",
                 event->subscribe.cur_notify);
        break;

    default:
        break;
    }
    return 0;
}

/* ========================================================================
 * Scanning & Connection (Central side) — finds and connects to real HRM
 * ======================================================================== */

static int
parse_adv_name(const uint8_t *data, uint8_t len, char *out, size_t out_sz)
{
    struct ble_hs_adv_fields fields;
    if (ble_hs_adv_parse_fields(&fields, data, len) != 0) return 0;
    if (fields.name && fields.name_len > 0) {
        size_t copy = fields.name_len < out_sz - 1 ? fields.name_len : out_sz - 1;
        memcpy(out, fields.name, copy);
        out[copy] = '\0';
        return (int)copy;
    }
    return 0;
}

static void start_scan(void)
{
    if (scanning) return;

    struct ble_gap_disc_params params;
    memset(&params, 0, sizeof(params));
    /* Passive scan in preset mode (we know the name, no need to send scan requests).
     * Active scan in dynamic mode to fetch names from scan response packets. */
    params.passive = dynamic_pair_mode ? 0 : 1;
    params.filter_duplicates = 1;
    /* Low duty cycle: scan 11.25ms every 90ms (~12.5%) to save power.
     * Units are 0.625ms ticks: itvl=144 (90ms), window=18 (11.25ms) */
    params.itvl = 144;
    params.window = 18;

    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &params,
                          central_gap_event, NULL);
    if (rc == 0) {
        scanning = true;
        if (dynamic_pair_mode) {
            ESP_LOGI(TAG, "Scanning for ANY HRM (dynamic pair)...");
        } else {
            ESP_LOGI(TAG, "Scanning for '%s'...", hrm_targets[current_target_idx]);
        }
    } else {
        ESP_LOGE(TAG, "ble_gap_disc failed: rc=%d", rc);
    }
}

static int
subscribe_hrm_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                 struct ble_gatt_attr *attr, void *arg)
{
    if (error->status == 0) {
        ESP_LOGI(TAG, "Subscribed to real HRM notifications!");
    } else {
        ESP_LOGE(TAG, "Subscribe failed: status=%d", error->status);
    }
    return 0;
}

static int
disc_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
            const struct ble_gatt_chr *chr, void *arg)
{
    if (error->status == 0 && chr != NULL) {
        ESP_LOGI(TAG, "Found HRM Measurement characteristic (val_handle=%d)", chr->val_handle);
        /* Subscribe: write 0x0001 to CCCD (handle + 1) */
        uint8_t notify_enable[2] = {0x01, 0x00};
        ble_gattc_write_flat(conn_handle, chr->val_handle + 1,
                             notify_enable, sizeof(notify_enable),
                             subscribe_hrm_cb, NULL);
    } else if (error->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "HRM characteristic discovery complete");
    } else {
        ESP_LOGE(TAG, "Characteristic discovery error: %d", error->status);
    }
    return 0;
}

static int
disc_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
            const struct ble_gatt_svc *svc, void *arg)
{
    if (error->status == 0 && svc != NULL) {
        ESP_LOGI(TAG, "Found HRM Service (start=%d end=%d)", svc->start_handle, svc->end_handle);
        /* Discover HRM Measurement characteristic within this service */
        ble_gattc_disc_chrs_by_uuid(conn_handle, svc->start_handle, svc->end_handle,
                                    &HRM_CHR_UUID.u, disc_chr_cb, NULL);
    } else if (error->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "Service discovery complete");
    } else {
        ESP_LOGE(TAG, "Service discovery error: %d", error->status);
    }
    return 0;
}

static int
central_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        char name[32] = {0};
        parse_adv_name(event->disc.data, event->disc.length_data, name, sizeof(name));

        bool match = false;
        if (dynamic_pair_mode) {
            /* In dynamic mode: accept any device that has a name AND either
             * advertises HRM UUID or matches any entry in predefined list.
             * As fallback, accept any named device not starting with "ESP-"
             * (to avoid connecting to ourselves or other repeaters). */
            if (name[0] && strncmp(name, "ESP-", 4) != 0) {
                /* First check if it advertises HRM service UUID */
                struct ble_hs_adv_fields adv_fields;
                if (ble_hs_adv_parse_fields(&adv_fields, event->disc.data,
                                            event->disc.length_data) == 0) {
                    for (int i = 0; i < adv_fields.num_uuids16; i++) {
                        if (adv_fields.uuids16[i].value == 0x180D) {
                            match = true;
                            break;
                        }
                    }
                }
                /* Also match if name matches any known HRM target */
                if (!match) {
                    for (int i = 0; i < NUM_HRM_TARGETS; i++) {
                        if (strstr(name, hrm_targets[i])) {
                            match = true;
                            break;
                        }
                    }
                }
            }
        } else {
            /* Preset mode — match substring */
            if (name[0] && strstr(name, hrm_targets[current_target_idx])) {
                match = true;
            }
        }

        if (match) {
            ESP_LOGI(TAG, "Found target HRM: '%s' rssi=%d", name, event->disc.rssi);
            strncpy(active_hrm_name, name, sizeof(active_hrm_name) - 1);
            /* Stop scanning and connect */
            ble_gap_disc_cancel();
            scanning = false;
            int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &event->disc.addr,
                                     5000, NULL, central_gap_event, NULL);
            if (rc != 0) {
                ESP_LOGE(TAG, "ble_gap_connect failed: rc=%d", rc);
                start_scan();
            }
        }
        break;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGI(TAG, "Scan complete");
        scanning = false;
        if (central_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            start_scan();
        }
        break;

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            central_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Connected to real HRM (handle=%d)", central_conn_handle);
            update_led_state();
            /* Update advertised name to reflect actual connected sensor */
            update_device_name(active_hrm_name);
            /* Restart advertising with new name */
            ble_gap_adv_stop();
            start_advertising();
            /* Discover Heart Rate Service */
            ble_gattc_disc_svc_by_uuid(central_conn_handle, &HRM_SVC_UUID.u,
                                       disc_svc_cb, NULL);
        } else {
            ESP_LOGW(TAG, "Failed to connect to HRM: %d, retrying...", event->connect.status);
            central_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            update_led_state();
            start_scan();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "Real HRM disconnected (reason=%d), re-scanning...",
                 event->disconnect.reason);
        central_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        latest_hrm_len = 0;
        led_state = LED_ERROR;
        vTaskDelay(pdMS_TO_TICKS(1000));
        start_scan();
        break;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        /* Received HR data from real HRM — forward to PC */
        uint16_t pkt_len = OS_MBUF_PKTLEN(event->notify_rx.om);
        if (pkt_len > sizeof(latest_hrm)) pkt_len = sizeof(latest_hrm);
        os_mbuf_copydata(event->notify_rx.om, 0, pkt_len, latest_hrm);
        latest_hrm_len = pkt_len;

        /* Parse BPM for logging */
        uint8_t flags = latest_hrm[0];
        uint16_t bpm;
        if (flags & 0x01) {
            bpm = latest_hrm[1] | (latest_hrm[2] << 8);
        } else {
            bpm = latest_hrm[1];
        }
        ESP_LOGI(TAG, "[%s → %s] HR: %d bpm", active_hrm_name, device_name, bpm);

        /* Flag error state for 0 or max BPM */
        if (bpm == 0 || bpm >= 255) {
            led_state = LED_ERROR;
        } else if (led_state == LED_ERROR) {
            /* Recover from error state */
            update_led_state();
        }

        /* Notify connected PC/phone */
        if (peripheral_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ble_gatts_chr_updated(hrm_chr_val_handle);
        }
        break;
    }

    default:
        break;
    }
    return 0;
}

/* ========================================================================
 * NimBLE Host Callbacks & Main
 * ======================================================================== */

static void ble_app_on_sync(void)
{
    ESP_LOGI(TAG, "NimBLE host synced");

    ble_addr_t addr;
    int rc = ble_hs_id_infer_auto(0, &addr.type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: rc=%d", rc);
        return;
    }
    rc = ble_hs_id_copy_addr(addr.type, addr.val, NULL);
    ESP_LOGI(TAG, "BLE addr: %02X:%02X:%02X:%02X:%02X:%02X",
             addr.val[5], addr.val[4], addr.val[3],
             addr.val[2], addr.val[1], addr.val[0]);

    /* Start advertising so PC can find us */
    start_advertising();

    /* Small delay, then start scanning for the real HRM */
    vTaskDelay(pdMS_TO_TICKS(200));
    start_scan();
}

static void ble_app_on_reset(int reason)
{
    ESP_LOGE(TAG, "NimBLE host reset, reason=%d", reason);
}

static void host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== HRM Repeater starting ===");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Load last used target from flash */
    nvs_load_target();

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", ret);
        return;
    }

    /* Configure NimBLE host */
    ble_hs_cfg.sync_cb  = ble_app_on_sync;
    ble_hs_cfg.reset_cb = ble_app_on_reset;

    /* Set device name */
    update_device_name(hrm_targets[current_target_idx]);
    ble_svc_gap_device_name_set(device_name);

    /* Initialize services */
    ble_svc_gap_init();
    ble_svc_gatt_init();

    /* Device Information Service (DIS) — shown in apps like Wahoo */
    ble_svc_dis_init();
    ble_svc_dis_manufacturer_name_set("ESP32-C6");
    ble_svc_dis_model_number_set("HRM-Repeater");
    ble_svc_dis_serial_number_set("1");
    ble_svc_dis_firmware_revision_set("1.0.0");
    ble_svc_dis_hardware_revision_set("ESP32-C6-DevKitM-1");
    ble_svc_dis_software_revision_set("NimBLE");

    /* Register our HRM GATT service */
    int rc = ble_gatts_count_cfg(gatt_services);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(gatt_services);
    assert(rc == 0);

    ESP_LOGI(TAG, "GATT services registered, starting host...");
    nimble_port_freertos_init(host_task);

    /* Start button task for target cycling */
    xTaskCreate(button_task, "button_task", 2048, NULL, 5, NULL);

    /* Start LED indicator task */
    xTaskCreate(led_task, "led_task", 2048, NULL, 3, NULL);
}
