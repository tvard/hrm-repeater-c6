/*
 * HRM Repeater for ESP32-C6
 *
 * Acts as BLE Central to connect to a real Heart Rate Monitor (e.g. H808S),
 * and simultaneously as BLE Peripheral advertising as "ESP-HRM" so that
 * a PC/phone/watch can read the forwarded heart rate data.
 *
 * Standard BLE Heart Rate Service (0x180D) is used on both sides.
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "HRM_REPEATER";

/* ---- Configuration ---- */
// #define TARGET_HRM_NAME    "H808S"       /* COOSPLO*/
#define TARGET_HRM_NAME    "16821-49"       /* MAGENE */
#define DEVICE_NAME        "ESP-HRM"     /* Name we advertise as */

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
    fields.name = (uint8_t *)DEVICE_NAME;
    fields.name_len = strlen(DEVICE_NAME);
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
        ESP_LOGI(TAG, "Advertising as '%s'...", DEVICE_NAME);
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
        } else {
            ESP_LOGW(TAG, "Peripheral connection failed, re-advertising...");
            start_advertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "PC/phone disconnected, re-advertising...");
        peripheral_conn_handle = BLE_HS_CONN_HANDLE_NONE;
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
    params.passive = 0;            /* Active scan to get names */
    params.filter_duplicates = 1;
    params.itvl = 0x0050;
    params.window = 0x0030;

    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &params,
                          central_gap_event, NULL);
    if (rc == 0) {
        scanning = true;
        ESP_LOGI(TAG, "Scanning for '%s'...", TARGET_HRM_NAME);
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
        if (name[0] && strstr(name, TARGET_HRM_NAME)) {
            ESP_LOGI(TAG, "Found target HRM: '%s' rssi=%d", name, event->disc.rssi);
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
            /* Discover Heart Rate Service */
            ble_gattc_disc_svc_by_uuid(central_conn_handle, &HRM_SVC_UUID.u,
                                       disc_svc_cb, NULL);
        } else {
            ESP_LOGW(TAG, "Failed to connect to HRM: %d, retrying...", event->connect.status);
            central_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            start_scan();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "Real HRM disconnected (reason=%d), re-scanning...",
                 event->disconnect.reason);
        central_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        latest_hrm_len = 0;
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
        ESP_LOGI(TAG, "HR: %d bpm", bpm);

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

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", ret);
        return;
    }

    /* Configure NimBLE host */
    ble_hs_cfg.sync_cb  = ble_app_on_sync;
    ble_hs_cfg.reset_cb = ble_app_on_reset;

    /* Set device name */
    ble_svc_gap_device_name_set(DEVICE_NAME);

    /* Initialize services */
    ble_svc_gap_init();
    ble_svc_gatt_init();

    /* Register our HRM GATT service */
    int rc = ble_gatts_count_cfg(gatt_services);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(gatt_services);
    assert(rc == 0);

    ESP_LOGI(TAG, "GATT services registered, starting host...");
    nimble_port_freertos_init(host_task);
}
