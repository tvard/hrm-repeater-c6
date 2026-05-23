/*
 * Minimal NimBLE scan test for ESP32-C6.
 * All roles enabled, no GATT server, just scan for BLE devices.
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
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "NIMBLE_SCAN_TEST";

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        char addr_str[18];
        const uint8_t *a = event->disc.addr.val;
        snprintf(addr_str, sizeof(addr_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                 a[5], a[4], a[3], a[2], a[1], a[0]);
        ESP_LOGI(TAG, "Found device: %s rssi=%d", addr_str, event->disc.rssi);
        break;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGI(TAG, "Scan complete, restarting...");
        break;
    default:
        break;
    }
    return 0;
}

static void start_scan(void)
{
    struct ble_gap_disc_params params;
    memset(&params, 0, sizeof(params));
    params.passive = 1;
    params.filter_duplicates = 1;
    params.itvl = 0x0050;
    params.window = 0x0030;

    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &params, gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc failed: rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "Scanning started (continuous passive)...");
    }
}

static void ble_app_on_sync(void)
{
    ESP_LOGI(TAG, "NimBLE host synced");

    int rc;
    ble_addr_t addr;
    rc = ble_hs_id_infer_auto(0, &addr.type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: rc=%d", rc);
        return;
    }
    rc = ble_hs_id_copy_addr(addr.type, addr.val, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_copy_addr failed: rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "BLE addr type=%d addr=%02X:%02X:%02X:%02X:%02X:%02X",
             addr.type, addr.val[5], addr.val[4], addr.val[3],
             addr.val[2], addr.val[1], addr.val[0]);

    /* Small delay before starting scan */
    vTaskDelay(pdMS_TO_TICKS(100));
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
    ESP_LOGI(TAG, "app_main started");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Calling nimble_port_init...");
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", ret);
        return;
    }

    /* Configure NimBLE host */
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_hs_cfg.reset_cb = ble_app_on_reset;

    /* Initialize GAP and GATT services (required even if unused) */
    ble_svc_gap_init();
    ble_svc_gatt_init();

    ESP_LOGI(TAG, "Starting NimBLE host task...");
    nimble_port_freertos_init(host_task);
}
