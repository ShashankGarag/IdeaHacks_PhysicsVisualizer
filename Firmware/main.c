/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

#include "MMA7361L_driver.h"


#define APP_ID_PLACEHOLDER 0
#define PROFILE_NUM 1
#define PROFILE_APP_IDX 0
#define SVC_INST_ID 0

enum {
    ACCEL_IDX_SVC,           // 0: The Service itself
    ACCEL_IDX_CHAR_DECL,     // 1: Characteristic Declaration
    ACCEL_IDX_CHAR_VAL,      // 2: Characteristic Value (Where the X,Y,Z data lives)
    ACCEL_IDX_CHAR_CFG,      // 3: Client Characteristic Configuration (Allows phone to subscribe to updates)
    ACCEL_IDX_NB             // 4: Total number of items
};


uint16_t accel_handle_table[ACCEL_IDX_NB];
uint16_t global_gatts_if = ESP_GATT_IF_NONE;
uint16_t global_conn_id = 0;
bool is_connected = false;

// A standard 16-bit UUID for our custom service and characteristic
static const uint16_t GATTS_SERVICE_UUID_TEST = 0x00FF;
static const uint16_t GATTS_CHAR_UUID_TEST    = 0xFF01;

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t accel_value[12] = {0};
static const uint8_t accel_ccc[2] = {0x00, 0x00}; // Notification config

static const esp_gatts_attr_db_t gatt_db[ACCEL_IDX_NB] =
{

    [ACCEL_IDX_SVC] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST), (uint8_t *)&GATTS_SERVICE_UUID_TEST}},


    [ACCEL_IDX_CHAR_DECL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},


    [ACCEL_IDX_CHAR_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST, ESP_GATT_PERM_READ,
      sizeof(accel_value), sizeof(accel_value), (uint8_t *)accel_value}},


    [ACCEL_IDX_CHAR_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(accel_ccc), (uint8_t *)accel_ccc}},
};

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static const char *CONN_TAG = "CONN_DEMO";
static const char device_name[] = "Accelerometer_Data";

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,  // 20ms
    .adv_int_max = 0x20,  // 20ms
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static uint8_t adv_raw_data[] = {
    0x02, ESP_BLE_AD_TYPE_FLAG, 0x06,
    0x0F, ESP_BLE_AD_TYPE_NAME_CMPL, 'B', 'l', 'u', 'e', 'd', 'r', 'o', 'i', 'd', '_', 'C', 'o', 'n', 'n',
    0x02, ESP_BLE_AD_TYPE_TX_PWR, 0x09,
};

void sensor_task(void *pvParameters) {
    MMA7361L accelerometer = {
        .x_chan = ADC_CHANNEL_3, 
        .y_chan = ADC_CHANNEL_4, 
        .z_chan = ADC_CHANNEL_5  
    };

    MMA7361L_Init (&accelerometer);

    uint8_t notify_data[12];

    while(1) {
        // Read the sensor
        adc_data current_g = read_adc(&accelerometer);

        if (is_connected) {

            memcpy(&notify_data[0], &current_g.voltage_x, 4);
            memcpy(&notify_data[4], &current_g.voltage_y, 4);
            memcpy(&notify_data[8], &current_g.voltage_z, 4);


            esp_ble_gatts_send_indicate(
                global_gatts_if, 
                global_conn_id, 
                accel_handle_table[ACCEL_IDX_CHAR_VAL],
                sizeof(notify_data), 
                notify_data, 
                false
            );
        }

        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

void app_main(void)
{
    esp_err_t ret;

    //initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(CONN_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(CONN_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&cfg);
    if (ret) {
        ESP_LOGE(CONN_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(CONN_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret) {
        ESP_LOGE(CONN_TAG, "%s gap register failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(CONN_TAG, "%s gatts register failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_ble_gatts_app_register(APP_ID_PLACEHOLDER);
    if (ret) {
        ESP_LOGE(CONN_TAG, "%s gatts app register failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_ble_gatt_set_local_mtu(500);
    if (ret) {
        ESP_LOGE(CONN_TAG, "set local  MTU failed, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_set_device_name(device_name);
    if (ret) {
        ESP_LOGE(CONN_TAG, "set device name failed, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_config_adv_data_raw(adv_raw_data, sizeof(adv_raw_data));
    if (ret) {
        ESP_LOGE(CONN_TAG, "config adv data failed, error code = %x", ret);
    }

    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        ESP_LOGI(CONN_TAG, "Advertising data set, status %d", param->adv_data_raw_cmpl.status);
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(CONN_TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
            break;
        }
        ESP_LOGI(CONN_TAG, "Advertising start successfully");
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(CONN_TAG, "Advertising stop failed, status %d", param->adv_stop_cmpl.status);
        }
        ESP_LOGI(CONN_TAG, "Advertising stop successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(CONN_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                    param->update_conn_params.status,
                    param->update_conn_params.conn_int,
                    param->update_conn_params.latency,
                    param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(CONN_TAG, "GATT server register, status %d, app_id %d", param->reg.status, param->reg.app_id);
        global_gatts_if = gatts_if;
        // TELL THE HARDWARE TO BUILD OUR TABLE
        esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, ACCEL_IDX_NB, SVC_INST_ID);
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        // THE HARDWARE FINISHED BUILDING THE TABLE
        ESP_LOGI(CONN_TAG, "The attribute table was created successfully!");
        if (param->add_attr_tab.status != ESP_GATT_OK){
            ESP_LOGE(CONN_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        } else {
            // Save the handles so we know exactly where to send our data
            memcpy(accel_handle_table, param->add_attr_tab.handles, sizeof(accel_handle_table));
            esp_ble_gatts_start_service(accel_handle_table[ACCEL_IDX_SVC]);
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(CONN_TAG, "Phone Connected!");
        global_conn_id = param->connect.conn_id;
        is_connected = true;
        
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        conn_params.latency = 0;
        conn_params.max_int = 0x20;
        conn_params.min_int = 0x10;
        conn_params.timeout = 400;
        esp_ble_gap_update_conn_params(&conn_params);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(CONN_TAG, "Phone Disconnected!");
        is_connected = false;
        esp_ble_gap_start_advertising(&adv_params);
        break;

    default:
        break;
    }
}
