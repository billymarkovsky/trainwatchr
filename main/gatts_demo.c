/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
*
* This demo showcases BLE GATT server. It can send adv data, be connected by client.
* Run the gatt_client demo, the client demo will automatically connect to the gatt_server demo.
* Client demo will enable gatt_server's notify after connection. The two devices will then exchange
* data.
*
****************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "driver/rtc_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_sleep.h"

#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "esp_mac.h"

#include "freertos/timers.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"

#include "s31fl3741a.h"
#include "station_map.h"

#include "sdkconfig.h"

#define GATTS_TAG "GATTS_DEMO"

///Declare the static function
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
// static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define GATTS_SERVICE_UUID_TEST_A   0x00FF
#define GATTS_CHAR_UUID_RED_LINE    0xFF01
#define GATTS_CHAR_UUID_BLUE_LINE   0xFE02
#define GATTS_CHAR_UUID_ORANGE_LINE  0xFC04
#define GATTS_CHAR_UUID_green_line 0xFD03
// #define GATTS_DESCR_UUID_TEST_A     0x3333
#define GATTS_NUM_HANDLE_TEST_A     10

static char test_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = "TrainWatchr Server";

#define TEST_MANUFACTURER_DATA_LEN  17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024


#define SCL_IO_PIN 26
#define SDA_IO_PIN 25
#define INPUT_BUTTON 19

#define MASTER_FREQUENCY 400000

#define PORT_NUMBER -1
#define LENGTH 48

#define DEFAULT_BRIGHTNESS 30

static uint8_t char1_str[] = {0x11,0x22};
static uint8_t char2_str[] = {0x34,0x56};
static uint8_t char3_str[] = {0x70,0x81};

struct received_data_t {
    unsigned char red_line[15];
    unsigned char orange_line[10];
    unsigned char blue_line[6];
    unsigned char green_line[26];
};

#define RED_LINE_PWM_BRIGHTNESS 30
#define ORANGE_LINE_PWM_BRIGHTNESS 30
#define BLUE_LINE_PWM_BRIGHTNESS 5
#define GREEN_LINE_PWM_BRIGHTNESS 3

static bool deviceConnected = false;
static bool leds_on_flag = true;
static bool power_on_sequence_complete = false;

static QueueHandle_t led_update_queue;



static esp_gatt_char_prop_t a_property = 0;
// static esp_gatt_char_prop_t b_property = 0;

static esp_attr_value_t gatts_demo_char1_val =
{
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str,
};

static esp_attr_value_t gatts_demo_char2_val =
{
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char2_str),
    .attr_value   = char2_str,
};

static esp_attr_value_t gatts_demo_char3_val =
{
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char3_str),
    .attr_value   = char3_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

#ifdef CONFIG_EXAMPLE_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
    /* Flags */
    0x02, ESP_BLE_AD_TYPE_FLAG, 0x06,               // Length 2, Data Type ESP_BLE_AD_TYPE_FLAG, Data 1 (LE General Discoverable Mode, BR/EDR Not Supported)
    /* TX Power Level */
    0x02, ESP_BLE_AD_TYPE_TX_PWR, 0xEB,             // Length 2, Data Type ESP_BLE_AD_TYPE_TX_PWR, Data 2 (-21)
    /* Complete 16-bit Service UUIDs */
    0x03, ESP_BLE_AD_TYPE_16SRV_CMPL, 0xAB, 0xCD    // Length 3, Data Type ESP_BLE_AD_TYPE_16SRV_CMPL, Data 3 (UUID)
};

static uint8_t raw_scan_rsp_data[] = {
    /* Complete Local Name */
    0x0F, ESP_BLE_AD_TYPE_NAME_CMPL, 'E', 'S', 'P', '_', 'G', 'A', 'T', 'T', 'S', '_', 'D', 'E', 'M', 'O'   // Length 15, Data Type ESP_BLE_AD_TYPE_NAME_CMPL, Data (ESP_GATTS_DEMO)
};
#else

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

// The length of adv data must be less than 31 bytes
//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
//adv data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    //.min_interval = 0x0006,
    //.max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define PROFILE_NUM 2
#define PROFILE_A_APP_ID 0
// #define PROFILE_B_APP_ID 1

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    // esp_bt_uuid_t char_uuid;
    esp_bt_uuid_t red_char_uuid;
    uint16_t red_char_handle;
    esp_bt_uuid_t blue_char_uuid;
    uint16_t blue_char_handle;
    esp_bt_uuid_t orange_char_uuid;
    uint16_t orange_char_handle;
    esp_bt_uuid_t green_char_uuid;
    uint16_t green_char_handle;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
// static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
//     [PROFILE_A_APP_ID] = {
//         .gatts_cb = gatts_profile_a_event_handler,
//         .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
//     }
// };
static struct gatts_profile_inst profile_a = {
    .gatts_cb = gatts_profile_a_event_handler,
    .gatts_if = ESP_GATT_IF_NONE
};

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;
// static prepare_type_env_t b_prepare_write_env;

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
#ifdef CONFIG_SET_RAW_ADV_DATA
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done==0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done==0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#endif
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TAG, "Advertising start successfully");
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed, status %d", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TAG, "Advertising stop successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTS_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                  param->update_conn_params.status,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "Packet length update, status %d, rx %d, tx %d",
                  param->pkt_data_length_cmpl.status,
                  param->pkt_data_length_cmpl.params.rx_len,
                  param->pkt_data_length_cmpl.params.tx_len);
        break;
    default:
        break;
    }
}

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp){
        if (param->write.is_prep) {
            if (param->write.offset > PREPARE_BUF_MAX_SIZE) {
                status = ESP_GATT_INVALID_OFFSET;
            } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                status = ESP_GATT_INVALID_ATTR_LEN;
            }
            if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem");
                    status = ESP_GATT_NO_RESOURCES;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            if (gatt_rsp) {
                gatt_rsp->attr_value.len = param->write.len;
                gatt_rsp->attr_value.handle = param->write.handle;
                gatt_rsp->attr_value.offset = param->write.offset;
                gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
                memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
                esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
                if (response_err != ESP_OK){
                    ESP_LOGE(GATTS_TAG, "Send response error\n");
                }
                free(gatt_rsp);
            } else {
                ESP_LOGE(GATTS_TAG, "malloc failed, no resource to send response error\n");
                status = ESP_GATT_NO_RESOURCES;
            }
            if (status != ESP_GATT_OK){
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;

        }else{
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
        ESP_LOGI(GATTS_TAG, "Dumping hex values of values to write");
        ESP_LOG_BUFFER_HEX(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
        ESP_LOGI(GATTS_TAG, "Finishing LED station dump");
    }else{
        ESP_LOGI(GATTS_TAG,"Prepare write cancel");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {

    static struct received_data_t station_data = {0};  


    ESP_LOGI(GATTS_TAG, "gatt event: %d", event);
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "GATT server register, status %d, app_id %d, gatts_if %d", param->reg.status, param->reg.app_id, gatts_if);
        profile_a.service_id.is_primary = true;
        profile_a.service_id.id.inst_id = 0x00;
        profile_a.service_id.id.uuid.len = ESP_UUID_LEN_16;
        profile_a.service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(test_device_name);
        if (set_dev_name_ret){
            ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
#ifdef CONFIG_EXAMPLE_SET_RAW_ADV_DATA
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        if (raw_adv_ret){
            ESP_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
        adv_config_done |= adv_config_flag;
        esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
        if (raw_scan_ret){
            ESP_LOGE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
        }
        adv_config_done |= scan_rsp_config_flag;
#else
        //config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= adv_config_flag;
        //config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;

#endif
        esp_ble_gatts_create_service(gatts_if, &profile_a.service_id, GATTS_NUM_HANDLE_TEST_A);
        break;
    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(GATTS_TAG, "Characteristic read, conn_id %d, trans_id %" PRIu32 ", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0xde;
        rsp.attr_value.value[1] = 0xed;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(GATTS_TAG, "Characteristic write, conn_id %d, trans_id %" PRIu32 ", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep){
            ESP_LOGI(GATTS_TAG, "value len %d, value ", param->write.len);
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
            if (profile_a.descr_handle == param->write.handle && param->write.len == 2){
                uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                if (descr_value == 0x0001){
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                        ESP_LOGI(GATTS_TAG, "Notification enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i%0xff;
                        }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, profile_a.char_handle,
                                                sizeof(notify_data), notify_data, false);
                    }
                }else if (descr_value == 0x0002){
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                        ESP_LOGI(GATTS_TAG, "Indication enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i%0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, profile_a.char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000){
                    ESP_LOGI(GATTS_TAG, "Notification/Indication disable");
                }else{
                    ESP_LOGE(GATTS_TAG, "Unknown descriptor value");
                    ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
                }

            }
        }
        if (param->write.handle == profile_a.red_char_handle) {
            memcpy(station_data.red_line, param->write.value, param->write.len);
        } else if (param->write.handle == profile_a.blue_char_handle) {
            memcpy(station_data.blue_line, param->write.value, param->write.len);
        } else if (param->write.handle == profile_a.orange_char_handle) {
            memcpy(station_data.orange_line, param->write.value, param->write.len);  
        } else if (param->write.handle == profile_a.green_char_handle) {
            memcpy(station_data.green_line, param->write.value, param->write.len);  
        }
        xQueueSend(led_update_queue,&station_data,1000);
        example_write_event_env(gatts_if, &a_prepare_write_env, param);
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(GATTS_TAG,"Execute write");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&a_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "MTU exchange, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "Service create, status %d, service_handle %d", param->create.status, param->create.service_handle);
        profile_a.service_handle = param->create.service_handle;
        
        // profile_a.char_uuid.len = ESP_UUID_LEN_16;
        // profile_a.char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

        //Define Characteristics
        profile_a.red_char_uuid.len = ESP_UUID_LEN_16;
        profile_a.blue_char_uuid.len = ESP_UUID_LEN_16;
        profile_a.orange_char_uuid.len = ESP_UUID_LEN_16;
        profile_a.green_char_uuid.len = ESP_UUID_LEN_16;
        

        profile_a.red_char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_RED_LINE;
        profile_a.blue_char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_BLUE_LINE;
        profile_a.orange_char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_ORANGE_LINE;
        profile_a.green_char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_green_line;

        esp_ble_gatts_start_service(profile_a.service_handle);
        a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

        esp_err_t add_char_ret;

        add_char_ret = esp_ble_gatts_add_char(profile_a.service_handle, &profile_a.red_char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        a_property,
                                                        &gatts_demo_char1_val, NULL);
        if (add_char_ret){
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
        }

        add_char_ret = esp_ble_gatts_add_char(profile_a.service_handle, &profile_a.blue_char_uuid,
                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                a_property,
                                                &gatts_demo_char2_val, NULL);
        if (add_char_ret){
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
        }

        add_char_ret = esp_ble_gatts_add_char(profile_a.service_handle, &profile_a.orange_char_uuid,
                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                a_property,
                                                &gatts_demo_char3_val, NULL);
        if (add_char_ret){
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
        }
        add_char_ret = esp_ble_gatts_add_char(profile_a.service_handle, &profile_a.green_char_uuid,
                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                a_property,
                                                &gatts_demo_char3_val, NULL);
        if (add_char_ret){
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT: {
        uint16_t length = 0;
        const uint8_t *prf_char;
    
        ESP_LOGI(GATTS_TAG, "Characteristic add, status %d, attr_handle %d, service_handle %d",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
    
        if (param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_RED_LINE) {
            profile_a.red_char_handle = param->add_char.attr_handle;
        } else if (param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_BLUE_LINE) {
            profile_a.blue_char_handle = param->add_char.attr_handle;
        } else if (param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_ORANGE_LINE) {
            profile_a.orange_char_handle = param->add_char.attr_handle;
        } else if (param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_green_line) {
            profile_a.green_char_handle = param->add_char.attr_handle;
        }
        else {
            ESP_LOGW(GATTS_TAG, "Unknown characteristic UUID");
        }
    
        profile_a.descr_uuid.len = ESP_UUID_LEN_16;
        profile_a.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
        if (get_attr_ret == ESP_FAIL) {
            ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
        }
    
        ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x", length);
        for (int i = 0; i < length; i++) {
            ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x", i, prf_char[i]);
        }
    
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(profile_a.service_handle, &profile_a.descr_uuid,
                                                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        if (add_descr_ret) {
            ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
        }
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        profile_a.descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(GATTS_TAG, "Descriptor add, status %d, attr_handle %d, service_handle %d",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "Service start, status %d, service_handle %d",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT: {
        deviceConnected = true;
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
        ESP_LOGI(GATTS_TAG, "Connected, conn_id %u, remote "ESP_BD_ADDR_STR"",
                 param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
        profile_a.conn_id = param->connect.conn_id;
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        deviceConnected = false;
        ESP_LOGI(GATTS_TAG, "Disconnected, remote "ESP_BD_ADDR_STR", reason 0x%02x",
                 ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG, "Confirm receive, status %d, attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK){
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            profile_a.gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {

        if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == profile_a.gatts_if) {
            if (profile_a.gatts_cb) {
                profile_a.gatts_cb(event, gatts_if, param);
            }
        }
        
    } while (0);
}

i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = PORT_NUMBER,
    .scl_io_num = SCL_IO_PIN,
    .sda_io_num = SDA_IO_PIN,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};


i2c_master_bus_handle_t bus_handle;

i2c_device_config_t U2_dev_cfg = {
    .dev_addr_length = 7,
    .device_address = 0x33,
    .scl_speed_hz = 400000,
};

i2c_master_dev_handle_t U2_dev_handle;

i2c_device_config_t U1_dev_cfg = {
    .dev_addr_length = 7,
    .device_address = 0x30,
    .scl_speed_hz = 400000,
};

i2c_master_dev_handle_t U1_dev_handle;

typedef struct {
    i2c_master_dev_handle_t U1;
    i2c_master_dev_handle_t U2;
} led_task_params_t;

void mbta_led_task(void *p) {
    led_task_params_t *params = (led_task_params_t *)p;
    i2c_master_dev_handle_t U1 = params->U1;
    i2c_master_dev_handle_t U2 = params->U2;

    struct received_data_t station_data;


    bool monitor_led = true;
    uint8_t brightness_count = DEFAULT_BRIGHTNESS; 
    bool brightness_count_ascending = false;
    bool led_tracker = false; //variable to track if leds are on, to avoid sending redundant I2C commands
    int j = 0;
    while (1) {
        if(leds_on_flag){
            if(!power_on_sequence_complete){
                for(int i = 0; i<red_line_len;i++ ){
                    setStation(red_line_stations[i],RED_LINE_PWM_BRIGHTNESS,U1,U2);
                    vTaskDelay(20/ portTICK_PERIOD_MS);
                }
                for (int i = 0; i<blue_line_len; i++){
                    setStation(blue_line_stations[i],BLUE_LINE_PWM_BRIGHTNESS,U1,U2);
                    vTaskDelay(20/ portTICK_PERIOD_MS);
                }
                for (int i = 0; i<orange_line_len; i++){
                    setStation(orange_line_stations[i],ORANGE_LINE_PWM_BRIGHTNESS,U1,U2);
                    vTaskDelay(20/ portTICK_PERIOD_MS);
                }
                for (int i = 0; i<main_green_len; i++){
                    setStation(main_green_line_stations[i],GREEN_LINE_PWM_BRIGHTNESS,U1,U2);
                    vTaskDelay(20/ portTICK_PERIOD_MS);
                }

                j = 0;

                while(j<d_green_len){
                    if(j<b_green_len){setStation(b_green_line_stations[j],GREEN_LINE_PWM_BRIGHTNESS,U1,U2);};
                    if(j<c_green_len){setStation(c_green_line_stations[j],GREEN_LINE_PWM_BRIGHTNESS,U1,U2);};
                    if(j<d_green_len){setStation(d_green_line_stations[j],GREEN_LINE_PWM_BRIGHTNESS,U1,U2);};
                    if(j<e_green_len){setStation(e_green_line_stations[j],GREEN_LINE_PWM_BRIGHTNESS,U1,U2);};
                    vTaskDelay(20/ portTICK_PERIOD_MS);
                    j++;
                }
                vTaskDelay(2000/ portTICK_PERIOD_MS);
                clearAllMatrix(U1);
                clearAllMatrix(U2);
                power_on_sequence_complete = true;
            }
            else{
                led_tracker = true;
                if(deviceConnected){
                    if(brightness_count != DEFAULT_BRIGHTNESS){
                        S31FL3741_setGlobalCurrent(DEFAULT_BRIGHTNESS,U1);
                        S31FL3741_setGlobalCurrent(DEFAULT_BRIGHTNESS,U2);
                    }
                    if(xQueueReceive(led_update_queue,&station_data,1000) && leds_on_flag){
                        updateLine(red_line_stations,station_data.red_line,red_line_len,RED_LINE_PWM_BRIGHTNESS, U1, U2);
                        updateLine(blue_line_stations,station_data.blue_line,blue_line_len,BLUE_LINE_PWM_BRIGHTNESS, U1, U2);
                        updateLine(orange_line_stations,station_data.orange_line,orange_line_len,ORANGE_LINE_PWM_BRIGHTNESS, U1, U2);
                        updateLine(all_green_line_stations,station_data.green_line,all_green_len,GREEN_LINE_PWM_BRIGHTNESS, U1, U2);
                    }
                }
                else{
                    S31FL3741_setGlobalCurrent(brightness_count,U1);
                    S31FL3741_setGlobalCurrent(brightness_count,U2);
                    ESP_LOGI("tag", "brightness: %d", brightness_count);
                    if(brightness_count < DEFAULT_BRIGHTNESS && brightness_count_ascending){
                        brightness_count = brightness_count + 1;
                    }
                    else if (brightness_count > 1 && !brightness_count_ascending) {
                        brightness_count = brightness_count - 1;
                        
                    }
                    else{
                        brightness_count_ascending = !brightness_count_ascending;
                    }
                    vTaskDelay(5);
                    
                    
                }
            }
        }
    
        vTaskDelay(30 / portTICK_PERIOD_MS);
    }
}

QueueHandle_t interputQueue;

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    int pinNumber = (int)args;
    xQueueSendFromISR(interputQueue, &pinNumber, NULL);
}

static volatile bool long_press_action_fired = false;

void long_press_timer_callback(TimerHandle_t xTimer) {
    long_press_action_fired = true; // Set the flag

    esp_err_t err = esp_ble_gap_start_advertising(&adv_params);
    if (err != ESP_OK) {
        ESP_LOGE("TAG", "Failed to start advertising: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI("TAG", "Advertising started due to long press.");
    }
}

void button_logic_task(void *p){
    int pinNumber;

    TimerHandle_t long_press_timer = xTimerCreate(
        "LongPressTimer",           // A text name for the timer (for debugging)
        pdMS_TO_TICKS(3000),        // The timer period in ticks
        pdFALSE,                    // Don't auto-reload, make it a one-shot timer
        (void *)0,                  // Timer ID (not used here)
        long_press_timer_callback   // The function to call when the timer expires
    );

    if (long_press_timer == NULL) {
        ESP_LOGE("TAG", "Failed to create the long press timer.");
    }

    
    while(1){
        if (xQueueReceive(interputQueue, &pinNumber, portMAX_DELAY)){

            if(gpio_get_level(INPUT_BUTTON) == 0){ 
                long_press_action_fired = false;
                xTimerStart(long_press_timer, 0);
            
            }
            else{
                xTimerStop(long_press_timer, 0);
                if (!long_press_action_fired) {
                    leds_on_flag = !leds_on_flag;
                    ESP_LOGE("TAG", "LED state changed");
                }
                if (leds_on_flag){
                    enableLEDDrivers();
                    ESP_LOGE("TAG", "LEDs turned on");
                }
                else{
                    clearAllMatrix(U1_dev_handle);
                    clearAllMatrix(U2_dev_handle);
                    disableLEDDrivers();
                    ESP_LOGE("TAG", "LEDs turned off");
                }
            }
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    esp_err_t ret;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &U1_dev_cfg, &U1_dev_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &U2_dev_cfg, &U2_dev_handle));

    enableLEDDrivers();
    S31FL3741_init(U2_dev_handle);
    S31FL3741_init(U1_dev_handle);
    
    
    ESP_LOGI("tag","log");
    S31FL3741_setGlobalCurrent(DEFAULT_BRIGHTNESS,U1_dev_handle);
    S31FL3741_setGlobalCurrent(DEFAULT_BRIGHTNESS,U2_dev_handle);
    ESP_LOGI("tag","test");

    struct received_data_t receive_data_buffer;

    interputQueue = xQueueCreate(10, sizeof(int));
    if (interputQueue == NULL) {
        ESP_LOGE(GATTS_TAG, "Failed to create LED update queue");
        abort();
    }

    esp_rom_gpio_pad_select_gpio(INPUT_BUTTON);
    gpio_reset_pin(INPUT_BUTTON);
    gpio_set_direction(INPUT_BUTTON, GPIO_MODE_INPUT);
    gpio_pullup_en(INPUT_BUTTON);
    gpio_pulldown_dis(INPUT_BUTTON);
    gpio_set_intr_type(INPUT_BUTTON, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(INPUT_BUTTON, gpio_interrupt_handler, (void *)INPUT_BUTTON);

    led_update_queue = xQueueCreate(10, sizeof(receive_data_buffer));
    if (led_update_queue == NULL) {
        ESP_LOGE(GATTS_TAG, "Failed to create LED update queue");
        abort();
    }

    static led_task_params_t led_task_params;
    led_task_params.U1 = U1_dev_handle;
    led_task_params.U2 = U2_dev_handle;



    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    #if CONFIG_EXAMPLE_CI_PIPELINE_ID
    memcpy(test_device_name, esp_bluedroid_get_example_name(), ESP_BLE_ADV_NAME_LEN_MAX);
    #endif

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
    

    xTaskCreate(mbta_led_task, "mbta_led_task", 8192, &led_task_params, 5, NULL);
    xTaskCreate(button_logic_task, "button_logic_task", 2048, NULL, 4, NULL);

    


    return;


}