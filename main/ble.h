
#ifndef BLE_H
#define BLE_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"

#define PROFILE_A_APP_ID 0

extern bool deviceConnected;
extern QueueHandle_t led_update_queue;
extern esp_ble_adv_params_t adv_params;

struct received_data_t {
    unsigned char red_line[15];
    unsigned char orange_line[10];
    unsigned char blue_line[6];
    unsigned char green_line[26];
};

typedef struct {
    uint8_t *prepare_buf;
    int      prepare_len;
} prepare_type_env_t;

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);


#endif // BLE_H
