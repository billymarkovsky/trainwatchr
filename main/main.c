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
#include "esp_check.h"

#include "s31fl3741a.h"
#include "station_map.h"
#include "ble.h"

#include "sdkconfig.h"

#define GATTS_TAG "GATTS_DEMO"

#define INPUT_BUTTON 19
#define SCL_IO_PIN 26
#define SDA_IO_PIN 25
#define MASTER_FREQUENCY 400000
#define PORT_NUMBER -1
#define LENGTH 48

#define DEFAULT_BRIGHTNESS 30
#define BRIGHTNESS_OPTIONS_LEN 4
int brightness_options[] = {0, DEFAULT_BRIGHTNESS/2, DEFAULT_BRIGHTNESS, 5*DEFAULT_BRIGHTNESS};


extern bool deviceConnected;
static bool leds_on_flag = true;
static bool power_on_sequence_complete = false;

#define RED_LINE_PWM_BRIGHTNESS 30
#define ORANGE_LINE_PWM_BRIGHTNESS 30
#define BLUE_LINE_PWM_BRIGHTNESS 5
#define GREEN_LINE_PWM_BRIGHTNESS 3

extern QueueHandle_t led_update_queue;

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
        else{
            if(led_tracker){
                clearAllMatrix(U1);
                clearAllMatrix(U2);
                led_tracker = false;
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
}

void button_logic_task(void *p){
    int pinNumber;
    int brightness = 1;


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
                ESP_LOGE( "BUTTON", "Button Press Detected");
                long_press_action_fired = false;
                xTimerStart(long_press_timer, 0);
                ESP_LOGE( "BUTTON", "Oops, timer started");
            
            }
            else{
                ESP_LOGE( "BUTTON", "Press was short");
                xTimerStop(long_press_timer, 0);

                if (!long_press_action_fired) {

                    if(brightness < BRIGHTNESS_OPTIONS_LEN-1){
                        brightness = brightness + 1;
                    }
                    else{
                        brightness = 0;
                    }
                    ESP_LOGE( "BUTTON", "Is the queue broken?");
                    //xQueueReset(led_update_queue);
                    ESP_LOGE( "BUTTON", "No");
                    S31FL3741_setGlobalCurrent(brightness_options[brightness],U1_dev_handle);
                    S31FL3741_setGlobalCurrent(brightness_options[brightness],U2_dev_handle);
                    xQueueReset(led_update_queue);
                }

                else{
                    ESP_LOGE( "BUTTON", "Long Press wooo!");
                    
                    gpio_isr_handler_remove(INPUT_BUTTON);

                    /* Enable wake up from GPIO */
                    if(gpio_wakeup_enable(INPUT_BUTTON, GPIO_INTR_LOW_LEVEL) == NULL) ESP_LOGE( "WAKEUP", "Enable gpio wakeup failed");
                    if(esp_sleep_enable_gpio_wakeup() == NULL)ESP_LOGE( "WAKEUP", "Configure gpio as wakeup source failed");


                    clearAllMatrix(U1_dev_handle);
                    clearAllMatrix(U2_dev_handle);
                    disableLEDDrivers();

                    esp_light_sleep_start();
                    if(gpio_wakeup_enable(INPUT_BUTTON, GPIO_INTR_LOW_LEVEL) == NULL) ESP_LOGE( "WAKEUP", "Disable gpio wakeup failed");
                    xQueueReset(interputQueue);
                    enableLEDDrivers();
                    gpio_isr_handler_add(INPUT_BUTTON, gpio_interrupt_handler, (void *)INPUT_BUTTON);
                    
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
    

    xTaskCreate(mbta_led_task, "mbta_led_task", 8192, &led_task_params, 4, NULL);
    xTaskCreate(button_logic_task, "button_logic_task", 2048, NULL, 5, NULL);

    


    return;


}