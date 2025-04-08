#include "s31fl3741a.h"
#include "station_map.h"
#include "driver/i2c_master.h"
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/touch_sensor.h"


#define SCL_IO_PIN 26
#define SDA_IO_PIN 25

#define MASTER_FREQUENCY 400000

#define PORT_NUMBER -1
#define LENGTH 48


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


void app_main(void){
 
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &U1_dev_cfg, &U1_dev_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &U2_dev_cfg, &U2_dev_handle));

    S31FL3741_init(U2_dev_handle);
    S31FL3741_init(U1_dev_handle);
    
    ESP_LOGI("tag","log");
    S31FL3741_setGlobalCurrent(50,U1_dev_handle);
    S31FL3741_setGlobalCurrent(50,U2_dev_handle);
    ESP_LOGI("tag","test");

    int red_line_size = 116;
    int blue_line_size = 48;
    int orange_line_size = 78;
    int j;
    while (1) {
    
        for(int i = 0; i<red_line_size;i++ ){
            setStation(red_line_stations[i],30,U1_dev_handle,U2_dev_handle);
            vTaskDelay(25/ portTICK_PERIOD_MS);
        }
        for (int i = 0; i<blue_line_size; i++){
            setStation(blue_line_stations[i],30,U1_dev_handle,U2_dev_handle);
            vTaskDelay(25/ portTICK_PERIOD_MS);
        }
        for (int i = 0; i<orange_line_size; i++){
            setStation(orange_line_stations[i],30,U1_dev_handle,U2_dev_handle);
            vTaskDelay(25/ portTICK_PERIOD_MS);
        }
        for (int i = 0; i<main_green_len; i++){
            setStation(main_green_line_stations[i],30,U1_dev_handle,U2_dev_handle);
            vTaskDelay(25/ portTICK_PERIOD_MS);
        }

        j = 0;

        while(j<d_green_len){
            if(j<b_green_len){setStation(b_green_line_stations[j],30,U1_dev_handle,U2_dev_handle);};
            if(j<c_green_len){setStation(c_green_line_stations[j],30,U1_dev_handle,U2_dev_handle);};
            if(j<d_green_len){setStation(d_green_line_stations[j],30,U1_dev_handle,U2_dev_handle);};
            if(j<e_green_len){setStation(e_green_line_stations[j],30,U1_dev_handle,U2_dev_handle);};
            vTaskDelay(25/ portTICK_PERIOD_MS);
            j++;
        }
        
        vTaskDelay(1000/ portTICK_PERIOD_MS);

        clearAllMatrix(U1_dev_handle);
        clearAllMatrix(U2_dev_handle);
        

    

     }

}

