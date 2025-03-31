#include "s31fl3741a.h"
#include "driver/i2c_master.h"
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#define SCL_IO_PIN 26
#define SDA_IO_PIN 25

#define MASTER_FREQUENCY 400000

#define PORT_NUMBER -1
#define LENGTH 48

i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = -1,
    .scl_io_num = 26,
    .sda_io_num = 25,
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
    S31FL3741_setGlobalCurrent(30,U1_dev_handle);
    S31FL3741_setGlobalCurrent(30,U2_dev_handle);
    ESP_LOGI("tag","test");

    while (1) {

    ESP_LOGI("tag","loop started");

	for(uint16_t i = 0; i < 351; i++)
	{
        ESP_LOGI("tag","i");
	    writeLED(i, 100, U1_dev_handle);
	 	vTaskDelay(50/ portTICK_PERIOD_MS);
        writeLED(i, 0, U1_dev_handle);
	}

    ESP_LOGI("tag","end of for loop");
    vTaskDelay(1000/ portTICK_PERIOD_MS);

    for(uint16_t i = 0; i < 351; i++)
	{
        ESP_LOGI("tag","i");
	    writeLED(i, 100, U2_dev_handle);
	 	vTaskDelay(50/ portTICK_PERIOD_MS);
        writeLED(i, 0, U2_dev_handle);
	}

    

     }

}

