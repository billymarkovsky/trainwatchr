#include "s31fl3741a.h"
#include "i2c_master.h"
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SCL_IO_PIN 26
#define SDA_IO_PIN 25

#define MASTER_FREQUENCY 400000

#define PORT_NUMBER -1
#define LENGTH 48

void app_main(void){

    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = PORT_NUMBER,
        .scl_io_num = SCL_IO_PIN,
        .sda_io_num = SDA_IO_PIN,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_device_config_t LED_DRIVER_TEST = {
        .dev_addr_length=7,         /*!< Select the address length of the slave device. */
        .device_address=0x33,                    /*!< I2C device raw address. (The 7/10 bit address without read/write bit) */
        .scl_speed_hz=MASTER_FREQUENCY,                     /*!< I2C SCL line frequency. */
        .scl_wait_us=0,                      /*!< Timeout value. (unit: us). Please note this value should not be so small that it can handle stretch/disturbance properly. If 0 is set, that means use the default reg value*/   
    };

    i2c_master_dev_handle_t led_driver_handle;
}