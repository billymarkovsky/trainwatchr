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

void app_main(void){

    S31FL3741_init();
    
    S31FL3741_setGlobalCurrent(0x00);
    setAllMatrix();


    while (1) {
        /* fade in LEDs using global current control */
	for(uint16_t i = 0; i < 256; i++)
	{
		S31FL3741_setGlobalCurrent(i);
		vTaskDelay(6/ portTICK_PERIOD_MS);
	}
    
    ESP_LOGI("tag","log");
	vTaskDelay(15000/ portTICK_PERIOD_MS);

	/* fade out LEDs using global current control */
	for(uint16_t i = 255; i > 0; i--)
	{
		S31FL3741_setGlobalCurrent(i);
		vTaskDelay(6/ portTICK_PERIOD_MS);
	}

    }

}

