#include "s31fl3741a.h"
#include "driver/gpio.h"

#define U1_ENABLE 7
#define U2_ENABLE 8


uint32_t S31FL3741_unlock(i2c_master_dev_handle_t dev_handle)
{
	uint32_t result = 0;
	uint8_t i2cBuffer[2];

	// Unlock the command register.
	i2cBuffer[0] = COMMANDREGISTER_WRITELOCK;
	i2cBuffer[1] = UNLOCK_KEY;

	result = i2c_master_transmit(dev_handle, i2cBuffer, 2, TIMEOUT);

	return result;
}

uint32_t S31FL3741_writeRegister(uint8_t reg, uint8_t *buffer, uint16_t length, bool send_stop, i2c_master_dev_handle_t dev_handle)
{
	uint32_t result = 0;
	uint8_t i2cBuffer[2];

	// Unlock the command register.
	result = S31FL3741_unlock(dev_handle);

	// Select register
	i2cBuffer[0] = COMMANDREGISTER;
	i2cBuffer[1] = reg;
	result = i2c_master_transmit(dev_handle, i2cBuffer, 2, TIMEOUT);

	/* write data */
	result = i2c_master_transmit(dev_handle, buffer, length, TIMEOUT);

	return result;
}

uint32_t S31FL3741_readRegister(uint8_t reg, uint8_t addr, uint8_t *buffer, uint16_t length, bool send_stop, i2c_master_dev_handle_t dev_handle)
{
	uint32_t result = 0;
	uint8_t i2cBuffer[2];

	// Unlock the command register.
	result = S31FL3741_unlock(dev_handle);

	// Select register
	i2cBuffer[0] = COMMANDREGISTER;
	i2cBuffer[1] = reg;
	result = i2c_master_transmit(dev_handle, i2cBuffer, 2, TIMEOUT);

	/* set address within page */
	result = i2c_master_transmit(dev_handle, &addr, 1, TIMEOUT);

	/* read requested data */
	result = i2c_master_receive(dev_handle, buffer, 1, TIMEOUT);

	return result;
}

uint32_t S31FL3741_setOperation(uint8_t mode, i2c_master_dev_handle_t dev_handle)
{
	uint32_t result = 0;
	uint8_t i2cBuffer[2];

	// Set Global Current Control Register
	i2cBuffer[0] = REG_CONFIGURATION;
	i2cBuffer[1] = mode;

	result = S31FL3741_writeRegister(PAGE_FUNCTION, i2cBuffer, 2, true, dev_handle);

	return result;
}

uint32_t S31FL3741_setGlobalCurrent(uint8_t value, i2c_master_dev_handle_t dev_handle)
{
	uint32_t result = 0;
	uint8_t i2cBuffer[2];

	// Set Global Current Control Register
	i2cBuffer[0] = REG_GLOBALCURRENT;
	i2cBuffer[1] = value;

	result = S31FL3741_writeRegister(PAGE_FUNCTION, i2cBuffer, 2, true, dev_handle);

	return result;
}

uint32_t writeScaling(uint16_t ledPosition, uint8_t scaleValue, i2c_master_dev_handle_t dev_handle)
{
	uint32_t result = 0;
	uint8_t i2cBuffer[2];
	uint8_t reg;

	if(ledPosition < MAX_LEDS)
	{
		// Select Scaling page register (PG2 or PG3)
		reg = (ledPosition < PAGE_ZERO_BOUNDARY ? PAGE_SCALING_0 : PAGE_SCALING_1);
		/* prepare data ... first byte sets address within page, second byte contains data to be written */
		i2cBuffer[0] = (uint8_t) (ledPosition < PAGE_ZERO_BOUNDARY ? ledPosition : ledPosition - PAGE_ZERO_BOUNDARY);
		i2cBuffer[1] = scaleValue;
		result = S31FL3741_writeRegister(reg, i2cBuffer, 2, true, dev_handle);
	}
	else result = 0xFFFF;

	return result;
}

uint32_t readScaling(uint16_t ledPosition, uint8_t *scaleValue, i2c_master_dev_handle_t dev_handle)
{
	uint32_t result = 0;
	uint8_t reg, addr;

	if(ledPosition < MAX_LEDS)
	{
		// Select Scaling page register (PG2 or PG3)
		reg = (ledPosition < PAGE_ZERO_BOUNDARY ? PAGE_SCALING_0 : PAGE_SCALING_1);
		// Set address for requested LED
		addr = (uint8_t) (ledPosition < PAGE_ZERO_BOUNDARY ? ledPosition : ledPosition - PAGE_ZERO_BOUNDARY);
		// read device
		result = S31FL3741_readRegister(reg, addr, scaleValue, 1, true, dev_handle);
	}
	else result = 0xFFFF;

	return result;
}

// Expects scaling buffer (max size = MAX_LEDS). Starting address of each page is automatically set to zero.
uint32_t writeGlobalScaling(uint8_t *buffer, uint16_t length, i2c_master_dev_handle_t dev_handle)
{
	uint32_t result = 0;
	uint8_t i2cBuffer[PAGE_ZERO_BOUNDARY + 1];

	if(length <= MAX_LEDS)
	{
		i2cBuffer[0] = 0; /* set starting address to zero */

		/* copy PAGE0 buffer contents to local i2cBuffer */
		for(uint8_t i = 0; i < PAGE_ZERO_BOUNDARY; i++)
		{
			i2cBuffer[i + 1] = buffer[i];
		}
		// select PAGE_SCALING_0 register and send starting address along with requested data
		result = S31FL3741_writeRegister(PAGE_SCALING_0, i2cBuffer, PAGE_ZERO_BOUNDARY + 1, true, dev_handle);

		/* copy PAGE1 buffer contents to local i2cBuffer */
		for(uint8_t i = 0; i < (MAX_LEDS - PAGE_ZERO_BOUNDARY); i++)
		{
			i2cBuffer[i + 1] = buffer[i + PAGE_ZERO_BOUNDARY];
		}
		// select PAGE_SCALING_1 register and send starting address along with requested data
		result = S31FL3741_writeRegister(PAGE_SCALING_1, i2cBuffer, (MAX_LEDS - PAGE_ZERO_BOUNDARY) + 1, true, dev_handle);
	}
	else result = 0xFFFF;

	return result;
}

// Expects pwm buffer (max size = MAX_LEDS). Starting address of each page is automatically set to zero.
uint32_t writeGlobalLED(uint8_t *buffer, uint16_t length, i2c_master_dev_handle_t dev_handle)
{
	uint32_t result = 0;
	uint8_t i2cBuffer[PAGE_ZERO_BOUNDARY + 1];

	if(length <= MAX_LEDS)
	{
		i2cBuffer[0] = 0; /* set starting address to zero */

		/* copy PAGE0 buffer contents to local i2cBuffer */
		for(uint8_t i = 0; i < PAGE_ZERO_BOUNDARY; i++)
		{
			i2cBuffer[i + 1] = buffer[i];
		}
		// select PAGE_PWM0 register and send starting address along with requested data
		result = S31FL3741_writeRegister(PAGE_PWM0, i2cBuffer, PAGE_ZERO_BOUNDARY + 1, true, dev_handle);

		/* copy PAGE1 buffer contents to local i2cBuffer */
		for(uint8_t i = 0; i < (MAX_LEDS - PAGE_ZERO_BOUNDARY); i++)
		{
			i2cBuffer[i + 1] = buffer[i + PAGE_ZERO_BOUNDARY];
		}
		// select PAGE_PWM1 register and send starting address along with requested data
		result = S31FL3741_writeRegister(PAGE_PWM1, i2cBuffer, (MAX_LEDS - PAGE_ZERO_BOUNDARY) + 1, true, dev_handle);
	}
	else result = 0xFFFF;

	return result;
}

uint32_t writeLED(uint16_t ledPosition, uint8_t pwmValue, i2c_master_dev_handle_t dev_handle)
{
	uint32_t result = 0;
	uint8_t i2cBuffer[2];
	uint8_t reg;

	if(ledPosition < MAX_LEDS)
	{
		// Select PWM page register (PG0 or PG1)
		reg = (ledPosition < PAGE_ZERO_BOUNDARY ? PAGE_PWM0 : PAGE_PWM1);
		/* prepare data ... first byte sets address within page, second byte contains data to be written */
		i2cBuffer[0] = (uint8_t) (ledPosition < PAGE_ZERO_BOUNDARY ? ledPosition : ledPosition - PAGE_ZERO_BOUNDARY);
		i2cBuffer[1] = pwmValue;
		result = S31FL3741_writeRegister(reg, i2cBuffer, 2, true, dev_handle);
	}
	else result = 0xFFFF;

	return result;
}

uint32_t readLED(uint16_t ledPosition, uint8_t *pwmValue, i2c_master_dev_handle_t dev_handle)
{
	uint32_t result = 0;
	uint8_t reg, addr;

	if(ledPosition < MAX_LEDS)
	{
		// Select PWM page register (PG0 or PG1)
		reg = (ledPosition < PAGE_ZERO_BOUNDARY ? PAGE_PWM0 : PAGE_PWM1);
		// Set address for requested LED
		addr = (uint8_t) (ledPosition < PAGE_ZERO_BOUNDARY ? ledPosition : ledPosition - PAGE_ZERO_BOUNDARY);
		// read device
		result = S31FL3741_readRegister(reg, addr, pwmValue, 1, true, dev_handle);
	}
	else result = 0xFFFF;

	return result;
}

uint32_t toggleLED(uint16_t ledPosition, i2c_master_dev_handle_t dev_handle)
{
	uint32_t result = 0;
	uint8_t data;

	if(ledPosition < MAX_LEDS)
	{
		result = readLED(ledPosition, &data, dev_handle);
		data = (0 == data ? 0xFF : 0);
		result = writeLED(ledPosition, data, dev_handle);
	}
	else result = 0xFFFF;

	return result;
}

uint32_t S31FL3741_init(i2c_master_dev_handle_t dev_handle)
{
	uint32_t result;
	uint8_t i2cBuffer[2];

	uint8_t ledMatrix[MAX_LEDS];

	result = S31FL3741_setOperation(NORMAL_OPERATION, dev_handle);

	result = S31FL3741_setGlobalCurrent(0xFF, dev_handle);

	// Set Pull up & Down for SWx CSy
	i2cBuffer[0] = REG_PULLDOWNUP;
	i2cBuffer[1] = PULLUP_PULLDOWN;
	result = S31FL3741_writeRegister(PAGE_FUNCTION, i2cBuffer, 2, true, dev_handle);

	// Set PWM Frequency
	i2cBuffer[0] = REG_PWMFREQ;
	i2cBuffer[1] = PWM_FREQUENCY;
	result = S31FL3741_writeRegister(PAGE_FUNCTION, i2cBuffer, 2, true, dev_handle);

	/* set scaling to 0xFF (max current/scaling) */
	for(uint16_t i = 0; i < sizeof(ledMatrix); i++)
	{
		ledMatrix[i] = 0xFF;
	}

	result = writeGlobalScaling(ledMatrix, sizeof(ledMatrix),dev_handle);

	/* extinguish all LEDs */
	clearAllMatrix(dev_handle);


	gpio_set_direction(U1_ENABLE, GPIO_MODE_OUTPUT);
	gpio_set_direction(U2_ENABLE, GPIO_MODE_OUTPUT);

	return result;
}

uint32_t clearAllMatrix(i2c_master_dev_handle_t dev_handle)
{
	uint32_t result;
	uint8_t ledMatrix[MAX_LEDS];

	/* set pwm values to zero (off) */
	for(uint16_t i = 0; i < sizeof(ledMatrix); i++)
	{
		ledMatrix[i] = 0x00;
	}

	result = writeGlobalLED(ledMatrix, sizeof(ledMatrix),dev_handle);

	return result;
}

uint32_t setAllMatrix(i2c_master_dev_handle_t dev_handle)
{
	uint32_t result;
	uint8_t ledMatrix[MAX_LEDS];

	/* set pwm values to requested value */
	for(uint16_t i = 0; i < sizeof(ledMatrix); i++)
	{
		ledMatrix[i] = 0xff;
	}

	result = writeGlobalLED(ledMatrix, sizeof(ledMatrix), dev_handle);

	return result;
}

uint32_t writeAllMatrix(uint8_t pwmValue, i2c_master_dev_handle_t dev_handle)
{
	uint32_t result;
	uint8_t ledMatrix[MAX_LEDS];

	/* set pwm values to requested value */
	for(uint16_t i = 0; i < sizeof(ledMatrix); i++)
	{
		ledMatrix[i] = pwmValue;
	}

	result = writeGlobalLED(ledMatrix, sizeof(ledMatrix), dev_handle);

	return result;
}

void enableLEDDrivers(){
	gpio_set_level(U1_ENABLE, 1);
	gpio_set_level(U2_ENABLE, 1);

}

void disableLEDDrivers(){
	gpio_set_level(U1_ENABLE, 0);
	gpio_set_level(U2_ENABLE, 0);

}