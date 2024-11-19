/*
 * WSEN_TIDS_TEMP_SENSOR.c
 *
 *  Created on: Nov 14, 2024
 *      Author: silvere
 */

#include "WSEN_TIDS_TEMP_SENSOR.h"
#include <libopencm3/stm32/i2c.h>
#include "drivers/uart_drv.h"
#include "drivers/timer_drv.h"


void wsenTdis_i2c1Setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_I2C1);

	// Setup SDA and SLC for I2C communication/
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, WSEN_TIDS_SCL);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, WSEN_TIDS_SDA);

	//Setup SDA and SCL pin as alternate function. //
	gpio_set_af(GPIOB, GPIO_AF4, WSEN_TIDS_SCL);
	gpio_set_af(GPIOB, GPIO_AF4, WSEN_TIDS_SDA);

	i2c_set_speed(I2C1, i2c_speed_sm_100k, 2);
	i2c_set_dutycycle(I2C1, I2C_CCR_DUTY);
	i2c_set_speed(I2C1,i2c_speed_sm_100k, 8);
	i2c_peripheral_enable(I2C1);

}


WSEN_TIDS_RESULT_t wsenTdis_readDeviceId(void)
{
	uint8_t data_buffer[4];
	WSEN_TIDS_RESULT_t status = WSEN_TIDS_ERROR;

	sleep_ms(15);
	data_buffer[0] = WSEN_TIDS_ID_REG;

	i2c_transfer7(I2C1, WSEN_TIDS_TEMP_SENSOR_ADDR0, data_buffer, 1, (data_buffer+1), 1);

	if (data_buffer[1] == WSEN_TIDS_ID_MAN)
	{
		status = WSEN_TIDS_OK;
	}
	else
	{
		printf("Wrong device ID\n");
		printf("No communication possible\n");
		status = WSEN_TIDS_ERROR;
	}
	return status;
}

/* Set temperature maximum and minimum threshold value
 * Max Threshold Temperature = (T_H_LIMIT - 63) ×0.64 °C
 * Min Threshold Temperature = (T_L_LIMIT - 63) ×0.64 °C
 *
 * 1 		<= threshold <= 255
 * -39.68°C <= threshold <= 122.88°C
 *
 *  */
void wsenTdis_setTempLimit(uint8_t h_limit, uint8_t l_limit)
{
	uint8_t data_buffer[2];

	if (wsenTdis_readDeviceId() == WSEN_TIDS_OK)
	{
		/* Set maximum Temperature threshold*/
		data_buffer[0] = WSEN_TIDS_T_H_LIMIT_REG;
		data_buffer[1] = h_limit;
		i2c_transfer7(I2C1, WSEN_TIDS_TEMP_SENSOR_ADDR0, data_buffer, 2, NULL, 0);


		/* Set minimum Temperature threshold*/
		data_buffer[0] = WSEN_TIDS_T_L_LIMIT_REG;
		data_buffer[1] = l_limit;
		i2c_transfer7(I2C1, WSEN_TIDS_TEMP_SENSOR_ADDR0, data_buffer, 2, NULL, 0);
	}
}

/* Set sensor into power down mode */
void wsenTdis_powerDownMode(void)
{
	uint8_t data_buffer[2];
	/* This set the FREERUN bit of the CTRL register to 0*/
	data_buffer[0] =  WSEN_TIDS_CTRL_REG;
	i2c_transfer7(I2C1, WSEN_TIDS_TEMP_SENSOR_ADDR0, data_buffer, 1, (data_buffer+1), 1);
	data_buffer[1] &= ~(1 << WSEN_TIDS_FREERUN);
	i2c_transfer7(I2C1, WSEN_TIDS_TEMP_SENSOR_ADDR0, data_buffer, 2, NULL, 0);
}

/* Set sensor into single conversion mode */
WSEN_TIDS_RESULT_t wsenTdis_singleMode(void)
{
	uint8_t data_buffer[2];
	WSEN_TIDS_STATUS_t status = WSEN_TIDS_BUSY;
	WSEN_TIDS_RESULT_t rt = WSEN_TIDS_ERROR;

	if(wsenTdis_readDeviceId() == WSEN_TIDS_OK)
	{
		/* Enable software reset */
		data_buffer[0] = WSEN_TIDS_SOFT_RESET_REG;
		data_buffer[1] = (1 << WSEN_TIDS_SW_RESET);
		i2c_transfer7(I2C1, WSEN_TIDS_TEMP_SENSOR_ADDR0, data_buffer, 2, NULL, 0);

		/* This disabels the SW reset */
		data_buffer[0] =  WSEN_TIDS_SOFT_RESET_REG;
		i2c_transfer7(I2C1, WSEN_TIDS_TEMP_SENSOR_ADDR0, data_buffer, 1, (data_buffer+1), 1);
		data_buffer[1] &= ~(1 << WSEN_TIDS_SW_RESET);
		i2c_transfer7(I2C1, WSEN_TIDS_TEMP_SENSOR_ADDR0, data_buffer, 2, NULL, 0);


		/* Enable continuous mode  set FREERUN and BDU to 1*/
		data_buffer[0] = WSEN_TIDS_CTRL_REG;
		data_buffer[1] = (1 << WSEN_TIDS_ONE_SHOT);
		i2c_transfer7(I2C1, WSEN_TIDS_TEMP_SENSOR_ADDR0, data_buffer, 2, NULL, 0);

		/* Read sensor status */
		data_buffer[0] =  WSEN_TIDS_STATUS_REG;
		i2c_transfer7(I2C1, WSEN_TIDS_TEMP_SENSOR_ADDR0, data_buffer, 1, (data_buffer+1), 1);

		if(data_buffer[1] == status)
		{
			printf("Single mode activate\n");
			rt = WSEN_TIDS_OK;
		}
		else{
			printf("Device busy\n");
			rt = WSEN_TIDS_ERROR;
		}
	}
	return rt;
}

/* Set sensor into continuous mode*/
void wsenTdis_continuousMode(uint8_t odr)
{
	uint8_t data_buffer[2];
	uint8_t tmp = 0;

	if(wsenTdis_readDeviceId() == WSEN_TIDS_OK)
	{
		/* Enable software reset */
		data_buffer[0] = WSEN_TIDS_SOFT_RESET_REG;
		data_buffer[1] = (1 << WSEN_TIDS_SW_RESET);
		i2c_transfer7(I2C1, WSEN_TIDS_TEMP_SENSOR_ADDR0, data_buffer, 2, NULL, 0);

		/* This disabels the SW reset */
		data_buffer[0] =  WSEN_TIDS_SOFT_RESET_REG;
		i2c_transfer7(I2C1, WSEN_TIDS_TEMP_SENSOR_ADDR0, data_buffer, 1, (data_buffer+1), 1);
		data_buffer[1] &= ~(1 << WSEN_TIDS_SW_RESET);
		i2c_transfer7(I2C1, WSEN_TIDS_TEMP_SENSOR_ADDR0, data_buffer, 2, NULL, 0);


		/* Enable continuous mode  set FREERUN and BDU to 1*/
		data_buffer[0] = WSEN_TIDS_CTRL_REG;
		data_buffer[1] = ((1 << WSEN_TIDS_FREERUN) | (1 << WSEN_TIDS_BDU));
		i2c_transfer7(I2C1, WSEN_TIDS_TEMP_SENSOR_ADDR0, data_buffer, 2, NULL, 0);


		/* SET output data rate */
		data_buffer[0] = WSEN_TIDS_CTRL_REG;
		i2c_transfer7(I2C1, WSEN_TIDS_TEMP_SENSOR_ADDR0, data_buffer, 1, (data_buffer+1), 1);
		tmp = data_buffer[1];
		data_buffer[1] &= odr;
		data_buffer[1] |= tmp;
		i2c_transfer7(I2C1, WSEN_TIDS_TEMP_SENSOR_ADDR0, data_buffer, 2, NULL, 0);
		printf("Continous mode Activated\n");
	}
}

void wsenTdis_readTemperature(float *temperature)
{
	uint8_t data_buffer[2];
	uint8_t temperature_l_data = 0;
	uint8_t temperature_h_data = 0;
	uint16_t raw_temperature = 0;

	/* Read Temperature L data */
	data_buffer[0] = WSEN_TIDS_DATA_T_L_REG;
	i2c_transfer7(I2C1, WSEN_TIDS_TEMP_SENSOR_ADDR0, data_buffer, 1, (data_buffer+1), 1);
	temperature_l_data = data_buffer[1];

	/* Read Temperature H data */
	data_buffer[0] =  WSEN_TIDS_DATA_T_H_REG;
	i2c_transfer7(I2C1, WSEN_TIDS_TEMP_SENSOR_ADDR0, data_buffer, 1, (data_buffer+1), 1);
	temperature_h_data = data_buffer[1];

	raw_temperature = (uint16_t)((temperature_h_data << 8) | temperature_l_data);

	*temperature = raw_temperature / 100.0;
}



