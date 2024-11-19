/*
 * WSEN_TIDS_TEMP_SENSOR.h
 *
 *  Created on: Nov 14, 2024
 *      Author: silvere
 */

#ifndef DRIVERS_WSEN_TIDS_TEMP_SENSOR_H_
#define DRIVERS_WSEN_TIDS_TEMP_SENSOR_H_


#include "global.h"

#define WSEN_TIDS_SDA 					GPIO9 //PB9
#define WSEN_TIDS_SCL 					GPIO8 //PB8

#define WSEN_TIDS_ID_REG 				0x01
#define WSEN_TIDS_T_H_LIMIT_REG 		0x02
#define WSEN_TIDS_T_L_LIMIT_REG 		0x03
#define WSEN_TIDS_CTRL_REG 				0x04
#define WSEN_TIDS_STATUS_REG 			0x05
#define WSEN_TIDS_DATA_T_L_REG  		0x06
#define WSEN_TIDS_DATA_T_H_REG 			0x07
#define WSEN_TIDS_SOFT_RESET_REG 		0x0C

#define WSEN_TIDS_ID_MAN 				0xA0

#define WSEN_TIDS_TEMP_SENSOR_ADDR0 	0x38
#define WSEN_TIDS_TEMP_SENSOR_ADDR1 	0x3F
#define WSEN_TIDS_SW_RESET 				2

/* Available Output data rate */
#define WSEN_TIDS_ODR_25_HZ  			0x0F
#define	WSEN_TIDS_ODR_50_HZ  			0x1F
#define	WSEN_TIDS_ODR_100_HZ 			0x2F
#define WSEN_TIDS_ODR_200_HZ 			0x3F

//#define WSEN_TIDS_FREERUN (1 << 2)
//#define WSEN_TIDS_BDU (1 << 6)

typedef struct {
	uint8_t data_t_h;
	uint8_t data_t_l;
}WSEN_TIDS_TEMP_RAW_DATA_t;


/* Status register */
typedef enum {
	WSEN_TIDS_BUSY= 0,
	WSEN_TIDS_UNDER_TLL,
	WSEN_TIDS_OVER_THL,
	WSEN_TIDS_MAX_STATUS
}WSEN_TIDS_STATUS_t;

/* Control register bits
 * For ODR, see macro on the top
 * */
typedef enum {
	WSEN_TIDS_ONE_SHOT = 0,
	WSEN_TIDS_FREERUN = 2,
	WSEN_TIDS_IF_ADD_INC = 3,
	WSEN_TIDS_BDU = 6,
	WSEN_TIDS_MAX_CTRL = 7
}WSEN_TIDS_CTRL_t;

typedef enum {
	WSEN_TIDS_ERROR = 0,
	WSEN_TIDS_OK
}WSEN_TIDS_RESULT_t;


void wsenTdis_i2c1Setup(void);


WSEN_TIDS_RESULT_t wsenTdis_readDeviceId(void);


/* Set temperature maximum and minimum threshold value
 * Max Threshold Temperature = (T_H_LIMIT - 63) ×0.64 °C
 * Min Threshold Temperature = (T_L_LIMIT - 63) ×0.64 °C
 *
 * 1 		<= threshold <= 255
 * -39.68°C <= threshold <= 122.88°C
 *
 *  */
void wsenTdis_setTempLimit(uint8_t h_limit, uint8_t l_limit);

/* Set sensor into power down mode */
void wsenTdis_powerDownMode(void);
/* Set sensor into single conversion mode */
WSEN_TIDS_RESULT_t wsenTdis_singleMode(void);
/* Set sensor into continuous mode*/
void wsenTdis_continuousMode(uint8_t odr);

void wsenTdis_readTemperature(float *temperature);



#endif /* DRIVERS_WSEN_TIDS_TEMP_SENSOR_H_ */
