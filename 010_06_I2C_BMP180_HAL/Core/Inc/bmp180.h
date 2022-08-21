/*
 * bmp180.h
 *
 *  Created on: 1 Haz 2022
 *      Author: samedB
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

#define BMP180_DEVICE_WRITE_ADDRESS 0xEE
#define BMP180_DEVICE_READ_ADDRESS 0xEF



void BMP180_Init();
void BMP180_GetCalibration();


#endif /* INC_BMP180_H_ */
