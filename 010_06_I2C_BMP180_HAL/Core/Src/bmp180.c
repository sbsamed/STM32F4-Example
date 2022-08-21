/*
 * bmp180.c
 *
 *  Created on: 1 Haz 2022
 *      Author: samedB
 */

#include "bmp180.h"

void BMP180_Init(){

	if( HAL_I2C_IsDeviceReady(&hi2c1, BMP180_DEVICE_WRITE_ADDRESS,1,100000 )!=HAL_OK ){
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,SET);
	}
}
