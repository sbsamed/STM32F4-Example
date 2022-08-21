/*
 * max6675k.c
 *
 *  Created on: 20 Ağu 2022
 *      Author: Samed
 */

#include "max6675k.h"


uint8_t MAX6675_RedRegister(uint16_t *reg){

	HAL_StatusTypeDef hal_Status=HAL_ERROR;
	uint8_t temperature[2]={0,0};

	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,RESET);
	hal_Status=HAL_SPI_Receive(&hspi1,temperature,2,100);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,SET);

	if(hal_Status==HAL_OK){
		if(temperature[1] & 0x04)
				return MAX6675_ERROR;
		*reg=(uint16_t)(temperature[1]>>3);
		*reg|=(uint16_t)(temperature[0]<<5);
		return MAX6675_OK;
	}
	return MAX6675_ERROR;
}
float MAX6675_ReadTemperature(uint8_t reg){
  return reg*0.25;
}
