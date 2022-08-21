/*
 * max6675k.h
 *
 *  Created on: 20 Aðu 2022
 *      Author: Samed
 */

#include "main.h"

#define MAX6675_OK 1
#define MAX6675_ERROR 0

#ifndef INC_MAX6675K_H_
#define INC_MAX6675K_H_

extern SPI_HandleTypeDef hspi1;

uint8_t MAX6675_RedRegister(uint16_t *reg);

float MAX6675_ReadTemperature(uint8_t deviceValue);




#endif /* INC_MAX6675K_H_ */
