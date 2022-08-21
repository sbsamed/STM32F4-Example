/*
 * nokia5110.h
 *
 *  Created on: 13 Aðu 2022
 *      Author: Samed
 */

#ifndef INC_NOKIA5110_H_
#define INC_NOKIA5110_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi2_tx;

uint8_t frameBuff[504];

bool nokia51110_Init(void);
void nokia51110_Reset(void);
bool nokia51110_Update(void);
void nokia51110_Clear(void);
bool nokia51110_Write(uint8_t data,uint8_t mode);
bool nokia51110_BufferWrite(uint8_t* data,uint8_t size);
void nokia5110_SetPixel(uint8_t x,uint8_t y, bool set);


#endif /* INC_NOKIA5110_H_ */
