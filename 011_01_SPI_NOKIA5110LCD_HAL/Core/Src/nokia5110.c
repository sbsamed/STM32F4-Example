/*
 * nokia5110.c
 *
 *  Created on: 13 Aðu 2022
 *      Author: Samed
 */

#include "nokia5110.h"

bool nokia51110_Init(){
	nokia51110_Reset();

	if(!(nokia51110_Write(0x21,0)))
		return false;
	 if(!(nokia51110_Write(0xC0,0)))
			return false;
	  if(!(nokia51110_Write(0x07,0)))
				return false;
	   if(!(nokia51110_Write(0x13,0)))
					return false;
	    if(!(nokia51110_Write(0x20,0)))
						return false;
	     if(!(nokia51110_Write(0x0C,0)))
							return false;
	     return true;

}

void nokia51110_Reset(){
   HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,RESET); //ce pin
   HAL_Delay(100);
   HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,SET);
}

bool nokia51110_Write(uint8_t data,uint8_t mode){

	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,RESET); // ce low

 if(mode=0){ // if send data
	 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,RESET); //dc low

 }else if(mode=1) // if send command
	 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,SET); //dc high
 else{
	 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,SET); //ce high
	 return false;
 }

   HAL_SPI_Transmit_DMA(&hspi2,&data,1);

   return true;


}
void nokia51110_Clear(void){
	for(int i=0;i<504;i++){
		frameBuff[i]=0;
	}
}
bool nokia51110_Update(void){

	//move to x=0
	if(!(nokia51110_Write(0x80,0)))
		return false;
	//move to y=0
		if(!(nokia51110_Write(0x40,0)))
		return false;
		if(!(nokia51110_BufferWrite(&frameBuff,504)))
		return false;

}
bool nokia51110_BufferWrite(uint8_t* data,uint8_t size){
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,RESET); // ce low
	 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,SET); //dc high

	 HAL_SPI_Transmit_DMA(&hspi2,data,size);
 return true;
}
void  nokia5110_SetPixel(uint8_t x,uint8_t y, bool set){
	uint8_t Bi;
	uint8_t By;


	By=(y/8)*84+x;
	Bi=(y % 8);

	if(set){
		frameBuff[By] |= (1<<Bi);

	}else{
		frameBuff[By] &= (1<<Bi);
	}







}
