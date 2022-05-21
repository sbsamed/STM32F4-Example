
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

int counter;
GPIO_InitTypeDef GPIO_InitStruct;
void GPIO_Config(){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);//clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);//clock enable


	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;

	GPIO_Init(GPIOE,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
		GPIO_InitStruct.GPIO_Pin= GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
		 |  GPIO_Pin_4 |  GPIO_Pin_5 |  GPIO_Pin_6   ;
		GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
		GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;

		GPIO_Init(GPIOA,&GPIO_InitStruct);

}
void delay(uint32_t time){
	while(time--);

}

int main(void)
{

  while (1)
  {
	  if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0)){
		  while( (GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0)) );
		  delay(1680000);
		  counter++;
	  }
	  switch(counter){
	  case 0:
	  {
		  GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
		              |  GPIO_Pin_4 | GPIO_Pin_6 );
		  GPIO_ResetBits(GPIOA,GPIO_Pin_5);
		  break;
	}
	  case 1:
	  {
	  		  GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
	  		              |  GPIO_Pin_4 | GPIO_Pin_6 );
	  		  GPIO_ResetBits(GPIOA,GPIO_Pin_5);
	  		  break;
	  	}
	  //case 2-3-4-5-6-7-8-9....... default:counter=0;
	  default:
	  {
		  counter=0;

	  }
	  }

  }
}

void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  return;
}


uint16_t EVAL_AUDIO_GetSampleCallBack(void){

  return -1;
}
