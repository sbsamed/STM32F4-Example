#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

GPIO_InitTypeDef GPIO_InitStruct;
EXTI_InitTypeDef EXTI_InitStruct; //interrup struct
NVIC_InitTypeDef NVIC_InitStruct; // nvic struct


void GPIO_Config(){

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD ,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/*EXTI CLOCK ENABLE*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);

	/*PORTD*/
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_12;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;

	GPIO_Init(GPIOD,&GPIO_InitStruct);
		/*PORTA*/
		GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN;
		GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0;
		GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_DOWN;
		GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;

		GPIO_Init(GPIOA,&GPIO_InitStruct);

}
void EXTI_Config(){

	/*EXTI CONFIG*/
	/*porta 0TH pin set for EXTI*/

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,GPIO_PinSource0);

	EXTI_InitStruct.EXTI_Line=EXTI_Line0;
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;

	EXTI_Init(&EXTI_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel= EXTI0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;

	NVIC_Init(&NVIC_InitStruct);
}
void delay(uint32_t time){
	while(time--);
}

void EXTI0_IRQHandler() //interrupt func.
{
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET){
		GPIO_ResetBits(GPIOD,GPIO_Pin_12);
		delay(168000);
		GPIO_SetBits(GPIOD,GPIO_Pin_12);
		delay(168000);
		EXTI_ClearITPendingBit(EXTI_Line0);
	}


}

int main(void)
{
	GPIO_Config();
	EXTI_Config();

  while (1)
  {
	  GPIO_SetBits(GPIOD,GPIO_Pin_All);
  }
}

void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  return;
}


uint16_t EVAL_AUDIO_GetSampleCallBack(void){

  return -1;
}
