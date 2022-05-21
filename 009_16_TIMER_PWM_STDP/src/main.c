
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

GPIO_InitTypeDef GPIO_InitStruct;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;
TIM_OCInitTypeDef TIM_OCInitStrcut;

void GPIO_Config(){

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);

	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);


	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_12 |GPIO_Pin_13 | GPIO_Pin_14 |GPIO_Pin_15;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOD,&GPIO_InitStruct);
}

void TIM_Config(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);

	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period=9999;
	TIM_TimeBaseInitStruct.TIM_Prescaler=8399;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;

	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStruct);

	TIM_Cmd(TIM4,ENABLE);

	TIM_OCInitStrcut.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStrcut.TIM_OutputState=ENABLE;
	TIM_OCInitStrcut.TIM_OCPolarity=TIM_OCPolarity_High;


}

int main(void)
{
	GPIO_Config();
	TIM_Config();

  while (1)
  {
	  //pd12
	    TIM_OCInitStrcut.TIM_Pulse=9999;
	  	TIM_OC1Init(TIM4,&TIM_OCInitStrcut);
	  	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable) ;// tim4 channel1 is enable

	  	//pd13
	    TIM_OCInitStrcut.TIM_Pulse=7499;
		TIM_OC2Init(TIM4,&TIM_OCInitStrcut);
		TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable); // tim4 channel2 is enable

		//pd14
		TIM_OCInitStrcut.TIM_Pulse=4999;
		TIM_OC3Init(TIM4,&TIM_OCInitStrcut);
		TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable) ;// tim4 channel3 is enable

		//pd15
		TIM_OCInitStrcut.TIM_Pulse=2499;
		TIM_OC4Init(TIM4,&TIM_OCInitStrcut);
		TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable); // tim4 channel4 is enable


  }
}



void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  /* TODO, implement your code here */
  return;
}

/*
 * Callback used by stm324xg_eval_audio_codec.c.
 * Refer to stm324xg_eval_audio_codec.h for more info.
 */
uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  /* TODO, implement your code here */
  return -1;
}
