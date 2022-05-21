
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

GPIO_InitTypeDef GPIO_InitStruct;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;
TIM_OCInitTypeDef TIM_OCInitStrcut;

void GPIO_Config(){

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);

	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);



	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_12;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOD,&GPIO_InitStruct);
}

void TIM_Config(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);

	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period=19999 ;
	TIM_TimeBaseInitStruct.TIM_Prescaler=83;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;

	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStruct);

	TIM_Cmd(TIM4,ENABLE);

	TIM_OCInitStrcut.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStrcut.TIM_OutputState=ENABLE;
	TIM_OCInitStrcut.TIM_OCPolarity=TIM_OCPolarity_High;


}
void delay(uint32_t time){

	while(time--);
}

int main(void)
{
	GPIO_Config();
	TIM_Config();

  while (1)
  {
	    TIM_OCInitStrcut.TIM_Pulse=500;
	  	TIM_OC1Init(TIM4,&TIM_OCInitStrcut);
	  	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable) ;// tim4 channel1 is enable
	  	delay(21000000);

		TIM_OCInitStrcut.TIM_Pulse=1500;
		TIM_OC1Init(TIM4,&TIM_OCInitStrcut);
	    TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable) ;// tim4 channel1 is enable
		delay(21000000);

		TIM_OCInitStrcut.TIM_Pulse=2500;
		TIM_OC1Init(TIM4,&TIM_OCInitStrcut);
	    TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable) ;// tim4 channel1 is enable
		delay(21000000);

  }
}

/*
 * Callback used by stm32f4_discovery_audio_codec.c.
 * Refer to stm32f4_discovery_audio_codec.h for more info.
 */
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
