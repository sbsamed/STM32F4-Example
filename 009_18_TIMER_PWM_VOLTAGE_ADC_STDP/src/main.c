#include "stm32f4xx.h"
#include "stm32f4_discovery.h"


GPIO_InitTypeDef GPIO_InitStruct;
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
TIM_OCInitTypeDef TIM_OCInitStruct;
ADC_InitTypeDef ADC_InitStruct;
ADC_CommonInitTypeDef  ADC_CommonInitStruct;

uint32_t adcValue,PwmValue;

void GPIO_Config(){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);


	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);

	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin= GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL,
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;

	GPIO_Init(GPIOD,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin= GPIO_Pin_0 ;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL,
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;

	GPIO_Init(GPIOA,&GPIO_InitStruct);

 }

void ADC_Config(){

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
////
	ADC_CommonInitStruct.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStruct.ADC_Mode=ADC_Mode_Independent;
	ADC_CommonInitStruct.ADC_Prescaler=ADC_Prescaler_Div4;

	ADC_CommonInit(&ADC_CommonInitStruct);
////
	ADC_InitStruct.ADC_Resolution=ADC_Resolution_12b;

	ADC_Init(ADC1,&ADC_InitStruct);

	ADC_Cmd(ADC1,ENABLE);
}

uint16_t READ_ADC(){

	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_56Cycles);

	ADC_SoftwareStartConv(ADC1);

	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);


	return ADC_GetConversionValue(ADC1);
}

void TIM_Config(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);

	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period=19999;
	TIM_TimeBaseInitStruct.TIM_Prescaler=83;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;

	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStruct);

	TIM_Cmd(TIM4,ENABLE);

	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState=ENABLE;
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;


}


uint32_t map(uint32_t A,uint32_t B,uint32_t C,uint32_t D,uint32_t E){

	return (A * E) / C;
}



int main(void)
{
	GPIO_Config();
	ADC_Config();
	TIM_Config();


  while (1)
  {
	  adcValue=READ_ADC();


	  PwmValue=map(adcValue,0,4095,0,19999);

	  //pd12
	    TIM_OCInitStruct.TIM_Pulse=PwmValue;
	  	TIM_OC1Init(TIM4,&TIM_OCInitStruct);
	  	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable) ;// tim4 channel1 is enable

	  	//pd13
	  	TIM_OCInitStruct.TIM_Pulse=PwmValue;
		TIM_OC2Init(TIM4,&TIM_OCInitStruct);
		TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable); // tim4 channel2 is enable

		//pd14
		TIM_OCInitStruct.TIM_Pulse=PwmValue;
		TIM_OC3Init(TIM4,&TIM_OCInitStruct);
		TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable) ;// tim4 channel3 is enable

		//pd15
		TIM_OCInitStruct.TIM_Pulse=PwmValue;
		TIM_OC4Init(TIM4,&TIM_OCInitStruct);
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
