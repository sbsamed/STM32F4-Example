#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

GPIO_InitTypeDef GPIO_InitStruct;
ADC_InitTypeDef ADC_InitStruct;
ADC_CommonInitTypeDef  ADC_CommonInitStruct;
DAC_InitTypeDef  DAC_InitStruct;

void GPIO_Config(){

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;

	GPIO_Init(GPIOA,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_4;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;

	GPIO_Init(GPIOA,&GPIO_InitStruct);


}
void ADC_Config(){

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);

	ADC_CommonInitStruct.ADC_Mode=ADC_Mode_Independent;
	ADC_CommonInitStruct.ADC_Prescaler=ADC_Prescaler_Div4;
	ADC_CommonInit(&ADC_CommonInitStruct);

	ADC_InitStruct.ADC_Resolution=ADC_Resolution_12b;
	ADC_Init(ADC1,&ADC_InitStruct);

	ADC_Cmd(ADC1,ENABLE);
}
void  DAC1_Config(){

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,ENABLE);

	DAC_InitStruct.DAC_OutputBuffer=DAC_OutputBuffer_Enable;
	DAC_InitStruct.DAC_Trigger=DAC_Trigger_None;
	DAC_InitStruct.DAC_WaveGeneration=DAC_WaveGeneration_None;

	DAC_Init(DAC_Channel_1,&DAC_InitStruct);

	DAC_Cmd(DAC_Channel_1,ENABLE);


}

uint16_t Read_Adc(){

	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_56Cycles);

	ADC_SoftwareStartConv(ADC1);

	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);

	return ADC_GetConversionValue(ADC1);
}
uint16_t adcvalue;
int main(void)
{


 GPIO_Config();
 ADC_Config();
 DAC1_Config();

  while (1)
  {
	  adcvalue=Read_Adc();
	  DAC_SetChannel1Data(DAC_Align_12b_R,adcvalue);
  }
}



void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){

  return;
}


uint16_t EVAL_AUDIO_GetSampleCallBack(void){

  return -1;
}
