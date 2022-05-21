#include "stm32f4xx.h"


GPIO_InitTypeDef GPIO_InitStruct;
ADC_InitTypeDef ADC_InitStruct;
ADC_CommonInitTypeDef  ADC_CommonInitStruct;

uint8_t adcValue;

void GPIO_Config(){

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);

	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;

	GPIO_Init(GPIOD,&GPIO_InitStruct);

        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

		GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AN;
		GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Pin=GPIO_Pin_12;
		GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
		GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;

		GPIO_Init(GPIOA,&GPIO_InitStruct);

}

void ADC_Config(){

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);

	ADC_CommonInitStruct.ADC_DMAAccessMode=ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStruct.ADC_Mode=ADC_Mode_Independent;
	ADC_CommonInitStruct.ADC_Prescaler=ADC_Prescaler_Div4;

	ADC_CommonInit(&ADC_CommonInitStruct);

	ADC_InitStruct.ADC_Resolution=ADC_Resolution_8b;

	ADC_Init(ADC1,&ADC_InitStruct);

	ADC_Cmd(ADC1,ENABLE);


}

void READ_ADC(){

	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1,  ADC_SampleTime_56Cycles);

	ADC_SoftwareStartConv(ADC1);

	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);

	adcValue=ADC_GetConversionValue(ADC1);
}

int main(void)
{
	 GPIO_Config();
	 ADC_Config();

  while (1)
  {
	  READ_ADC();
	  if(adcValue<50){
		  GPIO_ResetBits(GPIOD,GPIO_Pin_All);
	  }
	  else if(adcValue<=150){
			  GPIO_SetBits(GPIOD,GPIO_Pin_12|GPIO_Pin_13);
		  }
	  else if(adcValue<=250){
			  GPIO_SetBits(GPIOD,GPIO_Pin_14|GPIO_Pin_15);
		  }
	  else if(adcValue<=350){
	  			  GPIO_SetBits(GPIOD,GPIO_Pin_All);
	  		  }
  }
}



void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){

  return;
}


uint16_t EVAL_AUDIO_GetSampleCallBack(void){

  return -1;
}
