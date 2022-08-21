
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

GPIO_InitTypeDef GPIO_InitStruct;
SPI_InitTypeDef   SPI_InitStruct;
void GPIO_Config(){

	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);

	 GPIO_InitStruct.GPIO_Pin=  GPIO_Pin_12 | GPIO_Pin_13  | GPIO_Pin_14 | GPIO_Pin_15; //you use pin
	 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; //input-output settings
	 GPIO_InitStruct.GPIO_OType=  GPIO_OType_PP; //push pull
	 GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; //pull down or pull up or nopull
	 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	 GPIO_Init(GPIOD,&GPIO_InitStruct);


	 GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
	 GPIO_InitStruct.GPIO_Pin=  GPIO_Pin_3;
	 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	 GPIO_Init(GPIOE,&GPIO_InitStruct);

     GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
     GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
     GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);
     GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
	 GPIO_InitStruct.GPIO_Pin=  GPIO_Pin_5  | GPIO_Pin_6  | GPIO_Pin_7 ;
	 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	 GPIO_Init(GPIOA,&GPIO_InitStruct);

}

void SPI_Config(){

	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);

	 SPI_InitStruct.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_2;
	 SPI_InitStruct.SPI_CPHA=SPI_CPHA_2Edge;
	 SPI_InitStruct.SPI_CPOL=SPI_CPOL_High;
	 SPI_InitStruct.SPI_DataSize=SPI_DataSize_8b;
	 SPI_InitStruct.SPI_Direction=SPI_Direction_2Lines_FullDuplex;
	 SPI_InitStruct.SPI_FirstBit=SPI_FirstBit_MSB;
	 SPI_InitStruct.SPI_Mode=SPI_Mode_Master;
	 SPI_InitStruct.SPI_NSS= SPI_NSS_Soft |  SPI_NSSInternalSoft_Set;

	 SPI_Init(SPI1,&SPI_InitStruct);
	 SPI_Cmd(SPI1,ENABLE);
	 GPIO_SetBits(GPIOE,GPIO_Pin_3);


}
void SPI_Write(uint8_t address,uint8_t data){

	GPIO_ResetBits(GPIOE,GPIO_Pin_3);

	while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE));

	SPI_I2S_SendData(SPI1,address);

	while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXE));

	SPI_I2S_ReceiveData(SPI1);

	while(!SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE));

	SPI_I2S_SendData();


}
void SPI_Read(uint8_t address,uint8_t data){

	GPIO_ResetBits(GPIOE,GPIO_Pin);
}

int main(void)
{
	GPIO_Config();
	SPI_Config();
  while (1)
  {

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
