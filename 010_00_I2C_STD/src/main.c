#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

GPIO_InitTypeDef GPIO_InitStruct;
I2C_InitTypeDef  I2C_InitStruct;

uint8_t m_adress=0x4E;
int i;

void GPIO_Config(){

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//BUTTON
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//I2C

	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;

	GPIO_Init(GPIOA,&GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_I2C1); //SCL PÝN PB6
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_I2C1); //SDA PÝN PB7

	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
		GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
		GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;
		GPIO_Init(GPIOB,&GPIO_InitStruct);
}

void I2C_Config(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);

	I2C_InitStruct.I2C_Ack=ENABLE;
	I2C_InitStruct.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed=400000;
	I2C_InitStruct.I2C_DutyCycle=I2C_DutyCycle_2;
	I2C_InitStruct.I2C_Mode=I2C_Mode_I2C;
	I2C_InitStruct.I2C_OwnAddress1=0x00;

	I2C_Cmd(I2C1,ENABLE);
	I2C_Init(I2C1,ENABLE);

}

void I2C_Write(uint8_t adress, uint8_t data){

	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));  //I2C BAYRAÐI UYGUN MU HAT BOÞ MU

	I2C_GenerateSTART(I2C1,ENABLE);

	while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))); //ben master mod muyum?

	I2C_Send7bitAddress(I2C1,adress,I2C_Direction_Transmitter); //gönderici olarak  geliyorum

	while(!(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))); //gönderici olarak hazýr mýyým

	I2C_SendData(I2C1,data);

	I2C_GenerateSTOP(I2C1,ENABLE);

}
void delay(uint32_t time){
	while(time--);
}
int main(void)
{
	GPIO_Config();
	I2C_Config();

  while (1)
  {
	  if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)){
		  while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0));
		  i++;
		  delay(1680000);
	  }
	  switch(i){
	  case 0:
		  I2C_Write(m_adress,0x00);
		  break;
	  case 1:
	  	I2C_Write(m_adress,0x01);
	    break;
	  case 2:
	 	 I2C_Write(m_adress,0x02);
	 	 break;
	  case 3:
	 	I2C_Write(m_adress,0x03);
	 	 break;
	  default:
		  break;
	  }
  }
}



void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  /* TODO, implement your code here */
  return;
}


uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  /* TODO, implement your code here */
  return -1;
}
