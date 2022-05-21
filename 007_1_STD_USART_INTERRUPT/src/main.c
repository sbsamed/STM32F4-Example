#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

char str[50];
char message[100];
int i=0;

GPIO_InitTypeDef GPIO_InitStruct;

USART_InitTypeDef USART_InitStruct;

NVIC_InitTypeDef NVIC_InitStruct;


void Delay(uint32_t time)
{
	while(time--);
}

void GPIO_Config()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); //PA2-TX
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); //PA3-RX

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 ;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void USART_Config()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	USART_InitStruct.USART_BaudRate = 9600;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx ;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;

	USART_Init(USART2, &USART_InitStruct);

	USART_Cmd(USART2, ENABLE);

	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//MESAJ GELINCE INTERRUPA GIR
}

void NVIC_Config(){

	NVIC_InitStruct.NVIC_IRQChannel=USART2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;

	NVIC_Init(&NVIC_InitStruct);

}

//send message tx-rx
void USART2_IRQHandler(){


	if(USART_GetITStatus(USART2,USART_IT_RXNE)){

		char rx_buffer=USART_ReceiveData(USART2);

		if(rx_buffer != '\n'){
			 message[i]=rx_buffer;
			i++;
		}else{
			message[i] =rx_buffer;
			i=0;
			USART_Puts(USART2,message);
			for(int a=0;a<100;a++){
				message[a]='\0';
			}
		}


	}
}


void USART_Puts(USART_TypeDef* USARTx, volatile char *s)
{
	while(*s)
	{
		while(!(USARTx->SR & 0x00000040));
		USART_SendData(USARTx, *s);
		*s++;
	}
}

int main(void)
{
	GPIO_Config();
	USART_Config();
	NVIC_Config();
  while (1)
  {
	  sprintf(str, "denemev1\n");
	  //USART_Puts(USART2, str);
	  //Delay(8000000);
  }
}

void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size)
{
  /* TODO, implement your code here */
  return;
}

uint16_t EVAL_AUDIO_GetSampleCallBack(void)
{
  /* TODO, implement your code here */
  return -1;
}
