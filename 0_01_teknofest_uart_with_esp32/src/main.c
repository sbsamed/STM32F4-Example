#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include <stdlib.h>


#define bufferSize 100

#define motor1_in1 GPIO_Pin_0
#define motor1_in2 GPIO_Pin_1
#define motor2_in1 GPIO_Pin_2
#define motor2_in2 GPIO_Pin_3



char message[bufferSize];
char pwmPulseUart[10];
int pwmPulse1,pwmPulse2,pwmahmet;
char sendMqttMessage[bufferSize];
short i=0;
uint32_t delayCount;


GPIO_InitTypeDef GPIO_InitStruct;
USART_InitTypeDef USART_InitStruct;
NVIC_InitTypeDef NVIC_InitStruct;

TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
TIM_OCInitTypeDef TIM_OCInitStruct;



void GPIO_Config()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);


	//TIMER PWM PIN CONFIG GPIOD //PIN2,PIN13
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);

	//UART PIN CONFIG GPIOA   //PA2->TX      //PA3->RX
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); //PA2->TX
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); //PA3->RX

	//pwm pin
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin= GPIO_Pin_12 | GPIO_Pin_13 ;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL,
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;

	GPIO_Init(GPIOD,&GPIO_InitStruct);

	//uart pin
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 ;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(GPIOA, &GPIO_InitStruct);

	//motor enable pin
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin= GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL,
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;

	GPIO_Init(GPIOD,&GPIO_InitStruct);



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

	//�F COME MESSAGE GO INTERRUP
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
}

void NVIC_Config(){

	NVIC_InitStruct.NVIC_IRQChannel=USART2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;

	NVIC_Init(&NVIC_InitStruct);

}


//REC�EVE MESSAGE INTERRUPT GPIO->PA3
void USART2_IRQHandler(){

	if(USART_GetITStatus(USART2,USART_IT_RXNE)==SET){


		char rx_buffer=USART_ReceiveData(USART2);

		if(rx_buffer != '\n'){
			 message[i]=rx_buffer;
			i++;
		}else{
			message[i] =rx_buffer;
			i=0;

			USART_Puts(USART2,message);

			uartMessageDebug(message);

			//buffer is clear
			for(int a=0;a<bufferSize;a++){
				message[a]='\0';
			}

		}


	}
	USART_GetITStatus(USART2,USART_IT_RXNE)==RESET;
}
void uartMessageDebug(volatile char *s){

  if( s[0]=='i' && s[1]=='l' && s[2]=='e' && s[3]=='r'  && s[4]=='i'   ){
	 USART_Puts(USART2,"gelenVeri: ");
	 USART_Puts(USART2,"ileri");
	 forward();
    }
  else if( s[0]=='g' && s[1]=='e' && s[2]=='r' && s[3]=='i'   ){
	 USART_Puts(USART2,"gelenVeri: ");
	 USART_Puts(USART2,"geri");
	 back();
    }
  else if( s[0]=='s' && s[1]=='a' && s[2]=='g' ){
	 USART_Puts(USART2,"gelenVeri: ");
	 USART_Puts(USART2,"sag");
	 right();
      }
  else if( s[0]=='s' && s[1]=='o' && s[2]=='l' ){
	  USART_Puts(USART2,"gelenVeri: ");
	  USART_Puts(USART2,"sol");
	  	 left();
        }
  else if( s[0]=='d' && s[1]=='u' && s[2]=='r' ){
	 USART_Puts(USART2,"gelenVeri: ");
	USART_Puts(USART2,"dur");
	stop();
        }

  else if( s[0]=='h' && s[1]=='i' && s[2]=='z' ){
	 short k=3,m=0;

	 while(1){
		 if( s[k] != '\n'){
		  pwmPulseUart[m]=s[k];
		  k++;
		  m++;
		 }else {
		   pwmPulseUart[m] =s[k];
		   k=3;
		   m=0;
		   USART_Puts(USART2,"pwmhiz:\n");
		   USART_Puts(USART2,pwmPulseUart);

		   pwmahmet=atoi(pwmPulseUart);
		   pwmPulse1 = map(pwmahmet,0,100,0,20000);
		   pwmPulse2=pwmPulse1;
		   for(int a=0;a<10;a++){
			   pwmPulseUart[a]='\0';
		   	 }
		   break;
		  }

	 }

  }
}

//SEND MESSAGE UART P�N GPIO->PA2
void USART_Puts(USART_TypeDef* USARTx, volatile char *s)
{
	while(*s)
	{
		while(!(USARTx->SR & 0x00000040));
		USART_SendData(USARTx, *s);
		*s++;
	}
}

void forward(){
	GPIO_SetBits(GPIOD,motor1_in1);
	GPIO_ResetBits(GPIOD,motor1_in2);
	GPIO_SetBits(GPIOD,motor2_in1);
	GPIO_ResetBits(GPIOD,motor2_in2);

}
void back(){
	GPIO_SetBits(GPIOD,motor1_in2);
	GPIO_ResetBits(GPIOD,motor1_in1);
	GPIO_SetBits(GPIOD,motor2_in2);
	GPIO_ResetBits(GPIOD,motor2_in1);
}

void right(){
	GPIO_SetBits(GPIOD,motor1_in1);
	GPIO_ResetBits(GPIOD,motor1_in2);
	GPIO_SetBits(GPIOD,motor2_in1);
	GPIO_ResetBits(GPIOD,motor2_in2);
	pwmPulse1=(pwmPulse1 / 2);
}
void left(){
	GPIO_SetBits(GPIOD,motor1_in1);
	GPIO_ResetBits(GPIOD,motor1_in2);
	GPIO_SetBits(GPIOD,motor2_in1);
	GPIO_ResetBits(GPIOD,motor2_in2);
	pwmPulse2= (pwmPulse2 / 2);
}
void stop(){
	GPIO_ResetBits(GPIOD,motor1_in2);
	GPIO_ResetBits(GPIOD,motor1_in1);
	GPIO_ResetBits(GPIOD,motor2_in2);
	GPIO_ResetBits(GPIOD,motor2_in1);
}


void delay_ms(uint32_t time){

	delayCount=time;
	while(delayCount);
}

void SysTick_Handler()
{
	delayCount--;
}

int map(uint32_t A,uint32_t B,uint32_t C,uint32_t D,uint32_t E){

	return (A * E) / C;
}

 int main(void)
{

	SysTick_Config(SystemCoreClock/1000); //1 MS TIMER INTERRUP

	GPIO_Config();
	USART_Config();
	NVIC_Config();
	TIM_Config();




  while (1)
  {

	  //PD12 PWM MOTOR1
         TIM_OCInitStruct.TIM_Pulse=pwmPulse1;
		 TIM_OC1Init(TIM4,&TIM_OCInitStruct);
		 TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);

		//PD13 PWM MOTOR2
		 TIM_OCInitStruct.TIM_Pulse=pwmPulse2;
	     TIM_OC2Init(TIM4,&TIM_OCInitStruct);
		 TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable);



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