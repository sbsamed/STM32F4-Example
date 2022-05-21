#include "stm32f4xx.h"
#include "stm32f4_discovery.h"


void delay(uint32_t time){
	while(time--);
}
void CLOCK_Config()
{
	RCC->CR |= 0x00030000;	// HSEON and HSEONRDY enable
	while(!(RCC->CR & 0x00020000));	// HSEON Ready Flag wait
	RCC->CR |= 0x00080000;	// CSS Enable
	RCC->PLLCFGR |= 0x00400000;	// PLL e HSE seçtik
	RCC->PLLCFGR |= 0x00000004;	// PLL M = 4
	RCC->PLLCFGR |= 0x00005A00;	// Pll N = 168
	RCC->PLLCFGR |= 0x00000000;	// PLL p = 2
	RCC->CFGR |= 0x00000000;	// AHB Prescaler = 1
	RCC->CFGR |= 0x00080000;	// APB2 Prescaler = 2
	RCC->CFGR |= 0x00001400;	// APB1 Prescaler = 4
	RCC->CIR |= 0x00800000;		// CSS Flag clear



}
void GPIO_Config(){

	RCC->AHB1ENR=0x00000009;

	GPIOD->MODER=0x55000000;
	GPIOD->OSPEEDR=0xFF000000;
	GPIOD->OTYPER=0x00000000;
}
void EXTI_Config(){
	RCC->APB2ENR=0x00004000;

	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI2_IRQn);

	SYSCFG->EXTICR[0]=0x00000000;
	SYSCFG->EXTICR[1]=0x00000000;
	SYSCFG->EXTICR[2]=0x00000000;

	EXTI->IMR=0x00000007;
	EXTI->RTSR=0x00000007;

	NVIC_SetPriority(EXTI0_IRQn,0);
	NVIC_SetPriority(EXTI1_IRQn,5);
	NVIC_SetPriority(EXTI2_IRQn,3);


 }

void EXTI0_IRQHandler(){

	if( EXTI->PR & 0x00000001  ){
		int i=0;
		do {
		  GPIOD->ODR=0x00001000;
		  delay(1680000);
		  GPIOD->ODR=0x00001000;
		  delay(1680000);
		  i++;
		} while (i<5);
			EXTI->PR = (1<<0);
	}
}
void EXTI1_IRQHandler(){

	if( EXTI->PR & 0x00000002  ){
		int i=0;
		do {
		  GPIOD->ODR=0x00002000;
		  delay(1680000);
		  GPIOD->ODR=0x00000000;
		  delay(1680000);
		  i++;
		} while (i<5);
			EXTI->PR = (1<<1);
	}
}

void EXTI2_IRQHandler(){

	if( EXTI->PR & 0x00000008  ){
		int i=0;
		do {
		  GPIOD->ODR=0x00008000;
		  delay(1680000);
		  GPIOD->ODR=0x00000000;
		  delay(1680000);
		  i++;
		} while (i<5);
			EXTI->PR = (1<<2);
	}
}
int main(void)
{

	CLOCK_Config();
	GPIO_Config();
	EXTI_Config();
  while (1)
  {

  }
}


void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  return;
}


uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  return -1;
}
