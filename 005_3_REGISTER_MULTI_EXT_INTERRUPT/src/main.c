
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

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

	RCC->AHB1ENR=0x00000009;// GPIOA AND GPIOD ACTÝVE

	GPIOD->MODER= 0x55000000; //GPIOD PÝN12..15 OUTPUT
	GPIOD->OTYPER=0x00000000; //PUSH PULL
	GPIOD->OSPEEDR=0xFF000000;//VERY HIGH;


}
void EXTI_Config(){

	RCC->AHB2LPENR=0x00004000;//SYSCO EXT INTERRUPT ACTÝVE

	SYSCFG->EXTICR[0]=0x00000000; //PORTA ÝS EXTI ENABLE


	NVIC_EnableIRQ(EXTI0_IRQn); //extý line active
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI2_IRQn);

	NVIC_SetPriority(EXTI0_IRQn,0);
	NVIC_SetPriority(EXTI1_IRQn,1);
	NVIC_SetPriority(EXTI2_IRQn,2);

	EXTI->IMR=0x00000007; //ÝNTERRUP OR EVENT?
	EXTI->RTSR=0x00000007; //RÝSÝNG




}
void delay(uint32_t time){
	while(time--);
}
void 	EXTI0_IRQHandler(){

	if( EXTI->PR & 0x00000001  ){ //ÝF EXTI0 FLAG ÝS UP???
		GPIOD->ODR=0x00001000;
		delay(33600000);
		EXTI->PR=0x00000001;//flag is down
	}
}

void EXTI1_IRQHandler(){

	if(EXTI->PR & 0x00000002){
		GPIOD->ODR=0x000002000;
		delay(36800000);
		EXTI->PR= 0x00000002;
	}
}

void EXTI2_IRQHandler(){

	if(EXTI->PR & 0x00000004){
		GPIOD->ODR=0x00000004;
		delay(36800000);
		EXTI->PR=0x00000004;
	}
}
int main(void)
{

	CLOCK_Config();
	GPIO_Config();
	EXTI_Config();


  while (1)
  {
	  GPIOD->ODR=0x0000F000;//LED ÝS LOJIK 1 ON

  }
}


void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  return;
}


uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  return -1;
}
