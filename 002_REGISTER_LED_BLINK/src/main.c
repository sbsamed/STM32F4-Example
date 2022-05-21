
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

//Registers System Clock Config Func.
//8 000 000mhz -> 168 000 000mhz
void  RCC_Config(void){



//	RCC->CR &= 0x00000083; // RCC Clok Cont. Registers reset value
	RCC->CR &= ~ (1<<0); //HSI:OFF
	RCC->CR |= (1<<16); //HSE:ON

	RCC->CR |= (1<<19); //CSSON:Clock Security Bit is on

	while( ! (RCC->CR & (1<<17) ));   // if HSERDY=1 wait...

	//PLL SETTÝNGS FOR 168 000 000 HZ :

	RCC ->PLLCFGR = 0x00000000; //PLL SETTÝNGS ÝS RESET
	//PLL SOURCE EDÝT FOR HSE
	RCC ->PLLCFGR |= (1<<22);

	//PLLM=4
	RCC->PLLCFGR |= (4<<0); //PLLM 4
	 //PLLN =168
	RCC->PLLCFGR |= (168<<6); //PLLN 168
	//PLLP=2
	RCC->PLLCFGR &= ~(1<<16); //PLLP 2
	RCC->PLLCFGR &= ~(1<<17); //

	//PLL ON
	RCC->CR |=(1<<24);////PLL ON

	while( ! (RCC->CR & (1<<25) ));   // if PLLRDY=1 wait...

	RCC->CFGR &= ~(1<<0); //SYSTEM CLOCK IS PLL 10
	RCC->CFGR |=(1<<1); ////SYSTEM CLOCK IS PLL 10
	while( ! (RCC->CFGR & (1<<1)  ));   // if SYSTEM CLOCK IS PLL=1 wait...

}
void GPIO_Config(){

	RCC->AHB1ENR |= (1<<3); //AHB1ENR CLOCK,, D PORT ÝS ENABLE

	GPIOD->MODER |= (1<<24); //GPIO 12.PÝN OUTOPUT 01
	GPIOD->MODER &= ~(1<<25); //GPIO 12.PÝN OUTOPUT 01
	GPIOD->OSPEEDR |= 0xFF000000;
}

int main(void)
{

	RCC_Config();// my function is call

	SystemCoreClockUpdate(); //SYSTEM CLOCK ÝS UPDATE FOR MY FUNCTÝON

	GPIO_Config(); //port setting

  while (1)
  {
	GPIOD->ODR |= (1<<12); //POPRT D PÝN12 ÝS SET;

  }
}



void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){

  return;
}


uint16_t EVAL_AUDIO_GetSampleCallBack(void){

  return -1;
}
