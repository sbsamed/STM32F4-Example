
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"


void RCC_Config(){
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
void RANDOM_Config(){

	RCC->AHB2ENR|=1<<6;

	RNG->CR|=1<<2;
}

void delay(uint32_t time){
	while(time--);
}
int main(void)
{
	RCC_Config();
	RANDOM_Config();

  while (1)
  {
  int myrndnumber= RNG->DR;
  delay(21000000);
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
