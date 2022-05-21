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

	RCC->AHB1ENR = 0x00000009;	// GPIOA ve GPIOD aktif

	GPIOD->MODER = 0x55000000;	// 12,13,14,15. pins digital output
	GPIOD->OTYPER = 0x00000000;	// 12,13,14,15. pins Push pull
	GPIOD->OSPEEDR = 0xFF000000; // 12,13,14,15. pins 100MHz
	GPIOD->PUPDR = 0x00000000;	// 12,13,14,15. pins no pull

    RCC->AHB1ENR =0x00000009;


}
void GPIO_Config(){
	 RCC->AHB1ENR =0x00000009;
	 GPIOD->MODER=0x55000000;//12-13-14-15 PÝN LOJIK1
	 GPIOD->OTYPER=0x00000000; // PUSH PULL- OPEN DRAÝN
	 GPIOD->OSPEEDR=0xFF000000;// 100MHz
	 GPIOD->PUPDR=0x00000000;//

}
void delay(uint32_t time){
	while(time--);
}
int counter;
int main(void)
{
	CLOCK_Config();
	GPIO_Config();

  while (1)
  {
	  if(GPIOA->ODR & 0x00000001){
		  while(GPIOA->ODR & 0x00000001);
		  counter++;
		  delay(168000);
	  }
	  if(counter%2==0){
		  GPIOD->ODR=0x00000000;
	  } else{
		  GPIOD->ODR=0x000F0000;
	  }
  }
}



void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  return;
}


uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  return -1;
}
