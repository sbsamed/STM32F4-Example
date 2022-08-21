
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

void  RCC_Config(void){
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
	RCC->CFGR |=(1<<1); //SYSTEM CLOCK IS PLL 10
	while( ! (RCC->CFGR & (1<<1)  ));   // if SYSTEM CLOCK IS PLL=1 wait...

}
void FLASH_Unlocker(){

	while(!(FLASH->SR & 0x00010000)); //status control bsy bit

	FLASH->KEYR=0x45670123;
	FLASH->KEYR=0xCDEF89AB;

}

void FLASH_Erase(){

	while((FLASH->SR & 0x00010000)!=0); //status control bsy bit

	FLASH->CR |= 1<<1; //erase active
	FLASH->CR |= 11<<3; //sector 11 is delete
	FLASH->CR |= 1<<16; // start

	while((FLASH->SR & 0x00010000)!=0);

	if((FLASH->SR & 0x00000001) !=0){
		FLASH->SR |=  0x00000001;
	}else{

		//delete is not;
	}

	FLASH->CR &= ~(1<<1); //erase deactive
}
void FLASH_Write(uint32_t address, uint32_t data){

	while((FLASH->SR & 0x00010000)!=0); //status control bsy bit

	FLASH->CR|=1<<0; //pg set bit active
	FLASH->CR|=2<<8; //64 bit write


	*(__IO uint32_t*)address=data;

	while((FLASH->SR & 0x00010000)!=0);//status control bsy bit

	if((FLASH->SR & 0x00000001) !=0){
			FLASH->SR |=  0x00000001;
		}else{
			//write is not;
		}

	 FLASH->CR &=~(1<<0); //pg deactive
}

uint32_t FLASH_Read(uint32_t address){
	uint32_t myFlashData;
	myFlashData=*(uint32_t*)address;

	return myFlashData;
}

void FLASH_Locker(){
	while((FLASH->SR & 0x00010000)!=0); //status control bsy bit
  FLASH->CR |= 1<<31;


}

uint32_t data;
int main(void)
{
	RCC_Config();

	FLASH_Unlocker();
	FLASH_Erase();
	FLASH_Write(0x080E0000,0xCC);
	FLASH_Locker();
	FLASH_Unlocker();
	data=FLASH_Read(0x080E0000);
	FLASH_Locker();

  while (1)
  {

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
