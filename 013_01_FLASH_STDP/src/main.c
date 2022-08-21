#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

uint32_t data;

void FLASH_Write(uint32_t myFlashAddress, uint32_t flashData){

	FLASH_Unlock();

	FLASH_EraseSector(FLASH_Sector_11,VoltageRange_3);//32 bit sector 11

	FLASH_ProgramWord(myFlashAddress,flashData);

	FLASH_Lock();

}

uint32_t FLASH_Read(uint32_t myFlashAddress){

	uint32_t myflashdata=*(uint32_t *)myFlashAddress;

	return myflashdata;

}
int main(void)
{

	FLASH_Write(0x080E0000,0x55);

  while (1)
  {
	  data=FLASH_Read(0x080E0000);
  }
}



void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){

  return;
}


uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  return -1;
}
