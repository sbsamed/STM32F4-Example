/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

uint32_t mapCalculation(long A,long B,long C,long D,long E);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//motor pins


#define Kp 0.09
#define Kd 1.08
#define rightMaxSpeed 255
#define leftMaxSpeed 255
#define rightBaseSpeed 200
#define leftBaseSpeed 200
#define maxPwmPulse 18500

#define yukAlmaPini GPIO_PIN_14
#define yukBirakmaPini GPIO_PIN_13
GPIO_PinState  yukAlmaPiniDurumu=SET;
GPIO_PinState yukBirakmaPiniDurumu=SET;

#define durmaLediPini GPIO_PIN_4
#define sagLediPini GPIO_PIN_6
#define solLediPini GPIO_PIN_5
GPIO_PinState  durmaLediDurumu=SET;
GPIO_PinState sagLediDurumu=RESET;
GPIO_PinState solLediDurumu=RESET;

int leftMotorSpeed=0,rightMotorSpeed=0;

uint32_t pwmPulseRight1=0; //tim4 channel1
uint32_t pwmPulseLeft1=0; //tim4 channel2

uint32_t pwmPulseRight2=0; //tim4 channel3
uint32_t pwmPulseLeft2=0; // tim4 channel 4

int ledKontrolDurum=0;

int lastError = 0;
uint16_t qtrPosition=0;
char qtrPosition_char[10];
char rx_buffer[50];
char tx_buffer[50];//led
char rx_byte[1];
int size=0;
int count=0;


uint32_t countTimer;
int delay_ms(uint32_t time) {
	countTimer=time;
	while(countTimer);
	return 1;
}

void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  if(countTimer>0){
	  countTimer--;
  	}

  /* USER CODE END SysTick_IRQn 1 */
}
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

   char rx_byte[1];

	   HAL_UART_Receive(&huart3,&rx_byte[0],1,100);

	   rx_buffer[size++]=rx_byte[0];

	   if(rx_byte[0]=='\n'){

		   uartMessageDebugNrf(rx_buffer);

		   //buffer is clear
		  	  for(int a=0;a<50;a++){
		       	rx_buffer[a]='\0';
		       	}
		      size=0;
	 }



  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART2_IRQn 1 */
  /* USER CODE END USART2_IRQn 1 */
}

void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
   char rx_byte[1];

   HAL_UART_Receive(&huart2,&rx_byte[0],1,100);
   if(size<50){
	  rx_buffer[size++]=rx_byte[0];

	   if(rx_byte[0]=='\n'){

		   uartMessageDebugPid(rx_buffer);

		   //buffer is clear
		  	  for(int a=0;a<50;a++){
		       	rx_buffer[a]='\0';
		       	}
		      size=0;
	 }
}


  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  /* USER CODE END USART2_IRQn 1 */
}

void uartMessageDebugPid (volatile char *s){

   if( s[0]=='p' && s[1]=='s' && s[2]=='t' ){

	 short k=3,m=0;
	 while(1){
		 if( s[k] != '\n'){
			 qtrPosition_char[m]=s[k];
		  k++;
		  m++;
		 }else {
			 qtrPosition_char[m] =s[k];
		   k=3;
		   m=0;
		   qtrPosition=atoi(qtrPosition_char);

		   qtrPidPwm(qtrPosition);

		   for(int a=0;a<10;a++){
			   qtrPosition_char[a]='\0';
		   	 }

		   break;
		  }

	 }

	 ledStabil();

  }else if(s[0]=='d' && s[1]=='u' && s[2]=='r'){

	  stop();
	  durLedYak();


  }
  else if(s[0]=='s' && s[1]=='a' && s[2]=='g'){

  	 right();
  	sagLedYak();
    }
  else if(s[0]=='s' && s[1]=='o' && s[2]=='l'){

	  left();
	  solLedYak();

   }else if(s[0]=='y' && s[1]=='u' && s[2]=='k' && s[3]=='a' && s[4]=='l' ){
	   yukAl();
	   durLedYak();
    }
   else if(s[0]=='y' && s[1]=='u' && s[2]=='k' && s[3]=='b' && s[4]=='i'
		   && s[5]=='r' && s[6]=='a' && s[7]=='k'){
	   yukBirak();
	   durLedYak();
       }
   else if(s[0]=='y' && s[1]=='u' && s[2]=='k' && s[3]=='o' && s[4]=='k'  && s[5]=='e' ){
   	   yukStabil();
          }
   else if(s[0]=='i' && s[1]=='l' && s[2]=='e'&& s[3]=='r' && s[4]=='i'  ){
		  ileriNrf();
		  ledStabil();

	  }

     else{

    	 for(int a=0;a<50;a++){
    			 rx_buffer[a]='\0';
          	}
         size=0;
     }

         }

void uartMessageDebugNrf(volatile char *s){

  if(s[0]=='i' && s[1]=='l' && s[2]=='e'&& s[3]=='r' && s[4]=='i' ){
	  ileriNrf();
	  ledStabil();

  }else if(s[0]=='d' && s[1]=='u' && s[2]=='r'){
	  stop();
	  durLedYak();

  }
  else if(s[0]=='s' && s[1]=='a' && s[2]=='g'){
  	  rightNrf();
  	  sagLedYak();

    }
  else if(s[0]=='s' && s[1]=='o' && s[2]=='l'){

	  leftNrf();
	  solLedYak();
   } else if(s[0]=='g' && s[1]=='e' && s[2]=='r' && s[3]=='i'){
	   geriNrf();
	   }
   else if(s[0]=='y' && s[1]=='u' && s[2]=='k' && s[3]=='a' && s[4]=='l' ){
   	   yukAl();
   	durLedYak();
       }
      else if(s[0]=='y' && s[1]=='u' && s[2]=='k' && s[3]=='b' && s[4]=='i'
   		   && s[5]=='r' && s[6]=='a' && s[7]=='k'){
   	   yukBirak();
   	durLedYak();
          }
      else if(s[0]=='y' && s[1]=='u' && s[2]=='k' && s[3]=='o' && s[4]=='k'  && s[5]=='e' ){
      	   yukStabil();
             }
  else{
	  for(int a=0;a<50;a++){
	     rx_buffer[a]='\0';
	           	}
	          size=0;
     }


}

void qtrPidPwm(uint16_t position){

	  int error = position - 3500;
	  int motorSpeed = Kp * error + Kd * (error - lastError);
	  lastError = error;

	   rightMotorSpeed = rightBaseSpeed + motorSpeed;
	   leftMotorSpeed = leftBaseSpeed - motorSpeed;

	  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed;
	  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed;
	  if (rightMotorSpeed < 0) rightMotorSpeed = 0;
	  if (leftMotorSpeed < 0) leftMotorSpeed = 0;

	  if (rightMotorSpeed == 0) rightMotorSpeed = 100;
	  if (leftMotorSpeed == 0) leftMotorSpeed = 100;

	  mapPulse(rightMotorSpeed,leftMotorSpeed);

}
void mapPulse(rightMotorSpeed,leftMotorSpeed){

	 pwmPulseRight2=0; //tim4 channel3
	 pwmPulseLeft2=0; // tim4 channel4

	pwmPulseRight1=mapCalculation(rightMotorSpeed,0,rightMaxSpeed,0,maxPwmPulse);
	pwmPulseLeft1=mapCalculation(leftMotorSpeed,0,leftMaxSpeed,0,maxPwmPulse);



}
uint32_t mapCalculation(long A,long B,long C,long D,long E){

	return ((A*E)/C);

}

void stop(){

	 pwmPulseRight1=0; //tim4 channel1
	 pwmPulseLeft1=0; //tim4 channel2
	 pwmPulseRight2=0; //tim4 channel3
	 pwmPulseLeft2=0; // tim4 channel4

}
void right(){

	pwmPulseRight1=0; //tim4 channel1
    pwmPulseLeft1=mapCalculation(200,0,255,0,maxPwmPulse); //tim4 channel2
	pwmPulseRight2=mapCalculation(60,0,255,0,maxPwmPulse); //tim4 channel3
	pwmPulseLeft2=0; // tim4 channel4

}
void left(){

	pwmPulseRight1=mapCalculation(200,0,255,0,maxPwmPulse); //tim4 channel1
    pwmPulseLeft1=0; //tim4 channel2
    pwmPulseRight2=0; //tim4 channel3
    pwmPulseLeft2=mapCalculation(60,0,255,0,maxPwmPulse);; // tim4 channel4

}

/********NRF*******/

void  ileriNrf(){

	 pwmPulseRight1=mapCalculation(200,0,255,0,18500); //tim4 channel1
     pwmPulseLeft1=mapCalculation(200,0,255,0,18500); //tim4 channel2
     pwmPulseRight2=0; //tim4 channel3
     pwmPulseLeft2=0; // tim4 channel4

}
void rightNrf(){

	pwmPulseRight1=0; //tim4 channel1
    pwmPulseLeft1=mapCalculation(200,0,255,0,maxPwmPulse); //tim4 channel2
	pwmPulseRight2=mapCalculation(60,0,255,0,maxPwmPulse);; //tim4 channel3
	pwmPulseLeft2=0; // tim4 channel4

}
void leftNrf(){

	pwmPulseRight1=mapCalculation(200,0,255,0,maxPwmPulse); //tim4 channel1
    pwmPulseLeft1=0; //tim4 channel2
    pwmPulseRight2=0; //tim4 channel3
    pwmPulseLeft2=mapCalculation(60,0,255,0,maxPwmPulse);; // tim4 channel4

}
void geriNrf(){
	 pwmPulseRight1=0; //tim4 channel1
	 pwmPulseLeft1=0; //tim4 channel2
	 pwmPulseRight2=mapCalculation(180,0,255,0,19999);   //tim4 channel3
	 pwmPulseLeft2=mapCalculation(180,0,255,0,19999);  //tim4 channel4

}

void yukAl(){

	 yukAlmaPiniDurumu=RESET;
	 yukBirakmaPiniDurumu=SET;

}
void yukBirak(){
   yukAlmaPiniDurumu=SET;
   yukBirakmaPiniDurumu=RESET;

}

void yukStabil(){
	yukAlmaPiniDurumu=SET;
    yukBirakmaPiniDurumu=SET;
}

void durLedYak(){
     durmaLediDurumu=RESET;
	 sagLediDurumu=SET;
	 solLediDurumu=SET;
}
void solLedYak(){
	 durmaLediDurumu=SET;
     sagLediDurumu=SET;
     solLediDurumu=RESET;
}
void sagLedYak(){
	durmaLediDurumu=SET;
    sagLediDurumu=RESET;
    solLediDurumu=SET;
}
void ledStabil(){
	    durmaLediDurumu=SET;
	    sagLediDurumu=RESET;
	    solLediDurumu=RESET;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN , */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  //pwm motor
   HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
   HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);

  __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);//arduino mega
  __HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE); //nrf kablosuz



     pwmPulseRight1=0; //tim4 channel1
 	 pwmPulseLeft1=0; //tim4 channel2
 	 pwmPulseRight2=0; //tim4 channel3
 	 pwmPulseLeft2=0; // tim4 channel4

 	 // RESET OLUNCA ROLE ÇALIÞIYOR
 	 HAL_GPIO_WritePin(GPIOE,yukBirakmaPini,SET);
 	 HAL_GPIO_WritePin(GPIOE,yukAlmaPini,SET);
 	 //LED ROLE
 	  HAL_GPIO_WritePin(GPIOA,durmaLediPini,SET);
 	  HAL_GPIO_WritePin(GPIOA,sagLediPini,RESET);
 	  HAL_GPIO_WritePin(GPIOA,solLediPini,RESET);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

     __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,pwmPulseRight1);
     __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,pwmPulseLeft1);

     __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,pwmPulseRight2);
     __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,pwmPulseLeft2);

     HAL_GPIO_WritePin(GPIOE,yukAlmaPini,yukAlmaPiniDurumu);
     HAL_GPIO_WritePin(GPIOE,yukBirakmaPini,yukBirakmaPiniDurumu);

     HAL_GPIO_WritePin(GPIOE,durmaLediPini,durmaLediDurumu);
     HAL_GPIO_WritePin(GPIOE,sagLediPini,sagLediDurumu);
     HAL_GPIO_WritePin(GPIOE,solLediPini,solLediDurumu);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_13
                          |GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE4 PE5 PE6 PE13
                           PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_13
                          |GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

