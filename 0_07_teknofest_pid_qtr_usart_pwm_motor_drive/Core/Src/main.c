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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#define rightMotor1 GPIO_PIN_11
#define rightMotor2 GPIO_PIN_12
#define leftMotor1 GPIO_PIN_13
#define leftMotor2 GPIO_PIN_14

#define stmMaxPulse 8000

//pwm usart variable
uint32_t pulseUartSol=0, pulseUartSag=0;
char pwmPulseUartCharSol[10];
char pwmPulseUartCharSag[10];

//usart2 variable
char rx_buffer[50];
char rx_byte[1];
char tx_buffer[50];
int size=0;

uint32_t map(long A,long B,long C,long D,long E){

	return ((A*E)/C);

}

//usart2 interrupt
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
   char rx_byte[1];

   HAL_UART_Receive(&huart2,&rx_byte[0],1,100);

   rx_buffer[size++]=rx_byte[0];

   if(rx_byte[0]=='\n'){
	   //come message is send

	   //HAL_UART_Transmit(&huart2,&rx_buffer,size,100);
	   uartMessageDebug(rx_buffer);
	   //buffer is clear
	  	  for(int a=0;a<50;a++){
	       	rx_buffer[a]='\0';
	       	}
	      size=0;
   }

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  /* USER CODE END USART2_IRQn 1 */
}



void uartMessageDebug(volatile char *s){

	if( s[0]=='p' && s[1]=='i' && s[2]=='d'  && s[3]==',' ){
		forward();
		 int temp=0;
		 short k=4,mSol=0,mSag=0;
		 while(1){
			 if( s[k] != '\n'){
				 if(s[k] == ','){
					 temp = 1;
					 k++;
				 }
				 if(temp==1){
					 pwmPulseUartCharSol[mSol]=s[k];   //sol
					 mSol++;
				 }else{
					 pwmPulseUartCharSag[mSag]=s[k];  //sag
					 mSag++;
				 }
			       k++;
			 }else {
				pwmPulseUartCharSol[mSol] =s[k];
				pwmPulseUartCharSag[mSag] =s[k];

			   pulseUartSol=atoi(pwmPulseUartCharSol);
			   pulseUartSag=atoi(pwmPulseUartCharSag);

			   pulseUartSol= map(pulseUartSol,0,100,0,stmMaxPulse);
			   pulseUartSag =map(pulseUartSag,0,100,0,stmMaxPulse);

			   for(int a=0;a<10;a++){
				   pwmPulseUartCharSol[a]='\0';
				   pwmPulseUartCharSag[a]='\0';
			   }
			   temp = 1,k=4,mSol=0,mSag=0;
			   break;
			  }

		 }

	  }

else if( s[0]=='d' && s[1]=='u' && s[2]=='r' ){
	   stop();
    }
  else if( s[0]=='s' && s[1]=='a' && s[2]=='g' ){
	  right();
      }
  else if( s[0]=='s' && s[1]=='o' && s[2]=='l' ){
	  left();
    }

 }

void forward(){

	    HAL_GPIO_WritePin(GPIOE,rightMotor1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE,rightMotor2,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE,leftMotor1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE,leftMotor2,GPIO_PIN_SET);
}

void left(){

	HAL_GPIO_WritePin(GPIOE,rightMotor1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,rightMotor2,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE,leftMotor1,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,leftMotor2,GPIO_PIN_RESET);
	pulseUartSol= map(0,0,255,0,10000);
	pulseUartSag =map(240,0,255,0,10000);

}
void right(){
	HAL_GPIO_WritePin(GPIOE,rightMotor1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE,rightMotor2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE,leftMotor1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE,leftMotor2,GPIO_PIN_SET);
	pulseUartSol= map(240,0,255,0,10000);
	pulseUartSag =map(0,0,255,0,10000);

}
void stop(){
	HAL_GPIO_WritePin(GPIOE,rightMotor1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE,rightMotor2,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE,leftMotor1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE,leftMotor2,GPIO_PIN_RESET);
	pulseUartSol=0;
	pulseUartSag=0;
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

        HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
    	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
      __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);

	    HAL_GPIO_WritePin(GPIOE,rightMotor1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE,rightMotor2,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE,leftMotor1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE,leftMotor2, GPIO_PIN_RESET);

		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	     __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,10000);

		 __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,10000);
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE11 PE12 PE13 PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
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

