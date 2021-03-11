/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//void MX_I2C1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM13_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void MPU6050_Init(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);



short acceleStock[100][3];		//设置为全局变量，因为要在TIM4中断中传输其数据
short gyroStock[100][3];		//设置为全局变量，因为要在TIM4中断中传输其数据
float eulerStock[100][3];		//设置为全局变量，因为要在TIM4中断中传输其数据
short temperStock1[100];		//设置为全局变量，因为要在TIM4中断中传输其数据。存储还未处理过的数值
float temperStock2[100];		//设置为全局变量，因为要在TIM4中断中传输其数据。存储摄氏度
uint16_t currentCulcu;		//设置为全局变量，因为要在TIM4中断中传输其数据。记录当前100个数据中，有多少个数据被覆盖（前currentCulcu个数据）

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
  MX_I2C1_Init();
  MX_TIM10_Init();
  MX_USART1_UART_Init();
  MX_TIM13_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Init();
  mpu_dmp_init();
  HAL_TIM_IC_Start_IT(&htim10, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim13, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  for(currentCulcu=0; currentCulcu<100; currentCulcu++){
		  MPU_GetAcceler(&acceleStock[currentCulcu][0], &acceleStock[currentCulcu][1], &acceleStock[currentCulcu][2]);
		  MPU_GetGyro(&gyroStock[currentCulcu][0], &gyroStock[currentCulcu][1], &gyroStock[currentCulcu][2]);
		  mpu_dmp_get_data(&eulerStock[currentCulcu][0], &eulerStock[currentCulcu][1], &eulerStock[currentCulcu][2]);
		  MPU_GetTemper(&temperStock1[currentCulcu], &temperStock2[currentCulcu]);
	  }
    /* USER CODE BEGIN 3 */
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
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM1_UP_TIM10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  /* TIM8_UP_TIM13_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 3, 1);
  HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
//void MX_I2C1_Init(void)
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 8400-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 5000-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 4*8400-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 5*5000-1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED0_Pin */
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief MPU6050 Initialization Function
  * @param None
  * @retval None
  */
static void MPU6050_Init(void){
	uint8_t data;

	//1初始化I2C
	MX_I2C1_Init();

	//2复位MPU6050：一、将电源管理寄存器1的bit7写1（从第0位开始数起） 二、对电源管理寄存器1写0x00以唤醒MPU6050
	while(HAL_I2C_IsDeviceReady(&hi2c1, (MPU_Device_Addr<<1) | 1, 5, 0xffff) != HAL_OK);
	data = 0x80;
	while(HAL_I2C_Mem_Write(&hi2c1, (MPU_Device_Addr<<1) | 0, MPU_PowerManageReg1_Addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xffff) != HAL_OK);
	HAL_Delay(100);		//不知道为什么要延迟0.1s，应该没什么意义。。。。。
	while(HAL_I2C_IsDeviceReady(&hi2c1, (MPU_Device_Addr<<1) | 1, 5, 0xffff) != HAL_OK);
	data = 0x00;
	while(HAL_I2C_Mem_Write(&hi2c1, (MPU_Device_Addr<<1) | 0, MPU_PowerManageReg1_Addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xffff) != HAL_OK);

	//3设置陀螺仪配置寄存器、加速度配置寄存器的最大计数范围
	while(HAL_I2C_IsDeviceReady(&hi2c1, (MPU_Device_Addr<<1) | 1, 5, 0xffff) != HAL_OK);
	data = (0x03 << 3);		//为什么要左移三位？具体参见“STM32F407ZG中文开发手册库函数版本”中MPU6050的实验
	while(HAL_I2C_Mem_Write(&hi2c1, (MPU_Device_Addr<<1) | 0, MPU_GyroConfigReg_Addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xffff) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(&hi2c1, (MPU_Device_Addr<<1) | 1, 5, 0xffff) != HAL_OK);
	data = (0x00 << 3);		//为什么要左移三位？具体参见“STM32F407ZG中文开发手册库函数版本”中MPU6050的实验
	while(HAL_I2C_Mem_Write(&hi2c1, (MPU_Device_Addr<<1) | 0, MPU_AcceConfigReg_Addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xffff) != HAL_OK);

	//4关闭MPU的中断、AUX_I2C、FIFO
	while(HAL_I2C_IsDeviceReady(&hi2c1, (MPU_Device_Addr<<1) | 1, 5, 0xffff) != HAL_OK);
	data = 0x00;
	while(HAL_I2C_Mem_Write(&hi2c1, (MPU_Device_Addr<<1) | 0, MPU_InterruEnableReg_Addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xffff) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(&hi2c1, (MPU_Device_Addr<<1) | 1, 5, 0xffff) != HAL_OK);
	while(HAL_I2C_Mem_Write(&hi2c1, (MPU_Device_Addr<<1) | 0, MPU_UserControlReg_Addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xffff) != HAL_OK);
	while(HAL_I2C_IsDeviceReady(&hi2c1, (MPU_Device_Addr<<1) | 1, 5, 0xffff) != HAL_OK);
	while(HAL_I2C_Mem_Write(&hi2c1, (MPU_Device_Addr<<1) | 0, MPU_FIFOConfigReg_Addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xffff) != HAL_OK);

	//5设定陀螺仪采样率
	while(HAL_I2C_IsDeviceReady(&hi2c1, (MPU_Device_Addr<<1) | 1, 5, 0xffff) != HAL_OK);
	data = MPU_GetSMPLART_DIV(1, 50);
	while(HAL_I2C_Mem_Write(&hi2c1, (MPU_Device_Addr<<1) | 0, MPU_GyroSampleRateReg_Addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xffff) != HAL_OK);

	//6设定DLPF的带宽
	while(HAL_I2C_IsDeviceReady(&hi2c1, (MPU_Device_Addr<<1) | 1, 5, 0xffff) != HAL_OK);
	data = 0x04;		//这一步做的不是很完美，之后自己可以试着写一个根据陀螺仪输出频率、陀螺仪采样率得到DLPF进而得到CFG_DLPF的函数。。。。。
	while(HAL_I2C_Mem_Write(&hi2c1, (MPU_Device_Addr<<1) | 0, MPU_DLPFConfigReg_Addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xffff) != HAL_OK);

	//7设定时钟，使能陀螺仪、加速度寄存器
	while(HAL_I2C_IsDeviceReady(&hi2c1, (MPU_Device_Addr<<1) | 1, 5, 0xffff) != HAL_OK);
	while(HAL_I2C_Mem_Read(&hi2c1, (MPU_Device_Addr<<1) | 1, MPU_DeviceIDReg_Addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xffff) != HAL_OK);
	if(data == MPU_Device_Addr){
		while(HAL_I2C_IsDeviceReady(&hi2c1, (MPU_Device_Addr<<1) | 1, 5, 0xffff) != HAL_OK);
		data = 0x01;
		while(HAL_I2C_Mem_Write(&hi2c1, (MPU_Device_Addr<<1) | 0, MPU_PowerManageReg1_Addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xffff) != HAL_OK);
		while(HAL_I2C_IsDeviceReady(&hi2c1, (MPU_Device_Addr<<1) | 1, 5, 0xffff) != HAL_OK);
		data = 0x00;
		while(HAL_I2C_Mem_Write(&hi2c1, (MPU_Device_Addr<<1) | 0, MPU_PowerManageReg2_Addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xffff) != HAL_OK);
	}
}



//定时器中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	uint16_t i;
	if(htim == &htim10)
		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
	else if(htim == &htim13){
		printf("Data got from MPU6050: \r\n");
		for(i=0; i<100; i++){
			printf("第%d个数据：Accele(x,y,z) = (%hd, %hd, %hd)\r\n", i, acceleStock[i][0], acceleStock[i][1], acceleStock[i][2]);
		}
		for(i=0; i<100; i++){
			printf("第%d个数据：gyro(x,y,z) = (%hd, %hd, %hd)\r\n", i, gyroStock[i][0], gyroStock[i][1], gyroStock[i][2]);
		}
		for(i=0; i<100; i++){
			printf("第%d个数据：euler(pitch,roll,yaw) = (%f, %f, %f)\r\n", i, eulerStock[i][0], eulerStock[i][1], eulerStock[i][2]);
		}
		for(i=0; i<100; i++){
			printf("第%d个数据：temperature = %f\r\n", i, temperStock2[i]);
		}
		printf("当前覆盖了前%d个数据\r\n", currentCulcu);
		printf("***************************************************************\r\n\r\n");
	}
}



//printf函数重定向
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch,1,HAL_MAX_DELAY);
    return ch;
}



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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
