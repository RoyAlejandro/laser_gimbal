/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU9250.h"
#include <stdio.h>
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

osThreadId CapturaHandle;
osThreadId HombroHandle;
osThreadId CodoHandle;
osThreadId MunecaHandle;
osMessageQId ColaXHandle;
osMessageQId ColaYHandle;
osMessageQId ColaZHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
void StartCaptura(void const * argument);
void StartHombro(void const * argument);
void StartCodo(void const * argument);
void StartMuneca(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/******************************************************************************** VARS ***/
// Declaradas globales por captura de firstYAW
int16_t AccData[3], GyroData[3], MagData[3], firstYAW;
// Retardo de las tareas
int Gresponse = 10;
// Prescaler = 40
// Se queda ARR = 8000
// Centro de los Motores, Rango de 320-880 cuentas (0.8-2.2ms)
int centro=600;
// Límite de posición estable;
int limitStable = 400;


// Factor de resolución para el ángulo
//int incResolution=400;


/******************************************************************************** PRINTF ***/
//Retargets the C library printf function to the USART.
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
 PUTCHAR_PROTOTYPE
{
  /* write a character to the uart1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF); 
  return ch;
}

/******************************************************************************** SETPWM ***/
void setPWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint16_t pulse);



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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	
	printf("==========================================================  \n\r");
  printf("     Master en Electrónica y Telecomunicacion Aplicadas     \n\r");
  printf("          Sistemas Empotrados de Tiempo Real                \n\r");
  printf("                                                            \n\r");
  printf("                 PROYECTO GIMBAL                            \n\r");
  printf("                                              Roy Tarapuez  \n\r");
  printf("==========================================================\n\n\r");

	
	MPU9250_Init();

	// Capturamos el primer ángulo de giro de la base.
	MPU9250_GetData(AccData, GyroData, MagData);
	firstYAW = GyroData[0];


	/********************************************************************* INICIA ARTICULACIONES */
  
	// Inicializamos los motores a su centro
		setPWM(htim4, TIM_CHANNEL_1, 8000, centro);
  // El motor del codo tiene un offset
	setPWM(htim4, TIM_CHANNEL_2, 8000, centro);
  	setPWM(htim4, TIM_CHANNEL_3, 8000, centro);





  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of ColaX */
  osMessageQDef(ColaX, 16, uint32_t);
  ColaXHandle = osMessageCreate(osMessageQ(ColaX), NULL);

  /* definition and creation of ColaY */
  osMessageQDef(ColaY, 16, uint32_t);
  ColaYHandle = osMessageCreate(osMessageQ(ColaY), NULL);

  /* definition and creation of ColaZ */
  osMessageQDef(ColaZ, 16, uint32_t);
  ColaZHandle = osMessageCreate(osMessageQ(ColaZ), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Captura */
  osThreadDef(Captura, StartCaptura, osPriorityNormal, 0, 128);
  CapturaHandle = osThreadCreate(osThread(Captura), NULL);

  /* definition and creation of Hombro */
  osThreadDef(Hombro, StartHombro, osPriorityIdle, 0, 128);
  HombroHandle = osThreadCreate(osThread(Hombro), NULL);

  /* definition and creation of Codo */
  osThreadDef(Codo, StartCodo, osPriorityIdle, 0, 128);
  CodoHandle = osThreadCreate(osThread(Codo), NULL);

  /* definition and creation of Muneca */
  osThreadDef(Muneca, StartMuneca, osPriorityIdle, 0, 128);
  MunecaHandle = osThreadCreate(osThread(Muneca), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 40;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 8000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 600;
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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GY_CS_GPIO_Port, GY_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GY_CS_Pin */
  GPIO_InitStruct.Pin = GY_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GY_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*******************************************************************************************/
/******************************************************************************** SETPWM ***/
void setPWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint16_t pulse)
{
  HAL_TIM_PWM_Stop(&timer, channel); // stop generation of pwm
  TIM_OC_InitTypeDef sConfigOC;

  timer.Init.Period = period; // set the period duration
  HAL_TIM_PWM_Init(&timer); // reinititialise with new period value
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse; // set the pulse duration
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);
  HAL_TIM_PWM_Start(&timer, channel); // start pwm generation
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCaptura */
/*******************************************************************************************/
/******************************************************************************* CAPTURA ***/

/**
  * @brief  Function implementing the Captura thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartCaptura */

void StartCaptura(void const * argument)
{
  /* USER CODE BEGIN 5 */
	osStatus estado;
	
  /* Infinite loop */
  for(;;)
  {
		
		// Los valores que da el acelerómetro son proporcionales al 
		// Seno de ángulo que forma el vector gravedad con cada eje
		// AccData = 16384 * seno(theta) , Theta = ángulo formado con cada eje.
		// 16384 = obtenido en Datasheet y contrastado con STMStudio.
		MPU9250_GetData(AccData, GyroData, MagData);
	
		// Enviamos componente en X a su cola
		estado=osMessagePut(ColaXHandle,AccData[0],200);
    if (estado==osOK){ 
        printf("TX %5d ",(int16_t)AccData[0]);
    }
    else { printf(" * Cola llena\n\r"); }
		
		// Enviamos componente en Y a su cola
		estado=osMessagePut(ColaYHandle,AccData[1],200);
    if (estado==osOK){ 
        printf(" TY %5d ",(int16_t)AccData[1]);
    }
    else { printf(" * Cola llena\n\r"); }
		
		// Enviamos componente en Z del Gyro a su cola
		estado=osMessagePut(ColaZHandle,GyroData[0],200);
    if (estado==osOK){ 
        printf(" TZ %5d \n\r",(int16_t)GyroData[0]);
    }
    else { printf(" * Cola llena\n\r"); }
				
    
    osDelay(Gresponse);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartHombro */
/*******************************************************************************************/
/******************************************************************************** HOMBRO ***/
/**
* @brief Function implementing the Hombro thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHombro */

void StartHombro(void const * argument)
{
  /* USER CODE BEGIN StartHombro */
	osEvent  retvalue;
	int compZ, toIncrem=0;
  /* Infinite loop */
  for(;;)
  {
		
		retvalue=osMessageGet(ColaZHandle, osWaitForever);
		// Obtenemos la componente relativa en Z
		compZ=(int)retvalue.value.v - firstYAW;
		// Sensibilidad 250 con FS_SEL = 0 (datasheet)
		// version anterior (toIncrem = (int) (firstYAW - compZ)/10);
		// Se establece 25 como límite de estabilidad de los valores del Gyro
		// 25 es cte de Skinner, debería ser el rango / resolución para 1grado.
		if(compZ > 25)toIncrem = -1;
		if(compZ < -25) toIncrem = +1;
		if(compZ < 25 && compZ > -25) toIncrem = 0;
		// Incrementamos o reducimos el ciclo del PWM 
		TIM4->CCR1 = TIM4->CCR1 + toIncrem;
		printf("Hombro:%d \n\r",toIncrem);
		
    osDelay(Gresponse);
  }
  /* USER CODE END StartHombro */
}

/* USER CODE BEGIN Header_StartCodo */
/*******************************************************************************************/
/********************************************************************************** CODO ***/

/**
* @brief Function implementing the Codo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCodo */
void StartCodo(void const * argument)
{
  /* USER CODE BEGIN StartCodo */
	osEvent  retvalue;
  int compY, toIncrem;
	/* Infinite loop */
  for(;;)
  {
		
		retvalue=osMessageGet(ColaYHandle, osWaitForever);
		// Obtenemos la componente en Y
		compY=(int)retvalue.value.v;
		// con 300 tenemos un error de +- 1,049175689 grados en la posición estable.
		// es decir arcsin(300/sensibilidad) = 1,049175689 grados.
		// en versión anterior (toIncrem = (int)compY/incResolution);
		if (compY > limitStable)toIncrem = 1;
		if(compY < -limitStable) toIncrem = -1;
		if(compY < limitStable && compY > -limitStable) toIncrem = 0;
			
		
		TIM4->CCR2 = TIM4->CCR2 - toIncrem;
		
		printf("Codo: %d \n\r",toIncrem);		
    osDelay(Gresponse);
  }
  /* USER CODE END StartCodo */
}

/* USER CODE BEGIN Header_StartMuneca */
/*******************************************************************************************/
/******************************************************************************** MUNECA ***/
/**
* @brief Function implementing the Muneca thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMuneca */
void StartMuneca(void const * argument)
{
  /* USER CODE BEGIN StartMuneca */
	osEvent  retvalue;
	int compX, toIncrem;
  /* Infinite loop */
  for(;;)
  {
		
		retvalue=osMessageGet(ColaXHandle, osWaitForever);
		// Obtenemos la componente en X
		compX=(int)retvalue.value.v;
		// Versión anterior "con 300 tenemos un error de +- 1,049175689 grados en la posición estable."
		// es decir arcsin(300/sensibilidad) = 1,049175689 grados.
		// toIncrem = (int)compX/incResolution;
		// Conforme compX se acerque al limite de error los incrementos serán menores.
		
		// Se decidieron los incrementos en 1 ya que ARR no es demasiado grande y 
		// los incrementos anteriores hacían oscilar los motores al reducir Gresponse.
		if (compX > limitStable)toIncrem = 1;
		if(compX < -limitStable) toIncrem = -1;
		if(compX < limitStable && compX > -limitStable) toIncrem = 0;
		
		TIM4->CCR3 = TIM4->CCR3 + toIncrem;
		

		printf("Muneca: %d \n\r",toIncrem);
    osDelay(Gresponse);
  }
  /* USER CODE END StartMuneca */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
