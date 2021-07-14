/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "TJ_MPU6050.h"

#define M_PI        3.14159265358979323846;
#define mpu9265Address	0xD0
#define G_R 131.0
#define A_R 16384.0
#define RAD_A_DEG 57.295779
#define ema_alpha 0.8
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

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void crear(float **matriz, int fil, int col);
void valores(float **matriz, int fil, int col);
void crear_valores(float **matriz, int fil, int col);
void imprimir(float **matriz, int fil, int col);
void multiplicacion(float **matriz1, float **matriz2, float **matriz3, int fil1, int col1, int fil2, int col2);
void eye(float **matriz,int fil);
void copia(float **matriz1,float **matriz2,int fil,int col);
void inversa(float **matriz,float **m_inv, int fil);
void transpuesta(float **matriz,float **trans, int fil,int col);
void pseudoinversa(float **matriz, float **pseudo, int fil, int col);



void uprintf(char *str){
	HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str),100);
}
void uprintf2(char *str){
	HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str),100);
}

int SA1,SB1,SA2,SB2;
int count_1=0, count_2=0;
char buffer[32] = {0};
int ha=0,hb=0;
int bandera=0;
uint8_t RxData[11];

//PID
float v,vangular;
float v_2,vangular_2;
float eactual,eanterior=0,esum=0;
float eactual_2,eanterior_2=0,esum_2=0;
float sp = -5,sp_2=1.6;
int PID;
int PID_2;
float kp_2=3000,ki_2=60,kd_2=300;
float kp=3000,ki=60,kd=300;
int t;

uint8_t i2cBuf[8];
int16_t ax,ay,az,gx,gy,gz,gxa,gya,gza;
float Xaccel,Yaccel,Zaccel;
float Acc[2];
float Giro[3];
float angle[4];
float GiroN[3];

RawData_Def myAccelRaw, myGyroRaw;
ScaledData_Def myAccelScaled, myAcceScaledfilter, myGyroScaled, myGyroScaledfilter;



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	MPU_ConfigTypeDef myMpuConfig;
	
	
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_DMA(&huart2,RxData,1);
	HAL_UART_Receive_DMA(&huart3,RxData,1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//INIT CHANNEL 1
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);//INIT CHANNEL 2
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	TIM4->CCR3 = 0;
	TIM4->CCR4 = 0;
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	
	MPU6050_Init(&hi2c2);
	//2. Configure Accel and Gyro parameters
	myMpuConfig.Accel_Full_Scale = AFS_SEL_2g;
	myMpuConfig.ClockSource = Internal_8MHz;
	myMpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
	myMpuConfig.Gyro_Full_Scale = FS_SEL_250;
	myMpuConfig.Sleep_Mode_Bit = 0;  //1: sleep mode, 0: normal mode
	MPU6050_Config(&myMpuConfig);
	
	angle[2] = 0;
	
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (ha!=hb){
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
			t = 20;
			v=count_1/t;
			vangular = v * 1000 * 2* M_PI;
			vangular = vangular/3591;		
			v_2 = count_2/t;
			vangular_2 = v_2 * 1000 * 2* M_PI;
			vangular_2 = vangular_2/3591;		
			eactual = sp - vangular;
			PID = kp*eactual+ki*esum*t+kd*(eactual-eanterior)/t;
			eanterior = eactual;
			esum += eactual;		
			eactual_2 = sp_2 - vangular_2;
			PID_2 = kp_2 * eactual_2 + ki_2 * esum_2 * t + kd_2 * (eactual_2 - eanterior_2)/2;
			eanterior_2 = eactual_2;
			esum_2 += eactual_2;	
			if(PID>=65535){PID=65535;}
			else if(PID<=-65535){PID=-65535;}
			if(PID_2>=65535){PID_2=65535;}
			else if(PID_2<=-65535){PID_2=-65535;}
			//PA7-IN1 PA6-IN2
			//	1 			0 			-
			//	0				1 			+
			//PA4-IN4 PB3-IN3
			//	1				0				-
			//	0				1				+
			if(PID>0){
				TIM4->CCR3= PID;
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET); 
			}
			else if(PID<0){
				TIM4->CCR3= -1*PID;
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET); 
			}
			
			if(PID_2>0){
				TIM4->CCR4= PID_2;
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); 
			}
			else if(PID_2<0){
				TIM4->CCR4= -1*PID_2;
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); 
			}
	  hb = ha;
		count_1 = 0;
		count_2 = 0;

			
		MPU6050_Get_Accel_Cali(&myAccelScaled);
		MPU6050_Get_Gyro_Scale(&myGyroScaled);
		
		myAcceScaledfilter.x = ema_alpha * myAccelScaled.x + (1 - ema_alpha) * myAcceScaledfilter.x;
		myAcceScaledfilter.y = ema_alpha * myAccelScaled.y + (1 - ema_alpha) * myAcceScaledfilter.y;
		myAcceScaledfilter.z = ema_alpha * myAccelScaled.z + (1 - ema_alpha) * myAcceScaledfilter.z;

		myGyroScaledfilter.x = ema_alpha * myGyroScaled.x + (1 - ema_alpha) * myGyroScaledfilter.x;
		myGyroScaledfilter.y = ema_alpha * myGyroScaled.y + (1 - ema_alpha) * myGyroScaledfilter.y;
		myGyroScaledfilter.z = ema_alpha * myGyroScaled.z + (1 - ema_alpha) * myGyroScaledfilter.z;
		
		angle[2] = angle[2] + myGyroScaledfilter.z *	0.02;
		sprintf(buffer,"angulo: %f acc_x: %f acc_y: %f\n", angle[2],myAcceScaledfilter.x, myAcceScaledfilter.y);
		uprintf(buffer);
		
		}
	
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C2;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20404768;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 53999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 39;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB14 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : A1_Pin B1_Pin A2_Pin B2_Pin */
  GPIO_InitStruct.Pin = A1_Pin|B1_Pin|A2_Pin|B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  UNUSED(huart);

	bandera=1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim ->Instance == TIM2){
		//sprintf(buffer,"%d\n",count2);
		//uprintf(buffer);
		//count++;
		ha += 1;
		//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
		//TIM4->CCR3= 65535;
		//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
		//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET); 
		
	}
}

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_PIN){ 
	//ENCODER A1
	if( GPIO_PIN == A1_Pin){
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10)==GPIO_PIN_SET){
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10)!=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12)){
				count_1++;
			}
			else{
				count_1--;
			}
		}
		else{
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10)!=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12)){
				count_1++;
			}
			else{
				count_1--;
			}
		}
	}
	//ENCODER B1
	if(GPIO_PIN == B1_Pin){
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12)==GPIO_PIN_SET){
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10)==HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12)){
				count_1++;
			}
			else{
				count_1--;
			}
		}
		else{
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10)==HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12)){
				count_1++;
			}
			else{
				count_1--;
			}
		}		
	}
	//ENCODER A2
	if(GPIO_PIN == A2_Pin){
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14)==GPIO_PIN_SET){
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14)!=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_15)){
				count_2++;
			}
			else{
				count_2--;
			}
		}
		else{
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14)!=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_15)){
				count_2++;
			}
			else{
				count_2--;
			}
		}
		
	}
	if(GPIO_PIN == B2_Pin){
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_15)==GPIO_PIN_SET){
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14)==HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_15)){
				count_2++;
			}
			else{
				count_2--;
			}
		}
		else{
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14)==HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_15)){
				count_2++;
			}
			else{
				count_2--;
			}
		}
		
	}
	
	
	else{
		__NOP();
	}
}

void crear(float **matriz, int fil, int col){
	int i;
	for(i=0;i<fil;i++){
		matriz[i] = (float *) calloc(col,sizeof(float));
	}
}
void valores(float **matriz,int fil, int col){
	int i,j;
	for(i=0;i<fil;i++){
		for(j=0;j<col;j++){
			printf("%d,%d: ",i,j);
			scanf("%f",&matriz[i][j]);
		}
	}
}
void crear_valores(float **matriz,int fil, int col){
	crear(matriz,fil,col);
	valores(matriz,fil,col);
}
void imprimir(float **matriz, int fil, int col){
	int i,j;
	for(i=0;i<fil;i++){
		for(j=0;j<col;j++){
			printf("%f, ",matriz[i][j]);
		}
		printf("\n");
	}
}
void multiplicacion(float **matriz1, float **matriz2, float **matriz3, int fil1, int col1, int fil2, int col2){
	int n,m,i;
	float sum;
	for(n=0; n<fil1;n++){
		
		for(m=0; m<col2;m++){
			
			sum=0;
			for(i=0;i<col1;i++){
			
				sum += matriz1[n][i]*matriz2[i][m];			
			}
			matriz3[n][m] = sum;
		}
	}
}
void eye(float **matriz,int fil){
	crear(matriz,fil,fil);
	int i,j;
	for(i=0;i<fil;i++){
		for(j=0;j<fil;j++){
			if(i==j){
				matriz[i][j]=1;
			}
			else{
				matriz[i][j]=0;
			}
		}
	}
}
void copia(float **matriz1,float **matriz2,int fil,int col){
	int i,j;
	for(i=0;i<fil;i++){
		for(j=0;j<col;j++){
			matriz2[i][j]=matriz1[i][j];
		}
	}
}
void inversa(float **matriz,float **m_inv, int fil){
	float **identy = (float **) calloc(fil,sizeof(float *));
	float **copy = (float **) calloc(fil,sizeof(float *));
	float *b = (float *) calloc(fil,sizeof(float));	
	crear(copy,fil,fil);
	eye(identy,fil);
	int q,m,k,i,j;
	for(q=0; q<fil;q++){
		copia(matriz,copy,fil,fil);
		for(m=0;m<fil;m++){b[m]=identy[m][q];}
		for(k=0;k<(fil-1);k++){
			for(i=(k+1);i<fil;i++){
				float factor = copy[i][k]/copy[k][k];
				copy[i][k] = factor;
				for(j=(k+1);j<fil;j++){
					copy[i][j] = copy[i][j] -factor*copy[k][j];	
				}
			}	
		}
		for (i=1;i<fil;i++){

			float sum = b[i];
			for(j=0;j<=(i-1);j++){
				sum = sum - copy[i][j]*b[j];
			}
			b[i] = sum;
			
		}
		
		m_inv[fil-1][q] = b[fil-1]/copy[fil-1][fil-1];
		for(i=(fil-2);i>=0;i--){
			float sum=0;
			for(j = (i+1);j<fil;j++){
				sum = sum + copy[i][j] * m_inv[j][q];
			}
			m_inv[i][q] = (b[i]-sum)/copy[i][i];
		}	
	}
	free(identy);
	free(copy);
	free(b);	
}
void transpuesta(float **matriz,float **trans, int fil,int col){
	int i,j;
	for(i=0;i<fil;i++){
		for(j=0;j<col;j++){
			trans[j][i] = matriz[i][j]; 		
		}
	}
}
void pseudoinversa(float **matriz, float **pseudo, int fil, int col){
	//m = col , n = fil
	if (fil>col){
		
		float **trans = (float **) calloc(col,sizeof(float *));
		float **ct_c = (float **) calloc(col,sizeof(float *));
		float **ctc_inv = (float **) calloc(col,sizeof(float *));
		crear(trans,col,fil);
		crear(ct_c,col,col);
		crear(ctc_inv,col,col);	
		transpuesta(matriz,trans,fil,col);
		multiplicacion(trans,matriz,ct_c,col,fil,fil,col);
		inversa(ct_c,ctc_inv,col);
		multiplicacion(ctc_inv,trans,pseudo,col,col,col,fil);
	}
	else{
		float **trans = (float **) calloc(col,sizeof(float *));
		float **c_ct = (float **) calloc(fil,sizeof(float *));
		float **ctc_inv = (float **) calloc(fil,sizeof(float *));
		crear(trans,col,fil);
		crear(c_ct,fil,fil);
		crear(ctc_inv,fil,fil);
		transpuesta(matriz,trans,fil,col);
		multiplicacion(matriz,trans,c_ct,fil,col,col,fil);
		inversa(c_ct,ctc_inv,fil);
		multiplicacion(trans,ctc_inv,pseudo,col,fil,fil,fil);
	}
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
