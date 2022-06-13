/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "usbd_cdc_if.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

// klawiatura
int K[8];
GPIO_TypeDef* KI;
GPIO_TypeDef* KO;

// height angle, direction angle
int ha, da;

// LCD
char text[21];
char input[21];
char* ptr = input;

// status pozycji anteny
char status = 0;
// ilość otrzymanych zmiennych
char pos_stat = 0;
// N, E, kąt do równoleżnika
double latitude;
double longitude;
double direction;

// USB?
char dataT[70];
uint32_t Len;
uint8_t buffer[64];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#include "lcd.h"

void blink(void){
	HAL_GPIO_TogglePin(Test_LED_GPIO_Port, Test_LED_Pin);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(Test_LED_GPIO_Port, Test_LED_Pin);
	HAL_Delay(100);
}

// Serwo 1
void init_PWM(void){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	TIM3->CCR3 = 1500;

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	TIM4->CCR4 = 1500;
}

// od 0 do 90 stopni, 0 oznacza pion
void height_angle( int angle ){
	ha = angle;
	float tmp = 893.0/90.0;
	TIM3->CCR3 = 1607 + tmp*angle;
}

// od -135 do 135
void direction_angle( int angle ){
	da = angle;
	float tmp = 1000.0/135.0;
	TIM3->CCR3 = 1500 + tmp*angle;
}

void init_keyboard(void){
	K[0] = K1_Pin;
	K[1] = K2_Pin;
	K[2] = K3_Pin;
	K[3] = K4_Pin;
	K[4] = K5_Pin;
	K[5] = K6_Pin;
	K[6] = K7_Pin;
	K[7] = K8_Pin;

	KI = K1_GPIO_Port;
	KO = K5_GPIO_Port;
}

// Odczytaj numer przycisku klikniętego na klawiaturze (odczytany tylko o najniższym numerze)
// Numerowanie od lewego górnego rogu, od 0
int keyboard(void){
	for(int i=4; i<8; i++){
		HAL_GPIO_TogglePin(KO, K[i]);
		for(int j=0; j<4; j++)
			if( HAL_GPIO_ReadPin(KI, K[j]) ){
				HAL_GPIO_TogglePin(KO, K[i]);
				return (i-4)+j*4;
			}
		HAL_GPIO_TogglePin(KO, K[i]);
	}
	return -1;
}

void display_angle(){
	lcd_clear();
	sprintf(text, "Wysokość: %03d",ha);
	lcd_send_string(text);

	lcd_line(1);
	sprintf(text, "Kierunek: %03d",da);
	lcd_send_string(text);
}

void set_ANSTRA_pos(){
	lcd_clear();
	sprintf(text, "Pozycja ANSTRY:" );
	lcd_send_string(text);

	lcd_line(1);
}

void set_pos( int k ){
	if( k == 15 ){
		*ptr = '\0';
		double ret = strtod(input,&ptr);
		if( ptr == input )
			blink();

		ptr = input;
		if( pos_stat == 0 )
			latitude = ret;

		if( pos_stat == 1 )
			longitude = ret;

		if( pos_stat == 2 )
			direction = ret;

		pos_stat++;
		if( pos_stat == 3 ){
			status = 1;
			lcd_clear();
			sprintf(text, "%.5lf",latitude );
			lcd_send_string(text);
			lcd_line(1);
			sprintf(text, "%.5lf",longitude );
			lcd_send_string(text);
			lcd_line(2);
			sprintf(text, "%.5lf",direction );
			lcd_send_string(text);
			lcd_line(3);
		}

		lcd_line(pos_stat+1);

		return;
	}

	char c;
	switch(k){
		case 12:
			c = '-';
			break;
		case 14:
			c = '.';
			break;
		case 0:
			c = '1';
			break;
		case 1:
			c = '2';
			break;
		case 2:
			c = '3';
			break;
		case 4:
			c = '4';
			break;
		case 5:
			c = '5';
			break;
		case 6:
			c = '6';
			break;
		case 8:
			c = '7';
			break;
		case 9:
			c = '8';
			break;
		case 10:
			c = '9';
			break;
		case 13:
			c = '0';
			break;
		default:
			c = -1;
	}

	if( c > 0 ){

		lcd_send_data(c);
		*(ptr++) = c;
	}

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
//	init_keyboard();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  init_keyboard();
  init_PWM();
  lcd_init();

  height_angle(90);
  direction_angle(0);

  set_ANSTRA_pos();

  int k;
  int tmp;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  tmp = keyboard();
	  if( tmp >= 0 && k == -1 ){
		  if( !status )
			  set_pos(tmp);
	  }
	  k = tmp;
	  HAL_Delay(100);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Test_LED_GPIO_Port, Test_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, K8_Pin|K7_Pin|K6_Pin|K5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Test_LED_Pin */
  GPIO_InitStruct.Pin = Test_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Test_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : K1_Pin K2_Pin K3_Pin K4_Pin */
  GPIO_InitStruct.Pin = K1_Pin|K2_Pin|K3_Pin|K4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : K8_Pin K7_Pin K6_Pin K5_Pin */
  GPIO_InitStruct.Pin = K8_Pin|K7_Pin|K6_Pin|K5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
