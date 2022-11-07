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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "update.h"
#include "stdlib.h"
#include "sounds.h"

//#include "stdio.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint8_t screen_refresh = 0;
volatile uint8_t button_mode    = 0;
volatile uint8_t launch         = 0;

struct ball_info{
	uint16_t x;
	uint16_t y;
	uint8_t w;
	uint8_t h;
	int16_t angle;
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef status_now;
uint8_t i2cdata[10];
int16_t ax;
int16_t ay;
int16_t az;
int pad_x = 144;
int pad_y = 187;
int padW  = 32;
int padH  = 8;

FRESULT fres;
FATFS fs;
FIL fil;
UINT numwritten;
UINT numread;
uint8_t highscore[3];
uint8_t dataIn[3];
int currHS = 0;

#define screen_start (uint8_t*)0x20020000
uint8_t* scrn;
uint16_t score = 0;
uint8_t moving_now = 0;
uint8_t moving_up  = 0;

void updateBall(struct ball_info* ball_state, uint8_t speed, int padx, int pady, int16_t pad_ax,uint8_t moving_now1);
void BitBlit(unsigned char* ptr_screen, unsigned char* ptr_sprite, int x, int y, int sprite_w, int sprite_h);
void drawAll(int drawWhen1[],int drawWhen2[],int drawWhen3[],int drawWhen4[],int drawWhen5[]);
void clear(unsigned char* ptr_screen, int x, int y, int sprite_w, int sprite_h);
void drawHorizontalPart(unsigned char* ptr_screen, int x, int y, int col);
void drawVerticalPart(unsigned char* ptr_screen, int x, int y, int col);
void drawBrick(unsigned char* ptr_screen,int x, int y, int col);
void drawNumber(int num, int x, int y, int col);
void drawRow(int y,int col, int drawWhen[]);
void drawLetter(int let, int x, int y);
void drawScore(int num, int type);
void storeHighScore(int num);
void initializeHighScore();
void getInitialHighScore();
void gameOver(int col);
void titleScreen();
void gameReset();
void drawWall();


int lives = 3;
int ballH = 4;
int ballW = 4;

int drawWhen0[13] = {1,1,1,1,1,1,1,1,1,1,1,1,1};
int drawWhen1[13] = {1,1,1,1,1,1,1,1,1,1,1,1,1};
int drawWhen2[13] = {1,1,1,1,1,1,1,1,1,1,1,1,1};
int drawWhen3[13] = {1,1,1,1,1,1,1,1,1,1,1,1,1};
int drawWhen4[13] = {1,1,1,1,1,1,1,1,1,1,1,1,1};
int drawWhen5[13] = {1,1,1,1,1,1,1,1,1,1,1,1,1};
int eraseRow5[13] = {0,0,0,0,0,0,0,0,0,0,0,0,0};

uint8_t cmdbuffer[6];
uint8_t resp;
uint8_t outstr[1000];
uint32_t outlen;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	  uint8_t Ball[17] = {0x00, 0x0F, 0x0F, 0x00,
	   0x0F, 0x0B, 0x0B, 0x0F,
	   0x0F, 0x0B, 0x0B, 0x0F,
	   0x00, 0x0F, 0x0F, 0x00};

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
	  uint8_t Paddle[256] = {0x00, 0x00, 0x00, 0x06, 0x06, 0x06, 0x06, 0x10, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x10, 0x06, 0x06, 0x06, 0x06, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x10, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x10, 0x0C, 0x0C, 0x0C, 0x0C, 0x06, 0x00, 0x00,
	  0x00, 0x0B, 0x0C, 0x0F, 0x0F, 0x0F, 0x0F, 0x10, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x10, 0x0F, 0x0F, 0x0F, 0x0F, 0x06, 0x0B, 0x00,
	  0x0B, 0x0F, 0x0F, 0x0C, 0x0C, 0x0C, 0x0C, 0x10, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x10, 0x0C, 0x0C, 0x0C, 0x0C, 0x06, 0x0F, 0x0B,
	  0x0B, 0x0B, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x10, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x10, 0x0C, 0x0C, 0x0C, 0x0C, 0x06, 0x0B, 0x0B,
	  0x00, 0x0B, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x10, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x10, 0x0C, 0x0C, 0x0C, 0x0C, 0x06, 0x0B, 0x00,
	  0x00, 0x00, 0x0C, 0x06, 0x06, 0x06, 0x06, 0x10, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x16, 0x10, 0x06, 0x06, 0x06, 0x06, 0x06, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x06, 0x06, 0x06, 0x06, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x06, 0x06, 0x06, 0x06, 0x00, 0x00, 0x00,};

	  i2cdata[0] = 0x20;
	  i2cdata[1] = 0x47;
	  status_now = HAL_I2C_Master_Transmit(&hi2c1, 0x32, i2cdata, 2, 10);

	  i2cdata[0] = 0x23;
	  i2cdata[1] = 0x30;
	  status_now = HAL_I2C_Master_Transmit(&hi2c1, 0x32, i2cdata, 2, 10);

	  titleScreen();

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);

	  initializeHighScore();
      getInitialHighScore();
	  drawScore(currHS, 1);

  	  struct ball_info ball_now = {157,186, ballW, ballH, -30};
  	  drawWall();
  	  drawAll(drawWhen1,drawWhen2,drawWhen3,drawWhen4,drawWhen5);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if ( (launch == 0) ){
		  drawBrick((unsigned char*)0x20020000,ball_now.x,ball_now.y -1,0x00);//0x20020C80
		  ball_now.x     = 157;
		  ball_now.y     = 186;
		  ball_now.angle = -30;
		  moving_up 	 = 1;
		  BitBlit(screen_start,Ball,157,186,ball_now.w,ball_now.h);
	  }
	  if (button_mode == 0){
	  	  	if (screen_refresh){
				clear(screen_start, pad_x, pad_y, padW, padH);
				if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9)){
					pad_x = pad_x + 18;
					ax = 8;
					moving_now = 1;
				}
				else if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10)){
					pad_x = pad_x - 18;
					ax = -8;
					moving_now = -1;
				}
				else{
					moving_now = 0;
				}
										  if ( ball_now.angle > 0 ){moving_up 	 = 0;}
								     else if ( ball_now.angle < 0){moving_up 	 = 1;}
										  if ((pad_x + padW)>320 ){pad_x = 320-padW;}
										  if ( pad_x < 0 ) {pad_x = 0;}
	  									  clear(screen_start,ball_now.x,ball_now.y,ball_now.w,ball_now.h);
	  									  if (launch){
	  										  updateBall(&ball_now, 8,pad_x,pad_y,ax,moving_now);
	  										  BitBlit(screen_start,Ball,ball_now.x,ball_now.y,ball_now.w,ball_now.h);
	  									  }
	  									  screen_refresh = 0;
	  	  	}
	  	}
	    else if (button_mode == 1){
  			  i2cdata[0] = 0xA8;
  			  status_now = HAL_I2C_Master_Transmit(&hi2c1, 0x32, i2cdata, 1, 10);
  			  status_now = HAL_I2C_Master_Receive(&hi2c1, 0x32, i2cdata, 6, 10);

  			  ax  = *((int16_t*)i2cdata);
  			  ax /= 10;
  			  if ( ax >  159) ax =  159;
  			  if ( ax < -160) ax = -160;
  			  if (screen_refresh){
  				  //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
  			  clear(screen_start, pad_x, pad_y, padW, padH);
  			  HAL_Delay(1);
  			  if (ax >0)
  				  pad_x = pad_x + 18;
  			  if (ax <0)
  				  pad_x = pad_x -18;
										  if ( ball_now.angle > 0 ){moving_up 	 = 0;}
									 else if ( ball_now.angle < 0){moving_up 	 = 1;}
										  if ((pad_x + padW)>320 ){pad_x = 320-padW;}
										  if (pad_x < 0 ) {pad_x = 0;}
										  clear(screen_start,ball_now.x,ball_now.y,ball_now.w,ball_now.h);
										  if (launch){
											  updateBall(&ball_now, 8,pad_x,pad_y,ax,moving_now);
											  BitBlit(screen_start,Ball,ball_now.x,ball_now.y,ball_now.w,ball_now.h);
										  }
										  screen_refresh = 0;
			  }
	      }
	  	  else if(button_mode == 2){
	  		  if (screen_refresh){
	  			  	HAL_ADC_Start(&hadc1);
	  			  	HAL_ADC_PollForConversion(&hadc1, 10);
	  			  	uint16_t adcval = HAL_ADC_GetValue(&hadc1);

	  		  		clear(screen_start, pad_x, pad_y, padW, padH);
	  		  		if (adcval > 2042){
	  		  			ax = 8;
	  		  			pad_x = pad_x + 18;
	  		  		}
	  		  		if (adcval < 2042){
	  		  			ax = 8;
	  		  			pad_x = pad_x - 18;
	  		  		}

										  if ( ball_now.angle > 0 ){moving_up 	 = 0;}
									 else if ( ball_now.angle < 0){moving_up 	 = 1;}
										  if ((pad_x + padW)>320 ){pad_x = 320-padW;}
										  if (pad_x < 0 ) {pad_x = 0;}
										  clear(screen_start,ball_now.x,ball_now.y,ball_now.w,ball_now.h);
										  if(launch){
											  updateBall(&ball_now, 8,pad_x,pad_y,ax,moving_now);
											  BitBlit(screen_start,Ball,ball_now.x,ball_now.y,ball_now.w,ball_now.h);
										  }
										  screen_refresh = 0;
	  		  }
	  	  }
 	  BitBlit(screen_start, Paddle, pad_x, pad_y, padW, padH);
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

  /** Macro to configure the PLL multiplication factor 
  */
  __HAL_RCC_PLL_PLLM_CONFIG(16);
  /** Macro to configure the PLL clock source 
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);
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
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD9 PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
void BitBlit(unsigned char* ptr_screen, unsigned char* ptr_sprite, int x, int y, int sprite_w, int sprite_h){
	 unsigned char* screenPos = ptr_screen+x+y*320;
	 unsigned char* spritePos = ptr_sprite;

	 for(int i = 0; i < sprite_h; ++i){
		 for(int j = 0; j < sprite_w; ++j){
			 if(*spritePos != 0)
				 *(screenPos+i*320+j) = *spritePos;
		 }
		 spritePos = spritePos +1;
	 }
	 screenPos = screenPos + (320 - sprite_w);
}

void clear(unsigned char* ptr_screen, int x, int y, int sprite_w, int sprite_h){
	 unsigned char* screenPos = ptr_screen+x+y*320;

	 for(int i = 0; i < sprite_h; i++){
		 for(int j = 0; j < sprite_w; j++){
			 *(screenPos+i*320+j) = 0;
		 }
	}
}

void updateBall(struct ball_info* ball_state, uint8_t speed, int padx, int pady, int16_t pad_ax,uint8_t moving_now1){
	int ball_x = ball_state->x + deltaX(speed, ball_state->angle);
	int ball_y = ball_state->y + deltaY(speed, ball_state->angle);
	//////  Bounce off left and right  /////////////////////////////////////////////////////////////////////
	if(ball_x < 0){
		ball_state->angle = 180 - ball_state->angle;
		ball_state->x     = -ball_x;
	}
	else if(ball_x+ballW > 320){
		ball_state->angle = 180 - ball_state->angle;
		ball_state->x     = ball_x - 2*( (ball_x + ball_state->w) - 320);
	}
	else{
		ball_state->x = ball_x;
	}
	//////  Bounce off top and bottom  /////////////////////////////////////////////////////////////////////
	if (ball_y < 37){
		ball_state->angle = 360 - ball_state->angle;
		ball_state->y     = ball_y + 2*((ball_y + ball_state->h) - 35);
		drawWall();
	}
	else if(ball_y+ ball_state->h > 200){
		launch = 0;
		lives = lives - 1;

		storeHighScore(currHS);
		if (lives  == 0){
			int drawWhen[13] = {1,1,1,1,1,1,1,1,1,1,1,1,1};
				gameOver(0x31);
				HAL_Delay(30);
				gameOver(0x00);
				gameReset();
		}
	}
	//////   Bounce off paddle   /////////////////////////////////////////////////////////////////////
	else if((ball_x > padx) && (ball_x < (padx+padW)) && (ball_y + ball_state->h > pady) ){
		ball_state->angle = 360 - ball_state->angle;
		ball_state->y     = ball_y - 2*( (ball_y + ball_state->h) - pady);
		HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)paddle_hit, LEN_paddle_hit);
			if ((moving_now1 > 0) && ( ball_state->angle > 90 )){
				ball_state->angle = 180 - ball_state->angle;
			}
			else if ((moving_now1 < 0) && ( ball_state->angle < 90 )){
				ball_state->angle = 180 - ball_state->angle;
			}
	}
	else{
		ball_state->y = ball_y;
	}
	if (ball_state->angle > 180)
		ball_state->angle -= 360;
	if (ball_state->angle < -180)
		ball_state->angle += 360;
//###################  Bounce off bricks  ###################################################################
//************* code 5 pm start
	if (ball_state->angle < 0){
	if ( (ball_state->y - ball_state->h  < 64) ){
		div_t where_x5 = div(ball_state->x,24);
		if (drawWhen5[where_x5.quot] != 0){
		if (where_x5.quot > 12){where_x5.quot = 12;}
		int tx5 			   	   = (where_x5.quot ) * 24;
		ball_state->angle 		   = 360 - ball_state->angle;
		ball_state->y              = ball_y + 2*((ball_y + ball_state->h) - 64);
		score   				   = score + 10;
		drawScore(score,2);
		if (score > currHS){
			currHS = score;
			drawScore(currHS, 1);
		}
		if (eraseRow5[where_x5.quot]  == 1){
			drawWhen5[where_x5.quot]   = 0;
			drawBrick((unsigned char*)0x20024B00, tx5,0,0x00);
			HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)brick_break, LEN_brick_break);
		}
		else{
			drawBrick((unsigned char*)0x20024B00, tx5,0,0x07);
		}
		eraseRow5[where_x5.quot]   += 1;
		}
	}
	else if ( (ball_state->y - ball_state->h  < 72) ){
		div_t where_x4 = div(ball_state->x,24);
		if (drawWhen4[where_x4.quot] != 0){
		if (where_x4.quot > 12){where_x4.quot = 12;}
		int tx4 			   	   = (where_x4.quot) * 24;
		ball_state->angle 		   = 360 - ball_state->angle;
		drawWhen4[where_x4.quot]  = 0;
		score   				   = score + 7;
		drawScore(score,2);
		if (score > currHS){
			currHS = score;
			drawScore(currHS, 1);
		}
		HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)brick_break, LEN_brick_break);
		drawBrick((unsigned char*)0x20024B00, tx4,8,0x00);
		}
	}
	else if ( (ball_state->y - ball_state->h  < 80) ){
		div_t where_x3 = div(ball_state->x,24);
		if (drawWhen3[where_x3.quot] != 0){
		if (where_x3.quot > 12){where_x3.quot = 12;}
		int tx3 			   	   = (where_x3.quot) * 24;
		ball_state->angle 		   = 360 - ball_state->angle;
		drawWhen3[where_x3.quot] 	   = 0;
		score   					   = score + 5;
		drawScore(score,2);
		if (score > currHS){
			currHS = score;
			drawScore(currHS, 1);
		}
		HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)brick_break, LEN_brick_break);
		drawBrick((unsigned char*)0x20024B00, tx3,16,0x00);
		}
	}
	else if ( (ball_state->y - ball_state->h  < 88) ){
		div_t where_x2 = div(ball_state->x,24);
		if (drawWhen2[where_x2.quot] != 0){
		if (where_x2.quot > 12){where_x2.quot = 12;}
		int tx2 			   	   = (where_x2.quot) * 24;
		ball_state->angle 		   = 360 - ball_state->angle;
		drawWhen2[where_x2.quot]      = 0;
		score   					   = score + 3;
		drawScore(score,2);
		if (score > currHS){
			currHS = score;
			drawScore(currHS, 1);
		}
		HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)brick_break, LEN_brick_break);
		drawBrick((unsigned char*)0x20024B00, tx2,24,0x00);
		}
	}
	else if ( (ball_state->y  <= 96)  ){
		div_t where_x1 = div(ball_state->x,24);
		if (drawWhen1[where_x1.quot] != 0){
		if (where_x1.quot > 12){where_x1.quot = 12;}
		int tx1 			   	   = (where_x1.quot) * 24;
		ball_state->angle 		   = 360 - ball_state->angle;
		drawWhen1[where_x1.quot]   = 0;
		score                      = score + 1;
		drawScore(score,2);
		if (score > currHS){
			currHS = score;
			drawScore(currHS, 1);
		}
		HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)brick_break, LEN_brick_break);
		drawBrick((unsigned char*)0x20024B00, tx1,32,0x00);
	}
	}
}

if (ball_state->angle > 0){
	if (ball_state->y > 96){
		drawScore(score, 2);
	}
	else if ( (ball_state->y  >= 89 )  ){
		div_t where_x1 = div(ball_state->x,24);
		if (drawWhen1[where_x1.quot] != 0){
		if (where_x1.quot > 12){where_x1.quot = 12;}
		int tx1 			   	   = (where_x1.quot) * 24;
		ball_state->angle 		   = 360 - ball_state->angle;
		drawWhen1[where_x1.quot]   = 0;
		score                      = score + 1;
		drawScore(score,2);
		if (score > currHS){
			currHS = score;
			drawScore(currHS, 1);
		}
		HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)brick_break, LEN_brick_break);
		drawBrick((unsigned char*)0x20024B00, tx1,32,0x00);
		}
	}
	else if ( (ball_state->y  >= 82)  ){
	 div_t where_x2 = div(ball_state->x,24);
	 if (drawWhen2[where_x2.quot] != 0){
		if (where_x2.quot > 12){where_x2.quot = 12;}
		int tx2 			   	   = (where_x2.quot) * 24;
		ball_state->angle 		   = 360 - ball_state->angle;
		drawWhen2[where_x2.quot]   = 0;
		score                      = score + 3;
		drawScore(score,2);
		if (score > currHS){
			currHS = score;
			drawScore(currHS, 1);
		}
		HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)brick_break, LEN_brick_break);
		drawBrick((unsigned char*)0x20024B00, tx2,24,0x00);
	}
	}
	else if ( (ball_state->y  >= 74)  ){
	 div_t where_x3 = div(ball_state->x,24);
	 if (drawWhen3[where_x3.quot] != 0){
		if (where_x3.quot > 12){where_x3.quot = 12;}
		int tx3 			   	   = (where_x3.quot) * 24;
		ball_state->angle 		   = 360 - ball_state->angle;
		drawWhen3[where_x3.quot]   = 0;
		score                      = score + 5;
		drawScore(score,2);
		if (score > currHS){
			currHS = score;
			drawScore(currHS, 1);
		}
		HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)brick_break, LEN_brick_break);
		drawBrick((unsigned char*)0x20024B00, tx3,16,0x00);
	}
	}
	else if ( (ball_state->y  >= 66)  ){
	 div_t where_x4 = div(ball_state->x,24);
	 if (drawWhen4[where_x4.quot] != 0){
		if (where_x4.quot > 12){where_x4.quot = 12;}
		int tx4 			   	   = (where_x4.quot) * 24;
		ball_state->angle 		   = 360 - ball_state->angle;
		drawWhen4[where_x4.quot]   = 0;
		score                      = score + 7;
		drawScore(score,2);
		if (score > currHS){
			currHS = score;
			drawScore(currHS, 1);
		}
		HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)brick_break, LEN_brick_break);
		drawBrick((unsigned char*)0x20024B00, tx4,8,0x00);
	}
	}
	else if ( (ball_state->y  >60)  ){
	 div_t where_x5 = div(ball_state->x,24);
	 if (drawWhen5[where_x5.quot] != 0){
		if (where_x5.quot > 12){where_x5.quot = 12;}
		int tx5 			   	   = (where_x5.quot) * 24;
		ball_state->angle 		   = 360 - ball_state->angle;
		score                      = score + 10;
		drawScore(score,2);
		if (score > currHS){
			currHS = score;
			drawScore(currHS, 1);
		}
		if (eraseRow5[where_x5.quot] == 1){
			drawWhen5[where_x5.quot]  = 0;
			drawBrick((unsigned char*)0x20024B00, tx5,0,0x00);
			HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)brick_break, LEN_brick_break);
		}
		else{
			drawBrick((unsigned char*)0x20024B00, tx5,0,0x07);
		}
		eraseRow5[where_x5.quot] += 1;
		}
		}
	}
//************* code 5 pm end
}

void drawBrick(unsigned char* ptr_screen,int x, int y, int col){
	unsigned char* drawpos = ptr_screen+x+y*320;
		 for(int i = 0; i < 4; ++i){
			 for(int j = 0; j < 23; ++j){
					 *(drawpos+i*320+j) = col;
			 }
		 }
}

void drawRow(int y,int col, int drawWhen[]){
	for (int i = 0; i < 13; i++){
		int x = 0 + 24 * i*drawWhen[i];
		drawBrick((unsigned char*)0x20024B00,x,y,col);	// start at 60 rows, brick height = 4
	}
}

void drawAll(int drawWhen11[],int drawWhen22[],int drawWhen33[],int drawWhen44[],int drawWhen55[]){
	// fifth layer is boss layer = 2 hits i.e top not bottom
	  drawRow(0, 0x07,drawWhen11);
	  drawRow(8, 0x28,drawWhen22);
	  drawRow(16,0x2A,drawWhen33);
	  drawRow(24,0x2C,drawWhen44);
	  drawRow(32,0x31,drawWhen55);
}

void gameOver(int col){
		int drawWhen[13] = {1,1,1,1,1,1,1,1,1,1,1,1,1};
		for (int i = 0; i < 10; i++){
			drawRow(i*8, col, drawWhen);
			HAL_Delay(5);
		}
		HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)game_over, LEN_game_over);
		storeHighScore(currHS);
}

void drawWall(){
			unsigned char* ptr_st = (unsigned char*)0x20022BC0;
		 for(int i = 0; i < 3; ++i){
			 for(int j = 0; j < 320; ++j){
					 *(ptr_st+i*320+j) = 0x34;
			 }
		 }
}

void drawHorizontalPart(unsigned char* ptr_screen, int x, int y, int col){
	unsigned char* drawpos = ptr_screen+x+y*320;
	for(int i = 0; i < 2; ++i){
		for(int j = 0; j < 7; ++j){
		  *(drawpos+i*320+j) = col;
		}
	}
}

void drawVerticalPart(unsigned char* ptr_screen, int x, int y, int col){
	unsigned char* drawpos = ptr_screen+x+y*320;
	for(int i = 0; i < 6; ++i){
		for(int j = 0; j <2; ++j){
		  *(drawpos+i*320+j) = col;
		}
	}
}

void drawNumber(int num, int x, int y, int col){
	//unsigned char* screenPos = 0x20020C80+x+y*320;
	if (num == 1){
		drawVerticalPart((unsigned char*)0x20020C80,x,y,col);
		drawVerticalPart((unsigned char*)0x20020C80,x,y+6,col);
	}
	else if (num == 2){
		drawHorizontalPart((unsigned char*)0x20020C80,x, y,col);
		drawHorizontalPart((unsigned char*)0x20020C80,x,y+5,col);
		drawHorizontalPart((unsigned char*)0x20020C80,x,y+10,col);
		drawVerticalPart((unsigned char*)0x20020C80,x+5,y,col);
		drawVerticalPart((unsigned char*)0x20020C80,x,y+5,col);
	}
	else if (num == 3){
		drawHorizontalPart((unsigned char*)0x20020C80,x, y,col);
		drawHorizontalPart((unsigned char*)0x20020C80,x, y+5,col);
		drawHorizontalPart((unsigned char*)0x20020C80,x, y+10,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x+5, y,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x+5, y+5,col);
	}
	else if (num == 4){
		drawHorizontalPart((unsigned char*)0x20020C80,x, y+5,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x, y,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x+5, y,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x+5, y+5,col);
	}
	else if (num == 5){
		drawHorizontalPart((unsigned char*)0x20020C80,x, y,col);
		drawHorizontalPart((unsigned char*)0x20020C80,x, y+5,col);
		drawHorizontalPart((unsigned char*)0x20020C80,x, y+10,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x, y,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x+5, y+5,col);
	}
	else if (num == 6){
		drawHorizontalPart((unsigned char*)0x20020C80,x, y,col);
		drawHorizontalPart((unsigned char*)0x20020C80,x, y+5,col);
		drawHorizontalPart((unsigned char*)0x20020C80,x, y+10,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x, y,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x, y+5,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x+5, y+5,col);
	}
	else if (num == 7){
		drawHorizontalPart((unsigned char*)0x20020C80,x, y,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x+5, y,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x+5, y+5,col);
	}
	else if (num == 8){
		drawHorizontalPart((unsigned char*)0x20020C80,x, y,col);
		drawHorizontalPart((unsigned char*)0x20020C80,x, y+5,col);
		drawHorizontalPart((unsigned char*)0x20020C80,x, y+10,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x, y,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x+5, y,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x, y+5,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x+5, y+5,col);
	}
	else if (num == 9){
		drawHorizontalPart((unsigned char*)0x20020C80,x, y,col);
		drawHorizontalPart((unsigned char*)0x20020C80,x, y+5,col);
		drawHorizontalPart((unsigned char*)0x20020C80,x, y+10,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x, y,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x+5, y,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x+5, y,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x+5, y+5,col);
	}
	else if (num == 0){
		drawHorizontalPart((unsigned char*)0x20020C80,x, y,col);
		drawHorizontalPart((unsigned char*)0x20020C80,x, y+10,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x, y,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x, y+5,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x+5, y,col);
		drawVerticalPart(  (unsigned char*)0x20020C80,x+5, y+5,col);
	}
}

void drawScore(int num, int type){
	int x = 10;
	int col = 0x00;
	if (type == 1){
		x   = 290;
		col = 0x31;
	}
	else if (type == 2){
		x   = 30;
		col = 0x0F;
	}
	drawBrick((unsigned char*)0x20020C80, x-15,3,0x00);
	drawBrick((unsigned char*)0x20020C80, x-15,7,0x00);
	drawBrick((unsigned char*)0x20020C80, x-15,11,0x00);
	drawBrick((unsigned char*)0x20020C80, x-15,15,0x00);
	drawBrick((unsigned char*)0x20020C80, x-15,18,0x00);
	drawBrick((unsigned char*)0x20020C80, x-5,3,0x00);
	drawBrick((unsigned char*)0x20020C80, x-5,7,0x00);
	drawBrick((unsigned char*)0x20020C80, x-5,11,0x00);
	drawBrick((unsigned char*)0x20020C80, x-5,15,0x00);
	drawBrick((unsigned char*)0x20020C80, x-5,18,0x00);
	drawBrick((unsigned char*)0x20020C80, x+15,3,0x00);
	drawBrick((unsigned char*)0x20020C80, x+15,7,0x00);
	drawBrick((unsigned char*)0x20020C80, x+15,11,0x00);
	drawBrick((unsigned char*)0x20020C80, x+15,15,0x00);
	drawBrick((unsigned char*)0x20020C80, x+15,18,0x00);

	if (num < 10) {
		drawNumber(num, x+10, 5, col);
	}
	else if ( (num >= 10 ) && (num < 100) ){
		div_t num_x = div(num,10);
		drawNumber(num_x.quot,x,5, col);
		drawNumber(num_x.rem, x+10, 5, col);
	}
	else if ( num > 100 ){
		div_t num_x2 = div(num,100);
		drawNumber(num_x2.quot,x-10,5, col);
		int num2 = num - num_x2.quot * 100;
		div_t num_x3 = div(num2,10);
		drawNumber(num_x3.quot,x,5, col);
		drawNumber(num_x3.rem, x+10,5, col);
	}
}

void initializeHighScore(){
 highscore[0] = 0;
 highscore[1] = 2;
 highscore[2] = 3;

 	fres = f_mount(&fs, "",1 );
	fres = f_open(&fil, "hs.txt", FA_WRITE | FA_CREATE_ALWAYS);
	fres = f_write(&fil, highscore, 3, &numwritten);

 	f_close(&fil);
}

void getInitialHighScore(){
	fres = f_open(&fil, "hs.txt", FA_READ);
	fres = f_read(&fil, dataIn, 3, &numread);
	f_close(&fil);
	int ii;
	for (ii = 0; ii < 3; ii++)
		currHS = 10 * currHS + dataIn[ii];
}

void storeHighScore(int num){
 div_t hs_1 = div(num,100);
 highscore[0] = hs_1.quot;
 div_t hs_23 = div(num,10);
 highscore[1] = hs_23.quot;
 highscore[2] = hs_23.rem;

 fres = f_mount(&fs, "",1 );
 fres = f_open(&fil, "hs.txt", FA_WRITE | FA_CREATE_ALWAYS);
 fres = f_write(&fil, highscore, 3, &numwritten);
 f_close(&fil);
}

void drawLetter(int let, int x, int y){
	// let: 0 = B , 	1 = R,  2= I, 3= C, 4 = K, 5  = E, 6 = A
	int col = 0x23;

	if (let == 0){ // B
		drawHorizontalPart((unsigned char*)0x20020000,x,y,col);
		drawHorizontalPart((unsigned char*)0x20020000,x,y+5,col);
		drawHorizontalPart((unsigned char*)0x20020000,x,y+10,col);
		drawVerticalPart((unsigned char*)0x20020000,x,y,col);
		drawVerticalPart((unsigned char*)0x20020000,x,y+5,col);
		drawVerticalPart((unsigned char*)0x20020000,x+5,y,col);
		drawVerticalPart((unsigned char*)0x20020000,x+5,y+5,col);
	}
	else if (let == 1){ // R
		drawHorizontalPart((unsigned char*)0x20020000,x,y,col);
		drawHorizontalPart((unsigned char*)0x20020000,x,y+5,col);
		drawVerticalPart((unsigned char*)0x20020000,x,y,col);
		drawVerticalPart((unsigned char*)0x20020000,x,y+5,col);
		drawVerticalPart((unsigned char*)0x20020000,x+5,y,col);
		drawVerticalPart((unsigned char*)0x20020000,x+5,y+5,col);
	}
	else if (let == 2){ // I
		drawVerticalPart((unsigned char*)0x20020000,x,y+5,col);
		drawVerticalPart((unsigned char*)0x20020000,x,y,col);
	}
	else if (let == 3){ // C
		drawHorizontalPart((unsigned char*)0x20020000,x,y,col);
		drawHorizontalPart((unsigned char*)0x20020000,x,y+10,col);
		drawVerticalPart((unsigned char*)0x20020000,x,y,col);
		drawVerticalPart((unsigned char*)0x20020000,x,y+5,col);
	}
	else if (let == 4){ // K
		drawHorizontalPart((unsigned char*)0x20020000,x,y+5,col);
		drawVerticalPart((unsigned char*)0x20020000,x,y,col);
		drawVerticalPart((unsigned char*)0x20020000,x,y+5,col);
		drawVerticalPart((unsigned char*)0x20020000,x+5,y,col);
		drawVerticalPart((unsigned char*)0x20020000,x+5,y+5,col);
	}
	else if (let == 5){ // E
		drawHorizontalPart((unsigned char*)0x20020000,x,y,col);
		drawHorizontalPart((unsigned char*)0x20020000,x,y+5,col);
		drawHorizontalPart((unsigned char*)0x20020000,x,y+10,col);
		drawVerticalPart((unsigned char*)0x20020000,x,y,col);
		drawVerticalPart((unsigned char*)0x20020000,x,y+5,col);
	}
	else if (let == 6){ // A
		drawHorizontalPart((unsigned char*)0x20020000,x,y,col);
		drawHorizontalPart((unsigned char*)0x20020000,x,y+5,col);
		drawVerticalPart((unsigned char*)0x20020000,x,  y,col);
		drawVerticalPart((unsigned char*)0x20020000,x,  y+5,col);
		drawVerticalPart((unsigned char*)0x20020000,x+5,y,col);
	}
}

void titleScreen(){
	// let: 0 = B , 	1 = R,  2= I, 3= C, 4 = K, 5  = E, 6 = A
	drawLetter(0,100, 100);
	drawLetter(1,110, 100);
	drawLetter(2,120, 100);
	drawLetter(3,130, 100);
	drawLetter(4,140, 100);
	drawLetter(0,160, 100);
	drawLetter(1,170, 100);
	drawLetter(5,180, 100);
	drawLetter(6,190, 100);
	drawLetter(3,200, 100);
	drawLetter(4,210, 100);
	drawLetter(5,220, 100);
	drawLetter(1,230, 100);
	HAL_Delay(500);
	drawBrick((unsigned char*)0x20020000, 98, 99, 0x00);
	drawBrick((unsigned char*)0x20020000, 120, 99, 0x00);
	drawBrick((unsigned char*)0x20020000, 140, 99, 0x00);
	drawBrick((unsigned char*)0x20020000, 160, 99, 0x00);
	drawBrick((unsigned char*)0x20020000, 180, 99, 0x00);
	drawBrick((unsigned char*)0x20020000, 200, 99, 0x00);
	drawBrick((unsigned char*)0x20020000, 220, 99, 0x00);
		HAL_Delay(20);
	drawBrick((unsigned char*)0x20020000,  98, 103, 0x00);
	drawBrick((unsigned char*)0x20020000, 120, 103, 0x00);
	drawBrick((unsigned char*)0x20020000, 140, 103, 0x00);
	drawBrick((unsigned char*)0x20020000, 160, 103, 0x00);
	drawBrick((unsigned char*)0x20020000, 180, 103, 0x00);
	drawBrick((unsigned char*)0x20020000, 200, 103, 0x00);
	drawBrick((unsigned char*)0x20020000, 220, 103, 0x00);
		HAL_Delay(20);
	drawBrick((unsigned char*)0x20020000,  98, 107, 0x00);
	drawBrick((unsigned char*)0x20020000, 120, 107, 0x00);
	drawBrick((unsigned char*)0x20020000, 140, 107, 0x00);
	drawBrick((unsigned char*)0x20020000, 160, 107, 0x00);
	drawBrick((unsigned char*)0x20020000, 180, 107, 0x00);
	drawBrick((unsigned char*)0x20020000, 200, 107, 0x00);
	drawBrick((unsigned char*)0x20020000, 220, 107, 0x00);
		HAL_Delay(20);
	drawBrick((unsigned char*)0x20020000,  98, 111, 0x00);
	drawBrick((unsigned char*)0x20020000, 120, 111, 0x00);
	drawBrick((unsigned char*)0x20020000, 140, 111, 0x00);
	drawBrick((unsigned char*)0x20020000, 160, 111, 0x00);
	drawBrick((unsigned char*)0x20020000, 180, 111, 0x00);
	drawBrick((unsigned char*)0x20020000, 200, 111, 0x00);
	drawBrick((unsigned char*)0x20020000, 220, 111, 0x00);
}

void gameReset(){
	score = 0;
	lives = 3;
	for (int i = 0; i < 13; i ++){
		drawWhen0[i] = 1;
		drawWhen1[i] = 1;
		drawWhen2[i] = 1;
		drawWhen3[i] = 1;
		drawWhen4[i] = 1;
		drawWhen5[i] = 1;
		eraseRow5[i] = 0;
	}
	drawAll(drawWhen1,drawWhen2,drawWhen3,drawWhen4,drawWhen5);
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
