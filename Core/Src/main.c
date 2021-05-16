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
#include "st7789.h"
#include "fonts.h"
#include "stdio.h"
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
ADC_HandleTypeDef hadc2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
// НАСТРОЙК�? ОСНОВНІ //----------------------------
int color_tekst = YELLOW;//КОЛІР ТЕКСТУ
int setTemp = 320; //ЗАДАНА ТЕМПЕРАТУРА ПАЯЛЬН�?КА+ поправка
int maxTemp = 450;//МАКС�?МАЛЬНА ТЕМПЕРАТУРА ПАЯЛЬН�?КА
int minTemp = 100;//МІНІМАЛЬНА ТЕМПЕРАТУРА ПАЯЛЬН�?КА
int popravcaP=20;//4правка температури
int crocEncode = 5;//КРОК ЗМІН�? ТЕМПЕРАТУР�? ЗА 1 КРОК ЕНКОДЕРА
int setFen = 150;//ЗАДАНА ТЕМПЕРАТУРА ФЕНА
int maxFen = 400;//МАКС�?МАЛЬНА ТЕМПЕРАТУРА ФЕНА
int minFen = 50;//МІНІМАЛЬНА ТЕМПЕРАТУРА ФЕНА
int popravcaF=0;
int setAir = 50;
int minAir = 30;//мінімальга швидкість вентилятора
int EncRot = 1;//напрямок енкодера право = 1 , ліво = 0.
//ПЕРЕМІННІ//--------------------------------------
int EncVal;
void setPWM(uint16_t pwm_value);
//uint32_t adcResultn = 0;
float adcResult;
float adcResultf;
uint32_t i,d;
int32_t prevCounter = 0;//encoder
int32_t prevCounterz = 0;
int32_t currCounterz=0;
int32_t currCountern = 0;
uint32_t time_key1 = 0;
uint8_t short_state = 0;
uint8_t long_state = 0;
uint8_t menun=0;
uint8_t stopset=0;
uint8_t fanflag= 0;
uint8_t r = 0;
uint8_t set0 = 0;
uint8_t pflag = 0;
uint8_t flagfn = 0;
uint8_t culofn = 0;
uint8_t culoff = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void mytftset(void){
     ST7789_WriteString(35, 5, "HEATER_TEMP", Font_16x26, GREEN, BLACK);
     ST7789_WriteString(60, 84, "FEN_TEMP", Font_16x26, color_tekst, BLACK);
     ST7789_WriteString(65, 140, "AIRFLOW", Font_16x26, color_tekst, BLACK);
    ST7789_DrawRectangle(2, 2, 237, 78, WHITE);
 	ST7789_DrawRectangle(1, 1, 238, 79, WHITE);
 	ST7789_DrawRectangle(0, 0, 239, 80, WHITE);

 	ST7789_DrawRectangle(0, 81, 239, 239, WHITE);
 	ST7789_DrawRectangle(1, 81, 238, 238, WHITE);
 	ST7789_DrawRectangle(2, 81, 237, 237, WHITE);

    HAL_GPIO_WritePin(BLK_GPIO_Port, BLK_Pin,GPIO_PIN_SET);
}

void encoder(void) {
	if(set0==1){
		TIM2->CNT = 0;
    	set0=0;
    }
    int32_t currCounter = TIM2->CNT;

    if(currCounter != prevCounter) {
    	 prevCounter = currCounter;
    	 stopset = 1;
    if(menun==0){//паяльника температура
    currCounter=(currCounter + minTemp)/crocEncode;
    currCounter=currCounter*crocEncode;
    if(currCounter>maxTemp){
    	set0 = 1;
        }
    setTemp= currCounter + popravcaP;
	 char buff[16];
	 snprintf(buff, sizeof(buff), "%d", currCounter);
	 ST7789_WriteString(90, 35, buff ,Font_16x26, color_tekst, BLACK);

    }
    if(menun==1){//Фена температура

    currCounter=(currCounter + minFen)/crocEncode;
    currCounter=currCounter*crocEncode;
    if(currCounter>maxFen){
    	set0 = 1;
    }
    setFen= currCounter + popravcaF;
    char buff[16];
   	 snprintf(buff, sizeof(buff), "%d*C", currCounter);
   	 ST7789_WriteString(90, 115, buff ,Font_16x26, color_tekst, BLACK);
   	  if(setFen<99)ST7789_DrawFilledRectangle(90+16*4, 115,16,26,BLACK);//BLACK
    }
    if(menun==2 && fanflag==1){//Фена поток
        currCounter=(currCounter + minAir)/crocEncode;
        currCounter=currCounter*crocEncode;
        if(currCounter>100){
           set0=1;
           }
       // if(currCounter<30)
        setAir = currCounter;
       setPWM(setAir);
        char buff[16];
       	 snprintf(buff, sizeof(buff), "%d ", currCounter);
       	 ST7789_WriteString(90, 175, buff ,Font_16x26, color_tekst, BLACK);
        }
   // currCounterz = currCounter;
    }
    //BATN
    uint32_t ms = HAL_GetTick();
    uint8_t key1_state = HAL_GPIO_ReadPin(BATN_GPIO_Port, BATN_Pin); // подставить свой пин

    if(key1_state == 0 && !short_state && (ms - time_key1) > 50)
    {
      short_state = 1;
      long_state = 0;
      time_key1 = ms;
    }
    else if(key1_state == 0 && !long_state && (ms - time_key1) > 400)
    {
      long_state = 1;
     // действие на длинное нажатие
     if(menun==0){

    	 if(pflag==1){
    		 pflag=0;
    		 ST7789_DrawRectangle(1, 1, 238, 79, WHITE);
    		 ST7789_DrawRectangle(0, 0, 238, 80, WHITE);
    		 ST7789_DrawRectangle(2, 2, 237, 78, WHITE);
    	 }
    	 else {
    		 pflag=1;
    		 ST7789_DrawRectangle(1, 1, 238, 79, myred);
    		 ST7789_DrawRectangle(0, 0, 238, 80, MAGENTA);
    		 ST7789_DrawRectangle(2, 2, 237, 78, myred);
    	 }
     }
     if(menun==1 || menun==2){
    	 if(fanflag==1){
    		 fanflag=0;
    		 	ST7789_DrawRectangle(1, 81, 238, 238, WHITE);
    		 	ST7789_DrawRectangle(0, 81, 239, 239, WHITE);
    		 	ST7789_DrawRectangle(2, 81, 237, 237, WHITE);
    	 }
    	 else{
    		 fanflag=1;
    		 	ST7789_DrawRectangle(1, 81, 238, 238, myred);
    		 	ST7789_DrawRectangle(0, 81, 239, 239, myred);
    		 	ST7789_DrawRectangle(2, 81, 237, 237, myred);
    	 }
     }
    }
    else if(key1_state == 1 && short_state && (ms - time_key1) > 50)
    {
      short_state = 0;
      time_key1 = ms;

      if(!long_state)
      {
    menun++;
    if (menun==3)menun=0; // действие на короткое нажатие

    if(menun==0){
    	ST7789_WriteString(35, 5, "HEATER_TEMP", Font_16x26, GREEN, BLACK);
    	ST7789_WriteString(60, 84, "FEN_TEMP", Font_16x26, color_tekst, BLACK);
    	ST7789_WriteString(65, 140, "AIRFLOW", Font_16x26, color_tekst, BLACK);
    }
	if(menun==1){
		 ST7789_WriteString(35, 5, "HEATER_TEMP", Font_16x26, color_tekst, BLACK);
		 ST7789_WriteString(60, 84, "FEN_TEMP", Font_16x26, GREEN, BLACK);
		 ST7789_WriteString(65, 140, "AIRFLOW", Font_16x26, color_tekst, BLACK);
	 }
	 if(menun==2){
		 ST7789_WriteString(35, 5, "HEATER_TEMP", Font_16x26, color_tekst, BLACK);
		 ST7789_WriteString(60, 84, "FEN_TEMP", Font_16x26, color_tekst, BLACK);
		 ST7789_WriteString(65, 140, "AIRFLOW", Font_16x26, GREEN, BLACK);
	 }

      }
    }
}

void TermoControl(void){
if(pflag==1){
if(setTemp<=adcResult){//нгрів паяльник
	HAL_GPIO_WritePin(heatp_GPIO_Port, heatp_Pin,GPIO_PIN_SET);//pb5
	HAL_GPIO_WritePin(LEDD_GPIO_Port, LEDD_Pin,GPIO_PIN_RESET);//pa8
}
else {
	HAL_GPIO_WritePin(heatp_GPIO_Port, heatp_Pin,GPIO_PIN_RESET);
HAL_GPIO_WritePin(LEDD_GPIO_Port, LEDD_Pin,GPIO_PIN_SET);
}
}
if (pflag==0){
	HAL_GPIO_WritePin(heatp_GPIO_Port, heatp_Pin,GPIO_PIN_SET);//pb5
	HAL_GPIO_WritePin(LEDD_GPIO_Port, LEDD_Pin,GPIO_PIN_RESET);//pa8
}
//uint8_t culoff = 0;
if (fanflag==0){
	fenreset();
	flagfn=0;
	if(adcResultf>(minFen+30)){
		uint8_t flagfn=1;
		if(flagfn != culofn){
		culofn=flagfn;
		setAir=100;
		setPWM(setAir);
		}
	}

if(adcResultf<(minFen+20)){
	uint8_t flagff=1;
	  if(flagff != culoff) {
	culoff=flagff;
		setAir=0;
	setPWM(setAir);
	//culoff=0;
	  }

}}
if (fanflag==1){
	culoff=0;
	culofn=0;
	uint8_t flagf=1;
	if(flagf != flagfn){
	flagfn=flagf;
	setAir=50;
    setPWM(setAir);
	}
if(setFen<=adcResultf){//агрів фену !!!220V!!!
fenreset();
	}
else{

	HAL_GPIO_WritePin(FEN_GPIO_Port, FEN_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,GPIO_PIN_RESET);
}}
}

void fenreset(){
		HAL_GPIO_WritePin(FEN_GPIO_Port, FEN_Pin,GPIO_PIN_RESET);//pb9
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,GPIO_PIN_SET);//pc13
}

void setPWM(uint16_t value)
{
    int set = (6000*value)/100;
    TIM4->CCR1 = set;
}

void menu(void){
	if(stopset==0){//паяльника температура
	    if(adcResult<600){
		char buf[128];
		sprintf(buf, "%d*C ", ((uint32_t)adcResult-popravcaP));//
		ST7789_WriteString(90, 35, buf ,Font_16x26, color_tekst, BLACK);
	    }
		if(adcResult>600){
			ST7789_WriteString(90, 35, "NO   " ,Font_16x26, RED, BLACK);
			HAL_GPIO_WritePin(heatp_GPIO_Port, heatp_Pin,GPIO_PIN_RESET);
	    }
	}
	if(stopset==0){//Фена температура
		if(adcResultf<600){
		char buff[128];
		sprintf(buff, "%d*C ", ((uint32_t)adcResultf-popravcaF));//
		ST7789_WriteString(90, 115, buff ,Font_16x26, color_tekst, BLACK);
		}
		if(adcResultf>600){
		    ST7789_WriteString(90, 115, "NO   " ,Font_16x26, RED, BLACK);
		    HAL_GPIO_WritePin(FEN_GPIO_Port, FEN_Pin,GPIO_PIN_RESET);
		}

	}
	if(stopset==0){//поток повітря фена
        char buff[16];
       	 snprintf(buff, sizeof(buff), "%d", setAir);
       	 ST7789_DrawFilledRectangle(90+16, 175,48-16,26,BLACK);//BLACK
       	 ST7789_WriteString(90, 175, buff ,Font_16x26, color_tekst, BLACK);

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  ST7789_Init();

  mytftset();
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
  setPWM(0);
 // HAL_GPIO_WritePin(CUL_GPIO_Port, CUL_Pin,GPIO_PIN_SET);//pb5
 // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);//TIM_CHANNEL_1
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //if(d==10000||d==20000||d==40000||d==50000||d==60000){
      HAL_ADC_Start(&hadc1);
      HAL_ADC_PollForConversion(&hadc1, 100);
      int adcResultn = HAL_ADC_GetValue(&hadc1);
      HAL_ADC_Stop(&hadc1);
      adcResult = ((adcResultn*((3.274/4096)*4))*100)-popravcaP;//(4096*5)

      HAL_ADC_Start(&hadc2);
      HAL_ADC_PollForConversion(&hadc2, 100);
      int adcResultn1 = HAL_ADC_GetValue(&hadc2);
      HAL_ADC_Stop(&hadc2);
      adcResultf = ((adcResultn1*((3.274/4096)*4))*100)-popravcaF;//дільник 4 після заміни резисторів

      if(d==9000||d==19000||d==29000){
	  menu();
      }
	  encoder();
	  //setPWM(setAir);
//  setpwm();
	//  encoderinit();
	  if(d==30000){
		stopset=0;
		d=0;
	  }
	  TermoControl();
    	  d++;
    	  if(i==60000)i=0;
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
//  htim2.Init.Period = maxTemp-minTemp;

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  if(EncRot==1){
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  }
  if(EncRot==0){
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  }
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 69;
  htim4.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim4.Init.Period = 6000;
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
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BLK_Pin|FEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ST7789_CS_Pin|heatp_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LEDD_Pin|ST7789_DC_Pin|ST7789_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BLK_Pin heatp_Pin FEN_Pin */
  GPIO_InitStruct.Pin = BLK_Pin|heatp_Pin|FEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ST7789_CS_Pin */
  GPIO_InitStruct.Pin = ST7789_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ST7789_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LEDD_Pin */
  GPIO_InitStruct.Pin = LEDD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ST7789_DC_Pin */
  GPIO_InitStruct.Pin = ST7789_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ST7789_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ST7789_RST_Pin */
  GPIO_InitStruct.Pin = ST7789_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ST7789_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BATN_Pin */
  GPIO_InitStruct.Pin = BATN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BATN_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
