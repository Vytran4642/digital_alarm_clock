/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c_lcd.h"
#include <stdio.h>
#include <stdbool.h>

#define HIEN_THI 			0X00
#define CHINH_THU			0X10
#define CHINH_TANG_THU		0X101
#define CHINH_NGAY			0X11
#define CHINH_TANG_NGAY		0X111
#define CHINH_THANG			0X12
#define CHINH_TANG_THANG	0X121
#define CHINH_NAM			0X13
#define CHINH_TANG_NAM		0X131
#define	LUU_CHINH_NGAY		0X14

#define CHINH_GIO			0X20
#define CHINH_TANG_GIO		0X201
#define CHINH_PHUT			0X21
#define CHINH_TANG_PHUT		0X211
#define CHINH_GIAY			0X22
#define CHINH_TANG_GIAY		0X221
#define LUU_CHINH_GIO		0X23

#define HEN_THU				0X40
#define HEN_TANG_THU		0X401
#define HEN_GIO				0X41
#define HEN_TANG_GIO		0X411
#define HEN_PHUT			0X42
#define HEN_TANG_PHUT		0X421
#define HEN_GIAY			0X43
#define HEN_TANG_GIAY		0X431
#define LUU_HEN_GIO			0X44
#define ALARM				0x50
#define TEST				0X00

//OCTAVE 5
#define DO5 	 523
#define RE5 	 587
#define MI5	 	 659
#define FA5	 	 697
#define SOL5  	 784
#define LA5 	 880
#define SI5 	 988
//OCTAVE 6
#define DO6 	 523*2
#define RE6 	 587*2
#define MI6 	 659*2
#define FA6 	 698*2
#define SOL6 	 784*2
#define LA6 	 880*2
#define SI6 	 988*2
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
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void display_time(void);
void set_time(void);
void set_alarm_time(void);
void timer_hander(void);
void get_time(void);
void display_day(void);
void display_dmy(void);
void display_hms(void);
void set_ddmy(void);
void set_hms(void);
void get_alarm_time(void);
void display_alarm_hms(void);
void check_alarm(void);
void test(void);
void time_blink(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t BCDToDEC(uint8_t data){
	return ((data>>4)*10) +(data&0x0f);
}
uint8_t DECToBCD(uint8_t data){
	return ((data/10)<<4)|(data%10);
}

uint8_t sec;
uint8_t min;
uint8_t hour;
uint8_t day;
uint8_t date;
uint8_t month;
uint8_t year;

uint8_t sec_alarm;
uint8_t min_alarm;
uint8_t hour_alarm;
uint8_t day_alarm;
uint8_t date_alarm;

uint8_t t_i2c_buffer[7];
uint16_t current_mode = HIEN_THI;
uint32_t timer=0;
bool blink =0;
int i;
uint32_t period;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	static uint32_t timer=0;
	if(HAL_GetTick()- timer <40){
		return;
	}
	timer=HAL_GetTick();
	lcd_clear();
	// A0 DUOC NHAN
	if((GPIO_Pin == GPIO_PIN_0)&& (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1 )){
		switch(current_mode){
			case HIEN_THI:
				get_time();
				current_mode=CHINH_THU;
				break;
			case ALARM:
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
				current_mode=HIEN_THI;
				break;
			default:
				current_mode = HIEN_THI;
				break;
		}
	}
	// A1 DUOC NHAN
	if((GPIO_Pin == GPIO_PIN_1) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)== 1)){
		switch( current_mode) {
			case CHINH_GIO:
				current_mode = CHINH_PHUT;
				break;
			case CHINH_PHUT:
				current_mode = CHINH_GIAY;
				break;
			case CHINH_GIAY:
				current_mode = LUU_CHINH_GIO;
				break;
			case HIEN_THI:
				current_mode = CHINH_GIO;
				uint8_t set_time[7] ;
				HAL_I2C_Mem_Read(&hi2c1, 0x68<<1, 0, I2C_MEMADD_SIZE_8BIT, set_time, 7, 1000);
				sec = BCDToDEC(set_time[0]);
				min = BCDToDEC(set_time[1]);
				hour = BCDToDEC(set_time[2]);
				day = BCDToDEC(set_time[3]);
				date = BCDToDEC(set_time[4]);
				month = BCDToDEC(set_time[5]);
				year = BCDToDEC(set_time[6]);
				break;
			case CHINH_THU:
				current_mode=CHINH_NGAY;
				break;
			case CHINH_NGAY:
				current_mode=CHINH_THANG;
				break;
			case CHINH_THANG:
				current_mode=CHINH_NAM;
				break;
			case CHINH_NAM:
				current_mode=LUU_CHINH_NGAY;
				break;
			case HEN_GIO:
				current_mode=HEN_PHUT;
				break;
			case HEN_PHUT:
				current_mode=HEN_GIAY;
				break;
			case HEN_GIAY:
				current_mode=LUU_HEN_GIO;
				break;
			case ALARM:
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
				current_mode=HIEN_THI;
				break;
			default:
				current_mode = HIEN_THI;
				break;
		}
	}
	// A2 DUOC NHAN
	if((GPIO_Pin == GPIO_PIN_2) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)== 1)){
		switch( current_mode) {
			case CHINH_THU:
				current_mode = CHINH_TANG_THU;
				break;
			case CHINH_NGAY:
				current_mode = CHINH_TANG_NGAY;
				break;
			case CHINH_THANG:
				current_mode = CHINH_TANG_THANG;
				break;
			case CHINH_NAM:
				current_mode = CHINH_TANG_NAM;
				break;
			case CHINH_GIO:
				 current_mode = CHINH_TANG_GIO;
				 break;
			case CHINH_PHUT:
				 current_mode = CHINH_TANG_PHUT;
				 break;
			case CHINH_GIAY:
				 current_mode = CHINH_TANG_GIAY;
				 break;
			case HIEN_THI:
				current_mode=HEN_GIO;
				get_alarm_time();
				break;
			case HEN_GIO:
				 current_mode = HEN_TANG_GIO;
				 break;
			case HEN_PHUT:
				 current_mode = HEN_TANG_PHUT;
				 break;
			case HEN_GIAY:
				 current_mode = HEN_TANG_GIAY;
				 break;
			case ALARM:
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
				current_mode=HIEN_THI;
				break;
			default:
				current_mode = HIEN_THI;
				break;
		}
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
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	lcd_init();
	lcd_clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if(TEST == 0X01){
//		  	  set_time();
	 		  test();
	 		  while(1){
	 		  }
	 	  }
	  	  else{
	 	  	  timer_hander();
	 	  	  check_alarm();
	 	  	  time_blink();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA0 PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */
void set_time(void){
	t_i2c_buffer[0] = DECToBCD(0);
	t_i2c_buffer[1] = DECToBCD(54);
	t_i2c_buffer[2] = DECToBCD(12);
	t_i2c_buffer[3] = DECToBCD(7);
	t_i2c_buffer[4] = DECToBCD(7);
	t_i2c_buffer[5] = DECToBCD(1);
	t_i2c_buffer[6] = DECToBCD(23);
	HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0, I2C_MEMADD_SIZE_8BIT,t_i2c_buffer , 7, 1000);
	if(HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF){
		lcd_put_cur(0, 0);
		lcd_send_string("Write time");
		lcd_put_cur(1, 0);
		lcd_send_string("enrror!");
	}else{
		lcd_put_cur(0, 0);
		lcd_send_string("write time");
		lcd_put_cur(1, 0);
		lcd_send_string("complete!");
	}
	HAL_Delay(1000);
	lcd_clear();
}
void set_ddmy(void){
	t_i2c_buffer[0] = DECToBCD(day);
	t_i2c_buffer[1] = DECToBCD(date);
	t_i2c_buffer[2] = DECToBCD(month);
	t_i2c_buffer[3] = DECToBCD(year);
	HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x03, I2C_MEMADD_SIZE_8BIT,t_i2c_buffer , 4, 1000);
	lcd_clear();
	if(HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF){
		lcd_put_cur(0, 0);
		lcd_send_string("Write ddmy");
		lcd_put_cur(1, 0);
		lcd_send_string("enrror!");
	}else{
		lcd_put_cur(0, 0);
		lcd_send_string("write ddmy");
		lcd_put_cur(1, 0);
		lcd_send_string("complete!");
	}
	HAL_Delay(1000);
	lcd_clear();
	}
void set_hms(void){
	t_i2c_buffer[0] = DECToBCD(sec);
	t_i2c_buffer[1] = DECToBCD(min);
	t_i2c_buffer[2] = DECToBCD(hour);
	HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x00, I2C_MEMADD_SIZE_8BIT,t_i2c_buffer , 3, 1000);
	lcd_clear();
	if(HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF){
		lcd_put_cur(0, 0);
		lcd_send_string("Write buffer");
		lcd_put_cur(1, 0);
		lcd_send_string("enrror!");
	}else{
		lcd_put_cur(0, 0);
		lcd_send_string("write hms");
		lcd_put_cur(1, 0);
		lcd_send_string("complete!");
	}
	HAL_Delay(1000);
	lcd_clear();
	}
void display_day(void){
	char i2c_buf_0[6]={};
	if(day == 1){
	  lcd_put_cur(0,2);
	  lcd_send_string("CN");
	}
	else{
	  sprintf(i2c_buf_0,"T%d",day);
	  lcd_put_cur(0,2);
	  lcd_send_string(i2c_buf_0 );
	}
}
void display_dmy(void){
	char i2c_buf_1[20]={};
	if(year>9){
		sprintf(i2c_buf_1," %2d/%2d/20%2d",date,month, year);
		lcd_put_cur(0,4);
		lcd_send_string(i2c_buf_1);
	}else{
	sprintf(i2c_buf_1," %2d/%2d/200%1d",date,month, year);
	lcd_put_cur(0,4);
	lcd_send_string(i2c_buf_1);
	}
}
void display_hms(void){
	char i2c_buf_2[20]={};
	sprintf(i2c_buf_2,"%2dh:%2dm:%2ds", hour, min, sec );
	lcd_put_cur(1, 2);
	lcd_send_string(i2c_buf_2);
}
void display_alarm_hms(void){
	char i2c_buf_2[20]={};
	sprintf(i2c_buf_2,"%2dh:%2dm:%2ds", hour_alarm, min_alarm, sec_alarm );
	lcd_put_cur(1, 2);
	lcd_send_string(i2c_buf_2);
}
void display_time(void){
	get_time();
	display_day();
	display_dmy();
	display_hms();
	HAL_Delay(1000);
}
void set_alarm_time(void){
  t_i2c_buffer[0] = DECToBCD(sec_alarm);
  t_i2c_buffer[1] = DECToBCD(min_alarm);
  t_i2c_buffer[2] = DECToBCD(hour_alarm);
  HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x07, I2C_MEMADD_SIZE_8BIT,t_i2c_buffer , 3, 1000);
  if(HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF){
	lcd_put_cur(0,0);
	lcd_send_string("Write alarm");
	lcd_put_cur(1, 0);
	lcd_send_string("time enrror!");
	HAL_Delay(1000);
	lcd_clear();
  }
	lcd_put_cur(0, 0);
	lcd_send_string("Write alarm");
	lcd_put_cur(1, 0);
	lcd_send_string("time complete!");
	HAL_Delay(1000);
	lcd_clear();
}
void get_alarm_time(void){
	uint8_t get_time[7] ;
	HAL_I2C_Mem_Read(&hi2c1, 0x68<<1, 0x07, I2C_MEMADD_SIZE_8BIT, get_time, 3, 1000);
	sec_alarm = BCDToDEC(get_time[0]);
	min_alarm = BCDToDEC(get_time[1]);
	hour_alarm = BCDToDEC(get_time[2]);
	if(HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF){
		lcd_put_cur(0, 0);
		lcd_send_string("Read alarm");
		lcd_put_cur(1, 0);
		lcd_send_string("time complete!");
		HAL_Delay(1000);
		lcd_clear();
	}
}
void get_time(void){
	uint8_t get_time[7] ;
	HAL_I2C_Mem_Read(&hi2c1, 0x68<<1, 0, I2C_MEMADD_SIZE_8BIT, get_time, 7, 1000);
	sec = BCDToDEC(get_time[0]);
	min = BCDToDEC(get_time[1]);
	hour = BCDToDEC(get_time[2]);
	day = BCDToDEC(get_time[3]);
	date = BCDToDEC(get_time[4]);
	month = BCDToDEC(get_time[5]);
	year = BCDToDEC(get_time[6]);
	if(HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF){
	while((HAL_GetTick() - timer)<1000){
		lcd_send_string("Read time enrror!");
	}
	lcd_clear();
	}
}
void time_blink(void){
	if((HAL_GetTick()-timer)>500){
		blink =!blink;
		timer = HAL_GetTick();
	}
}
void timer_hander(void){
	switch(current_mode){
		case HIEN_THI:
			display_time();
			break;
		case CHINH_THU:
			if(blink ==1 ){
				display_day();
				display_dmy();
			}
			else{
				lcd_put_cur(0, 2);
				lcd_send_string("  ");
			}
			break;
		case CHINH_TANG_THU:
			day++;
			if (day>7){
				day=1;
			}
			current_mode = CHINH_THU;
			break;
		case CHINH_NGAY:
			if(blink ==1 ){
			display_day();
			display_dmy();
			}
			else{
			lcd_put_cur(0,5);
			lcd_send_string("  ");
			}
			break;
		case CHINH_TANG_NGAY:
			date++;
			if( (month == 1) | (month == 3) | (month == 5) | (month == 7) | (month == 8) | (month == 10) | (month == 12)){
				if (date > 31){
					date =1;
				}
			}
			if ( (month == 4) | (month == 6) | (month == 9) | (month == 11)){
				if (date >30){
					date = 1;
				}
			}
			if (month == 2){
				if (year%4 == 0){
					if (date > 29){
						date = 1;
					}
				}
				else {
					if (date > 28){
						date = 1;
					}
				}
			}
			current_mode = CHINH_NGAY;
			break;
		case CHINH_THANG:
			if(blink ==1 ){
			display_day();
			display_dmy();
			}
			else{
			lcd_put_cur(0,8);
			lcd_send_string("  ");
			}
			break;
		case CHINH_TANG_THANG:
			month++;
			if(month>12){
				month=1;
			}
			current_mode=CHINH_THANG;
			break;
		case CHINH_NAM:
			if(blink ==1 ){
			display_day();
			display_dmy();
			}
			else{
			lcd_put_cur(0,11);
			lcd_send_string("    ");
			}
			break;
		case CHINH_TANG_NAM:
			year++;
			if(year>30){
				year=1;
			}
			current_mode=CHINH_NAM;
			break;
		case LUU_CHINH_NGAY:
			set_ddmy();
			current_mode=HIEN_THI;
			break;
		case CHINH_GIO:
			if(blink ==1 ){
			display_hms();
			}
			else{
			lcd_put_cur(1, 2);
			lcd_send_string("   ");
			}
			break;
		case CHINH_TANG_GIO:
			hour++;
			if(hour>23){
				hour=0;
			}
			current_mode=CHINH_GIO;
			break;
		case CHINH_PHUT:
			if(blink ==1 ){
			display_hms();
			}
			else{
			lcd_put_cur(1, 6);
			lcd_send_string("   ");
			}
			break;
		case CHINH_TANG_PHUT:
			min++;
			if(min>59){
				min=0;
			}
			current_mode=CHINH_PHUT;
			break;
		case CHINH_GIAY:
			if(blink ==1 ){
			display_hms();
			}
			else{
			lcd_put_cur(1, 10);
			lcd_send_string("   ");
			}
			break;
		case CHINH_TANG_GIAY:
			sec++;
			if(sec>59){
				sec=0;
			}
			current_mode=CHINH_GIAY;
			break;
		case LUU_CHINH_GIO:
			set_hms();
			current_mode=HIEN_THI;
			break;
		case HEN_GIO:
			if(blink ==1 ){
			display_alarm_hms();
			}
			else{
			lcd_put_cur(1, 2);
			lcd_send_string("   ");
			}
			break;
		case HEN_TANG_GIO:
			hour_alarm++;
			if(hour_alarm>23){
				hour_alarm=0;
			}
			current_mode=HEN_GIO;
			break;
		case HEN_PHUT:
			if(blink ==1 ){
			display_alarm_hms();
			}
			else{
			lcd_put_cur(1, 6);
			lcd_send_string("   ");
			}
			break;
		case HEN_TANG_PHUT:
			min_alarm++;
			if(min_alarm>59){
				min_alarm=0;
			}
			current_mode=HEN_PHUT;
			break;
		case HEN_GIAY:
			if(blink ==1 ){
			display_alarm_hms();
			}
			else{
			lcd_put_cur(1, 10);
			lcd_send_string("   ");
			}
			break;
		case HEN_TANG_GIAY:
			sec_alarm++;
			if(sec_alarm>59){
				sec_alarm=0;
			}
			current_mode=HEN_GIAY;
			break;
		case LUU_HEN_GIO:
			set_alarm_time();
			current_mode=HIEN_THI;
			break;
		case ALARM:
//			for(i=0;i<16;i++){
//				  period=8000000/f[i];
//				__HAL_TIM_SET_AUTORELOAD(&htim2,period );
//				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4, period*0.7);
//				get_time();
//				display_day();
//				display_dmy();
//				display_hms();
//				HAL_Delay(200);
//				if(i == 16 ){
//					i=0;
//				}
//			}
			for(i=2000;i<6000;i=i+200){
				  period= i;
				__HAL_TIM_SET_AUTORELOAD(&htim2,period );
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4, period*0.5);
				get_time();
				display_day();
				display_dmy();
				display_hms();
				HAL_Delay(100);
			}
			break;
		default:
			current_mode=HIEN_THI;
			break;
	}

}
void check_alarm(void){
//	get_time();
	if((hour==hour_alarm) & (min==min_alarm) & (sec==sec_alarm) ){
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
		current_mode =ALARM;
	}
}
void test(void){
	while(1){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
		HAL_Delay(500);
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
