/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
void SysTick_Delay(uint32_t delay_ms);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define THRESHOLD_CURRENT 4.170
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float adc_vol,adc_cur;
int adc_val,cnt,k;
uint32_t press_time1,press_time2,press_time3,press_time4;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/


/* USER CODE BEGIN PFP */
void auto_window_gpio(void);
void auto_up_down(void);
void auto_motor_up(void);
void auto_motor_down(void);
void auto_motor_stop(void);
void SysTick_Init(void);
void SystemClock_Config(void);
void adc_init(void);
int CURRENT(void);
void anti_pinch(void);
uint16_t ADC_READ(void);
void delay_ms(int);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
auto_window_gpio();
SysTick_Init();
adc_init();
//SystemClock_Config();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  auto_up_down();
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void auto_window_gpio(void)
{
RCC->AHBENR|=(1<<18);
GPIOB->MODER&=(~(1<<3))&(~(1<<2));//PB1 INPUT (DOOR SWITCH)
GPIOB->MODER&=(~(1<<4))&(~(1<<5));//PB2 INPUT (DOOR SWITCH)
GPIOB->MODER&=(~(1<<15))&(~(1<<14));//PB7 INPUT (LIMIT SWITCH)
GPIOB->MODER&=(~(1<<8))&(~(1<<9));//PB4 INPUT(LIMIT SWITCH)
GPIOB->MODER|=(1<<10);//PB5 OUTPUT(RELAY R1)
GPIOB->MODER|=(1<<12);//PB6 OUTPUT(RELAY R2)
GPIOB->PUPDR|=1<<3;//PB1 PULL-DOWN
GPIOB->PUPDR|=1<<5;//PB2 PULL-DOWN
GPIOB->PUPDR|=1<<15;//PB7 PULL-DOWN
GPIOB->PUPDR|=1<<9;//PB4 PULL-DOWN

GPIOB->PUPDR|=1<<10;//PB5 PULL-UP(Relay 1)
GPIOB->PUPDR|=1<<12;//PB6 PULL-UP(Relay 2)

GPIOB->ODR|=(1<<5)|(1<<6);//Initially realay should OFF
}
/*-----------------------------------------------------------------------------------*/
void auto_up_down(void)
{
 sw1:if((GPIOB->IDR>>1 &1)==1)
	{
	 k=1;
	 delay_ms(5);
	 if((GPIOB->IDR>>1&1)==0)
	 {
		 goto sw1;
	 }
	 press_time1=0;
	 while((GPIOB->IDR>>1&1)==1)
	 {
		 SysTick_Delay(1);
		 press_time1++;
		 if(press_time1>500)
		 {
			 if((GPIOB->IDR>>7&1)==1)
			 {
				 goto sw1;
			 }
			 else
			 {
				 k=22;
			 auto_motor_down();
			 }
			 while(!(GPIOB->IDR>>7&1))
			 {
				 press_time4=0;
				 l:if((GPIOB->IDR>>2&1)==1)
				 {
					 delay_ms(5);
					 if((GPIOB->IDR>>2&1)==0)
					 {
						 goto l;
					 }
					 while((GPIOB->IDR>>2&1)==1)
					 {
						 SysTick_Delay(1);
						 press_time4++;
						 if(press_time4>10)
						 {
							 auto_motor_stop();
							 while(GPIOB->IDR>>2&1);
							 goto sw1;
						 }
					 }

				 }
			 }
			 auto_motor_stop();
			 while((GPIOB->IDR>>1&1));
			 press_time1=0;
		 }
	 }
	}
	else
	{
		k=9;
		auto_motor_stop();
	}
 sw2:if((GPIOB->IDR>>2 &1)==1)
	{
		 delay_ms(5);
		 if((GPIOB->IDR>>2&1)==0)
		 {
			 goto sw2;
		 }
		 press_time2=0;
		 while((GPIOB->IDR>>2&1)==1)
		 {
			 SysTick_Delay(1);
			 press_time2++;
			 if(press_time2>500)
			 {
				 if((GPIOB->IDR>>4&1)==1)
				 {
					 goto sw1;
				 }
				 else
				 {
					 k=4;
				 auto_motor_up();
				 }
				 while((!(GPIOB->IDR>>4&1))&&(CURRENT()))
				 {
					 press_time3=0;
					 l1:if((GPIOB->IDR>>1&1)==1)
					 {
						 delay_ms(5);
						 if((GPIOB->IDR>>1&1)==0)
						 {
							goto l1;
						 }
						 while((GPIOB->IDR>>1&1)==1)
						 {
							 SysTick_Delay(1);
							 press_time3++;
							 if(press_time3>10)
							 {
								 auto_motor_stop();
								 while(GPIOB->IDR>>1&1);
								 goto sw1;
							 }
						 }
					 }
				 }
				 auto_motor_stop();
				 while(GPIOB->IDR>>2&1);
			 }
		 }
	}
	else
	{
		auto_motor_stop();
	}


}
/************************************ANTI-PINCH*********************************/
void anti_pinch(void)
{
auto_motor_stop();
delay_ms(1);
auto_motor_down();
delay_ms(50);
auto_motor_stop();
while(GPIOB->IDR>>2 &1);
}
/*-------------------------------------------------------------------------------------*/
void auto_motor_up(void)
{
    GPIOB->ODR|=(1<<6);//RELAY R1 OFF
    delay_ms(1);
	GPIOB->BSRR|=(1<<5)<<16;//RELAY R2 ON
}
/*-------------------------------------------------------------------------------------*/
void auto_motor_down(void)
{
    GPIOB->ODR|=(1<<5);//RELAY R2 OFF
    delay_ms(1);
	GPIOB->BSRR|=(1<<6)<<16;//RELAY R1 ON
}
/*-------------------------------------------------------------------------------------*/
void auto_motor_stop(void)
{
	GPIOB->ODR|=(1<<5)|(1<<6);//BOTH RELAY OFF
	cnt=0;adc_cur=0;
}
/*--------------------------------------------------------------------------------------*/
void SysTick_Init(void)
{
    SysTick->LOAD = 8000 - 1;// Set reload register for 1 ms delay
    SysTick->VAL = 0;// Clear the current value register
    SysTick->CTRL |= (1 << 2) | (1 << 0);// Set the clock source to processor clock (AHB) and enable SysTick
}
/*---------------------------------------------------------------------------------------*/
void SysTick_Delay(uint32_t delay_ms)
{
    for (uint32_t i = 0; i < delay_ms; i++)
    {
        SysTick->VAL = 0;// Clear the current value register
        while (!(SysTick->CTRL & (1 << 16))); // Wait until the COUNTFLAG is set
    }
}
/************************* ADC INIT ********************************************/
void adc_init(void)
{

	RCC->APB2ENR|=(1<<9);
	RCC->AHBENR|=(1<<18);
	GPIOB->MODER|=0X3;
	ADC1->CR |= 1<<31; /* (4) */
	while ((ADC1->CR>>31 & 1) != 0);
	//ADC1->SMPR |= 1<<0|1<<1|1<<0;
	ADC1->CFGR2&=(~(1<<31))&(~(1<<30));//ASYN CLK
	ADC1->CFGR1&=(~(1<<0));//SCAN CHNL FROM 0 TO 18
	ADC1->CFGR1|=(1<<13);
	ADC1->CR|=(1<<0);
	ADC1->CHSELR|=(1<<8);
	while(!(ADC1->ISR>>0 &1));
}
/*****************************ADC READ ****************************************/
uint16_t ADC_READ()
{
	    ADC1->CR |= (1<<2); // START COVR
	    while (!(ADC1->ISR>>2&1)); // END CONVER
	    return ADC1->DR; // READ COVERT VAL
}
/*----------------------------------------------------------------------------*/
/****************************CURRENT READ ***********************************/
int CURRENT(void)
{
   adc_val=0;
   if(cnt==0)
   {
	   delay_ms(10);
	   cnt++;
   }
   for(int i=0;i<10;i++)
   {
	   adc_val=adc_val+ADC_READ();
   }
   //adc_val=ADC_READ();
   adc_val=adc_val/10;
   adc_vol=(adc_val*(3.6/4095.0))-0.52;
   adc_cur=(adc_vol-2.5)/0.066;
   if(adc_cur<0)
   {
	   adc_cur=-adc_cur;
   }
     if(adc_cur>THRESHOLD_CURRENT)
     {
  	   anti_pinch();
  	   return 0;
     }
  return 1;
}
void delay_ms(int dlyms)
{
	dlyms*=12000;
	while(dlyms--);
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
