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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "hx711.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint32_t	data[2];

char yazi1 [32] = " ";
char yazi2 [32] = " ";
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adcBuffer[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc -> Instance == ADC1)
	{
		data[0] = adcBuffer[1];
		data[1] = adcBuffer[0];
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t zaman =2 ; 
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
void USART_TX();
HAL_ADC_Start_DMA(&hadc,(uint32_t*)adcBuffer,2);
void EXTI4_15_IRQHandler2();
HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//USART_TX("Hi\n");
HX711 sensor;
sensor.PD_SCK_PinType=GPIOF;
sensor.PD_SCK_PinNumber=GPIO_PIN_0;
sensor.DOUT_PinType=GPIOF;
sensor.DOUT_PinNumber=GPIO_PIN_1;
sensor.mode=0;
int read = HX711_Read(&sensor);
int netkg = (read-158100)/473.43+160;
char gram[10];
		sprintf(gram,"%d\n",netkg);
	/*  USART_TX(gram);
		USART_TX("gram\n");
		HAL_Delay(500);*/
		sprintf(yazi1,"%d\n",data[0]);
		sprintf(yazi2,"%d\n",data[1]);
	/*	USART_TX(yazi1);
		USART_TX(yazi2);*/
	htim3.Instance->CCR1=5;//PWM deki ON Kismin süresi 5 milisaniyedir.
       for(int i = 0; i<zaman; i++)
{           
	USART_TX(yazi1);//ADC1
 USART_TX(yazi2);//ADC2
 USART_TX(gram);//agirlik için
 USART_TX("5\n");    
HAL_Delay(1000);	
}	
    htim3.Instance->CCR1=6;//PWM deki ON Kismin süresi 6 milisaniyedir.
       for(int i = 0; i<zaman; i++)
{           

USART_TX(yazi1);
USART_TX(yazi2);
 USART_TX(gram);//agirlik için
 USART_TX("6\n");       
HAL_Delay(1000);
}
     htim3.Instance->CCR1=7;//PWM deki ON Kismin süresi 7 milisaniyedir.
        for(int i = 0; i<zaman; i++)
{           
  USART_TX(yazi1);
 USART_TX(yazi2);
USART_TX(gram);//agirlik için
USART_TX("7\n");;
HAL_Delay(1000);                                              
}
     htim3.Instance->CCR1=8;//PWM deki ON Kismin süresi 8 milisaniyedir.
        for(int i = 0; i<zaman; i++)
{           
USART_TX(yazi1);
USART_TX(yazi2);
USART_TX(gram);//agirlik için
USART_TX("8\n");       
HAL_Delay(1000);
}
      htim3.Instance->CCR1=9;//PWM deki ON Kismin süresi 9 milisaniyedir.
        for(int i = 0; i<zaman; i++)
 {           
USART_TX(yazi1);
USART_TX(yazi2);
USART_TX(gram);//agirlik için
USART_TX("9\n");
HAL_Delay(1000);
}
        htim3.Instance->CCR1=10;//PWM deki ON Kismin süresi 10 milisaniyedir.
        for(int i = 0; i<zaman; i++)
{           
USART_TX(yazi1);
USART_TX(yazi2);
USART_TX(gram);//agirlik için
USART_TX("10\n");
HAL_Delay(1000);
}
//pwm duty si,akim1,akim2,agirlik,*/
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void EXTI4_15_IRQHandler2(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */
HAL_TIM_PWM_Stop_IT(&htim3,TIM_CHANNEL_1);
USART_TX("EMT...\n");
while(1)
  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */

}
void ToggleLED (void)
{
 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
 HAL_Delay(500);
 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
 HAL_Delay(500);
}
void USART_TX(uint8_t Veri1[])
{
	HAL_UART_Transmit(&huart1,Veri1,strlen(Veri1),1000);
	HAL_Delay(100);
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
