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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>

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
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart2;
#define LENGTH  10  //���ջ�������С
/* USER CODE END PM */
uint8_t RxBuffer[LENGTH];//���ջ�����
uint8_t RecCount = 0;//������������
uint8_t flag = 0;//������ɱ�־��0��ʾδ������ɣ�1��ʾ�������
uint8_t crr =0;//CRR�ı�ռ�ձ�
/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_DMA_Init();
  /* USER CODE BEGIN 2 */
	printf("Plaese input the frequency and the Duty,like the '100 40':\r\n");
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);//ʹ���ź�IDLE�ж�
	HAL_UART_Receive_DMA(&huart2,(uint8_t*)RxBuffer,LENGTH);///����DMA����
	//HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//������ʱ��2��ͨ��1���PWM�ź�
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		if(flag == 1)  //�ж������Ƿ���ɽ���
		{
			int a[10] = {0};
			int i;
			flag = 0; //�����־λ
			RecCount = LENGTH - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);//�ѽ������ݵ�������������ȥDMA�������д���������
			HAL_UART_Transmit_DMA(&huart2,(uint8_t*)RxBuffer,RecCount);//����DMA��ʽ�����ܵ�����ԭ������PC
			RecCount = 0;
			int fre = 0;//Ƶ��
			int duty = 0;//ռ�ձ�
			for(i = 0;RxBuffer[i]!=' ';i++)//ʶ���Ƶ��
			{
					fre =10*arr +(int)RxBuffer[i];
			}
			arr = 10000/fre -1;
			for(i++;i<RecCount;i++)//ʶ���ռ�ձ�
			{
					duty =10*duty +(int)RxBuffer[i];
			}
			crr = duty*(arr+1)/100;
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,crr);//��ͨ��1��ռ�ձ�
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,crr);//��ͨ��2��ռ�ձ�
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//������ʱ��2��ͨ��1���PWM�ź�
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);//������ʱ��2��ͨ��2���PWM�ź�
			__HAL_DMA_DISABLE(&hdma_usart2_rx);//����DISABLE������DMA�жϻص�����������һ����һ�ε�DMA����
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//DMA�н������жϻص�����
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart2)
{
	if(huart2 ->Instance == USART2)
	{
		HAL_UART_Receive_DMA(huart2,(uint8_t*)RxBuffer,LENGTH);//��������DMA���գ�׼����һ�����ݴ���
	}
}
//�����жϻص�����
void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart2)
{
	flag = 1;//���ý��ձ�־λ
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
