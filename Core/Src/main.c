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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "max6675.h"
#include "tim.h"
#include "stm32f103xb.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define k2		  HAL_GPIO_ReadPin(K2_GPIO_Port,K2_Pin)
#define k3		  HAL_GPIO_ReadPin(K3_GPIO_Port,K3_Pin)
#define k4		  HAL_GPIO_ReadPin(K4_GPIO_Port,K4_Pin)
#define k5		  HAL_GPIO_ReadPin(K5_GPIO_Port,K5_Pin)
#define PWM_MAX 	19999			//PWM最大值
#define SET_TEMPER	4200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
PID pid;
uint16_t temper_buff[5];
uint32_t temper_long;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//将数组从小到大排序
//*******************************************************************************************
void sort(uint16_t a[],uint8_t n)		
{
    uint16_t temp;
	uint8_t i,j;
    for(i=0;i<n-1;i++)		//i<n-1是因为数组的最后一位已经在之前的运算中接受了排序。
    {
        for(j=i+1;j<n;j++)
        {
            if(a[i]>a[j])
            {temp=a[i];a[i]=a[j];a[j]=temp;}	
        }
    }
}

//-----------去掉最大和最小，求平均------------------------
uint32_t median(uint16_t a[], uint16_t n)
{
	uint32_t value_long;
	uint16_t remove=1;		//头尾各去除几个
	
	sort(a, n);
	value_long = 0;
	for(uint8_t i=remove;i<(n-remove);i++)
	{
		value_long += a[i];
	}
	value_long /= (n-(remove*2));
	return value_long;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
    uint8_t i = 0;
    float temper;
    uint8_t t1 = 0, t2 = 0, t3 = 0,t4=0;
	uint8_t count = 0;
    pid.P = 20;
    pid.I = 1.3f;
    pid.D = 0;
    pid.OutputMax = 19999.0f;	//设置PWM停止输出的脉宽值
	pid.IntegralMax = 2.0f;		
	pid.OutputMin = 0;
	uint16_t temper_int;
	float pwm_pid;
	uint16_t pwm_val;
	uint8_t switch_power=1;		//加热开关
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
    MX_USART1_UART_Init();
    MX_TIM4_Init();
    /* USER CODE BEGIN 2 */
	pwm_val = PWM_MAX - PWM_MAX / 6;
    TIM4->CCR3 = pwm_val;
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
		if(t4++ == 20)		//200ms的周期进入一次
		{
			t4=0;
			temper = read_temper();
			temper_int = temper * 100;
			temper_buff[count++] = temper_int;
		}
		if(count == 5)
		{
			count = 0;
			temper_long = median(temper_buff,5);			//中位值滤波
		}
        if((t2++ == 100) && (switch_power))					//1s的周期进入一次
        {
            t2 = 0;
			pwm_pid = PID_Cal(&pid,temper_long,SET_TEMPER);	//目标温度：42℃
			pwm_val = PWM_MAX - (uint16_t)(pwm_pid);		//计算当前脉宽值(因为PID计算出来的值是反的)
			TIM4->CCR3 = pwm_val;							//写入pwm寄存器，执行输出
        }
        //-------key---------------
        if(k2 == GPIO_PIN_RESET)
        {
			while (k2 == GPIO_PIN_RESET);
            pid.I = pid.I-0.1f;
        }
        if(k3 == GPIO_PIN_RESET)
        {
            while (k3 == GPIO_PIN_RESET);
            pid.I = pid.I+0.1f;
        }
        if(k4 == GPIO_PIN_RESET)
        {
            while (k4 == GPIO_PIN_RESET);
            TIM4->CCR3 = PWM_MAX;
			switch_power = !switch_power;	//开关量取反
        }

        if(t1++ == 10)		//100ms的周期进入一次
        {
            t1 = 0;
            HAL_GPIO_TogglePin(D1_GPIO_Port, D1_Pin);
        }
		if(t3++ == 100)		//1s的周期进入一次
		{
			t3 = 0;
			printf("i=%d temper=%d℃ pwm_val=%d KI=%f \r\n", i++, (uint16_t)temper_long,pwm_val,pid.I);
		}
        HAL_Delay(10);
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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
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
