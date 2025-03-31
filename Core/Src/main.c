/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static struct {
	uint8_t on;
	uint8_t finish;
	float current;
	float target;
	float p_k;
	float i_k;
	float d_k;
	float error;
	float sum_error;
	float dif_error;
	float out;
	float max_sum_error;
	float min_output;
	float max_output;
	float error_end;
}PID_speed;

uint32_t TxMailbox = 0;

static  CAN_TxHeaderTypeDef angle = {
  .DLC = 4,
  .StdId = 0x12,
  .IDE = CAN_ID_STD,
  .RTR = CAN_RTR_DATA,
  .TransmitGlobalTime = 0,
};
static uint8_t angle_request[4] = {
  0x04, 0x12, 0x01, 0x00
};

static  CAN_TxHeaderTypeDef speed = {
  .DLC = 4,
  .StdId = 0x12,
  .IDE = CAN_ID_STD,
  .RTR = CAN_RTR_DATA,
  .TransmitGlobalTime = 0,
};
uint8_t speed_request[7] = {
  0x04, 0x12, 0x0A, 0x00
};

static CAN_RxHeaderTypeDef encoder_answer = {
  .DLC = 7,
  .StdId = 0x12,
  .IDE = CAN_ID_STD,
  .RTR = CAN_RTR_DATA,
  .Timestamp = 0x00,
};
uint8_t encoder_data[7];

matlab_serial_t serial;
float matlab_pid_ref[2] = {0.0, 0.0};
float matlab_pid_duty = 0.0;
uint32_t hal_delay = 0;

float angle_recv = 0.0;
float speed_recv = 0.0;
uint32_t err_c = 0x00;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

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
static void driver_steer_set_duty(float duty)
{
	if(duty > 1.0) duty = 1.0;
	if(duty < -1.0) duty = -1.0;
  if(isnan(duty) || isinf(duty)) duty = 0.0;


	if(duty >= 0.0)
	{
    TIM4->CCR1 = (uint32_t)(TIM4->ARR * duty);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	} else
	{
    TIM4->CCR1 = (uint32_t)(TIM4->ARR * -duty);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
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
  MX_DMA_Init();
  MX_CAN_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
  matlab_serial_init_hil(serial, huart1, 0x3A, 0x0D0A, matlab_pid_ref, matlab_pid_duty);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t prev_tick, cur_tick, err_count_tick;
  cur_tick = HAL_GetTick();
  prev_tick = HAL_GetTick();
  // err_count_tick = HAL_GetTick();
  //matlab_pid_ref[0] = 1.570785;
  //status_t ret = RECEIVING_ERROR;
  //driver_steer_set_duty(0.1);
  while (1)
  {
    cur_tick = HAL_GetTick();
    if(cur_tick - prev_tick >= 10)
    {
      prev_tick = cur_tick;
      HAL_CAN_AddTxMessage(&hcan, &angle, angle_request, &TxMailbox);
      HAL_CAN_AddTxMessage(&hcan, &speed, speed_request, &TxMailbox);
      matlab_pid_ref[0] = angle_recv;
      matlab_pid_ref[1] = speed_recv;
      matlab_serial_send(&serial, 1);
    }
    driver_steer_set_duty(matlab_pid_duty);

    //matlab_serial_send(&serial, 10);
    // cur_tick = HAL_GetTick();
    // ret = matlab_serial_receive_common(&serial, 10);
    // driver_steer_set_duty(matlab_pid_duty);
    // if(ret != STATUS_OK) err_c++;
    // if( cur_tick - prev_tick >= 10)
    // {
    //   prev_tick = cur_tick;


    //   matlab_pid_ref[1] = speed_recv;
    //   matlab_serial_send(&serial, 10);
    // }
    // if(cur_tick - err_count_tick >= 1000)
    // {
    //   err_count_tick = cur_tick;
    //   err_c = 0;
    // }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


// Receive to IDLE flag
status_t ret = RECEIVING_ERROR;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  (void)Size; // warning avoiding
  if(huart == &huart1)
  {
    ret = matlab_serial_parse_rx(&serial);
    if(ret != STATUS_OK)
    {
      err_c++;
    }
  }
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
}

/* USER CODE BEGIN 4 */
void set_duty(float duty)
{
	if(duty > 1.0) duty = 1.0;
	if(duty < -1.0) duty = -1.0;

	if(duty >= 0.0)
	{
    TIM4->CCR1 = (uint32_t)(TIM4->ARR * duty);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	} else
	{
			TIM4->CCR1 = (uint32_t)(TIM4->ARR * -duty);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	}
}
int32_t data_speed_in = 0x00;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &encoder_answer, encoder_data) == HAL_OK)
  {
    uint8_t message_type = encoder_data[2];

    switch(message_type)
    {
      case 0x01:
      {
        uint32_t data_angle_in = 0x00;
        data_angle_in += ((uint32_t)encoder_data[6]) << 24;
        data_angle_in += ((uint32_t)encoder_data[5]) << 16;
        data_angle_in += ((uint32_t)encoder_data[4]) << 8;
        data_angle_in += ((uint32_t)encoder_data[3]);
  
        angle_recv = ((float)data_angle_in) * 360.0 / 1024.0; 
        break;
      }
      case 0x0A:
      {
        data_speed_in = 0x00;
        data_speed_in += ((uint32_t)encoder_data[6]) << 24;
        data_speed_in += ((uint32_t)encoder_data[5]) << 16;
        data_speed_in += ((uint32_t)encoder_data[4]) << 8;
        data_speed_in += ((uint32_t)encoder_data[3]);
  
        speed_recv = ((float)data_speed_in) * 0.61328125;         
        break;
      }
    }
  }
}

void pid_init(void)
{
  PID_speed.p_k = 0.140;
  PID_speed.i_k = 0.0;
  PID_speed.d_k = 0.0;
  PID_speed.error = 0.0;
  PID_speed.sum_error = 0.0;
  PID_speed.dif_error = 0.0;
  PID_speed.max_output = 0.50;
  PID_speed.min_output = 0.01;
  PID_speed.max_sum_error = 5.0;
  PID_speed.out = 0.0;
  PID_speed.on = 0;
  PID_speed.error_end = 0.0;
}

void pid_calc(float speed)
{
	PID_speed.current = speed;
	PID_speed.dif_error = PID_speed.error;
	PID_speed.error = PID_speed.target - PID_speed.current;

	if(PID_speed.on)
	{
		PID_speed.sum_error += PID_speed.error;
		if(PID_speed.sum_error > PID_speed.max_sum_error) PID_speed.sum_error = PID_speed.max_sum_error;
		if(PID_speed.sum_error < -PID_speed.max_sum_error) PID_speed.sum_error = -PID_speed.max_sum_error;
	}

	PID_speed.out = 0.0;
	PID_speed.out += PID_speed.p_k * PID_speed.error;
	PID_speed.out += PID_speed.i_k * PID_speed.sum_error;
	PID_speed.out += PID_speed.d_k * PID_speed.dif_error;

	if(PID_speed.out > PID_speed.max_output) PID_speed.out = PID_speed.max_output; else
	if(PID_speed.out < -PID_speed.max_output) PID_speed.out = -PID_speed.max_output;

	if((PID_speed.out < PID_speed.min_output) && (PID_speed.out > -PID_speed.min_output))
		PID_speed.out = 0.0;

	if((PID_speed.error_end > PID_speed.error) && (-PID_speed.error_end < PID_speed.error))
	{
		PID_speed.finish = 1;
	} else PID_speed.finish = 0;

	if(PID_speed.on)
	{
		set_duty(PID_speed.out);
	} else 	set_duty(0.0);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
