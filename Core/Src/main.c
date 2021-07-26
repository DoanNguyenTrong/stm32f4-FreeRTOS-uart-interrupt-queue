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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rtc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include<string.h>
#include<stdint.h>
#include<stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct CMD {
	uint8_t CMD_NUM;
	uint8_t CMD_ARGS[10];
} CMD_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define osFeature_MailQ   1

#define osFeature_MailQ   1
#define LED_ON_CMD			 1
#define LED_OFF_CMD			 2
#define LED_TOGGLE_CMD		 3
#define LED_TOGGLE_OFF_CMD	 4
#define LED_READ_STATUS_CMD	 5
#define RTC_PRINT_DATETIME	 6


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

osThreadId menuDisplayTaskHandle;
osThreadId cmdHandlingTaskHandle;
osThreadId cmdProcessingTaHandle;
osThreadId uartWriteTaskHandle;

osMailQId uartQueueHandle;
osMailQId cmdQueueHandle;
/* USER CODE BEGIN PV */

TimerHandle_t timerHandle = NULL;

RTC_HandleTypeDef   hrtc;
char usr_msg[250] = {0};
uint8_t cmd_buffer[20];
uint8_t cmd_len = 0;
uint8_t byte = 0;
// Menu
char menu[] = {"\
\r\nLED_ON					----> 1\
\r\nLED_OFF					----> 2\
\r\nLED_TOGGLE				----> 3\
\r\nLED_TOGGLE_OFF			----> 4\
\r\nLED_READ_STATUS		----> 5\
\r\nRTC_PRINT_DATETIME		----> 6\
\r\nEXIT						----> 7\
\r\nType your option here: "};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
void StartMenuTask(void const * argument);
void StartCmdHandling(void const * argument);
void StartCmdProcessing(void const * argument);
void StartUartWrite(void const * argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

void printMsg(char *msg);


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
  MX_USART2_UART_Init();
  MX_RTC_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  // This is needed to wake up UART interrupt
  // It's format has to be consistent with one in HAL_UART_RxCpltCallback
  HAL_UART_Receive_IT(&huart2, &byte, 1);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of uartQueue */
  osMailQDef(uartQueue, 10, char *);
  uartQueueHandle = osMailCreate(osMailQ(uartQueue), NULL);

  /* definition and creation of cmdQueue */
  osMailQDef(cmdQueue, 10, CMD_t *);
  cmdQueueHandle = osMailCreate(osMailQ(cmdQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  if (uartQueueHandle && cmdQueueHandle){
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of menuDisplayTask */
  osThreadDef(menuDisplayTask, StartMenuTask, osPriorityBelowNormal, 1, 512);
  menuDisplayTaskHandle = osThreadCreate(osThread(menuDisplayTask), NULL);

  /* definition and creation of cmdHandlingTask */
  osThreadDef(cmdHandlingTask, StartCmdHandling, osPriorityNormal, 2, 512);
  cmdHandlingTaskHandle = osThreadCreate(osThread(cmdHandlingTask), NULL);

  /* definition and creation of cmdProcessingTa */
  osThreadDef(cmdProcessingTa, StartCmdProcessing, osPriorityNormal, 2, 512);
  cmdProcessingTaHandle = osThreadCreate(osThread(cmdProcessingTa), NULL);

  /* definition and creation of uartWriteTask */
  osThreadDef(uartWriteTask, StartUartWrite, osPriorityNormal, 2, 512);
  uartWriteTaskHandle = osThreadCreate(osThread(uartWriteTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  }
  else{
	sprintf(usr_msg, "\r\nFailed to create queues!\r\n");
  }
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
	__HAL_RCC_USART2_CLK_ENABLE();
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */



  /* USER CODE END USART2_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitTypeDef GPIO_InitStruct_UART2;

  /**USART2 GPIO Configuration
  PA2     ------> USART2_TX
  PA3     ------> USART2_RX
  */
	GPIO_InitStruct_UART2.Pin = GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct_UART2.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct_UART2.Pull = GPIO_PULLUP;
	GPIO_InitStruct_UART2.Speed = GPIO_SPEED_LOW;
	GPIO_InitStruct_UART2.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_UART2);


}

/* USER CODE BEGIN 4 */


uint8_t arraySize(char * array) {
	uint8_t i = 0;
	while (array[i] != '\0') i++;
	return i;
}

void printMsg(char *msg)
{
	uint8_t length = arraySize(msg);
//	while ( __HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) != SET);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, length, 100);
//  HAL_UART_Transmit(&huart2,(uint8_t *) msg, sizeof(msg), portMAX_DELAY);
//  HAL_UART_TxCpltCallback(&huart2); // wait for transfer completion
}


void make_led_on(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}
void make_led_off(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

void led_toggle(TimerHandle_t xTimer){
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

void led_toggle_start(uint16_t duration){
	if (timerHandle == NULL){
		// Create Software Timer
		timerHandle = xTimerCreate("LED timer", duration, pdTRUE, NULL, led_toggle);
		osTimerStart(timerHandle, portMAX_DELAY);
	}
	else{
		osTimerStart(timerHandle, portMAX_DELAY);
	}
}


void led_toggle_stop(){
	xTimerStop(timerHandle, portMAX_DELAY);
}

void read_led_status(char *task_msg){
	sprintf(task_msg, "\r\nLED status is: %d\r\n", HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5));
	osMailPut( uartQueueHandle, &task_msg );
}
void rtc_print_status(char *task_msg){
	RTC_TimeTypeDef RTC_time;
	RTC_DateTypeDef RTC_date;

	HAL_RTC_GetTime( &hrtc, &RTC_time, RTC_FORMAT_BCD);
	HAL_RTC_GetDate( &hrtc, &RTC_date, RTC_FORMAT_BCD);

	sprintf(task_msg, "\r\nTime: %02d-%02d-%02d\r\nDate: %02d-%02d-%02d\r\n",
          RTC_time.Hours, RTC_time.Minutes, RTC_time.Seconds,
          RTC_date.Month, RTC_date.Date, RTC_date.Year);
	osMailPut(uartQueueHandle, &task_msg );
}


void print_error_msg(char *task_msg){
	sprintf(task_msg, "\r\nInvalid command received\r\n");
	osMailPut(uartQueueHandle, &task_msg );
}

/** @brief Convert from ASCI to uint8_t.
 *  character to int value
 * @param buffer - character
 * @retval uint8_t value
 */
uint8_t getCommandCode(uint8_t *buffer){
	//return buffer - 48;  // ASCI char to uint8_t
	return buffer[0] - 48;  // ASCI char to uint8_t
}

void getArguments(uint8_t *buffer){
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMenuTask */
/**
  * @brief  Function implementing the menuDisplayTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMenuTask */
void StartMenuTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	for(;;)
	  {
		osMailPut( uartQueueHandle, &menu );
			// Wait here until someone notify
		xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
	  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCmdHandling */
/**
* @brief Function implementing the cmdHandlingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCmdHandling */
void StartCmdHandling(void const * argument)
{
  /* USER CODE BEGIN StartCmdHandling */
  /* Infinite loop */
	// REF: https://www.keil.com/pack/doc/cmsis/RTOS/html/group__CMSIS__RTOS__Mail.html
	uint8_t command_code = 0;
	CMD_t *new_cmd;
  for(;;)
  {
	// Wait for someone to notify
	xTaskNotifyWait(0,0,NULL, portMAX_DELAY);

	// Create new_cmd and allocate memory
	//new_cmd = (CMD_t *) pvPortMalloc( sizeof(CMD_t));
	new_cmd = osMailAlloc(cmdQueueHandle, osWaitForever);       // Allocate memory



	taskENTER_CRITICAL(); // disable interrupt

	uint8_t pos = 0;
	while (cmd_buffer[pos] == '\r' || cmd_buffer[pos] == '\n'){
		pos ++;
	}

	// Display cmd_buffer
	// DEBUGGING
//	sprintf(usr_msg, "\r\ncmd_buffer %d-%d-%s.\r\n", pos,(uint8_t)cmd_buffer[pos],cmd_buffer);
//	printMsg(usr_msg);


	command_code = (uint8_t)cmd_buffer[pos] - 48;
	new_cmd->CMD_NUM = command_code;

	// Open function for future features
	getArguments(new_cmd->CMD_ARGS);

	/*
	 * Show code to screen
	 */

	sprintf(usr_msg, "\r\nTask in queue: %d\r\n", new_cmd->CMD_NUM);
	printMsg(usr_msg);

	taskEXIT_CRITICAL();

	// Put command to queue
	osMailPut(cmdQueueHandle, new_cmd);                         // Send Mail
	osThreadYield();

  }
  /* USER CODE END StartCmdHandling */
}

/* USER CODE BEGIN Header_StartCmdProcessing */
/**
* @brief Function implementing the cmdProcessingTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCmdProcessing */
void StartCmdProcessing(void const * argument)
{
  /* USER CODE BEGIN StartCmdProcessing */
  /* Infinite loop */
	char task_msg[50];
	uint16_t toggle_duration = 1000;
	osEvent event;
	CMD_t *new_cmd;

	while(1){
		event = osMailGet(cmdQueueHandle, osWaitForever);  // wait for mail
		if (event.status == osEventMail) {
			new_cmd = event.value.p;

			sprintf(usr_msg, "\r\nExecuting task %d ...\r\n", new_cmd->CMD_NUM);
			printMsg(usr_msg);

			switch (new_cmd->CMD_NUM){
			case LED_ON_CMD:
				make_led_on();
				break;
			case LED_OFF_CMD:
				make_led_off();
			break;
			case LED_TOGGLE_CMD:
				led_toggle_start(toggle_duration);
				break;
			case LED_TOGGLE_OFF_CMD:
				led_toggle_stop();
				break;
			case LED_READ_STATUS_CMD:
				read_led_status(task_msg);
				break;
			case RTC_PRINT_DATETIME:
				rtc_print_status(task_msg);
				break;
			default:
				print_error_msg(task_msg);
				break;
			}

			osMailFree(cmdQueueHandle, new_cmd);  // free memory allocated for mail
		}
	}
  /* USER CODE END StartCmdProcessing */
}

/* USER CODE BEGIN Header_StartUartWrite */
/**
* @brief Function implementing the uartWriteTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartWrite */
void StartUartWrite(void const * argument)
{
  /* USER CODE BEGIN StartUartWrite */
  /* Infinite loop */
	for(;;)
	  {
		osEvent event = osMailGet(uartQueueHandle, osWaitForever);
		char *received = (char *)event.value.p;
		if (event.status == osOK || event.status == osEventMail){
			printMsg( "\r\n===========Application===========\r\n" );
		  printMsg( received );
		}
		else{
		  sprintf(usr_msg, "\r\nError code %d!!", event.status);
		  printMsg( usr_msg );
		}
		// Free the memory block to be reused
		osMailFree(uartQueueHandle, received);

	  }
  /* USER CODE END StartUartWrite */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
