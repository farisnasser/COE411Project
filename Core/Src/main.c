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
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_i2c.h"     // LCD over I2C
#include "rfid.h"        // RFID reader interface
#include "keypad.h"      // Matrix keypad interface
#include <string.h>
#include <stdio.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ---- Servo + Buzzer helper functions ----

// Servo: convert desired angle (0–180°) to PWM pulse width on TIM1 CH1
void Servo_SetAngle(float angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    // Map 0–180° → 1000–2000 µs pulse (standard hobby servo range)
    uint32_t pulse = 1000 + (uint32_t)((angle / 180.0f) * 1000.0f);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}

// Move servo to locked position (e.g., 0°) and print debug message
void Door_Lock(void) {
    Servo_SetAngle(0);
    HAL_Delay(300);
    printf("🔒 Door locked\n");
}

// Move servo to unlocked position (e.g., 180°) and print debug message
void Door_Unlock(void) {
    Servo_SetAngle(180);
    HAL_Delay(300);
    printf("🔓 Door unlocked\n");
}

// ---- Buzzer setup (TIM3 CH1) ----
#define BUZZ_TIM      htim3
#define BUZZ_CHANNEL  TIM_CHANNEL_1

// Event strings sent over UART3 to ESP12F (for MQTT / logging on ESP side)
#define RFID_SCANNED "RFID_SCANNED"
#define PIN_CORRECT "PIN_CORRECT"
#define PIN_WRONG "PIN_WRONG"
#define RFID_BAD "RFID_BAD"  // Define the RFID_BAD event
#define ACCESS_GRANTED "ACCESS_GRANTED"
#define ACCESS_DENIED "ACCESS_DENIED"

// Helper: send an event string to ESP via USART3 (each message ends with '\n')
void send_event(char* event_message) {
    // Send the event message to USART3 (which is connected to ESP12F)
    HAL_UART_Transmit(&huart3, (uint8_t*)event_message, strlen(event_message), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t*)"\n", 1, HAL_MAX_DELAY);  // Send a newline for easier parsing
}

// Set buzzer output frequency by configuring TIM3 period and duty cycle
static inline void buzzer_set_freq(uint32_t freq_hz) {
    if (freq_hz == 0) {
        // 0 Hz → turn buzzer “off” (no toggling)
        __HAL_TIM_SET_COMPARE(&BUZZ_TIM, BUZZ_CHANNEL, 0);
        return;
    }
    // Compute timer period for a given frequency assuming 1 MHz timer base
    uint32_t period = 1000000UL / freq_hz;
    if (period < 2) period = 2;
    __HAL_TIM_SET_AUTORELOAD(&BUZZ_TIM, period - 1);
    __HAL_TIM_SET_COMPARE(&BUZZ_TIM, BUZZ_CHANNEL, period / 4); // ~25% duty cycle
}

// Convenience: stop buzzer sound (duty cycle = 0)
static inline void buzzer_off(void) {
    __HAL_TIM_SET_COMPARE(&BUZZ_TIM, BUZZ_CHANNEL, 0);
}

// Play a single tone at freq_hz for ms milliseconds, then short pause
static void play_tone(uint32_t freq_hz, uint32_t ms) {
    buzzer_set_freq(freq_hz);
    HAL_Delay(ms);
    buzzer_off();
    HAL_Delay(30);
}

// Success chime: short ascending melody
void chime_ok(void) {
    play_tone(784, 120);
    play_tone(988, 120);
    play_tone(1175, 160);
}

// Error chime: descending / “wrong” sound
void chime_error(void) {
    play_tone(330, 150);
    play_tone(247, 250);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // (Place for early user variables if needed)
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();   // Initialize HAL library, SysTick, and reset peripherals

  /* USER CODE BEGIN Init */
  // (Optional user init before clock setup)
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();  // Configure PLL and bus clocks

  /* USER CODE BEGIN SysInit */
  // (Optional: system-level user init after clock)
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();  // UART2 (often used for debug / printf)
  MX_I2C1_Init();         // I2C1 (for LCD)
  MX_TIM3_Init();         // Timer 3 (buzzer PWM)
  MX_TIM1_Init();         // Timer 1 (servo PWM)
  MX_USART1_UART_Init();  // UART1 (RFID reader)
  MX_USART3_UART_Init();  // UART3 (ESP12F WiFi/MQTT bridge)
  /* USER CODE BEGIN 2 */
  // Start PWM outputs for servo and buzzer timers
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   // Servo
  HAL_TIM_PWM_Start(&BUZZ_TIM, BUZZ_CHANNEL); // Buzzer

  // UART startup message (debug) over USART2
  char start_msg[] = "\r\n=== Smart Door System Booted ===\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)start_msg, strlen(start_msg), HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();    // Create FreeRTOS tasks, queues, etc.

  /* Start scheduler */
  osKernelStart();       // Hand control to FreeRTOS (tasks now run)

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Should never execute if FreeRTOS is running correctly
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// (Extra user functions can go here)
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();   // Disable interrupts
  while (1)
  {
      // Stay here forever on critical error
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
