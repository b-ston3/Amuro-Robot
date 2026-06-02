/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "motor-control.h"
#include "encoder_calib.h"
#include "uart_debug.h"
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
char string_buf[100];

volatile uint32_t adc[6];
uint32_t buffer[6];
volatile uint8_t conversion_done_flag = 1;

EncoderCalib_t encoder_left;
EncoderCalib_t encoder_right;

// State Machine für Kalibrierung
typedef enum {
	STATE_IDLE, STATE_CALIB_RUNNING, STATE_CALIB_DONE, STATE_NORMAL_OPERATION
} RobotState_t;

RobotState_t robot_state = STATE_IDLE;
uint32_t calib_timeout = 0;

// Minimale Zeit für volle Umdrehung (in ms)
// 0.5 U/s = 2 Sekunden pro Umdrehung + Puffer
#define CALIB_MIN_TIME_MS 5000  // 5 Sekunden (großzügig für imperfekte weiße Flächen)
#define CALIB_MAX_TIME_MS 15000 // 15 Sekunden Timeout (falls was schiefgeht)
/*
 * Speicher	Channel	PIN		Sensor
 * adc[0]	IN5		PA0		lineSensor_middle
 * adc[1]	IN6		PA1		encoder_left
 * adc[2]	IN8		PA3		lineSensor_right
 * adc[3]	IN9		PA4		battery (voltage)
 * adc[4]	IN10 	PA5 	encoder_right
 * adc[5]	IN12	PA7		lineSensor_left
 */
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
int main(void) {

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
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1, buffer, 6);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	EncoderCalib_Init(&encoder_left);
	EncoderCalib_Init(&encoder_right);

	EncoderCalib_StartCalibration(&encoder_left);
	EncoderCalib_StartCalibration(&encoder_right);
	robot_state = STATE_CALIB_RUNNING;
	calib_timeout = HAL_GetTick();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (conversion_done_flag) {
			conversion_done_flag = 0;

			HAL_ADC_Start_DMA(&hadc1, buffer, 6);

			// === KALIBRIERUNGS-STATE MACHINE ===
			if (robot_state == STATE_CALIB_RUNNING) {

				motorControl('r', 'f', 5);
				motorControl('l', 'f', 5);
				HAL_Delay(5000);

				bool left_done = EncoderCalib_AddSample(&encoder_left, adc[1]);
				bool right_done = EncoderCalib_AddSample(&encoder_right,
						adc[4]);

				// Universelle Print-Funktion verwenden!
				uart_print_encoder_calib(adc[1], adc[4],
						encoder_left.sample_count);

				// Prüfe Timeout
				uint32_t elapsed = HAL_GetTick() - calib_timeout;
				if (elapsed > CALIB_MAX_TIME_MS) {
					uart_printf(
							"ERROR: Calibration timeout! Elapsed: %lu ms\r\n",
							elapsed);
					robot_state = STATE_IDLE;
					motorControl('r', 'f', 0);
					motorControl('l', 'f', 0);
				}
				// Mindestzeit erfüllt UND alle Samples gesammelt?
				else if (elapsed >= CALIB_MIN_TIME_MS && left_done
						&& right_done) {
					uart_printf(
							"Calibration data collected! Time: %lu ms, Samples: %u\r\n",
							elapsed, encoder_left.sample_count);
					robot_state = STATE_CALIB_DONE;
				}

			} else if (robot_state == STATE_CALIB_DONE) {

				EncoderCalib_Finalize(&encoder_left, 0.1f);
				EncoderCalib_Finalize(&encoder_right, 0.1f);

				uart_printf("\n===== CALIBRATION COMPLETE =====\r\n");
				EncoderCalib_PrintCalibData(&encoder_left);
				EncoderCalib_PrintCalibData(&encoder_right);
				uart_printf("================================\r\n");

				motorControl('r', 'f', 0);
				motorControl('l', 'f', 0);

				robot_state = STATE_NORMAL_OPERATION;
			} else if (robot_state == STATE_NORMAL_OPERATION) {

				EncoderCalib_UpdateState(&encoder_left, adc[1]);
				EncoderCalib_UpdateState(&encoder_right, adc[4]);

				// Auch hier universelle Print-Funktion!
				uart_print_encoder_state(adc[1], adc[4],
						EncoderCalib_GetState(&encoder_left),
						EncoderCalib_GetState(&encoder_right));

				motorControl('r', 'f', 10);
				motorControl('l', 'f', 10);
			} else {
				// STATE_IDLE: nur einfache ADC Ausgabe
				uart_print_adc(adc[1], adc[4]);
			}
			/* USER CODE END WHILE */
		}
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE
			| RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}

	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc1) {
	for (int i = 0; i < 6; i++) {
		adc[i] = buffer[i];
	}
	conversion_done_flag = 1;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
