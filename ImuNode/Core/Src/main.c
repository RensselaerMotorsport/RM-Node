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
#include "can.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include "asm330lhh_reg.h"

#include "stm32f0xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_BUS hi2c1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define    BOOT_TIME            10 //ms
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float_t acceleration_mg[3];
static float_t angular_rate_mdps[3];
static float_t temperature_degC;
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
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
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;//Alternate function push-pull
  	GPIO_InitStruct.Pull = GPIO_NOPULL;
  	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  	GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
  	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  	CAN_TxHeaderTypeDef TxHeader;
  	uint8_t TxData[8];
  	uint32_t TxMailbox;

  	TxHeader.IDE = CAN_ID_STD;
  	TxHeader.StdId = 0x446;
  	TxHeader.RTR = CAN_RTR_DATA;
  	TxHeader.DLC = 2;

  	TxData[0] = 50;
  	TxData[1] = 0xAA;

  	stmdev_ctx_t dev_ctx;
  	dev_ctx.write_reg = platform_write;
  	dev_ctx.read_reg = platform_read;
  	dev_ctx.mdelay = platform_delay;
  	dev_ctx.handle = &SENSOR_BUS;
  	/* Wait sensor boot time */
  	platform_delay(BOOT_TIME);
  	/* Check device ID */
  	asm330lhh_device_id_get(&dev_ctx, &whoamI);

  	if (whoamI != ASM330LHH_ID)
  	  while (1);

  	  /* Restore default configuration */
  	asm330lhh_reset_set(&dev_ctx, PROPERTY_ENABLE);

  	do {
  	  asm330lhh_reset_get(&dev_ctx, &rst);
  	} while (rst);

  	/* Start device configuration. */
  	asm330lhh_device_conf_set(&dev_ctx, PROPERTY_ENABLE);
  	/* Enable Block Data Update */
  	asm330lhh_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  	/* Set Output Data Rate */
  	asm330lhh_xl_data_rate_set(&dev_ctx, ASM330LHH_XL_ODR_12Hz5);
  	asm330lhh_gy_data_rate_set(&dev_ctx, ASM330LHH_GY_ODR_12Hz5);
  	/* Set full scale */
  	asm330lhh_xl_full_scale_set(&dev_ctx, ASM330LHH_2g);
  	asm330lhh_gy_full_scale_set(&dev_ctx, ASM330LHH_2000dps);
  	/* Configure filtering chain(No aux interface)
  	 * Accelerometer - LPF1 + LPF2 path
  	 */
  	asm330lhh_xl_hp_path_on_out_set(&dev_ctx, ASM330LHH_LP_ODR_DIV_100);
  	asm330lhh_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint8_t reg;
	  	  /* Read output only if new xl value is available */
	  	  asm330lhh_xl_flag_data_ready_get(&dev_ctx, &reg);

	  	  if (reg) {
	  	    /* Read acceleration field data */
	  	    memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
	  	    asm330lhh_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
	  	    acceleration_mg[0] =
	  	      asm330lhh_from_fs2g_to_mg(data_raw_acceleration[0]);
	  	    acceleration_mg[1] =
	  	      asm330lhh_from_fs2g_to_mg(data_raw_acceleration[1]);
	  	    acceleration_mg[2] =
	  	      asm330lhh_from_fs2g_to_mg(data_raw_acceleration[2]);
	  	    snprintf((char *)tx_buffer, sizeof(tx_buffer),
	  	              "Acceleration [mg]:%i\t%i\t%i\r\n",
	  	              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
	  	      tx_com(tx_buffer, strlen((char const *)tx_buffer));
	  	    }

	  	    asm330lhh_gy_flag_data_ready_get(&dev_ctx, &reg);

	  	    if (reg) {
	  	      /* Read angular rate field data */
	  	      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
	  	      asm330lhh_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
	  	      angular_rate_mdps[0] =
	  	        asm330lhh_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
	  	      angular_rate_mdps[1] =
	  	        asm330lhh_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
	  	      angular_rate_mdps[2] =
	  	        asm330lhh_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);
	  	      snprintf((char *)tx_buffer, sizeof(tx_buffer),
	  	              "Angular rate [mdps]:%i\t%i\t%i\r\n",
	  	              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
	  	      tx_com(tx_buffer, strlen((char const *)tx_buffer));
	  	    }

	  	    //asm330lhh_temp_flag_data_ready_get(&dev_ctx, &reg);

	  	    //if (reg) {
	  	      /* Read temperature data */
	  	      /*memset(&data_raw_temperature, 0x00, sizeof(int16_t));
	  	      asm330lhh_temperature_raw_get(&dev_ctx, &data_raw_temperature);
	  	      temperature_degC = asm330lhh_from_lsb_to_celsius(
	  	                           data_raw_temperature);
	  	      snprintf((char *)tx_buffer, sizeof(tx_buffer),
	  	              "Temperature [degC]:%6.2f\r\n", temperature_degC);
	  	      tx_com(tx_buffer, strlen((char const *)tx_buffer));
	  	    }*/
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
	{
	HAL_I2C_Mem_Write(handle, ASM330LHH_I2C_ADD_L, reg,
            I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
	return 0;
	}

	static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
	{
	  HAL_I2C_Mem_Read(handle, ASM330LHH_I2C_ADD_L, reg,
	                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	  return 0;
	}

	static void tx_com(uint8_t *tx_buffer, uint16_t len)
	{
	 HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
	}

	static void platform_delay(uint32_t ms)
	{
	  HAL_Delay(ms);
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
