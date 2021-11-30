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
#include "i2c.h"
#include "rtc.h"
#include "subghz.h"
#include "usart.h"
#include "gpio.h"
#include "tmp117.h"
#include "lps33hw_reg.h"
#include "stdio.h"
#include "radio.h"
#include "lora.h"
#include "radio_conf.h"
#include "radio_driver.h" 
#include "mw_log_conf.h"


/* semihosting Initializing */
//extern void initialise_monitor_handles(void);

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RADIO_MODE_STANDBY_RC        0x02
#define RADIO_MODE_STANDBY_HSE32     0x03
#define RADIO_MODE_STANDBY_FS        0x04
#define RADIO_MODE_STANDBY_RX        0x05
#define RADIO_MODE_TX                0x06

#define RADIO_STATUS_DATA_READY      0x02
#define RADIO_STATUS_TIMEOUT         0x03
#define RADIO_STATUS_PROC_ERROR      0x04
#define RADIO_STATUS_EXEC_FAILURE    0x05
#define RADIO_STATUS_TX_DONE         0x06

#define RADIO_MODE_BITFIELD          0x70
#define RADIO_STATUS_BITFIELD        0x0E

#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SUBGHZ_HandleTypeDef hsubghz;

/* USER CODE BEGIN PV */
uint8_t RadioCmd[3] = {0x00, 0x00, 0x00};
uint8_t RadioResult = 0x00;
uint8_t RadioParam  = 0x00;
uint8_t RadioMode   = 0x00;
uint8_t RadioStatus = 0x00;

uint8_t TXbuf[4] = { 0x0A, 0x0B, 0x0C, 0x0D };
uint16_t Size = NELEMS(TXbuf);

char buf[50] = {0};

/* USER CODE END PV */

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

  static float P = 0.0;
  static float T1 = 0.0;
  static float T2 = 0.0;


  static RTC_TimeTypeDef sTime;
  static RTC_DateTypeDef sDate;

  static stmdev_ctx_t pressure;
  static uint32_t rawP; 
  static int16_t rawT; 

  static uint8_t whoamI, rst;

  /* ARM Semihosting 
  initialise_monitor_handles();
  printf("ARM Semihosting Enabled...\n"); */

  /* MCU Configuration-------------------------------------------------------- */

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_SUBGHZ_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();

   /* Configure LED2 & LED3 */
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);
  
  /* Peripheral Configuration-------------------------------------------------------- */
  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 15, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0);

  /* Check TMP117 device ID */
  TMP117_get_ID_Register(hi2c2);  

  /* Init pressure sensor structure */
  pressure.write_reg = platform_write;
  pressure.read_reg = platform_read;
  pressure.handle = &hi2c2;

  /* Check LPS33HW device ID */
  whoamI = 0;
  lps33hw_device_id_get(&pressure, &whoamI);

  if ( whoamI != LPS33HW_ID )
    while (1); /*manage here device not found */

  /* Restore default configuration 
  lps33hw_reset_set(&pressure, PROPERTY_ENABLE);
  do {
    lps33hw_reset_get(&pressure, &rst);
  } while (rst);
  */

  /* Enable Block Data Update */
  //lps33hw_block_data_update_set(&pressure, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lps33hw_data_rate_set(&pressure, LPS33HW_ODR_1_Hz);

  /*## 0 - Wakeup the SUBGHZ Radio ###########################################*/
  /* Set Sleep Mode */
  if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_SLEEP, &RadioParam, 1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Set Standby Mode */
  if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_STANDBY, &RadioParam, 1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Retrieve Status from SUBGHZ Radio */
  if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, &RadioResult, 1) != HAL_OK)
  {
    Error_Handler();
  }
  else
  {
    /* Format Mode and Status receive from SUBGHZ Radio */
    RadioMode   = ((RadioResult & RADIO_MODE_BITFIELD) >> 4); 
    
    /* Check if SUBGHZ Radio is in RADIO_MODE_STANDBY_RC mode */
    if(RadioMode != RADIO_MODE_STANDBY_RC)
    {
      Error_Handler();
    }
  }

  LoRaInit();

  /* Infinite loop */
  while (1)
  {
 
  /*## 1 - Write to SUBGHZ Radio TX Buffer ################################*/
  if ( HAL_SUBGHZ_WriteBuffer(&hsubghz, 0, &TXbuf[0], Size) != HAL_OK)
  {
    Error_Handler();
  }

  /*## 2 - Set a TX on SUBGHZ Radio side #####################################*/
  /* Set Tx Mode. RadioCmd = 0x00 Timeout deactivated */
  if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TX, RadioCmd, 3) != HAL_OK)
  {
    Error_Handler();
  }

  /*## 3 - Get TX status from SUBGHZ Radio side ##############################*/
  /* Check that TX is well ongoing (RADIO_MODE_TX), wait end of transfer */

  /* Reset RadioResult */
  RadioResult = 0x00;

   /* Retrieve Status from SUBGHZ Radio */
  if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, &RadioResult, 1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Format Mode and Status receive from SUBGHZ Radio */
  RadioMode   = ((RadioResult & RADIO_MODE_BITFIELD) >> 4);
  RadioStatus = ((RadioResult & RADIO_STATUS_BITFIELD) >> 1);

  if (RadioMode == RADIO_MODE_TX)
  {
    /* Wait end of transfer. SUBGHZ Radio go in Standby Mode */
    do
    {
      /* Reset RadioResult */
      RadioResult = 0x00;

      /* Retrieve Status from SUBGHZ Radio */
      if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, &RadioResult, 1) != HAL_OK)
      {
        Error_Handler();
      }

      /* Format Mode and Status receive from SUBGHZ Radio */
      RadioMode   = ((RadioResult & RADIO_MODE_BITFIELD) >> 4); 
      RadioStatus = ((RadioResult & RADIO_STATUS_BITFIELD) >> 1);
    }
    while (RadioMode != RADIO_MODE_STANDBY_RC);
  }
  else
  {
    /* Call Error Handler; LED1 blinking */
    //Error_Handler();
  }

  /* Check if TX is well done  (SUBGHZ Radio already in Standby mode) */
  if (RadioStatus == RADIO_STATUS_TX_DONE)
  {
    /* Turn LED2 on */
    BSP_LED_On(LED2);
  }
  else
  {
    /* Call Error Handler; LED1 blinking */
   // Error_Handler();
  }

    T1 = TMP117_get_Temperature(hi2c2);

    lps33hw_pressure_offset_get(&pressure, &rawT );
    lps33hw_pressure_ref_get(&pressure, &rawP );

    lps33hw_pressure_raw_get( &pressure, &rawP )  ;
    P = ((float)rawP) / 4096; // 4096 LSB/hPa

    lps33hw_temperature_raw_get( &pressure, &rawT );
    T2 = ((float)rawT) / 100; // 100 LSB/degC


    memset(buf, 0, sizeof(buf));
    snprintf(buf, sizeof(buf), "T1: %.3f C, T2: %.3f C, P: %.3f hPa \r\n", T1, T2, P );
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), 1000);

    BSP_LED_Off(LED2);

  if (READ_BIT(RTC->SR, RTC_SR_WUTF) != 0U)
  {
    BSP_LED_On(LED3);

  /* Clear the WAKEUPTIMER Flag */
  WRITE_REG(RTC->SCR, RTC_SCR_CWUTF);

  /* Change RTC state */
  hrtc.State = HAL_RTC_STATE_READY;

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    memset(buf, 0, sizeof(buf));
    snprintf(buf, sizeof(buf), "%02d:%02d\r\n", sTime.Minutes, sTime.Seconds );
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), 1000);

  }

    HAL_Delay(1000);
    BSP_LED_Off(LED3);



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

  /** Configure LSE Drive Capability
  */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int __io_putchar(int ch)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
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
  while(1)
  {
    BSP_LED_Toggle(LED3);
    HAL_Delay(500);
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
