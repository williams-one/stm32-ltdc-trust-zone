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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32u5xx_ll_rcc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Non-secure Vector table to jump to (internal Flash Bank2 here)             */
/* Caution: address must correspond to non-secure internal Flash where is     */
/*          mapped in the non-secure vector table                             */
#define VTOR_TABLE_NS_START_ADDR  0x08200000UL

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void NonSecure_Init(void);
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_GTZC_S_Init(void);
static void MX_ICACHE_Init(void);
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
  /* SAU/IDAU, FPU and interrupts secure/non-secure allocation setup done */
/* in SystemInit() based on partition_stm32u5g9xx.h file's definitions. */

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the System Power */
  SystemPower_Config();

  /* Configure the system clock */
  SystemClock_Config();
  /* GTZC initialisation */
  MX_GTZC_S_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ICACHE_Init();
  /* USER CODE BEGIN 2 */
  LL_RCC_ConfigSecure(LL_RCC_ALL_NSEC);
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /*************** Setup and jump to non-secure *******************************/

  NonSecure_Init();

  /* Non-secure software does not return, this code is not executed */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief  Non-secure call function
  *         This function is responsible for Non-secure initialization and switch
  *         to non-secure state
  * @retval None
  */
static void NonSecure_Init(void)
{
  funcptr_NS NonSecure_ResetHandler;

  SCB_NS->VTOR = VTOR_TABLE_NS_START_ADDR;

  /* Set non-secure main stack (MSP_NS) */
  __TZ_set_MSP_NS((*(uint32_t *)VTOR_TABLE_NS_START_ADDR));

  /* Get non-secure reset handler */
  NonSecure_ResetHandler = (funcptr_NS)(*((uint32_t *)((VTOR_TABLE_NS_START_ADDR) + 4U)));

  /* Start non-secure state software application */
  NonSecure_ResetHandler();
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/**
  * @brief GTZC_S Initialization Function
  * @param None
  * @retval None
  */
static void MX_GTZC_S_Init(void)
{

  /* USER CODE BEGIN GTZC_S_Init 0 */

  /* USER CODE END GTZC_S_Init 0 */

  MPCBB_ConfigTypeDef MPCBB_Area_Desc = {0};

  /* USER CODE BEGIN GTZC_S_Init 1 */

  /* USER CODE END GTZC_S_Init 1 */
  if (HAL_GTZC_TZSC_ConfigPeriphAttributes(GTZC_PERIPH_TIM3, GTZC_TZSC_PERIPH_SEC|GTZC_TZSC_PERIPH_NPRIV) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_GTZC_TZSC_ConfigPeriphAttributes(GTZC_PERIPH_OTG, GTZC_TZSC_PERIPH_SEC|GTZC_TZSC_PERIPH_NPRIV) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_GTZC_TZSC_ConfigPeriphAttributes(GTZC_PERIPH_AES, GTZC_TZSC_PERIPH_SEC|GTZC_TZSC_PERIPH_NPRIV) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_GTZC_TZSC_ConfigPeriphAttributes(GTZC_PERIPH_HASH, GTZC_TZSC_PERIPH_SEC|GTZC_TZSC_PERIPH_NPRIV) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_GTZC_TZSC_ConfigPeriphAttributes(GTZC_PERIPH_RNG, GTZC_TZSC_PERIPH_SEC|GTZC_TZSC_PERIPH_NPRIV) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_GTZC_TZSC_ConfigPeriphAttributes(GTZC_PERIPH_SAES, GTZC_TZSC_PERIPH_SEC|GTZC_TZSC_PERIPH_NPRIV) != HAL_OK)
  {
    Error_Handler();
  }
  MPCBB_Area_Desc.SecureRWIllegalMode = GTZC_MPCBB_SRWILADIS_ENABLE;
  MPCBB_Area_Desc.InvertSecureState = GTZC_MPCBB_INVSECSTATE_NOT_INVERTED;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[0] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[1] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[2] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[3] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[4] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[5] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[6] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[7] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[8] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[9] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[10] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[11] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[12] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[13] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[14] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[15] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[16] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[17] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[18] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[19] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[20] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[21] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[22] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[23] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[24] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[25] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[26] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[27] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[28] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[29] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[30] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[31] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[32] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[33] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[34] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[35] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[36] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[37] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[38] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[39] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[40] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[41] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[42] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[43] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[44] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[45] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[46] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[47] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[48] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[49] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[50] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[51] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[0] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[1] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[2] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[3] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[4] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[5] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[6] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[7] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[8] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[9] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[10] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[11] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[12] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[13] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[14] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[15] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[16] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[17] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[18] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[19] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[20] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[21] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[22] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[23] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[24] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[25] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[26] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[27] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[28] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[29] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[30] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[31] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[32] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[33] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[34] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[35] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[36] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[37] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[38] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[39] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[40] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[41] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[42] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[43] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[44] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[45] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[46] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[47] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[48] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[49] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[50] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[51] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_LockConfig_array[0] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_LockConfig_array[1] =   0x00000000;
  if (HAL_GTZC_MPCBB_ConfigMem(SRAM3_BASE, &MPCBB_Area_Desc) != HAL_OK)
  {
    Error_Handler();
  }
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[0] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[1] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[2] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[3] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[4] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[5] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[6] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[7] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[8] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[9] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[10] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[11] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[12] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[13] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[14] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[15] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[16] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[17] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[18] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[19] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[20] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[21] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[22] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[23] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[24] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[25] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[26] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[27] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[28] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[29] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[30] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[31] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[32] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[33] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[34] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[35] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[36] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[37] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[38] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[39] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[40] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[41] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[42] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[43] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[44] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[45] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[46] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[47] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[48] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[49] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[50] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[51] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[0] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[1] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[2] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[3] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[4] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[5] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[6] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[7] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[8] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[9] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[10] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[11] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[12] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[13] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[14] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[15] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[16] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[17] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[18] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[19] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[20] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[21] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[22] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[23] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[24] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[25] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[26] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[27] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[28] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[29] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[30] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[31] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[32] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[33] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[34] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[35] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[36] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[37] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[38] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[39] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[40] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[41] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[42] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[43] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[44] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[45] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[46] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[47] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[48] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[49] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[50] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[51] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_LockConfig_array[0] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_LockConfig_array[1] =   0x00000000;
  if (HAL_GTZC_MPCBB_ConfigMem(SRAM5_BASE, &MPCBB_Area_Desc) != HAL_OK)
  {
    Error_Handler();
  }
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[0] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[1] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[2] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[3] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[4] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[5] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[6] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[7] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[8] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[9] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[10] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[11] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[12] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[13] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[14] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[15] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[16] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[17] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[18] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[19] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[20] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[21] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[22] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[23] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[24] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[25] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[26] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[27] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[28] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[29] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[30] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_SecConfig_array[31] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[0] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[1] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[2] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[3] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[4] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[5] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[6] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[7] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[8] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[9] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[10] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[11] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[12] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[13] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[14] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[15] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[16] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[17] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[18] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[19] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[20] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[21] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[22] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[23] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[24] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[25] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[26] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[27] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[28] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[29] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[30] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_PrivConfig_array[31] =   0x00000000;
  MPCBB_Area_Desc.AttributeConfig.MPCBB_LockConfig_array[0] =   0x00000000;
  if (HAL_GTZC_MPCBB_ConfigMem(SRAM6_BASE, &MPCBB_Area_Desc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN GTZC_S_Init 2 */

  /* USER CODE END GTZC_S_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);

  /*IO attributes management functions */
  HAL_GPIO_ConfigPinAttributes(GPIOE, LCD_ON_Pin|BL_CTRL_Pin|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0, GPIO_PIN_NSEC);

  /*IO attributes management functions */
  HAL_GPIO_ConfigPinAttributes(GPIOF, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_NSEC);

  /*IO attributes management functions */
  HAL_GPIO_ConfigPinAttributes(GPIOB, GPIO_PIN_2|GPIO_PIN_9, GPIO_PIN_NSEC);

  /*IO attributes management functions */
  HAL_GPIO_ConfigPinAttributes(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GREEN_LED_Pin
                          |GPIO_PIN_6, GPIO_PIN_NSEC);

  /*IO attributes management functions */
  HAL_GPIO_ConfigPinAttributes(GPIOH, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_NSEC);

  /*IO attributes management functions */
  HAL_GPIO_ConfigPinAttributes(GPIOI, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_NSEC);

  /*IO attributes management functions */
  HAL_GPIO_ConfigPinAttributes(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_NSEC);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RED_LED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
