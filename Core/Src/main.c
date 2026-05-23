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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PIN_SENSOR_LLUVIA GPIO_PIN_1
#define PIN_SENSOR_DHT22  GPIO_PIN_2
#define PUERTO_SENSORES   GPIOA
#define DHT22_TIMEOUT_US  3000u
#define MPU6050_DIR_7BIT_0      (0x68u)
#define MPU6050_DIR_7BIT_1      (0x69u)
#define MPU6050_DIR_HAL_0       (MPU6050_DIR_7BIT_0 << 1)
#define MPU6050_DIR_HAL_1       (MPU6050_DIR_7BIT_1 << 1)
#define MPU6050_REG_WHO_AM_I   (0x75u)
#define MPU6050_REG_PWR_MGMT_1 (0x6Bu)
#define MPU6050_REG_ACCEL_XOUT_H (0x3Bu)
#define ESTADO_RIESGO_VERDE    (0u)
#define ESTADO_RIESGO_AMARILLO (1u)
#define ESTADO_RIESGO_ROJO     (2u)
#define LLUVIA_INTENSIDAD_NULA   (0u)
#define LLUVIA_INTENSIDAD_LEVE   (1u)
#define LLUVIA_INTENSIDAD_MEDIA  (2u)
#define LLUVIA_INTENSIDAD_FUERTE (3u)
#define ADC_CANAL_HUMEDAD_SUELO ADC_CHANNEL_0
#define ADC_CANAL_LLUVIA_AO     ADC_CHANNEL_3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint16_t humedad_suelo_adc_crudo = 0;
volatile float humedad_suelo_voltaje = 0.0f;
volatile float humedad_suelo_porcentaje = 0.0f;
volatile uint8_t lluvia_do_crudo = 0;
volatile uint8_t lluvia_activa = 0;
volatile uint32_t lluvia_do_crudo_dbg = 0;
volatile uint32_t lluvia_activa_dbg = 0;
volatile uint16_t lluvia_ao_adc_crudo = 0;
volatile float lluvia_ao_voltaje = 0.0f;
volatile float lluvia_ao_nivel_pct = 0.0f;
volatile float lluvia_ao_nivel_agua_pct = 0.0f;
volatile float lluvia_ao_nivel_agua_calibrado_pct = 0.0f;
volatile uint16_t lluvia_ao_adc_ref_seco = 3850;
volatile uint16_t lluvia_ao_adc_ref_mojado = 3600;
volatile float umbral_lluvia_leve_pct = 20.0f;
volatile float umbral_lluvia_media_pct = 50.0f;
volatile float umbral_lluvia_fuerte_pct = 80.0f;
volatile uint32_t lluvia_intensidad_dbg = LLUVIA_INTENSIDAD_NULA;
volatile float temperatura_ambiente_c = 0.0f;
volatile float humedad_ambiente_pct = 0.0f;
volatile uint32_t dht22_lectura_valida_dbg = 0;
volatile uint32_t dht22_checksum_ok_dbg = 0;
volatile uint32_t dht22_muestreo_us_dbg = 48;
volatile uint32_t mpu6050_conectado_dbg = 0;
volatile uint32_t mpu6050_whoami_dbg = 0;
volatile uint32_t mpu6050_direccion_dbg = 0;
volatile int16_t  mpu6050_acel_x = 0;
volatile int16_t  mpu6050_acel_y = 0;
volatile int16_t  mpu6050_acel_z = 0;
volatile int16_t  mpu6050_gyro_x = 0;
volatile int16_t  mpu6050_gyro_y = 0;
volatile int16_t  mpu6050_gyro_z = 0;
static uint16_t mpu6050_direccion_activa_hal = 0;
volatile float umbral_humedad_alta_pct = 70.0f;
volatile float umbral_humedad_saturada_pct = 85.0f;
volatile int16_t umbral_vibracion_gyro_abs = 250;
volatile uint32_t vibracion_anomala_dbg = 0;
volatile uint32_t estado_riesgo_dbg = ESTADO_RIESGO_VERDE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM14_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void SensorHumedadSuelo_Actualizar(void);
static void SensorLluvia_Actualizar(void);
static uint16_t ADC_LeerCanal(uint32_t canal);
static void DHT22_DelayUs(uint32_t us);
static uint8_t DHT22_LeerBit(void);
static uint8_t DHT22_LeerDatos(uint8_t datos[5]);
static void SensorDHT22_Actualizar(void);
static void SensorMPU6050_Inicializar(void);
static void SensorMPU6050_Actualizar(void);
static void EstadoRiesgo_Actualizar(void);
static void Bluetooth_EnviarTrama(void);

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
  MX_ADC_Init();
  MX_TIM14_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim14);
  SensorMPU6050_Inicializar();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    SensorHumedadSuelo_Actualizar();
    SensorLluvia_Actualizar();
    SensorDHT22_Actualizar();
    SensorMPU6050_Actualizar();
    EstadoRiesgo_Actualizar();
    Bluetooth_EnviarTrama();
    HAL_Delay(1000);
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

  /** Initializes the CPU, AHB and APB buses clocks
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

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 47;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* PA1 = entrada digital del sensor de lluvia (DO de YL-83). */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void SensorHumedadSuelo_Actualizar(void)
{
  humedad_suelo_adc_crudo = ADC_LeerCanal(ADC_CANAL_HUMEDAD_SUELO);
  humedad_suelo_voltaje = (3.3f * (float)humedad_suelo_adc_crudo) / 4095.0f;

  /* Initial inverse mapping: wet ~= low ADC, dry ~= high ADC. */
  humedad_suelo_porcentaje = 100.0f - (((float)humedad_suelo_adc_crudo / 4095.0f) * 100.0f);

  if (humedad_suelo_porcentaje < 0.0f)
  {
    humedad_suelo_porcentaje = 0.0f;
  }
  else if (humedad_suelo_porcentaje > 100.0f)
  {
    humedad_suelo_porcentaje = 100.0f;
  }
}

static void SensorLluvia_Actualizar(void)
{
  /*
   * En la mayoria de modulos YL-83 con comparador LM393:
   * DO = 0 cuando detecta agua (lluvia activa),
   * DO = 1 cuando no hay lluvia.
   */
  lluvia_do_crudo = (uint8_t)HAL_GPIO_ReadPin(PUERTO_SENSORES, PIN_SENSOR_LLUVIA);
  lluvia_activa = (lluvia_do_crudo == GPIO_PIN_RESET) ? 1u : 0u;
  lluvia_do_crudo_dbg = (uint32_t)lluvia_do_crudo;
  lluvia_activa_dbg = (uint32_t)lluvia_activa;

  lluvia_ao_adc_crudo = ADC_LeerCanal(ADC_CANAL_LLUVIA_AO);
  lluvia_ao_voltaje = (3.3f * (float)lluvia_ao_adc_crudo) / 4095.0f;
  lluvia_ao_nivel_pct = ((float)lluvia_ao_adc_crudo / 4095.0f) * 100.0f;
  lluvia_ao_nivel_agua_pct = 100.0f - lluvia_ao_nivel_pct;

  /* Nivel calibrado usando referencias en campo (seco/mojado). */
  if (lluvia_ao_adc_ref_seco > lluvia_ao_adc_ref_mojado)
  {
    float rango = (float)(lluvia_ao_adc_ref_seco - lluvia_ao_adc_ref_mojado);
    lluvia_ao_nivel_agua_calibrado_pct =
      (((float)lluvia_ao_adc_ref_seco - (float)lluvia_ao_adc_crudo) / rango) * 100.0f;
  }
  else
  {
    lluvia_ao_nivel_agua_calibrado_pct = 0.0f;
  }

  if (lluvia_ao_nivel_agua_calibrado_pct < 0.0f)
  {
    lluvia_ao_nivel_agua_calibrado_pct = 0.0f;
  }
  else if (lluvia_ao_nivel_agua_calibrado_pct > 100.0f)
  {
    lluvia_ao_nivel_agua_calibrado_pct = 100.0f;
  }

  if (lluvia_activa_dbg == 0u)
  {
    lluvia_intensidad_dbg = LLUVIA_INTENSIDAD_NULA;
  }
  else if (lluvia_ao_nivel_agua_calibrado_pct >= umbral_lluvia_fuerte_pct)
  {
    lluvia_intensidad_dbg = LLUVIA_INTENSIDAD_FUERTE;
  }
  else if (lluvia_ao_nivel_agua_calibrado_pct >= umbral_lluvia_media_pct)
  {
    lluvia_intensidad_dbg = LLUVIA_INTENSIDAD_MEDIA;
  }
  else if (lluvia_ao_nivel_agua_calibrado_pct >= umbral_lluvia_leve_pct)
  {
    lluvia_intensidad_dbg = LLUVIA_INTENSIDAD_LEVE;
  }
  else
  {
    lluvia_intensidad_dbg = LLUVIA_INTENSIDAD_NULA;
  }
}

static uint16_t ADC_LeerCanal(uint32_t canal)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  uint16_t valor = 0;

  sConfig.Channel = canal;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    return 0;
  }

  HAL_ADC_Start(&hadc);
  if (HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK)
  {
    valor = (uint16_t)HAL_ADC_GetValue(&hadc);
  }
  HAL_ADC_Stop(&hadc);
  return valor;
}

static void DHT22_DelayUs(uint32_t us)
{
  __HAL_TIM_SET_COUNTER(&htim14, 0u);
  while (__HAL_TIM_GET_COUNTER(&htim14) < us)
  {
  }
}

static uint8_t DHT22_LeerBit(void)
{
  uint32_t timeout = 0;

  /* Espera flanco a nivel alto (fin del pulso bajo de inicio de bit). */
  while (HAL_GPIO_ReadPin(PUERTO_SENSORES, PIN_SENSOR_DHT22) == GPIO_PIN_RESET)
  {
    if (++timeout > 1000u)
    {
      return 0u;
    }
  }

  /* Muestreo del bit (ajustable para corregir checksum). */
  DHT22_DelayUs(dht22_muestreo_us_dbg);
  if (HAL_GPIO_ReadPin(PUERTO_SENSORES, PIN_SENSOR_DHT22) == GPIO_PIN_SET)
  {
    /* Espera a que baje para finalizar la ventana del bit. */
    timeout = 0;
    while (HAL_GPIO_ReadPin(PUERTO_SENSORES, PIN_SENSOR_DHT22) == GPIO_PIN_SET)
    {
      if (++timeout > 1000u)
      {
        break;
      }
    }
    return 1u;
  }
  return 0u;
}

static uint8_t DHT22_LeerDatos(uint8_t datos[5])
{
  uint32_t timeout = 0;
  uint8_t i = 0;
  uint8_t j = 0;

  memset(datos, 0, 5);

  /* 1) Solicitud al sensor: pull-down >= 1ms. */
  GPIOA->MODER &= ~(GPIO_MODER_MODER2_Msk);
  GPIOA->MODER |= (1u << GPIO_MODER_MODER2_Pos); /* PA2 output */
  HAL_GPIO_WritePin(PUERTO_SENSORES, PIN_SENSOR_DHT22, GPIO_PIN_RESET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(PUERTO_SENSORES, PIN_SENSOR_DHT22, GPIO_PIN_SET);
  DHT22_DelayUs(30u);

  /* 2) Cambia pin a entrada para recibir respuesta. */
  GPIOA->MODER &= ~(GPIO_MODER_MODER2_Msk); /* PA2 input */

  /* Respuesta sensor: bajo ~80us y alto ~80us. */
  timeout = 0;
  while (HAL_GPIO_ReadPin(PUERTO_SENSORES, PIN_SENSOR_DHT22) == GPIO_PIN_SET)
  {
    if (++timeout > DHT22_TIMEOUT_US)
    {
      return 0u;
    }
  }
  timeout = 0;
  while (HAL_GPIO_ReadPin(PUERTO_SENSORES, PIN_SENSOR_DHT22) == GPIO_PIN_RESET)
  {
    if (++timeout > DHT22_TIMEOUT_US)
    {
      return 0u;
    }
  }
  timeout = 0;
  while (HAL_GPIO_ReadPin(PUERTO_SENSORES, PIN_SENSOR_DHT22) == GPIO_PIN_SET)
  {
    if (++timeout > DHT22_TIMEOUT_US)
    {
      return 0u;
    }
  }

  /* 3) Lee 40 bits (5 bytes). */
  for (i = 0; i < 5; i++)
  {
    for (j = 0; j < 8; j++)
    {
      datos[i] <<= 1;
      datos[i] |= DHT22_LeerBit();
    }
  }

  return 1u;
}

static void SensorDHT22_Actualizar(void)
{
  uint8_t datos[5];
  uint16_t hum_cruda = 0;
  int16_t temp_cruda = 0;
  uint8_t checksum = 0;

  dht22_lectura_valida_dbg = (uint32_t)DHT22_LeerDatos(datos);
  dht22_checksum_ok_dbg = 0u;

  if (dht22_lectura_valida_dbg == 0u)
  {
    return;
  }

  checksum = (uint8_t)(datos[0] + datos[1] + datos[2] + datos[3]);
  if (checksum != datos[4])
  {
    return;
  }

  dht22_checksum_ok_dbg = 1u;

  hum_cruda = (uint16_t)(((uint16_t)datos[0] << 8) | datos[1]);
  humedad_ambiente_pct = ((float)hum_cruda) / 10.0f;

  temp_cruda = (int16_t)(((uint16_t)(datos[2] & 0x7Fu) << 8) | datos[3]);
  temperatura_ambiente_c = ((float)temp_cruda) / 10.0f;
  if ((datos[2] & 0x80u) != 0u)
  {
    temperatura_ambiente_c = -temperatura_ambiente_c;
  }
}

static void SensorMPU6050_Inicializar(void)
{
  uint8_t whoami = 0u;
  uint8_t salir_sleep = 0u;

  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_DIR_HAL_0, MPU6050_REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &whoami, 1, 100) == HAL_OK)
  {
    mpu6050_direccion_activa_hal = MPU6050_DIR_HAL_0;
    mpu6050_direccion_dbg = MPU6050_DIR_7BIT_0;
  }
  else if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_DIR_HAL_1, MPU6050_REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &whoami, 1, 100) == HAL_OK)
  {
    mpu6050_direccion_activa_hal = MPU6050_DIR_HAL_1;
    mpu6050_direccion_dbg = MPU6050_DIR_7BIT_1;
  }

  if (mpu6050_direccion_activa_hal != 0u)
  {
    mpu6050_whoami_dbg = whoami;
    if ((whoami == MPU6050_DIR_7BIT_0) || (whoami == MPU6050_DIR_7BIT_1))
    {
      if (HAL_I2C_Mem_Write(&hi2c1, mpu6050_direccion_activa_hal, MPU6050_REG_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &salir_sleep, 1, 100) == HAL_OK)
      {
        mpu6050_conectado_dbg = 1u;
      }
    }
  }
}

static void SensorMPU6050_Actualizar(void)
{
  uint8_t buffer[14];

  if (mpu6050_conectado_dbg == 0u)
  {
    return;
  }

  if (HAL_I2C_Mem_Read(&hi2c1, mpu6050_direccion_activa_hal, MPU6050_REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, buffer, 14, 100) != HAL_OK)
  {
    return;
  }

  mpu6050_acel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
  mpu6050_acel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
  mpu6050_acel_z = (int16_t)((buffer[4] << 8) | buffer[5]);

  mpu6050_gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
  mpu6050_gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
  mpu6050_gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);
}

static void EstadoRiesgo_Actualizar(void)
{
  int16_t abs_gx = (mpu6050_gyro_x < 0) ? (int16_t)(-mpu6050_gyro_x) : mpu6050_gyro_x;
  int16_t abs_gy = (mpu6050_gyro_y < 0) ? (int16_t)(-mpu6050_gyro_y) : mpu6050_gyro_y;
  int16_t abs_gz = (mpu6050_gyro_z < 0) ? (int16_t)(-mpu6050_gyro_z) : mpu6050_gyro_z;

  vibracion_anomala_dbg = ((abs_gx > umbral_vibracion_gyro_abs) ||
                           (abs_gy > umbral_vibracion_gyro_abs) ||
                           (abs_gz > umbral_vibracion_gyro_abs)) ? 1u : 0u;

  if ((humedad_suelo_porcentaje >= umbral_humedad_saturada_pct) &&
      (lluvia_activa_dbg == 1u) &&
      (vibracion_anomala_dbg == 1u))
  {
    estado_riesgo_dbg = ESTADO_RIESGO_ROJO;
  }
  else if ((humedad_suelo_porcentaje >= umbral_humedad_alta_pct) ||
           (lluvia_activa_dbg == 1u) ||
           (vibracion_anomala_dbg == 1u))
  {
    estado_riesgo_dbg = ESTADO_RIESGO_AMARILLO;
  }
  else
  {
    estado_riesgo_dbg = ESTADO_RIESGO_VERDE;
  }
}

static void Bluetooth_EnviarTrama(void)
{
  char mensaje[256];
  int longitud = 0;
  int hs_x10 = (int)(humedad_suelo_porcentaje * 10.0f);
  int lla_x10 = (int)(lluvia_ao_nivel_agua_calibrado_pct * 10.0f);
  int ta_x10 = (int)(temperatura_ambiente_c * 10.0f);
  int ha_x10 = (int)(humedad_ambiente_pct * 10.0f);

  longitud = snprintf(
    mensaje,
    sizeof(mensaje),
    "HS_X10:%d,HS_ADC:%u,LLD:%lu,LLA_X10:%d,LLA_ADC:%u,TA_X10:%d,HA_X10:%d,AX:%d,AY:%d,AZ:%d,GX:%d,GY:%d,GZ:%d\r\n",
    hs_x10,
    (unsigned int)humedad_suelo_adc_crudo,
    (unsigned long)lluvia_activa_dbg,
    lla_x10,
    (unsigned int)lluvia_ao_adc_crudo,
    ta_x10,
    ha_x10,
    (int)mpu6050_acel_x,
    (int)mpu6050_acel_y,
    (int)mpu6050_acel_z,
    (int)mpu6050_gyro_x,
    (int)mpu6050_gyro_y,
    (int)mpu6050_gyro_z
  );

  if (longitud > 0)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)mensaje, (uint16_t)longitud, 100);
  }
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
