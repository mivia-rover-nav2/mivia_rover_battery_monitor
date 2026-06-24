/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Lipo Multi-Profile Voltmeter with FD CAN Transmission
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "I2C_screen.h"
#include "CAN_exchange.h"
#include "battery_profile.h"
#include "display_update.h"
#include "CAN_debug_screen.h" 
#include <string.h>
#include <stdio.h>

#include "stdio.h"

int _write(int file, char *ptr, int len)
{
  (void)file;
  for (int i = 0; i < len; i++)
  {
    ITM_SendChar((uint32_t)*ptr++);
  }
  return len;
}


// Change to correspond to your battery pack configuration.
#define CURRENT_BATTERY_TYPE    BATTERY_TYPE_3S

// STM32 G4 series clock and timing configuration constants
#define ADC_SAMPLE_PERIOD_MS    50U
#define CAN_TX_PERIOD_MS        1000U
#define CAN_POLL_PERIOD_MS      100U

#define ADC_VREF                3.3f
#define ADC_MAX_VALUE           4095U
#define FILTER_WINDOW_SIZE      32U

// Peripheral handles
ADC_HandleTypeDef  hadc1;
I2C_HandleTypeDef  hi2c1;
FDCAN_HandleTypeDef hfdcan1;

// Global state variables
static volatile float          currentVoltage = 0.0f;
static volatile uint32_t       rawAdcValue    = 0U;
static volatile Battery_Info_t batteryInfo    = {0.0f, BATTERY_STATE_OK};

static int32_t displayWholeVolt = 0;
static int32_t displayFracVolt  = 0;

static uint32_t adcHistory[FILTER_WINDOW_SIZE] = {0};
static uint32_t adcHistoryIndex = 0U;

static uint32_t lastAdcTime     = 0U;
static uint32_t lastCanTxTime   = 0U;
static uint32_t lastCanPollTime = 0U;
static uint32_t lastDisplayTime = 0U;

static uint8_t is_screen_connected = 1U;

// Function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static uint32_t Filter_GetAverage(uint32_t newValue);
static void Calculate_Display_Voltage_Parts(float voltage);


int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();

  if (CAN_Exchange_Init() != HAL_OK)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    Error_Handler();
  }

  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  CAN_Exchange_Trigger_Startup_Probe();
  CAN_Exchange_Poll();

  // Calculate the voltage divider ratio for the current battery type. Go to battery_profile.c to adjust the ratio for different voltage divider configurations.
  float currentDividerRatio = Battery_Profile_Get_Divider_Ratio(CURRENT_BATTERY_TYPE);

  // Calibrate the ADC before starting regular sampling
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    Error_Handler();
  }

  // Initialize the I2C display and check if it is connected
  if (SSD1306_Init(&hi2c1) != HAL_OK)
  {
    is_screen_connected = 0U;
  }

  // Initial ADC sample to populate the filter history and get an initial voltage reading
  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 100U) == HAL_OK)
  {
    uint32_t initialSample = HAL_ADC_GetValue(&hadc1);

    for (uint32_t i = 0U; i < FILTER_WINDOW_SIZE; i++)
    {
      adcHistory[i] = initialSample;
    }
    rawAdcValue = initialSample;

    float adcVoltage = ((float)rawAdcValue / (float)ADC_MAX_VALUE) * ADC_VREF;
    currentVoltage   = adcVoltage / currentDividerRatio;

    batteryInfo = Battery_Profile_Get_Info(currentVoltage, CURRENT_BATTERY_TYPE);
    Calculate_Display_Voltage_Parts(currentVoltage);
  }
  HAL_ADC_Stop(&hadc1);

  // Initialize timing variables
  uint32_t bootTime   = HAL_GetTick();
  lastAdcTime         = bootTime;
  lastCanTxTime       = bootTime;
  lastCanPollTime     = bootTime;
  lastDisplayTime     = bootTime;

  // Initial display render
  Display_Update_Render(&hi2c1, displayWholeVolt, displayFracVolt,
                        rawAdcValue, (const Battery_Info_t *)&batteryInfo,
                        is_screen_connected);


  // Main loop: ADC sampling, CAN polling, CAN transmission, and display update
  while (1)
  {
    uint32_t currentTime = HAL_GetTick();

    // TASK 1: ADC at 20 Hz - Sample the ADC every 50 ms and update the filtered voltage reading
    if ((currentTime - lastAdcTime) >= ADC_SAMPLE_PERIOD_MS)
    {
      lastAdcTime = currentTime;

      HAL_ADC_Start(&hadc1);
      if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK)
      {
        uint32_t freshSample = HAL_ADC_GetValue(&hadc1);
        rawAdcValue = Filter_GetAverage(freshSample);

        float adcVoltage = ((float)rawAdcValue / (float)ADC_MAX_VALUE) * ADC_VREF;
        currentVoltage   = adcVoltage / currentDividerRatio;

        batteryInfo = Battery_Profile_Get_Info(currentVoltage, CURRENT_BATTERY_TYPE);
        Calculate_Display_Voltage_Parts(currentVoltage);
      }
      HAL_ADC_Stop(&hadc1);
    }

     // TASK 2: CAN Poll 10 Hz - Poll the CAN bus for status updates every 100 ms
    if ((currentTime - lastCanPollTime) >= CAN_POLL_PERIOD_MS)
    {
      lastCanPollTime = currentTime;
      CAN_Exchange_Poll();
    }

    // TASK 3: CAN TX 1 Hz - Send voltage and percentage data over CAN bus every second
    if ((currentTime - lastCanTxTime) >= CAN_TX_PERIOD_MS)
    {
      lastCanTxTime = currentTime;
      CAN_Exchange_Transmit_Data(currentVoltage, batteryInfo.percentage); 
    }

    // TASK 4: Display 4 Hz
    if ((currentTime - lastDisplayTime) >= 250U)
    {
      lastDisplayTime = currentTime;
      Display_Update_Render(&hi2c1, displayWholeVolt, displayFracVolt, rawAdcValue, (const Battery_Info_t *)&batteryInfo, is_screen_connected);
    }
  }
}

/* ============================================================================
 * Helper functions
 * ========================================================================== */

static void Calculate_Display_Voltage_Parts(float voltage)
{
  displayWholeVolt = (int32_t)voltage;
  displayFracVolt  = (int32_t)(((voltage - (float)displayWholeVolt) * 1000.0f) + 0.51f);

  if (displayFracVolt >= 1000) { displayWholeVolt++; displayFracVolt = 0; }
  if (displayFracVolt < 0)     { displayFracVolt = -displayFracVolt; }
}

static uint32_t Filter_GetAverage(uint32_t newValue)
{
  adcHistory[adcHistoryIndex] = newValue;
  adcHistoryIndex = (adcHistoryIndex + 1U) % FILTER_WINDOW_SIZE;

  uint32_t sum = 0U;
  for (uint32_t i = 0U; i < FILTER_WINDOW_SIZE; i++) { sum += adcHistory[i]; }
  return (sum / FILTER_WINDOW_SIZE);
}

/* ============================================================================
 * Peripheral initialization functions
 * ========================================================================== */
static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef   multimode = {0};
  ADC_ChannelConfTypeDef sConfig   = {0};

  hadc1.Instance                   = ADC1;
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait      = DISABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.NbrOfConversion       = 1;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun               = ADC_OVR_DATA_PRESERVED;

  // Calibrate the ADC before starting regular sampling
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  { 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); 
    Error_Handler(); }
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    Error_Handler();
  }

  sConfig.Channel      = ADC_CHANNEL_1;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); Error_Handler(); }
}

/* ============================================================================
 * I2C1 init function - For SSD1306 OLED 128x64 display
 * ========================================================================== */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance             = I2C1;
  hi2c1.Init.Timing          = 0x40B285C2;
  hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress1     = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); Error_Handler(); }
}

/* ============================================================================
 * GPIO init function
 * ========================================================================== */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  gpio.Pin   = GPIO_PIN_5;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin  = GPIO_PIN_13;
  gpio.Mode = GPIO_MODE_IT_FALLING;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &gpio);
}

/* ============================================================================
 * System Clock Configuration
 * ========================================================================== */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef       RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef       RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit     = {0};

  /* ── Voltage scaling : SCALE1_BOOST requis pour 170 MHz ── */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    Error_Handler();
  }

  /* ── Oscillateur : HSI 16 MHz → PLL → 170 MHz ── */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV4;   /* 16 MHz / 4  =  4 MHz  */

  // PLL configuration for 170 MHz SYSCLK
  // Prescaler (PLLM) = 4 → 16 MHz / 4 = 4 MHz
  // Multiplier (PLLN) = 85 → 4 MHz × 85 = 340 MHz
  // Output divider (PLLR) = 2 → 340 MHz / 2 = 170 MHz SYSCLK
  RCC_OscInitStruct.PLL.PLLN            = 85U;              /*  4 MHz × 85 = 340 MHz */
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;   /* 340 MHz / 2 = 170 MHz */
  RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;   /* 340 MHz / 2 = 170 MHz */
  RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;   /* 340 MHz / 2 = 170 MHz → SYSCLK */

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {.Pin = GPIO_PIN_5, .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL};
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    Error_Handler();
  }

  /* ── Bus clocks : SYSCLK=170 MHz, AHB=170 MHz, APB1=170 MHz, APB2=170 MHz ── */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  |
                                     RCC_CLOCKTYPE_SYSCLK |
                                     RCC_CLOCKTYPE_PCLK1  |
                                     RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;    /* HCLK  = 170 MHz */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;      /* PCLK1 = 170 MHz → FDCAN */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;      /* PCLK2 = 170 MHz */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    Error_Handler();
  }

  /* ── FDCAN clock : PCLK1 = 170 MHz ── */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PCLK1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    Error_Handler();
  }

} 

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}