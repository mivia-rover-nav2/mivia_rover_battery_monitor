/**
 ******************************************************************************
 * @file    stm32g4xx_hal_msp.c
 * @brief   HAL MSP module — low-level hardware initialization for STM32G4 series.
 ******************************************************************************
 */

#include "main.h"

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

/* ============================================================================
 * ADC MSP
 * ========================================================================== */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  if (hadc->Instance == ADC1)
  {
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    PeriphClkInit.Adc12ClockSelection  = RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin  = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1)
  {
    __HAL_RCC_ADC12_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);
  }
}

/* ============================================================================
 * I2C MSP
 * ========================================================================== */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (hi2c->Instance == I2C1)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9; 
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_I2C1_CLK_ENABLE();
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    __HAL_RCC_I2C1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
  }
}

/* ============================================================================
 * FDCAN MSP
 * ========================================================================== */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
  if (hfdcan->Instance == FDCAN1)
  {
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    /* CORRECTION 5 — TX (PA12): NOPULL (actively driven by transceiver) */
    gpio.Pin       = GPIO_PIN_12;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* CORRECTION 5 — RX (PA11): PULLUP (holds line recessive when idle) */
    gpio.Pin       = GPIO_PIN_11;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &gpio);
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan)
{
  if (hfdcan->Instance == FDCAN1)
  {
    __HAL_RCC_FDCAN_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
  }
}
