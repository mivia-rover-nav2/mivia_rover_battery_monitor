/**
 ******************************************************************************
 * @file    display_update.h
 * @brief   Display update logic for voltage and battery status rendering
 ******************************************************************************
 */

#ifndef DISPLAY_UPDATE_H
#define DISPLAY_UPDATE_H

#include "main.h"
#include "battery_profile.h"
#include "CAN_exchange.h"

/**
  * @brief  Renders all display parameters onto the SSD1306 buffer and transmits.
  * @param  hi2c              Pointer to the initialized I2C hardware handle
  * @param  wholeVolt         Pre-extracted whole number portion of the battery voltage
  * @param  fracVolt          Pre-extracted fractional milli-volt portion of the battery voltage
  * @param  rawAdc            Latest filtered raw integer ADC sample value
  * @param  info              Pointer to the structured battery capacity profiling context
  * @param  screen_connected  Boolean validation flag indicating if I2C display responded on boot
  * @retval None
  */
void Display_Update_Render(I2C_HandleTypeDef *hi2c, int32_t wholeVolt, int32_t fracVolt, uint32_t rawAdc, const Battery_Info_t *info, uint8_t screen_connected);

#endif /* DISPLAY_UPDATE_H */