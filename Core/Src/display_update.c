/**
 ******************************************************************************
 * @file    display_update.c
 * @brief   Display UI update logic for rendering live voltage, raw ADC values,
 * battery profile states, charge bars, and CAN communication status.
 ******************************************************************************
 */

#include "display_update.h"
#include "I2C_screen.h"
#include "CAN_exchange.h"
#include "CAN_debug_screen.h" 
#include <stdio.h>

// Define a flag to enable or disable the CAN debug screen rendering
#define ACTIVATE_CAN_DEBUG_SCREEN 0

/**
  * @brief  Renders all display tracking metrics onto the localized SSD1306 frame
  * buffer, then flushes the RAM layout to physical hardware via I2C.
  * @param  hi2c             Pointer to the initialized I2C peripheral hardware engine
  * @param  wholeVolt        Pre-extracted whole number portion of the battery voltage
  * @param  fracVolt         Pre-extracted fractional milli-volt portion of the battery voltage
  * @param  rawAdc           Current low-pass filtered integer value from the ADC converter
  * @param  info             Pointer to the structure holding battery state and charge percentage
  * @param  screen_connected Boolean flag indicating if a physical display is responding
  * @retval None
  */
void Display_Update_Render(I2C_HandleTypeDef *hi2c,
                           int32_t            wholeVolt,
                           int32_t            fracVolt,
                           uint32_t           rawAdc,
                           const Battery_Info_t *info,
                           uint8_t            screen_connected)
{
  char buf[32];

  if (!screen_connected) return;

  CAN_Diag_t canDiag = CAN_Exchange_Get_Status();



  // Check if CAN debug screen should be displayed
  if (CAN_Debug_Display_Screen(hi2c, &canDiag))
  {
    // Debug screen was displayed, skip normal display rendering
    return;
  }

  // Clear the internal graphical display memory array buffer before drawing a fresh frame //
  SSD1306_ClearBuffer();

  /* ========================================================================== */
  /* HEADER UI SECTION (Title, Separation Line, and Raw ADC Values)             */
  /* ========================================================================== */
  SSD1306_DrawString(2, 0, "Voltmeter", 1, SSD1306_WHITE);

  for (int16_t px = 0; px < 128; px++)
  {
    SSD1306_DrawPixel(px, 10, SSD1306_WHITE);
  }

  snprintf(buf, sizeof(buf), "ADC:%lu", rawAdc);
  SSD1306_DrawString(72, 0, buf, 1, SSD1306_WHITE);

  /* ========================================================================== */
  /* PRIMARY MEASUREMENT SECTION (Live Pack Voltage Rendering)                  */
  /* ========================================================================== */
  snprintf(buf, sizeof(buf), "V:%ld.%03ldV", wholeVolt, fracVolt);
  SSD1306_DrawString(0, 16, buf, 2, SSD1306_WHITE);

  /* ========================================================================== */
  /* BATTERY METRICS STATE MACHINE                                               */
  /* ========================================================================== */
  int32_t intPercentage = (int32_t)(info->percentage + 0.5f);
  if (intPercentage > 100) { intPercentage = 100; }
  if (intPercentage < 0)   { intPercentage = 0;   }

  if (info->state == BATTERY_STATE_DEAD)
  {
    SSD1306_DrawString(0, 36, "B:DEAD", 2, SSD1306_WHITE);
  }
  else if (info->state == BATTERY_STATE_ERROR)
  {
    SSD1306_DrawString(0, 36, "B:ERR", 2, SSD1306_WHITE);
  }
  else
  {
    snprintf(buf, sizeof(buf), "B:%ld%%", intPercentage);
    SSD1306_DrawString(0, 36, buf, 2, SSD1306_WHITE);
  }

  /* ========================================================================== */
  /* GRAPHICAL FUEL GAUGE BAR SECTION                                           */
  /* ========================================================================== */
  SSD1306_DrawRect(2, 56, 118, 7, SSD1306_WHITE);

  int16_t fillW = (int16_t)((116.0f * (float)intPercentage) / 100.0f);

  if (fillW > 0 && info->state == BATTERY_STATE_OK)
  {
    SSD1306_FillRect(3, 57, fillW, 5, SSD1306_WHITE);
  }

  /* ========================================================================== */
  /* CAN STATUS SECTION                                                         */
  /* ========================================================================== */
  CAN_Status_t canState = canDiag.status;

  SSD1306_DrawString(76, 36, "CAN STATE", 1, SSD1306_WHITE);

  /* Primary status label — maps directly to the CAN state machine output.
   * ACK is now driven by TX Event FIFO, so CAN_STATUS_OK is only set when
   * a frame has been physically acknowledged by a node on the bus.           */
  switch (canState)
  {
    case CAN_STATUS_OK:
      SSD1306_DrawString(78, 46, "  OK    ", 1, SSD1306_WHITE);
      break;

    case CAN_STATUS_PENDING:
      SSD1306_DrawString(78, 46, " PENDING", 1, SSD1306_WHITE);
      break;

    case CAN_STATUS_DISABLED:
      SSD1306_DrawString(78, 46, "  OFF   ", 1, SSD1306_WHITE);
      break;

    case CAN_STATUS_ACK_ERR:
      // No TX Event FIFO entry received — frame not acknowledged.
      // Causes: no listener, GND missing, bitrate mismatch.        
      SSD1306_DrawString(78, 46, "Err:ACK ", 1, SSD1306_WHITE);
      break;

    case CAN_STATUS_BUS_OFF:
      // TEC exceeded 255 — node is off the bus.
      // Recovery is automatic after 128×11 recessive bits.          
      SSD1306_DrawString(78, 46, "BUS:OFF ", 1, SSD1306_WHITE);
      break;

    case CAN_STATUS_STUFF_ERR:
      // >5 identical consecutive bits — bitrate or termination issue. 
      SSD1306_DrawString(78, 46, "Err:STUF", 1, SSD1306_WHITE);
      break;

    case CAN_STATUS_FORM_ERR:
      // Fixed-format field violated — bitrate or signal integrity.    
      SSD1306_DrawString(78, 46, "Err:FORM", 1, SSD1306_WHITE);
      break;

    case CAN_STATUS_BIT1_ERR:
      // Sent recessive, read dominant — short circuit or bus contention. 
      SSD1306_DrawString(78, 46, "Err:BIT1", 1, SSD1306_WHITE);
      break;

    case CAN_STATUS_BIT0_ERR:
      // Sent dominant, read recessive — open circuit or broken TX pin.  
      SSD1306_DrawString(78, 46, "Err:BIT0", 1, SSD1306_WHITE);
      break;

    case CAN_STATUS_CRC_ERR:
      // CRC mismatch — bitrate or sample point mismatch, or EMI.        
      SSD1306_DrawString(78, 46, "Err:CRC ", 1, SSD1306_WHITE);
      break;

    default:
      SSD1306_DrawString(78, 46, "Err:UNKW", 1, SSD1306_WHITE);
      break;
  }

  /* ========================================================================== */
  /* POST-OK ERROR FALLBACK                                                     */
  /* Safety net: overrides OK if underlying error counters are non-zero.        */
  /* Root cause must be fixed in CAN_exchange.c, not here.                     */
  /* ========================================================================== */
  if (canState == CAN_STATUS_OK)
  {
    if (canDiag.tx_error_cnt > 0U)
    {
      // TEC non-zero despite OK status — residual errors from a past fault. 
      SSD1306_DrawString(78, 46, "Wrn:TEC ", 1, SSD1306_WHITE);
    }
    else if (canDiag.rx_error_cnt > 0U)
    {
      // REC non-zero — receiving malformed frames from another node.        
      SSD1306_DrawString(78, 46, "Wrn:REC ", 1, SSD1306_WHITE);
    }
    else if (!canDiag.ack_seen)
    {
      // TX Event FIFO never fired — ack_seen was never set to 1.
      SSD1306_DrawString(78, 46, "Wrn:ACK ", 1, SSD1306_WHITE);
    }
  }

  /* ========================================================================== */
  /* HARDWARE OUTPUT FLASH SEGMENT                                              */
  /* ========================================================================== */
  if (screen_connected)
  {
    SSD1306_Display(hi2c);
  }
}