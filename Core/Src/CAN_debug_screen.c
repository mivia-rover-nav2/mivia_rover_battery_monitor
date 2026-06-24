/**
 ******************************************************************************
 * @file    CAN_debug_screen.c
 * @brief   CAN debug screen implementation for SSD1306 I2C OLED (128x64)
 *          Optimized for small display with color-aware architecture
 ******************************************************************************
 */

#include "CAN_debug_screen.h"
#include "I2C_screen.h"
#include <stdio.h>
#include <string.h>

/* ============================================================================
 * Private helpers
 * ========================================================================== */

static const char* CAN_Debug_Status_Name(CAN_Status_t status)
{
  switch (status)
  {
    case CAN_STATUS_OK:         return "OK";
    case CAN_STATUS_PENDING:    return "PENDING";
    case CAN_STATUS_ACK_ERR:    return "ACK_ERROR";
    case CAN_STATUS_BUS_OFF:    return "BUS_OFF";
    case CAN_STATUS_DISABLED:   return "DISABLED";
    case CAN_STATUS_STUFF_ERR:  return "STUFF_ERR";
    case CAN_STATUS_FORM_ERR:   return "FORM_ERR";
    case CAN_STATUS_BIT1_ERR:   return "BIT1_ERR";
    case CAN_STATUS_BIT0_ERR:   return "BIT0_ERR";
    case CAN_STATUS_CRC_ERR:    return "CRC_ERR";
    default:                    return "UNKNOWN";
  }
}

/* ============================================================================
 * Error descriptions (short, fits 128px width)
 * ========================================================================== */

const char* CAN_Debug_Get_Error_Description(CAN_Status_t status)
{
  switch (status)
  {
    case CAN_STATUS_ACK_ERR:
      return "No listeners";
    case CAN_STATUS_BUS_OFF:
      return "Bus isolated";
    case CAN_STATUS_STUFF_ERR:
      return "Bit stuffing";
    case CAN_STATUS_FORM_ERR:
      return "Bad frame format";
    case CAN_STATUS_BIT1_ERR:
      return "Bit recessive fail";
    case CAN_STATUS_BIT0_ERR:
      return "Bit dominant fail";
    case CAN_STATUS_CRC_ERR:
      return "CRC checksum bad";
    case CAN_STATUS_DISABLED:
      return "CAN not enabled";
    default:
      return "Unknown error";
  }
}

/* ============================================================================
 * Recommended actions (short, fits 128px width)
 * ========================================================================== */

const char* CAN_Debug_Get_Recommended_Action(CAN_Status_t status)
{
  switch (status)
  {
    case CAN_STATUS_ACK_ERR:
      return "Check connections";
    case CAN_STATUS_BUS_OFF:
      return "Wait for recovery";
    case CAN_STATUS_STUFF_ERR:
      return "Check baud rate";
    case CAN_STATUS_FORM_ERR:
      return "Check signal level";
    case CAN_STATUS_BIT1_ERR:
      return "Check for shorts";
    case CAN_STATUS_BIT0_ERR:
      return "Check GND / TX pin";
    case CAN_STATUS_CRC_ERR:
      return "Review baud/clock";
    case CAN_STATUS_DISABLED:
      return "Reinit CAN module";
    default:
      return "Contact support";
  }
}

/* ============================================================================
 * LEC (Last Error Code) name decoder
 * ========================================================================== */

static const char* CAN_Debug_Get_LEC_Name(uint8_t lec)
{
  switch (lec)
  {
    case 0: return "No Error";
    case 1: return "Bit Stuffing";
    case 2: return "Frame Error";
    case 3: return "ACK Error";
    case 4: return "Bit1 Error";
    case 5: return "Bit0 Error";
    case 6: return "CRC Error";
    case 7: return "No Change";
    default: return "?";
  }
}

/* ============================================================================
 * ACK status string builder
 * ========================================================================== */

static void CAN_Debug_Build_ACK_String(char *buf, size_t len, 
                                        const CAN_Diag_t *canDiag)
{
  if (canDiag->ack_seen)
  {
    uint32_t time_diff = HAL_GetTick() - canDiag->last_ack_tick;
    snprintf(buf, len, "ACK: YES (%lums ago)", time_diff);
  }
  else
  {
    snprintf(buf, len, "ACK: WAITING...");
  }
}

/* ============================================================================
 * Main debug screen display function
 * Display Layout (128x64 pixels):
 * 
 * y=0   [Header - Status name + architecture indicator]  <- COLORED
 * y=10  [Error description]                              <- COLORED
 * y=20  [Error counters: TEC / REC]
 * y=30  [LEC code + ACK status]
 * y=40  [Spacer]
 * y=48  [Recommended action]
 * y=56  [Status bar - recovery indicator]
 * ========================================================================== */

uint8_t CAN_Debug_Display_Screen(I2C_HandleTypeDef *hi2c, const CAN_Diag_t *canDiag)
{
  /* Only show debug if there's an actual error */
  if (canDiag->status == CAN_STATUS_OK || canDiag->status == CAN_STATUS_PENDING || ACTIVATE_CAN_DEBUG_SCREEN == 0)
  {
    return 0U;  /* No debug screen needed */
  }

  /* Clear display buffer */
  SSD1306_ClearBuffer();

  /* ========================================================================
   * LINE 0 (y=0): HEADER - Status with architecture indicator
   *               Use different color to highlight from normal display
   * ======================================================================== */ 

  //Center name of status on line 0
  const char *status_name = CAN_Debug_Status_Name(canDiag->status);
  size_t status_len = strlen(status_name);
  int16_t x1_offset = (128 - (status_len * FONT_WIDTH)) / 2;  // Centered horizontally
  // Draw line above and below the status name
  for (int16_t px = 0; px < 128; px++)
  {
    SSD1306_DrawPixel(px, 0, SSD1306_WHITE);  // Top line
    SSD1306_DrawPixel(px, 14, SSD1306_WHITE); // Bottom line
  }
  SSD1306_DrawString(x1_offset, 4, CAN_Debug_Status_Name(canDiag->status), 1 , SSD1306_WHITE);


  /* ========================================================================
   * LINE 1 (y=10): ERROR HEADER - Short description of the error
   * ======================================================================== */
     const char *status_description = CAN_Debug_Get_Error_Description(canDiag->status);
  size_t desc_len = strlen(status_description);
  int16_t x2_offset = (128 - (desc_len * FONT_WIDTH)) / 2;  // Centered horizontally
  const char *desc = CAN_Debug_Get_Error_Description(canDiag->status);
  SSD1306_DrawString(x2_offset, 16, (char*)desc, 1, SSD1306_WHITE);


  /* ========================================================================
   * LINE 2 (y=20): ERROR COUNTERS
   * ======================================================================== */
  char err_buf[32];
  snprintf(err_buf, sizeof(err_buf), "TEC:%3u  REC:%3u", 
           canDiag->tx_error_cnt, canDiag->rx_error_cnt);
  SSD1306_DrawString(0, 26, err_buf, 1, SSD1306_WHITE);

  /* ========================================================================
   * LINE 3 (y=30): LEC CODE + ACK STATUS
   * ======================================================================== */
  char status_buf[32];
  const char *lec_str = CAN_Debug_Get_LEC_Name(canDiag->lec);
  const char *ack_indicator = canDiag->ack_seen ? "OK" : "NO";
  
  snprintf(status_buf, sizeof(status_buf), "LEC:%s  ACK:%s", 
           lec_str, ack_indicator);
  SSD1306_DrawString(0, 36, status_buf, 1, SSD1306_WHITE);

  /* ========================================================================
   * LINE 4 (y=48): RECOMMENDED ACTION
   * ======================================================================== */
  const char *action = CAN_Debug_Get_Recommended_Action(canDiag->status);
  SSD1306_DrawString(0, 48, (char*)action, 1, SSD1306_WHITE);

  /* ========================================================================
   * LINE 5 (y=56): STATUS BAR - Recovery indicator
   * ======================================================================== */
  char status_bar[32];
  
  if (canDiag->ack_seen)
  {
    snprintf(status_bar, sizeof(status_bar), "Recovery in progress...");
  }
  else
  {
    snprintf(status_bar, sizeof(status_bar), "Waiting for recovery...");
  }
  
  SSD1306_DrawString(0, 56, status_bar, 1, SSD1306_WHITE);

  /* ========================================================================
   * SEND TO DISPLAY
   * ======================================================================== */
  SSD1306_Display(hi2c);

  /* Log for debugging */
  printf("[CAN DEBUG SCREEN] %s\n  -> %s\n  -> TEC:%u REC:%u LEC:%s\n",
         CAN_Debug_Status_Name(canDiag->status),
         action,
         canDiag->tx_error_cnt,
         canDiag->rx_error_cnt,
         lec_str);

  return 1U;  /* Debug screen was displayed */
}

/* ============================================================================
 * Optional: Get full diagnostic string (for logging/UART)
 * ========================================================================== */

void CAN_Debug_Print_Full_Diagnostics(const CAN_Diag_t *canDiag)
{
  printf("\n");
  printf("========== CAN BUS DIAGNOSTICS ==========\n");
  printf("Status:     %s\n", CAN_Debug_Status_Name(canDiag->status));
  printf("Description: %s\n", CAN_Debug_Get_Error_Description(canDiag->status));
  printf("Action:     %s\n", CAN_Debug_Get_Recommended_Action(canDiag->status));
  printf("TX Errors:  %u\n", canDiag->tx_error_cnt);
  printf("RX Errors:  %u\n", canDiag->rx_error_cnt);
  printf("Last Error Code: %s\n", CAN_Debug_Get_LEC_Name(canDiag->lec));
  printf("ACK Seen:   %s\n", canDiag->ack_seen ? "YES" : "NO");
  if (canDiag->ack_seen)
  {
    printf("ACK Timing: %lums ago\n", HAL_GetTick() - canDiag->last_ack_tick);
  }
  printf("=========================================\n\n");
}
