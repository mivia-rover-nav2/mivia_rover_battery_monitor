/**
 ******************************************************************************
 * @file    CAN_debug_screen.h
 * @brief   CAN debug screen display for I2C OLED (SSD1306)
 ******************************************************************************
 */

#ifndef __CAN_DEBUG_SCREEN_H
#define __CAN_DEBUG_SCREEN_H

#include "I2C_screen.h"
#include "CAN_exchange.h"

/**
 * @brief Display CAN debug screen when bus is not OK or PENDING
 * 
 * @param hi2c Pointer to I2C handle for display communication
 * @param canDiag Current CAN diagnostic status
 * @retval 1 if debug screen was displayed, 0 if not needed
 */
uint8_t CAN_Debug_Display_Screen(I2C_HandleTypeDef *hi2c, const CAN_Diag_t *canDiag);

/**
 * @brief Get human-readable error description
 * 
 * @param status CAN status enum
 * @retval Pointer to error description string
 */
const char* CAN_Debug_Get_Error_Description(CAN_Status_t status);

/**
 * @brief Get recommended action for current error
 * 
 * @param status CAN status enum
 * @retval Pointer to action string
 */
const char* CAN_Debug_Get_Recommended_Action(CAN_Status_t status);

#endif /* __CAN_DEBUG_SCREEN_H */
