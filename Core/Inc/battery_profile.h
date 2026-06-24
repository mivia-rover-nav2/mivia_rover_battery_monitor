/**
 ******************************************************************************
 * @file    battery_profile.c
 * @brief   Battery profile mapping functions for voltage-to-percentage translation
 ******************************************************************************
 */

#ifndef BATTERY_PROFILE_H
#define BATTERY_PROFILE_H

#include <stdint.h>

/* ========================================================================== */
/* BATTERY HARDWARE CONFIGURATION                                             */
/* ========================================================================== */

// CHange value to correspond to your voltage divider in amont of the ADC input.
// For example, if you have a 100k/27k voltage divider, the ratio is 27k / (100k + 27k) = 0.21259843f

#define VOLTAGE_DIVIDER_RATIO   0.21259843f
#define BATTERY_TABLE_SIZE      21U

/* ========================================================================== */
/* PUBLIC API MODULES                                                         */
/* ========================================================================== */

/* Supported battery pack configurations */
typedef enum {
    BATTERY_TYPE_1S = 0,
    BATTERY_TYPE_2S,
    BATTERY_TYPE_3S,
    BATTERY_TYPE_4S
} Battery_Type_t;

/* State enumerations for clear error interpretation */
typedef enum {
    BATTERY_STATE_DEAD = 0,
    BATTERY_STATE_ERROR,
    BATTERY_STATE_OK,
} Battery_State_t;

/* Return structure holding calculation outputs */
typedef struct {
    float percentage;
    Battery_State_t state;
} Battery_Info_t;

/**
  * @brief  Returns the hardware voltage divider ratio for a specific battery type.
  * @param  type Selection configuration flag (1S to 4S)
  * @retval Computed hardware scale attenuation ratio factor
  */
float Battery_Profile_Get_Divider_Ratio(Battery_Type_t type);

/**
  * @brief  Translates an unattenuated pack voltage into a structure holding 
  * both percentage and discrete diagnostic status fields.
  * @param  voltage True calculated voltage of the battery pack
  * @param  type    Selection configuration flag (1S to 4S)
  * @retval Battery_Info_t Computed charge metrics layout
  */
Battery_Info_t Battery_Profile_Get_Info(float voltage, Battery_Type_t type);

#endif /* BATTERY_PROFILE_H */