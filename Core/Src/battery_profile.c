/**
 ******************************************************************************
 * @file    battery_profile.c
 * @brief   Multi-profile implementation for 1S, 2S, 3S, and 4S battery configurations
 ******************************************************************************
 */

#include "battery_profile.h"

// Shared target output capacity percentage scale (common across variants)
static const float batteryPercentTable[BATTERY_TABLE_SIZE] = {
      0.0f,    5.0f,   10.0f,   15.0f,   20.0f,
     25.0f,   30.0f,   35.0f,   40.0f,   45.0f,
     50.0f,   55.0f,   60.0f,   65.0f,   70.0f,
     75.0f,   80.0f,   85.0f,   90.0f,   95.0f,
    100.0f
};

// 1S Battery Configuration Lookups (Nominal 3.7V, Max 4.2V)
static const float voltageTable_1S[BATTERY_TABLE_SIZE] = {
    3.000f,  3.400f,  3.500f,  3.600f,  3.650f,
    3.675f,  3.700f,  3.725f,  3.750f,  3.775f,
    3.800f,  3.825f,  3.850f,  3.875f,  3.900f,
    3.950f,  4.000f,  4.050f,  4.100f,  4.150f,
    4.200f
};

// 2S Battery Configuration Lookups (Nominal 7.4V, Max 8.4V)
static const float voltageTable_2S[BATTERY_TABLE_SIZE] = {
    6.000f,  6.800f,  7.000f,  7.200f,  7.300f,
    7.350f,  7.400f,  7.450f,  7.500f,  7.550f,
    7.600f,  7.650f,  7.700f,  7.750f,  7.800f,
    7.900f,  8.000f,  8.100f,  8.200f,  8.300f,
    8.400f
};

// 3S Battery Configuration Lookups (Nominal 11.1V, Max 12.6V)
static const float voltageTable_3S[BATTERY_TABLE_SIZE] = {
    9.000f,  10.200f, 10.500f, 10.800f, 10.950f,
    11.025f, 11.100f, 11.175f, 11.250f, 11.325f,
    11.400f, 11.475f, 11.550f, 11.625f, 11.700f,
    11.850f, 12.000f, 12.150f, 12.300f, 12.450f,
    12.600f
};

// 4S Battery Configuration Lookups (Nominal 14.8V, Max 16.8V)
static const float voltageTable_4S[BATTERY_TABLE_SIZE] = {
    12.000f, 13.600f, 14.000f, 14.400f, 14.600f,
    14.700f, 14.800f, 14.900f, 15.000f, 15.100f,
    15.200f, 15.300f, 15.400f, 15.500f, 15.800f,
    15.800f, 16.000f, 16.200f, 16.400f, 16.600f,
    16.800f
};

float Battery_Profile_Get_Divider_Ratio(Battery_Type_t type)
{
  switch (type)
  {
    case BATTERY_TYPE_1S: return 0.60000000f; // 150 / (150 + 100)
    case BATTERY_TYPE_2S: return 0.30069930f; // 43  / (43  + 100)
    case BATTERY_TYPE_3S: return 0.21259843f; // 27  / (27  + 100)
    case BATTERY_TYPE_4S: return 0.18032787f; // 22  / (22  + 100)
    default:              return 1.0f;
  }
}

Battery_Info_t Battery_Profile_Get_Info(float voltage, Battery_Type_t type)
{
  Battery_Info_t result = {0.0f, BATTERY_STATE_OK};
  /* Default fallback context pointer */
  const float *pTable = voltageTable_3S; 
  float deadThreshold = 9.0f;
  float errorThreshold = 11.0f;

  // 1. Map context variables dynamically according to profile type choice
  switch (type)
  {
    // deadThreshold : 2.5V per cell, errorThreshold : 3.0V per cell
    case BATTERY_TYPE_1S:
      pTable = voltageTable_1S;
      deadThreshold = 2.5f;
      errorThreshold = 3.0f;
      break;
    case BATTERY_TYPE_2S:
      pTable = voltageTable_2S;
      deadThreshold = 5.0f;
      errorThreshold = 6.0f;
      break;
    case BATTERY_TYPE_3S:
      pTable = voltageTable_3S;
      deadThreshold = 7.5f;
      errorThreshold = 9.0f;
      break;
    case BATTERY_TYPE_4S:
      pTable = voltageTable_4S;
      deadThreshold = 10.0f;
      errorThreshold = 12.0f;
      break;
  }

  // 2. Evaluate explicit layout boundary protection conditions
  if (voltage < deadThreshold)
  {
    result.percentage = 0.0f;
    result.state = BATTERY_STATE_DEAD;
    return result;
  }
  else if (voltage < errorThreshold)
  {
    result.percentage = 0.0f;
    result.state = BATTERY_STATE_ERROR;
    return result;
  }

  // 3. Handle out-of-bounds upper clamping safeguards
  if (voltage >= pTable[BATTERY_TABLE_SIZE - 1U]) 
  {
    result.percentage = batteryPercentTable[BATTERY_TABLE_SIZE - 1U];
    result.state = BATTERY_STATE_OK;
    return result;
  }

  // 4. Perform localized linear piece-wise curve interpolation math
  for (uint32_t i = 0U; i < (BATTERY_TABLE_SIZE - 1U); i++)
  {
    if (voltage >= pTable[i] && voltage <= pTable[i + 1U])
    {
      float deltaV = pTable[i + 1U] - pTable[i];
      if (deltaV <= 0.0001f) 
      {
        result.percentage = batteryPercentTable[i];
        return result;
      }
      
      float slope = (batteryPercentTable[i + 1U] - batteryPercentTable[i]) / deltaV;
      result.percentage = batteryPercentTable[i] + (slope * (voltage - pTable[i]));
      return result;
    }
  }

  return result;
}