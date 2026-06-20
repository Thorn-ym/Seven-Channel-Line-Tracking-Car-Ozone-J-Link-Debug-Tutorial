/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : line_tracker.c
  * @brief          : Seven channel digital line tracker input.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "line_tracker.h"

volatile LineTracker_t g_line =
{
  .raw_mask = 0U,             // Raw bitmask of the line tracker channels (1 = line detected, 0 = no line)
  .active_mask = 0U,          // Bitmask of active line tracker channels (1 = line detected, 0 = no line)
  .active_count = 0U,         // Number of active line tracker channels
  .line_seen = 0U,            // Flag indicating if a line is seen
  .active_low = 0U,           // 0 = active high, 1 = active low
  .position = 0,              // Calculated position of the line
  .error = 0,
  .sample_tick = 0U,
  .right_angle_enable = 1U,
  .right_angle_detected = 0U,
  .right_angle_direction = 0,
  .right_angle_error = 50,
};

static GPIO_TypeDef * const line_ports[LINE_TRACKER_CHANNELS] =
{
  TRACK1_GPIO_Port,
  TRACK2_GPIO_Port,
  TRACK3_GPIO_Port,
  TRACK4_GPIO_Port,
  TRACK5_GPIO_Port,
  TRACK6_GPIO_Port,
  TRACK7_GPIO_Port,
};

static const uint16_t line_pins[LINE_TRACKER_CHANNELS] =
{
  TRACK1_Pin,
  TRACK2_Pin,
  TRACK3_Pin,
  TRACK4_Pin,
  TRACK5_Pin,
  TRACK6_Pin,
  TRACK7_Pin,
};

static const int16_t line_weights[LINE_TRACKER_CHANNELS] =
{
  50,
  33,
  16,
  0,
  -16,
  -33,
  -50,
};

void LineTracker_Init(void)
{
  LineTracker_Update();
}

void LineTracker_Update(void)
{
  uint8_t raw_mask = 0U;
  uint8_t active_mask = 0U;
  uint8_t active_count = 0U;
  int16_t error = 0;
  int32_t weighted_sum = 0;
  uint32_t index = 0U;

  for (index = 0U; index < LINE_TRACKER_CHANNELS; index++)
  {
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(line_ports[index], line_pins[index]);
    uint8_t is_set = (pin_state == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t is_active = 0U;

    if (is_set != 0U)
    {
      raw_mask |= (uint8_t)(1U << index);
    }

    is_active = (g_line.active_low != 0U) ? (uint8_t)(is_set == 0U) : is_set;
    if (is_active != 0U)
    {
      active_mask |= (uint8_t)(1U << index);
      weighted_sum += line_weights[index];
      active_count++;
    }
  }

  g_line.raw_mask = raw_mask;
  g_line.active_mask = active_mask;
  g_line.active_count = active_count;
  g_line.line_seen = (active_count > 0U) ? 1U : 0U;

  if (active_count > 0U)
  {
    error = (int16_t)(weighted_sum / (int32_t)active_count);

    g_line.right_angle_detected = 0U;
    g_line.right_angle_direction = 0;

    if (g_line.right_angle_enable != 0U)
    {
      uint8_t left_right_angle = ((active_mask & 0x0FU) == 0x0FU) ? 1U : 0U;
      uint8_t right_right_angle = ((active_mask & 0x78U) == 0x78U) ? 1U : 0U;

      if ((left_right_angle != 0U) && (right_right_angle == 0U))
      {
        error = g_line.right_angle_error;
        g_line.right_angle_detected = 1U;
        g_line.right_angle_direction = 1;
      }
      else if ((right_right_angle != 0U) && (left_right_angle == 0U))
      {
        error = (int16_t)-g_line.right_angle_error;
        g_line.right_angle_detected = 1U;
        g_line.right_angle_direction = -1;
      }
    }

    g_line.position = error;
    g_line.error = error;
  }
  else
  {
    g_line.right_angle_detected = 0U;
    g_line.right_angle_direction = 0;
  }

  g_line.sample_tick++;
}
