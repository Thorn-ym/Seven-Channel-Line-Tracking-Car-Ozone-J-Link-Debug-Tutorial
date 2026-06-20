/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : line_tracker.h
  * @brief          : Seven channel digital line tracker input.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define LINE_TRACKER_CHANNELS 7U

typedef struct
{
  uint8_t raw_mask;
  uint8_t active_mask;
  uint8_t active_count;
  uint8_t line_seen;
  uint8_t active_low;
  int16_t position;
  int16_t error;
  uint32_t sample_tick;
  uint8_t right_angle_enable;
  uint8_t right_angle_detected;
  int8_t right_angle_direction;
  int16_t right_angle_error;
} LineTracker_t;

extern volatile LineTracker_t g_line;

void LineTracker_Init(void);
void LineTracker_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* LINE_TRACKER_H */
