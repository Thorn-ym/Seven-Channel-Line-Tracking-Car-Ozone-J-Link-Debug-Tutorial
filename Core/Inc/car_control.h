/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : car_control.h
  * @brief          : Two motor speed PID control for TB6612 encoder car.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define CAR_CONTROL_PERIOD_MS     10U
#define CAR_PWM_MAX               3199
#define CAR_PID_OUTPUT_MAX        3199.0f
#define CAR_PID_INTEGRAL_MAX      3199.0f

typedef enum
{
  CAR_MODE_DISABLED = 0,
  CAR_MODE_OPEN_LOOP = 1,
  CAR_MODE_SPEED_PID = 2,
  CAR_MODE_LINE_FOLLOW = 3
} CarMode_t;

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral;
  float previous_error;
  float output_limit;
  float integral_limit;
} CarPid_t;

typedef struct
{
  CarPid_t pid;
  int32_t target_counts;
  int32_t measured_counts;
  int32_t encoder_delta;
  int32_t encoder_total;
  int16_t encoder_raw;
  int16_t encoder_last;
  int16_t manual_pwm;
  int16_t pwm_output;
  uint8_t invert_motor;
  uint8_t invert_encoder;
} CarMotor_t;

typedef struct
{
  CarPid_t pid;
  int32_t base_counts;
  int32_t correction_counts;
  int32_t left_target_counts;
  int32_t right_target_counts;
  uint8_t line_lost_stop;
} CarLineFollow_t;

typedef struct
{
  CarMode_t mode;
  uint32_t control_period_ms;
  uint32_t control_tick;
  uint8_t driver_enabled;
  CarMotor_t left;
  CarMotor_t right;
  CarLineFollow_t line;
} CarControl_t;

extern volatile CarControl_t g_car;

void Car_Init(void);
void Car_ControlStep(void);
void Car_Stop(void);
void Car_SetSpeedTargets(int32_t left_counts, int32_t right_counts);

#ifdef __cplusplus
}
#endif

#endif /* CAR_CONTROL_H */
