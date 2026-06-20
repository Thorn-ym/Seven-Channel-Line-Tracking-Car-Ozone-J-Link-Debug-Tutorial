/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : car_control.c
  * @brief          : Two motor speed PID control for TB6612 encoder car.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "car_control.h"
#include "line_tracker.h"

volatile CarControl_t g_car =
{
  .mode = CAR_MODE_OPEN_LOOP,      //小车控制模式。0=禁用停车，1=开环PWM，2=速度PID闭环
  .control_period_ms = CAR_CONTROL_PERIOD_MS,     //控制周期，单位毫秒。这个值需要根据实际情况调整，过大可能导致响应慢，过小可能导致CPU占用过高
  .control_tick = 0U,              //控制周期计数。TIM4 每 10ms 加 1，用来判断程序和中断是否在跑
  .driver_enabled = 0U,           //驱动使能状态。0=禁用，1=使能。禁用时电机不转，且不响应PID控制
  .left =
  {
    .pid =
    {
      .kp = 180,
      .ki = 0.7,
      .kd = 4,
      .integral = 3199,                               //PID积分初始值。这个值需要根据实际情况调整，过大可能导致震荡，过小可能导致响应慢
      .previous_error = 0.0f,                         //PID初始参数。这个值需要根据实际情况调整，过大可能导致震荡，过小可能导致响应慢
      .output_limit = CAR_PID_OUTPUT_MAX,             //PID输出限幅。这个值需要根据实际情况调整，过大可能导致震荡，过小可能导致响应慢
      .integral_limit = CAR_PID_INTEGRAL_MAX,         //PID输出限幅和积分限幅。这个值需要根据实际情况调整，过大可能导致震荡，过小可能导致响应慢
    },
    .target_counts = 45,            //速度目标，单位是每控制周期的编码器计数增量。比如每10ms 1000计数增量就是1000*100=100000计数每秒
    .measured_counts = 0,           //速度测量，单位同上。每个控制周期更新一次
    .encoder_delta = 0,             //编码器增量，单位是编码器计数。每个控制周期更新一次，正负表示转向
    .encoder_total = 0,           //编码器总计数，单位是编码器计数。上电或复位后从0开始，正负表示转向。每个控制周期更新一次
    .encoder_raw = 0,             //编码器原始计数值，单位是编码器计数。上电或复位后从0开始，正负表示转向。每个控制周期更新一次
    .encoder_last = 0,             //编码器上一个周期的计数值，单位是编码器计数。每个控制周期更新一次
    .manual_pwm = 2600,            //开环控制时的PWM值，范围-1000~1000，正负表示转向。这个值会被Car_SetSpeedTargets覆盖。
    .pwm_output = 0,               //最终输出的PWM值，范围-1000~1000，正负表示转向。每个控制周期更新一次。
    .invert_motor = 1,          //电机反转标志。0=不反转，1=反转。反转后正PWM变负PWM，负PWM变正PWM
    .invert_encoder = 0U,       //编码器反转标志。0=不反转，1=反转。反转后编码器增量和总计数的正负相反
  },
  .right =
  {
    .pid =
    {
      .kp = 400,
      .ki = 0.6,
      .kd = 40,
      .integral = 3199,
      .previous_error = 0.0f,
      .output_limit = CAR_PID_OUTPUT_MAX,
      .integral_limit = CAR_PID_INTEGRAL_MAX,
    },
    .target_counts = 46,
    .measured_counts = 0,
    .encoder_delta = 0,
    .encoder_total = 0,
    .encoder_raw = 0,
    .encoder_last = 0,
    .manual_pwm = 2600,
    .pwm_output = 0,
    .invert_motor = 1,
    .invert_encoder = 1,
  },
  .line =
  {
    .pid =
    {
      .kp = 1.3f,
      .ki = 0.08f,
      .kd = 0.2f,
      .integral = 0.0f,
      .previous_error = 0.0f,
      .output_limit = 30.0f,
      .integral_limit = 1000.0f,
    },
    .base_counts = 46,
    .correction_counts = 0,
    .left_target_counts = 0,
    .right_target_counts = 0,
    .line_lost_stop = 1U,
  },
};

static int16_t Car_LimitPwm(int32_t pwm);
static float Car_AbsFloat(float value);
static float Car_LimitFloat(float value, float limit);
static int16_t Car_PidStep(volatile CarMotor_t *motor);
static int32_t Car_LinePidStep(int32_t error);
static void Car_ResetPid(volatile CarMotor_t *motor);
static void Car_ResetLinePid(void);
static void Car_UpdateEncoder(volatile CarMotor_t *motor, TIM_HandleTypeDef *htim);
static void Car_SetDriverEnable(uint8_t enable);
static void Car_ApplyMotor(volatile CarMotor_t *motor,
                           TIM_HandleTypeDef *htim,
                           uint32_t channel,
                           GPIO_TypeDef *in1_port,
                           uint16_t in1_pin,
                           GPIO_TypeDef *in2_port,
                           uint16_t in2_pin);

void Car_Init(void)
{
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);

  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  __HAL_TIM_SET_COUNTER(&htim2, 0U);

  g_car.left.encoder_last = 0;
  g_car.right.encoder_last = 0;
  g_car.left.encoder_raw = 0;
  g_car.right.encoder_raw = 0;

  Car_Stop();
  HAL_TIM_Base_Start_IT(&htim4);
}

void Car_ControlStep(void)
{
  int16_t left_pwm = 0;
  int16_t right_pwm = 0;

  g_car.control_tick++;

  Car_UpdateEncoder(&g_car.left, &htim1);
  Car_UpdateEncoder(&g_car.right, &htim2);
  LineTracker_Update();

  switch (g_car.mode)
  {
    case CAR_MODE_OPEN_LOOP:
      left_pwm = Car_LimitPwm(g_car.left.manual_pwm);
      right_pwm = Car_LimitPwm(g_car.right.manual_pwm);
      break;

    case CAR_MODE_SPEED_PID:
      left_pwm = Car_PidStep(&g_car.left);
      right_pwm = Car_PidStep(&g_car.right);
      break;

    case CAR_MODE_LINE_FOLLOW:
      if ((g_line.line_seen != 0U) || (g_car.line.line_lost_stop == 0U))
      {
        int32_t correction = Car_LinePidStep((int32_t)g_line.error);
        int32_t left_target = g_car.line.base_counts - correction;
        int32_t right_target = g_car.line.base_counts + correction;

        g_car.line.correction_counts = correction;
        g_car.line.left_target_counts = left_target;
        g_car.line.right_target_counts = right_target;
        g_car.left.target_counts = left_target;
        g_car.right.target_counts = right_target;

        left_pwm = Car_PidStep(&g_car.left);
        right_pwm = Car_PidStep(&g_car.right);
      }
      else
      {
        g_car.line.correction_counts = 0;
        g_car.line.left_target_counts = 0;
        g_car.line.right_target_counts = 0;
        g_car.left.target_counts = 0;
        g_car.right.target_counts = 0;
        Car_ResetLinePid();
        Car_ResetPid(&g_car.left);
        Car_ResetPid(&g_car.right);
        left_pwm = 0;
        right_pwm = 0;
      }
      break;

    case CAR_MODE_DISABLED:
    default:
      Car_ResetLinePid();
      Car_ResetPid(&g_car.left);
      Car_ResetPid(&g_car.right);
      left_pwm = 0;
      right_pwm = 0;
      break;
  }

  g_car.left.pwm_output = left_pwm;
  g_car.right.pwm_output = right_pwm;

  if (g_car.mode == CAR_MODE_DISABLED)
  {
    Car_SetDriverEnable(0U);
  }
  else
  {
    Car_SetDriverEnable(1U);
  }

  Car_ApplyMotor(&g_car.left,
                 &htim3,
                 TIM_CHANNEL_1,
                 AIN1_GPIO_Port,
                 AIN1_Pin,
                 AIN2_GPIO_Port,
                 AIN2_Pin);
  Car_ApplyMotor(&g_car.right,
                 &htim3,
                 TIM_CHANNEL_2,
                 BIN1_GPIO_Port,
                 BIN1_Pin,
                 BIN2_GPIO_Port,
                 BIN2_Pin);
}

void Car_Stop(void)
{
  g_car.mode = CAR_MODE_DISABLED;
  g_car.left.target_counts = 0;
  g_car.right.target_counts = 0;
  g_car.left.manual_pwm = 0;
  g_car.right.manual_pwm = 0;
  g_car.left.pwm_output = 0;
  g_car.right.pwm_output = 0;
  Car_ResetPid(&g_car.left);
  Car_ResetPid(&g_car.right);
  Car_SetDriverEnable(0U);
  Car_ApplyMotor(&g_car.left, &htim3, TIM_CHANNEL_1, AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin);
  Car_ApplyMotor(&g_car.right, &htim3, TIM_CHANNEL_2, BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin);
}

void Car_SetSpeedTargets(int32_t left_counts, int32_t right_counts)
{
  g_car.left.target_counts = left_counts;
  g_car.right.target_counts = right_counts;
  g_car.mode = CAR_MODE_SPEED_PID;
}

static int16_t Car_LimitPwm(int32_t pwm)
{
  if (pwm > CAR_PWM_MAX)
  {
    return CAR_PWM_MAX;
  }

  if (pwm < -CAR_PWM_MAX)
  {
    return -CAR_PWM_MAX;
  }

  return (int16_t)pwm;
}

static float Car_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float Car_LimitFloat(float value, float limit)
{
  float positive_limit = Car_AbsFloat(limit);

  if (positive_limit <= 0.0f)
  {
    return 0.0f;
  }

  if (value > positive_limit)
  {
    return positive_limit;
  }

  if (value < -positive_limit)
  {
    return -positive_limit;
  }

  return value;
}

static int16_t Car_PidStep(volatile CarMotor_t *motor)
{
  float error = (float)motor->target_counts - (float)motor->measured_counts;
  float derivative = error - motor->pid.previous_error;
  float output = 0.0f;

  motor->pid.integral += error;
  motor->pid.integral = Car_LimitFloat(motor->pid.integral, motor->pid.integral_limit);

  output = (motor->pid.kp * error) +
           (motor->pid.ki * motor->pid.integral) +
           (motor->pid.kd * derivative);

  motor->pid.previous_error = error;
  output = Car_LimitFloat(output, motor->pid.output_limit);

  return Car_LimitPwm((int32_t)output);
}

static int32_t Car_LinePidStep(int32_t error_counts)
{
  volatile CarPid_t *pid = &g_car.line.pid;
  float error = (float)error_counts;
  float derivative = error - pid->previous_error;
  float output = 0.0f;

  pid->integral += error;
  pid->integral = Car_LimitFloat(pid->integral, pid->integral_limit);

  output = (pid->kp * error) +
           (pid->ki * pid->integral) +
           (pid->kd * derivative);

  pid->previous_error = error;
  output = Car_LimitFloat(output, pid->output_limit);

  return (int32_t)output;
}

static void Car_ResetPid(volatile CarMotor_t *motor)
{
  motor->pid.integral = 0.0f;
  motor->pid.previous_error = 0.0f;
}

static void Car_ResetLinePid(void)
{
  g_car.line.pid.integral = 0.0f;
  g_car.line.pid.previous_error = 0.0f;
}

static void Car_UpdateEncoder(volatile CarMotor_t *motor, TIM_HandleTypeDef *htim)
{
  int16_t now = (int16_t)__HAL_TIM_GET_COUNTER(htim);
  int16_t delta16 = (int16_t)(now - motor->encoder_last);
  int32_t delta = (int32_t)delta16;

  motor->encoder_last = now;
  motor->encoder_raw = now;

  if (motor->invert_encoder != 0U)
  {
    delta = -delta;
  }

  motor->encoder_delta = delta;
  motor->measured_counts = delta;
  motor->encoder_total += delta;
}

static void Car_SetDriverEnable(uint8_t enable)
{
  g_car.driver_enabled = (enable != 0U) ? 1U : 0U;
  HAL_GPIO_WritePin(STBY_GPIO_Port,
                    STBY_Pin,
                    (enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Car_ApplyMotor(volatile CarMotor_t *motor,
                           TIM_HandleTypeDef *htim,
                           uint32_t channel,
                           GPIO_TypeDef *in1_port,
                           uint16_t in1_pin,
                           GPIO_TypeDef *in2_port,
                           uint16_t in2_pin)
{
  int16_t pwm = motor->pwm_output;
  uint32_t duty = 0U;
  GPIO_PinState in1 = GPIO_PIN_RESET;
  GPIO_PinState in2 = GPIO_PIN_RESET;

  if (motor->invert_motor != 0U)
  {
    pwm = (int16_t)-pwm;
  }

  if (pwm > 0)
  {
    in1 = GPIO_PIN_SET;
    duty = (uint32_t)pwm;
  }
  else if (pwm < 0)
  {
    in2 = GPIO_PIN_SET;
    duty = (uint32_t)(-pwm);
  }

  if (g_car.mode == CAR_MODE_DISABLED)
  {
    in1 = GPIO_PIN_RESET;
    in2 = GPIO_PIN_RESET;
    duty = 0U;
  }

  HAL_GPIO_WritePin(in1_port, in1_pin, in1);
  HAL_GPIO_WritePin(in2_port, in2_pin, in2);
  __HAL_TIM_SET_COMPARE(htim, channel, duty);
}
