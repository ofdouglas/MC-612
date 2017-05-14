

#include "common.h"

#define DUTY_SCALE	400	// Max duty cycle + 1
#define ENCODER_COUNTS  3200	// Counts per revolution
#define CONTROL_RATE	200	// Frequency of control loop execution (Hz)
#define CONTROL_CEILING 1000.0	// maximum controller output value


// Scaling factor for encoder speed conversions
// N ticks/sample * [CONTROL_RATE samples/sec * revs/tick * seconds/min]
#define RPM_COEFFICIENT (F16((1.0/ENCODER_COUNTS) * CONTROL_RATE * 60))

// Velocity controller loop gains
#define V_PID_KP		(F16(20.0))
#define V_PID_KI		(F16(100.0 / CONTROL_RATE))
#define V_PID_KD		(F16(1.0 / CONTROL_RATE))

// Position controller loop gains
#define P_PID_KP		(F16(5.0))
#define P_PID_KI		(F16(0.1 / CONTROL_RATE))
#define P_PID_KD		(F16(5.0 / CONTROL_RATE))


fix16_t goal_speed = F16(0);


// Return (a-b) modulo m
int modular_subtract(int a, int b, int m)
{
  if (a >= b)
    return a - b;
  else
    return m - b + a;
}

#define A  0.2
#define A0 F16(A)
#define A1 F16(1-A)
fix16_t recursive_lpf(fix16_t x)
{
  static fix16_t y;

  y = fix16_add(fix16_smul(A0, x), fix16_smul(A1, y));
  return y;
}


// return encoder velocity, in RPM
fix16_t encoder_get_velocity(void)
{
  static int old_pos;
  int new_pos, diff_pos;

  new_pos = timer_get_counter(TIM3);

  int dir = TIM_CR1(TIM3) & BIT4;
  if (dir) {
    diff_pos = -modular_subtract(old_pos, new_pos, ENCODER_COUNTS);
    gpio_set(GPIOC, LED_BLUE_BIT);
  }
  else {
    diff_pos = modular_subtract(new_pos, old_pos, ENCODER_COUNTS);
    gpio_clear(GPIOC, LED_BLUE_BIT);
  }

  // clockwise
  //diff_pos = modular_subtract(new_pos, old_pos, ENCODER_COUNTS);
  
  old_pos = new_pos;

  //  xprintf("%d\n", diff_pos);
  
  fix16_t diff = fix16_smul(fix16_from_int(diff_pos), RPM_COEFFICIENT);
  return recursive_lpf(diff);
}

int encoder_get_position(void)
{
  return timer_get_counter(TIM3);
}


void set_pwm(fix16_t controller_output)
{
  // Scale the controller output so that CONTROL_CEILING corresponds
  // to the maximum duty cycle
  controller_output = fix16_mul(controller_output,
				F16((DUTY_SCALE) / CONTROL_CEILING));
  int duty = fix16_to_int(fix16_abs(controller_output));
  if (duty >= DUTY_SCALE)
    duty = DUTY_SCALE - 1;

  if (controller_output > 0) {
    // clockwise
    timer_set_oc_value(TIM2, TIM_OC2, duty);
    timer_set_oc_value(TIM2, TIM_OC3, 0);
  } else {
    // counter-clockwise
    timer_set_oc_value(TIM2, TIM_OC2, 0);
    timer_set_oc_value(TIM2, TIM_OC3, duty);
  }
}


fix16_t velocity_controller(fix16_t goal_speed)
{
  static fix16_t integral;
  fix16_t velocity, error;

  velocity = encoder_get_velocity();
  error = fix16_ssub(goal_speed, velocity);
  integral = fix16_add(integral, fix16_smul(error, V_PID_KI));
  return fix16_add(fix16_smul(error, V_PID_KP), integral);
}


fix16_t position_controller(int goal_position)
{
  static fix16_t error_prev;
  int position, cw_error, ccw_error;
  fix16_t error, derivative, proportional, result;

  // Choose the shorter of the clockwise and counterclockwise difference
  // between present and desired position 
  position = encoder_get_position();
  cw_error = modular_subtract(position, goal_position, ENCODER_COUNTS);
  ccw_error = modular_subtract(goal_position, position, ENCODER_COUNTS);
  if (cw_error < ccw_error)
    error = fix16_from_int(-cw_error);
  else
    error = fix16_from_int(ccw_error);

  xprintf("%d %d %d\n", cw_error, ccw_error, error >> 16);
  //  integral = fix16_add(integral, fix16_smul(error, P_PID_KI));
  
  derivative = fix16_smul(fix16_ssub(error, error_prev), P_PID_KD);
  proportional = fix16_smul(error, P_PID_KP);
  result = fix16_add(derivative, proportional);
  error_prev = error;
  
  //xprintf("%d %d\n", error >> 16, result >> 16);
  return result;
}

QueueHandle_t cmd_queue;

// control the motor
// positive goal_speed: clockwise motion, TIM_OC2 is on
void motor_control_task(void * foo)
{
  (void) foo;

  cmd_queue = xQueueCreate(5, sizeof(struct motion_cmd));
  
  fix16_t controller_output;

  struct motion_cmd mc = {
    .cmd = CMD_NEUTRAL,
    .arg = 0
  };
  
  while(1) {
    vTaskDelay((const TickType_t)(configTICK_RATE_HZ / CONTROL_RATE));

    if (uxQueueMessagesWaiting(cmd_queue))
      xQueueReceive(cmd_queue, &mc, 0);

    switch (mc.cmd) {
    case CMD_NEUTRAL:	// coast / idle
      controller_output = 0;
      set_pwm(controller_output);
      break;

    case CMD_BRAKE:	// hard stop
      timer_set_oc_value(TIM2, TIM_OC2, DUTY_SCALE - 1);
      timer_set_oc_value(TIM2, TIM_OC3, DUTY_SCALE - 1);
      break;

    case CMD_VELOCITY:	// constant velocity
      controller_output = velocity_controller(mc.arg);
      set_pwm(controller_output);
      break;
      
    case CMD_POSITION:	// constant position
      controller_output = position_controller(fix16_to_int(mc.arg));
      set_pwm(controller_output);
      break;
    }

    
    
    //    xprintf("%d, %d\n", position, duty);
    //    xprintf("%d.%d %c%d\n", velocity >> 16, velocity & 0xffff,
    //	    output2 > 0 ? '+' : '-', duty);
    
    // Emergency stop
    if (gpio_get(GPIOA, GPIO0))
      break;
  }
  
  gpio_set(GPIOC, LED_BLUE_BIT);
  timer_set_oc_value(TIM2, TIM_OC2, 0);
  timer_set_oc_value(TIM2, TIM_OC3, 0);  
  
  // Do nothing while e-stopped
    while (1)
      ;
}
