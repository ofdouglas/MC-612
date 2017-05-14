

#include "common.h"

#define DUTY_SCALE	400	// Max duty cycle + 1
#define ENCODER_COUNTS  3200	// Counts per revolution
#define CONTROL_RATE	1000	// Frequency of control loop execution (Hz)

// Scaling factor for encoder speed conversions
#define RPS_COEFFICIENT (F16((1.0/ENCODER_COUNTS) * CONTROL_RATE))

// Control loop gains
#define PID_KP		(F16(10))
#define PID_KI		(F16(5))
#define PID_KD		(F16(1))



fix16_t goal_speed = F16(0);


// Return (a-b) modulo m
int modular_subtract(int a, int b, int m)
{
  if (a >= b)
    return a - b;
  else
    return m - b + a;
}


fix16_t encoder_get_velocity(void)
{
  static int old_pos;
  int new_pos, diff_pos;

  new_pos = timer_get_counter(TIM3);
  diff_pos = modular_subtract(new_pos, old_pos, ENCODER_COUNTS);
  old_pos = new_pos;

  return fix16_smul(fix16_from_int(diff_pos), RPS_COEFFICIENT);
}


// control the motor
void motor_control_task(void * foo)
{
  (void) foo;

  while(1) {
    vTaskDelay((const TickType_t)(configTICK_RATE_HZ / CONTROL_RATE));

    fix16_t velocity = encoder_get_velocity();
    fix16_t error = fix16_ssub(goal_speed, velocity);
    fix16_t output1 = fix16_smul(error, PID_KP);
    fix16_t output2 = fix16_mul(output1, F16((100.0 * DUTY_SCALE) / fix16_one));

    int duty = fix16_to_int(output2);
    if (duty >= DUTY_SCALE)
      duty = DUTY_SCALE - 1;
    
    timer_set_oc_value(TIM2, TIM_OC2, duty);
    timer_set_oc_value(TIM2, TIM_OC3, 0);

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
