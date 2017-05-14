#include "common.h"

// Set duty cycle of H-bridge
void pwm_set_output(fix16_t setpoint)
{
  // Scale the controller output so that CONTROL_CEILING corresponds
  // to the maximum duty cycle
  setpoint = fix16_mul(setpoint,
				F16((DUTY_SCALE) / CONTROL_CEILING));
  int duty = fix16_to_int(fix16_abs(setpoint));
  if (duty >= DUTY_SCALE)
    duty = DUTY_SCALE - 1;

  if (setpoint > 0) {
    // clockwise
    timer_set_oc_value(TIM2, TIM_OC2, duty);
    timer_set_oc_value(TIM2, TIM_OC3, 0);
  } else {
    // counter-clockwise
    timer_set_oc_value(TIM2, TIM_OC2, 0);
    timer_set_oc_value(TIM2, TIM_OC3, duty);
  }
}

void pwm_setup(void)
{
  // Configure timer to count up in edge-aligned mode; no input clock divider
  rcc_periph_clock_enable(RCC_TIM2);
  timer_reset(TIM2);
  timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

  // 20 kHz / 50 us PWM cycles
  timer_set_prescaler(TIM2, 0);
  timer_set_period(TIM2, 399);

  // Enable PWM outputs. 'Break' output must be enabled, although though the
  // datasheet doesn't bother to mention this fact...
    
  timer_set_oc_mode(TIM2, TIM_OC2, TIM_OCM_PWM1);
  timer_set_oc_value(TIM2, TIM_OC2, 0);
  timer_enable_oc_output(TIM2, TIM_OC2);
  
  timer_set_oc_mode(TIM2, TIM_OC3, TIM_OCM_PWM1);
  timer_set_oc_value(TIM2, TIM_OC3, 0);
  timer_enable_oc_output(TIM2, TIM_OC3);
 
  //  Uncomment if using tim1_cc_isr() for debugging:
  //  timer_enable_irq(TIM2, TIM_DIER_CC1IE);
  //  nvic_enable_irq(NVIC_TIM2_CC_IRQ);
  
  timer_enable_counter(TIM2);
}
