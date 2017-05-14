
/*******************************************************************************
 * encoder.c - Position and velocity sensing using a quadrature encoder
 * Author: Oliver Douglas
 *
 ******************************************************************************/

#include "common.h"

// Scaling factor for encoder position conversions (ticks <-> degrees)
#define DEGS_PER_TICK	(F16(360.0/ENCODER_COUNTS))
#define TICKS_PER_DEG	(F16(ENCODER_COUNTS/360.0))

// Scaling factor for encoder speed conversions (ticks -> RPM)
// N ticks/sample * [CONTROL_RATE samples/sec * revs/tick * seconds/min]
#define RPM_COEFFICIENT (F16((1.0/ENCODER_COUNTS) * CONTROL_RATE * 60))



/*******************************************************************************
 * Encoder position functions
 ******************************************************************************/

fix16_t encoder_ticks_to_degs(int ticks)
{
  return fix16_smul(DEGS_PER_TICK, fix16_from_int(ticks));
}


int encoder_degs_to_ticks(fix16_t degs)
{
  return fix16_to_int(fix16_smul(degs, TICKS_PER_DEG));
}


int encoder_get_position(void)
{
  return timer_get_counter(TIM3);
}



/*******************************************************************************
 * Encoder velocity functions
 ******************************************************************************/

// Filter coefficients
#define LPF_A  0.2
#define LPF_A0 F16(LPF_A)
#define LPF_A1 F16(1-LPF_A)

// First-order IIR low-pass filter for the encoder velocity.
// This helps reduce the noise inherent in the velocity estimation
// algorithm at low speeds.
static fix16_t recursive_lpf(fix16_t x)
{
  static fix16_t y;

  y = fix16_add(fix16_smul(LPF_A0, x), fix16_smul(LPF_A1, y));
  return y;
}

// Return encoder velocity, in RPM, by counting the number of encoder ticks per
// time window. This function must be called at regular intervals
// (once per iteration of the control loop)
fix16_t encoder_get_velocity(void)
{
  static int old_pos;
  int new_pos, diff_pos;

  new_pos = timer_get_counter(TIM3);

  int dir = TIM_CR1(TIM3) & BIT4;
  if (dir) {
    diff_pos = -MODULAR_SUBTRACT(old_pos, new_pos, ENCODER_COUNTS);
    gpio_set(GPIOC, LED_BLUE_BIT);
  }
  else {
    diff_pos = MODULAR_SUBTRACT(new_pos, old_pos, ENCODER_COUNTS);
    gpio_clear(GPIOC, LED_BLUE_BIT);
  }

  // clockwise
  //diff_pos = modular_subtract(new_pos, old_pos, ENCODER_COUNTS);
  
  old_pos = new_pos;

  //  xprintf("%d\n", diff_pos);
  
  fix16_t diff = fix16_smul(fix16_from_int(diff_pos), RPM_COEFFICIENT);
  return recursive_lpf(diff);
}



/*******************************************************************************
 * Encoder configuration functions
 ******************************************************************************/

// The output gear ratio is 50:1, so our encoder counts per rev of the
// output shaft is 64 * 50 = 3200
void encoder_setup(void)
{
  rcc_periph_clock_enable(RCC_TIM3);
  timer_set_period(TIM3, ENCODER_COUNTS);
  timer_slave_set_mode(TIM3, 0x3);
  timer_ic_set_input(TIM3, TIM_IC1, TIM_IC_IN_TI1);
  timer_ic_set_input(TIM3, TIM_IC1, TIM_IC_IN_TI2);
  timer_enable_counter(TIM3);
}


