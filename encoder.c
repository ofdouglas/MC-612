/*******************************************************************************
 * encoder.c - Position and velocity sensing using a quadrature encoder
 * Author: Oliver Douglas
 * Target: STM32F051R8, using libopencm3
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

// Convert a position measurement from encoder ticks into degrees.
// The encoder ticks input should be less than ENCODER_COUNT.
fix16_t encoder_ticks_to_degs(int ticks)
{
  return fix16_smul(DEGS_PER_TICK, fix16_from_int(ticks));
}

// Convert a position measurement from degrees into encoder ticks.
// The degrees input should be less than 360.
int encoder_degs_to_ticks(fix16_t degs)
{
  return fix16_to_int(fix16_smul(degs, TICKS_PER_DEG));
}

// Return the encoder position, in terms of encoder ticks.
int encoder_get_position(void)
{
  return timer_get_counter(TIM3);
}



/*******************************************************************************
 * Encoder velocity functions
 ******************************************************************************/

// Return the direction of the last encoder position counter change (1=up, 0=down)
int encoder_get_direction(void)
{
  return TIM_CR1(TIM3) & BIT4;
}

// Filter coefficients
#define LPF_A  0.2
#define LPF_A0 F16(LPF_A)
#define LPF_A1 F16(1-LPF_A)

// First-order IIR low-pass filter for the encoder velocity. This helps 
// reduce the quantization noise which is prsent in the velocity estimation
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
  static int prev_pos;
  int cur_pos, diff_pos, dir;

  // Read our current position and direction of travel from the encoder
  cur_pos = encoder_get_position();
  dir = encoder_get_direction();

  // For any two position measurements, there are two valid ways of calculating
  // the position difference: clockwise and counterclockwise. We use the travel
  // direction to pick which one. The relation between dir and the direction of
  // travel (CW / CCW) will be determined by which encoder phase (A or B) is
  // connected to which channel of the encoder timer.
  if (dir) {
    // counterclockwise in our setup
    diff_pos = -MODULAR_SUBTRACT(prev_pos, cur_pos, ENCODER_COUNTS);
  }
  else {
    // clockwise in our setup
    diff_pos = MODULAR_SUBTRACT(cur_pos, prev_pos, ENCODER_COUNTS);
  }

  // Store the current position for use next time.
  prev_pos = cur_pos;

  // Convert the position difference into degrees and apply a low-pass filter
  // before returning it, to reduce quantization noise.
  return recursive_lpf(fix16_smul(fix16_from_int(diff_pos), RPM_COEFFICIENT));
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


