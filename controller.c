/*******************************************************************************
 * controller.c - Closed loop position and velocity control
 * Author: Oliver Douglas
 *
 ******************************************************************************/

#include "common.h"

QueueHandle_t cmd_queue;


// Velocity PID controller loop gains
#define V_PID_KP		(F16(20.0))
#define V_PID_KI		(F16(100.0 / CONTROL_RATE))
#define V_PID_KD		(F16(1.0 / CONTROL_RATE))

// Position PID controller loop gains
#define P_PID_KP		(F16(15.0))
#define P_PID_KI		(F16(10.0 / CONTROL_RATE))
#define P_PID_KD		(F16(25.0 / CONTROL_RATE))



/*******************************************************************************
 * PID Controllers
 ******************************************************************************/

static fix16_t velocity_controller(fix16_t goal_velocity, fix16_t real_velocity)
{
  static fix16_t error_prev, integral;
  fix16_t error, proportional, derivative;

  error = fix16_ssub(goal_velocity, real_velocity);

  proportional = fix16_smul(error, V_PID_KP);
  integral = fix16_add(integral, fix16_smul(error, V_PID_KI));
  derivative = fix16_smul(V_PID_KD, fix16_ssub(error, error_prev));
  error_prev = error;
  
  return fix16_add(fix16_add(proportional, integral), derivative);
}


static fix16_t position_controller(int goal_position, int real_position)
{
  static fix16_t error_prev, integral;
  fix16_t error, proportional, derivative;
  int cw_error, ccw_error;


  // Choose the shorter of the clockwise and counterclockwise difference
  // between present and desired position 
  cw_error = MODULAR_SUBTRACT(real_position, goal_position, ENCODER_COUNTS);
  ccw_error = MODULAR_SUBTRACT(goal_position, real_position, ENCODER_COUNTS);
  if (cw_error < ccw_error)
    error = fix16_from_int(-cw_error);
  else
    error = fix16_from_int(ccw_error);

  proportional = fix16_smul(error, P_PID_KP);
  integral = fix16_add(integral, fix16_smul(error, P_PID_KI));
  derivative = fix16_smul(P_PID_KD, fix16_ssub(error, error_prev));
  error_prev = error;
  
  return fix16_add(fix16_add(proportional, integral), derivative);
}



/*******************************************************************************
 * Top-level controller
 ******************************************************************************/

// control the motor
// positive goal_speed: clockwise motion, TIM_OC2 is on
void motor_control_task(void * log_queue_void)
{
  cmd_queue = xQueueCreate(5, sizeof(struct motion_cmd));

  QueueHandle_t log_queue = (QueueHandle_t) log_queue_void;
  struct ctrl_log log;
  
  fix16_t ctrl_output;
  fix16_t velocity;
  int position;

  struct motion_cmd mc = {
    .cmd = CMD_BRAKE,
    .arg = 0
  };
  
  while(1) {
    vTaskDelay((const TickType_t)(configTICK_RATE_HZ / CONTROL_RATE));

    if (uxQueueMessagesWaiting(cmd_queue))
      xQueueReceive(cmd_queue, &mc, 0);

    // The controller never uses both velocity and position simultaneously,
    // but must make both available to the logging task at all times
    velocity = encoder_get_velocity();
    position = encoder_get_position();
    
    switch (mc.cmd)
      {
      case CMD_BRAKE:	// hard stop - short the motor terminals
	timer_set_oc_value(TIM2, TIM_OC2, 0);
	timer_set_oc_value(TIM2, TIM_OC3, 0);
	break;

      case CMD_VELOCITY:	// constant velocity control
	ctrl_output = velocity_controller(mc.arg, velocity);
	pwm_set_output(ctrl_output);
	break;
      
      case CMD_POSITION:	// constant position control
	ctrl_output = position_controller(encoder_degs_to_ticks(mc.arg), position);
	pwm_set_output(ctrl_output);
	break;
      }

    // Send the current position, velocity, and controller output to
    // the logging task
    log.position = encoder_ticks_to_degs(position);
    log.velocity = velocity;
    log.ctrl_output = ctrl_output;
    if (uxQueueSpacesAvailable(log_queue))
      xQueueSend(log_queue, &log, 0);
    
    // Emergency stop button
    if (gpio_get(GPIOA, GPIO0))
      mc.cmd = CMD_BRAKE;
  }
}
