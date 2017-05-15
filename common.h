/*******************************************************************************
 * common.h - Project definitions and function prototypes
 * Author: Oliver Douglas
 *
 ******************************************************************************/

#ifndef _COMMON_H
#define _COMMON_H

#include <stdint.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "xprintf.h"

#include "circbuf.h"

#define FIXMATH_NO_64BIT 1
#include "fix16.h"

#define STM32F0
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dbgmcu.h>



/*******************************************************************************
 * IO pin definitions
 ******************************************************************************/

#define LED_GPIO	GPIOC
#define LED_BLUE_BIT	BIT8
#define LED_GREEN_BIT	BIT9

#define PWM_TIMER	TIM1
#define PWM_TIMER_RCC	RCC_TIM1

#define PWM_GPIO	GPIOA
#define PWM_GPIO_RCC	RCC_GPIOA
#define PWM_BIT		BIT8

#define TIMING_GPIO	GPIOC
#define CONTROL_TASK_BIT BIT10
#define CMDLINE_TASK_BIT BIT11
#define LOGGING_TASK_BIT BIT12

/* I/O pin usage table
 *
 *  PWM			ENCODER			USART
 *   TIM2_CH2 [PA1]	 TIM3_CH1 [PA6]		 USART1_TX [PA9]
 *   TIM2_CH3 [PA2]	 TIM3_CH2 [PA7]		 USART1_RX [PA10]
 *  
 *  LEDS		PUSHBUTTON		TASK TIMING
 *   GREEN_LED [PC8]	 USER_BTN [PA0]		 CONTROL_TASK [PC.10]
 *   BLUE_LED  [PC9]				 CMDLINE_TASK [PC.11]
 *						 LOGGING_TASK [PC.12]
 */



/*******************************************************************************
 * Motion control definitions
 ******************************************************************************/

#define ENCODER_COUNTS  3200	// Counts per revolution
#define CONTROL_RATE	200	// Frequency of control loop execution (Hz)
#define CONTROL_CEILING 1000.0	// maximum controller output value
#define DUTY_SCALE	400	// Max duty cycle + 1
#define RPM_MAX		200

// Motion commands passed from the command line interface to the controller
enum { CMD_BRAKE, CMD_VELOCITY, CMD_POSITION };
       
struct motion_cmd {
  int cmd;
  fix16_t arg;
};

// Controller output and sensor data, passed from the controller to the logger
struct ctrl_log {
  fix16_t position;
  fix16_t velocity;
  fix16_t ctrl_output;
};



/*******************************************************************************
 * USART definitions
 ******************************************************************************/

#define USART_RX_BUF_LEN 64
#define USART_TX_BUF_LEN 64
#define USART_BAUD_RATE  115200



/*******************************************************************************
 * Encoder functions
 ******************************************************************************/

fix16_t encoder_ticks_to_degs(int ticks);
int encoder_degs_to_ticks(fix16_t degs);
int encoder_get_position(void);
fix16_t encoder_get_velocity(void);
void encoder_setup(void);



/*******************************************************************************
 * PWM functions
 ******************************************************************************/

void pwm_setup(void);
void pwm_set_output(fix16_t setpoint);



/*******************************************************************************
 * USART functions
 ******************************************************************************/

void usart_putchar(unsigned char c);
int usart_getchar(void);
void usart_setup(void);



/*******************************************************************************
 * Miscellaneous definitions
 ******************************************************************************/
#define DEBUG		0
#define MAX_CMD_LEN	3
#define MAX_ARG_LEN	20

// Return (a-b) modulo m
#define MODULAR_SUBTRACT(a,b,m) ((a) >= (b) ? (a) - (b) : (m) - (b) + (a))



#endif // _COMMON_H
