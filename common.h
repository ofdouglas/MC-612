#ifndef _DEFS_H
#define _DEFS_H

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

#define LED_GPIO	GPIOC
#define LED_BLUE_BIT	BIT8
#define LED_GREEN_BIT	BIT9

#define PWM_TIMER	TIM1
#define PWM_TIMER_RCC	RCC_TIM1

#define PWM_GPIO	GPIOA
#define PWM_GPIO_RCC	RCC_GPIOA
#define PWM_BIT		BIT8

#define DEBUG		0

struct print_msg {
  const char * fmt_str;
  fix16_t arg;
};


enum { CMD_NEUTRAL, CMD_BRAKE, CMD_VELOCITY, CMD_POSITION };
       
struct motion_cmd {
  int cmd;
  fix16_t arg;
};

/*  USART
 *   USART1_TX [PA9]
 *   USART1_RX [PA10]
 *
 *  PWM
 *   TIM2_CH2 [PA1]
 *   TIM2_CH3 [PA2]
 *  
 *  ENCODER
 *   TIM3_CH1 [PA6]
 *   TIM4_CH2 [PA7]
 *
 *  LEDS
 *   GREEN_LED [PC8]
 *   BLUE_LED  [PC9]
 *
 *  PUSHBUTTON
 *   USER_BUTTON [PA0]
 */


#endif // _DEFS_H
