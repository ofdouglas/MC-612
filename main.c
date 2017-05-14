/*******************************************************************************
 * main.c - Motor controller setup
 * Author: Oliver Douglas
 *
 ******************************************************************************/

#include "common.h"


extern void motor_control_task(void * foo);
extern void cmd_line_task(void * foo);
extern void logging_task(void * foo);

extern void pwm_timer_config(void);
extern void encoder_timer_config(void);
extern void usart_config(void);

QueueHandle_t log_queue;

void assertion_failed(void)
{
  xputs("assertion failed\n");
  taskDISABLE_INTERRUPTS();
  while (1)
    ; // do nothing
}


// Toggle green LED to show that the system is still running
void toggle_task(void *foo)
{
  (void)foo;
  
  while (1) {
    vTaskDelay((const TickType_t) 250);
    gpio_toggle(GPIOC, LED_GREEN_BIT);
  }
}


void clock_setup(void)
{
  // Enable GPIOA, GPIOC
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOC);

  // Enable USART1
  rcc_periph_clock_enable(RCC_USART1);
}


void io_pin_setup(void)
{
  // Configure PA.9, PA.10 as USART1 Tx and Rx
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
  gpio_set_af(GPIOA, GPIO_AF1, GPIO9 | GPIO10);
  
  // Configure PA.1, PA.2 as PWM channels 2, 3 on TIM2
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO1 | GPIO2);
  gpio_set_af(GPIOA, GPIO_AF2, GPIO1 | GPIO2);

  // Configure PC.8, PC.9 as GPIO outputs for LEDs
  gpio_mode_setup(LED_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
		  LED_GREEN_BIT | LED_BLUE_BIT);

  // Configure PA.0 as GPIO input for the pushbutton
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);
  
  // Configure PA.6, PA.7 as Encoder inputs for TIM3
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
  gpio_set_af(GPIOA, GPIO_AF1, GPIO6 | GPIO7);
}


int main(void)
{
  // Allow the debugger to work during sleep modes
  DBGMCU_CR = DBGMCU_CR_STOP | DBGMCU_CR_STANDBY;
  
  clock_setup();
  io_pin_setup();
  usart_setup();
  pwm_setup();
  encoder_setup();

  log_queue = xQueueCreate(10, sizeof(struct ctrl_log));
  
  xTaskCreate(toggle_task, "", configMINIMAL_STACK_SIZE,
	      NULL, 1, NULL);
  
  xTaskCreate(motor_control_task, "", configMINIMAL_STACK_SIZE,
    	      log_queue, 3, NULL);

  xTaskCreate(cmd_line_task, "", configMINIMAL_STACK_SIZE,
	      NULL, 2, NULL);
  
  xTaskCreate(logging_task, "", configMINIMAL_STACK_SIZE,
	      log_queue, 2, NULL);
  
  vTaskStartScheduler();
  
  while (1)
    ;	// The scheduler does not return
}





