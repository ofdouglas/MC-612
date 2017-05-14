

#include "common.h"

extern void usart_config(void);
extern void usart_putchar(unsigned char c);
extern void motor_control_task(void * foo);
extern void serial_write_task(void * foo);
extern void serial_read_task(void * foo);


void assertion_failed(void)
{
  xputs("assertion failed\n");
  taskDISABLE_INTERRUPTS();
  while (1)
    ; // do nothing
}


void pwm_timer_config(void)
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


// The output gear ratio is 50:1, so our encoder counts per rev of the
// output shaft is 64 * 50 = 3200
void encoder_timer_config(void)
{
  rcc_periph_clock_enable(RCC_TIM3);
  timer_set_period(TIM3, 3199);
  timer_slave_set_mode(TIM3, 0x3);
  timer_ic_set_input(TIM3, TIM_IC1, TIM_IC_IN_TI1);
  timer_ic_set_input(TIM3, TIM_IC1, TIM_IC_IN_TI2);
  timer_enable_counter(TIM3);
}


// Toggle green LED
void toggle_task(void *foo)
{
  (void)foo;
  
  while (1) {
    gpio_toggle(GPIOC, LED_GREEN_BIT);
    vTaskDelay((const TickType_t) 250);
  }
}


void clock_config(void)
{
  // Enable GPIOA, GPIOC
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOC);

  // Enable USART1
  rcc_periph_clock_enable(RCC_USART1);
}


void io_pin_config(void)
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
  
  clock_config();
  io_pin_config();
  usart_config();
  pwm_timer_config();
  encoder_timer_config();

  xTaskCreate(toggle_task, "", configMINIMAL_STACK_SIZE,
	      NULL, 1, NULL);
  
  xTaskCreate(motor_control_task, "", configMINIMAL_STACK_SIZE,
  	      NULL, 3, NULL);

  xTaskCreate(serial_write_task, "", configMINIMAL_STACK_SIZE,
	      NULL, 2, NULL);
  
  xTaskCreate(serial_read_task, "", configMINIMAL_STACK_SIZE,
	      NULL, 2, NULL);
  
  vTaskStartScheduler();
  
  while (1)
    ;	// The scheduler does not return
}





