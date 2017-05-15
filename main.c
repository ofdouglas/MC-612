/*******************************************************************************
 * main.c - Motor controller setup
 * Author: Oliver Douglas
 * Target: STM32F051R8, using libopencm3
 ******************************************************************************/

#include "common.h"



/*******************************************************************************
 * Debugging
 ******************************************************************************/

void assertion_failed(const char *file, int line)
{
  xprintf("assertion failed: %s:%d\n", file, line);
  taskDISABLE_INTERRUPTS();
  while (1)
    ; // do nothing
}

// Toggle green LED to show that the system is still running
// NOTE: creating a task solely for this purpose wastes considerable memory.
// In the future, replace this with FreeRTOS' vApplicationIdleHook() mechanism.
void toggle_task(void *foo)
{
  (void)foo;
  
  while (1) {
    vTaskDelay((const TickType_t) 250);
    gpio_toggle(GPIOC, LED_GREEN_BIT);
  }
}



/*******************************************************************************
 * Basic hardware setup
 ******************************************************************************/

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

  // Configure PC.10, PC.11, PC.12 as GPIO outputs for task timing
  gpio_mode_setup(TIMING_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
		  CONTROL_TASK_BIT | CMDLINE_TASK_BIT | LOGGING_TASK_BIT);
  
  // Configure PA.0 as GPIO input for the pushbutton
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);
  
  // Configure PA.6, PA.7 as Encoder inputs for TIM3
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
  gpio_set_af(GPIOA, GPIO_AF1, GPIO6 | GPIO7);
}



/*******************************************************************************
 * Application entry point
 ******************************************************************************/

// Synchronization objects to be created
SemaphoreHandle_t print_mutex;
SemaphoreHandle_t usart_rx_sem;
QueueHandle_t cmd_queue;
QueueHandle_t log_queue;

// Tasks from other modules, to be created
extern void motorctrl_task(void * foo);
extern void cmdline_task(void * foo);
extern void logging_task(void * foo);

int main(void)
{
  // Allow the debugger to work during sleep modes
  DBGMCU_CR = DBGMCU_CR_STOP | DBGMCU_CR_STANDBY;

  // Setup peripherals
  clock_setup();
  usart_setup();
  io_pin_setup();
  pwm_setup();
  encoder_setup();
  
  xdev_out(usart_putchar);
  xputs("\nMC-612 Motor Controller Online\n");
  xputs("==============================\n\n");

  // Allocate synchronization objects
  print_mutex = xSemaphoreCreateMutex();
  usart_rx_sem = xSemaphoreCreateCounting(10, 0);
  cmd_queue = xQueueCreate(4, sizeof(struct motion_cmd));
  log_queue = xQueueCreate(4, sizeof(struct ctrl_log));

  // Allocate tasks (highest priority first)
  xTaskCreate(motorctrl_task, "", 128, log_queue, 4, NULL);
  xTaskCreate(logging_task, "", 64, log_queue, 3, NULL);
  xTaskCreate(cmdline_task, "", 128, NULL, 2, NULL);
  xTaskCreate(toggle_task, "", 64, NULL, 1, NULL);

  // Start the RTOS
  vTaskStartScheduler();
  
  while (1)
    ;	// The scheduler does not return
}





