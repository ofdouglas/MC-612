/*******************************************************************************
 * cmdline.c - Command line interface and logging
 * Author: Oliver Douglas
 * Target: STM32F051R8, using libopencm3
 ******************************************************************************/

#include "common.h"

extern SemaphoreHandle_t usart_rx_sem;
extern SemaphoreHandle_t print_mutex;
extern QueueHandle_t cmd_queue;
extern QueueHandle_t log_queue;

// Logging enable flag that is set by cmd_line_task and read by logging_task. 
static volatile int do_logging;



/*******************************************************************************
 * String handling functions from C stdlib
 ******************************************************************************/

int isdigit(int c) {
  return c >= '0' && c <= '9';
}

int isspace(int c) {
  return c == ' ' || c == '\t' || c == '\n';
}

int strncmp(const char * a, const char * b, size_t n)
{
  int diff;
  
  for (size_t i = 0; i < n; n++, a++, b++) {
    diff = *a - *b;
    if (diff || (*a == 0))
      break;
  }
  return diff;
}

// 'atof' for fix16_t (adapted from K&R C)
fix16_t atofix16(char * s)
{
  fix16_t whole, frac, sign;
  int i; 

  for (i = 0; isspace(s[i]); i++)
    ; // skip white space
  sign = (s[i] == '-') ? F16(-1) : F16(1);
  
  if (s[i] == '+' || s[i] == '-')
    i++;
  
  for (whole = 0; isdigit(s[i]); i++)
    whole = 10 * whole + (s[i] - '0');
  
  if (s[i] == '.')
    i++;
  
  for (frac = 0; isdigit(s[i]); i++)
    frac = 10 * frac + (s[i] - '0');

  return fix16_mul(sign, fix16_add(whole << 16, frac));
}



/*******************************************************************************
 * Command structure and implementation
 ******************************************************************************/

// Set the motor velocity, in RPM (begin closed-loop velocity control)
int cmd_set_rpm(fix16_t arg)
{
  struct motion_cmd mc = {
    .cmd = CMD_VELOCITY,
    .arg = arg
  };
  
  return xQueueSend(cmd_queue, &mc, portMAX_DELAY);
}

// Set the motor position, in degrees (begin closed-loop position control)
int cmd_set_pos(fix16_t arg)
{
  struct motion_cmd mc = {
    .cmd = CMD_POSITION,
    .arg = arg
  };
  
  return xQueueSend(cmd_queue, &mc, portMAX_DELAY);
}

// Hard-stop the motor (short both motor terminals together). This will stop
// the motor faster than 'rpm=0' due to the lag in the PID controller, but
// after the transient, the results will be very similar. The motor will
// remain shorted until a new command is issued.
int cmd_break(fix16_t arg)
{
  (void) arg;
  
  struct motion_cmd mc = {
    .cmd = CMD_BRAKE,
    .arg = 0
  };
  
  return xQueueSend(cmd_queue, &mc, portMAX_DELAY);
}

// Enable or disable logging output to the serial terminal
int cmd_toggle_log(fix16_t arg)
{
  (void) arg;
  
  do_logging ^= 1;
  return do_logging;
}

struct command {
  const char name[MAX_CMD_LEN];
  int (* cmd_func)(fix16_t);
};

struct command cmd_table[] = {
  { .name = "rpm", cmd_set_rpm },
  { .name = "pos", cmd_set_pos },
  { .name = "brk", cmd_break },
  { .name = "tlg", cmd_toggle_log }
};

// Return the index of 'str' in the command table, or -1 if it isn't found.
int cmd_index_lookup(const char *str)
{
  for (size_t i = 0; i < sizeof(cmd_table) / sizeof(cmd_table[0]); i++)
    if (strncmp(cmd_table[i].name, str, MAX_CMD_LEN) == 0)
      return i;
  
  return -1;
}



/*******************************************************************************
 * Command line read and parse functions
 ******************************************************************************/

// Attempt to fill cmd_buf and arg_buf from the USART receive buffer
// return -1 on syntax error
int read_line(char *cmd_buf, char *arg_buf)
{
  int i, c;

  // Ensure previous contents are ignored if nothing is found
  cmd_buf[0] = '\0';
  arg_buf[0] = '\0';
  
  // Copy characters into the command buffer and null-terminate it
  for (i = 0; i < MAX_CMD_LEN; i++) {
    // Abort without an error if the command string is empty.
    // Abort with an error if the full command length is not found.
    if ((c = usart_getchar()) == '\n')
      return i == 0 ? 0 : -1;
    cmd_buf[i] = (char) c;
  }
  cmd_buf[i] = '\0';

  // This character determines whether there is a command or not
  c = usart_getchar();
  
  if (c == '\n') {		// just a command with no args
    return 0;
  }
  else if (c == '=') {		// an argument is included
    for (i = 0; i < MAX_ARG_LEN; i++) {
      if ((c = usart_getchar()) == '\n') {
	arg_buf[i] = '\0';
	return 0;
      }
      arg_buf[i] = c;
    }
  }

  // If we reached this point, then either the syntax was invalid or the arg
  // exceeded the buffer size, which is an error. Remove the remainder of this
  // erroneous command. The arg_buf should be ignored by the caller,
  // we but null-terminate it anyways just to be safe.
  arg_buf[i] = '\0';
  
  while (c != -1 && c != '\n')
    c = usart_getchar();
  return -1;
}


void cmd_line_task(void * foo)
{
  (void) foo;

  char cmd_buf[MAX_CMD_LEN + 1];
  char arg_buf[MAX_ARG_LEN + 1];

  // Check memory usage. We are not using dynamic memory allocation for
  // anything once the scheduler has started, but since the scheduler allocates
  // some memory for itself, we have to check usage inside a task.
  //
  // The only reason that a 'heap' and non-static versions of xTaskCreate, etc,
  // were used, is that they are simpler to get working than the truly static
  // allocation versions.
  //
  // Because no more memory is allocated, the free heap amount can be small.
  if (DEBUG)
    xprintf("free heap: %d\n", xPortGetFreeHeapSize());
  
  while (1) {
    // Block indefinitely until a full line is received
    xSemaphoreTake(usart_rx_sem, portMAX_DELAY);

    // Begin task timing
    gpio_set(TIMING_GPIO, CMDLINE_TASK_BIT);
    
    fix16_t arg = 0;
    int rv = read_line(cmd_buf, arg_buf);
    char * response = NULL;
    
    if (rv) {
      response = "parse error\n";
    }
    else {
      int cmd_index = cmd_index_lookup(cmd_buf);
      if (cmd_index == -1) {
	response = "invalid command\n";
      }
      else {
	response = "ack\n";
	arg = atofix16(arg_buf);
	(cmd_table[cmd_index].cmd_func)(arg);
      }
    }

    if (response) {
      xSemaphoreTake(print_mutex, portMAX_DELAY);
      xputs(response);
      xSemaphoreGive(print_mutex);
    }


    if (DEBUG) {
      xSemaphoreTake(print_mutex, portMAX_DELAY);
      if (cmd_buf[0] && arg_buf[0])
	xprintf("cmd: %s  arg: %d.%d\n", cmd_buf, arg >> 16, arg & 0xffff);
      else if (cmd_buf[0])
	xprintf("cmd: %s\n", cmd_buf);
      xSemaphoreGive(print_mutex);
    }

    
    // End task timing
    gpio_clear(TIMING_GPIO, CMDLINE_TASK_BIT);
  }
}



/*******************************************************************************
 * Logging of sensor data and controller output 
 ******************************************************************************/

void logging_task(void * foo)
{
  (void) foo;
  
  struct ctrl_log log;

  while (1) {
    xQueueReceive(log_queue, &log, portMAX_DELAY);

    // Begin task timing
    gpio_set(TIMING_GPIO, LOGGING_TASK_BIT);
    
    if (do_logging) {
      xSemaphoreTake(print_mutex, portMAX_DELAY);
      xprintf("%#d.%d %d.%d %d.%d\n",
	      log.position >> 16, log.position & 0xffff,
	      log.velocity >> 16, log.velocity & 0xffff,
	      log.ctrl_output >> 16, log.ctrl_output & 0xffff);
      xSemaphoreGive(print_mutex);
    }

    // End task timing
    gpio_clear(TIMING_GPIO, LOGGING_TASK_BIT);
  }
}

