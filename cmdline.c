/*******************************************************************************
 * cmdline.c - Command line interface and logging
 * Author: Oliver Douglas
 *
 ******************************************************************************/

#include "common.h"

extern SemaphoreHandle_t usart_rx_sem;

extern QueueHandle_t cmd_queue;

extern int usart_getchar(void);
extern void usart_putchar(unsigned char c);



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

static int do_logging;

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

  xdev_out(usart_putchar);
  xputs("MC-612 Motor Controller Online\n");
  xputs("==============================\n");
  
  char cmd_buf[MAX_CMD_LEN + 1];
  char arg_buf[MAX_ARG_LEN + 1];

  while (1) {
    // Block indefinitely until a full line is received
    xSemaphoreTake(usart_rx_sem, portMAX_DELAY);
    
    int rv = read_line(cmd_buf, arg_buf);
    fix16_t arg;
    
    if (rv)
      xputs("parse error\n");
    else {
      int cmd_index = cmd_index_lookup(cmd_buf);
      if (cmd_index == -1)
	xputs("invalid command\n");
      else {
	arg = atofix16(arg_buf);
	(cmd_table[cmd_index].cmd_func)(arg);
      }
      if (DEBUG) {
	if (cmd_buf[0] && arg_buf[0])
	  xprintf("cmd: %s  arg: %d.%d\n", cmd_buf, arg >> 16, arg & 0xffff);
	else if (cmd_buf[0])
	  xprintf("cmd: %s\n", cmd_buf);
      }
    }
  }
}


/*******************************************************************************
 * Logging of sensor data and controller output 
 ******************************************************************************/

void logging_task(void * log_queue_void)
{
  
  QueueHandle_t log_queue = (QueueHandle_t) log_queue_void;
  struct ctrl_log log;

  while (1) {
    xQueueReceive(log_queue, &log, portMAX_DELAY);
    if (do_logging) {
      xprintf("%d.%d %d.%d %d.%d\n",
	      log.position >> 16, log.position & 0xffff,
	      log.velocity >> 16, log.velocity & 0xffff,
	      log.ctrl_output >> 16, log.ctrl_output & 0xffff);
    }
  }
}

