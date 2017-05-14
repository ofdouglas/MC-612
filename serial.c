#include "common.h"


#define MAX_CMD_LEN 3
#define MAX_ARG_LEN 10

extern fix16_t goal_speed;


#define RX_BUF_LEN 64
#define TX_BUF_LEN 64

static char rx_buf_mem[RX_BUF_LEN];
static char tx_buf_mem[TX_BUF_LEN];

static struct circbuf rx_buf = {
  .head = 0,
  .tail = 0,
  .size = RX_BUF_LEN,
  .mem = rx_buf_mem
};

static struct circbuf tx_buf = {
  .head = 0,
  .tail = 0,
  .size = TX_BUF_LEN,
  .mem = tx_buf_mem
};

// Incremented when a line is received, decremented when the line is processed
static SemaphoreHandle_t rx_sem;

// Pass 'deferred printf' messages to the serial_write_task
QueueHandle_t lpq;

// 
void usart1_isr(void)
{
  int c;
  
  // character received:
  if (usart_get_interrupt_source(USART1, USART_ISR_RXNE)) {
    c = USART1_RDR;
    circbuf_add(&rx_buf, (char) c);

    // full line received: wake the writer task
    if (c == '\n')
      xSemaphoreGiveFromISR(rx_sem, NULL);
  }

  // transmit buffer empty:
  if (usart_get_interrupt_source(USART1, USART_ISR_TXE)) {

    // this will be -1 if the buffer was empty, but that should never happen...
    c = circbuf_remove(&tx_buf);
    USART1_TDR = (char) c;
    
    if (circbuf_is_empty(&tx_buf))
      usart_disable_tx_interrupt(USART1);
  }
}

void usart_putchar(unsigned char c)
{
  circbuf_add(&tx_buf, c);
  usart_enable_tx_interrupt(USART1);
}



void usart_config(void)
{
  // Configure the USART in asynchronous 8n1 mode
  usart_set_baudrate(USART1, 460800);
  usart_set_databits(USART1, 8);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_stopbits(USART1, 1);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART1, USART_MODE_TX_RX);
  usart_enable(USART1);

  rx_sem = xSemaphoreCreateBinary();

  usart_enable_rx_interrupt(USART1);
  nvic_enable_irq(NVIC_USART1_IRQ);
}


int isdigit(int c) {
  return c >= '0' && c <= '9';
}

int isspace(int c) {
  return c == ' ' || c == '\t' || c == '\n';
}


// adapted from K&R
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

extern QueueHandle_t cmd_queue;

int cmd_set_rpm(fix16_t arg)
{
  struct motion_cmd mc = {
    .cmd = CMD_VELOCITY,
    .arg = arg
  };
  
  xQueueSend(cmd_queue, &mc, portMAX_DELAY);
  
  return 0;
}

int cmd_set_pos(fix16_t arg)
{
  struct motion_cmd mc = {
    .cmd = CMD_POSITION,
    .arg = arg
  };
  
  xQueueSend(cmd_queue, &mc, portMAX_DELAY);
  
  return 0;
}
int cmd_stop(fix16_t arg)
{
  struct motion_cmd mc = {
    .cmd = CMD_BRAKE,
    .arg = 0
  };
  
  xQueueSend(cmd_queue, &mc, portMAX_DELAY);
  
  
  return 0;
}

struct command {
  const char name[MAX_CMD_LEN];
  int (* cmd_func)(fix16_t);
};

struct command cmd_table[] = {
  { .name = "rpm", cmd_set_rpm },
  { .name = "pos", cmd_set_pos },
  { .name = "stp", cmd_stop }
};


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
    if ((c = circbuf_remove(&rx_buf)) == '\n')
      return i == 0 ? 0 : -1;
    cmd_buf[i] = (char) c;
  }
  cmd_buf[i] = '\0';

  // This character determines whether there is a command or not
  c = circbuf_remove(&rx_buf);
  
  if (c == '\n') {		// just a command with no args
    return 0;
  }
  else if (c == '=') {		// an argument is included
    for (i = 0; i < MAX_ARG_LEN; i++) {
      if ((c = circbuf_remove(&rx_buf)) == '\n') {
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
    c = circbuf_remove(&rx_buf);
  return -1;
}



int cmd_index_lookup(const char *str)
{
  for (size_t i = 0; i < sizeof(cmd_table) / sizeof(cmd_table[0]); i++)
    if (strncmp(cmd_table[i].name, str, MAX_CMD_LEN) == 0)
      return i;
  
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
    xSemaphoreTake(rx_sem, portMAX_DELAY);
    
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

      if (cmd_buf[0] && arg_buf[0])
	xprintf("cmd: %s  arg: %d.%d\n", cmd_buf, arg >> 16, arg & 0xffff);
      else if (cmd_buf[0])
	xprintf("cmd: %s\n", cmd_buf);
    }
  }
}



void logging_task(void * foo)
{
  (void) foo;

  while (1) {
  }
}
