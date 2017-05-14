/*******************************************************************************
 * usart.c - RS-232 serial communication driver
 * Author: Oliver Douglas
 *
 ******************************************************************************/

#include "common.h"


// Incremented when a line is received, decremented when the line is processed
SemaphoreHandle_t usart_rx_sem;


/*******************************************************************************
 * FIFO transmit and receive buffers 
 ******************************************************************************/

static char rx_buf_mem[USART_RX_BUF_LEN];
static char tx_buf_mem[USART_TX_BUF_LEN];

static struct circbuf rx_buf = {
  .head = 0,
  .tail = 0,
  .size = USART_RX_BUF_LEN,
  .mem = rx_buf_mem
};

static struct circbuf tx_buf = {
  .head = 0,
  .tail = 0,
  .size = USART_TX_BUF_LEN,
  .mem = tx_buf_mem
};



/*******************************************************************************
 * Transmit and receive functions
 ******************************************************************************/

// Move characters between the USART RX/TX registers and the software buffers
// * Upon receiving a newline ('\n'), wake the command line task
// * Disable TX interrupt source when the software TX buffer has been emptied
 void usart1_isr(void)
{
  int c;
  
  // Character received - transfer it from the USART to the RX buffer
  if (usart_get_interrupt_source(USART1, USART_ISR_RXNE)) {
    c = USART1_RDR;
    circbuf_add(&rx_buf, (char) c);

    // A full line has been received: wake the command line task
    if (c == '\n')
      xSemaphoreGiveFromISR(usart_rx_sem, NULL);
  }

  // Character transmission - move it from the TX buffer to the USART
  if (usart_get_interrupt_source(USART1, USART_ISR_TXE)) {

    // 'c' will be -1 if the buffer was empty, but that should never happen...
    c = circbuf_remove(&tx_buf);
    USART1_TDR = (char) c;

    // If we have emptied the TX buffer, we're done transmitting
    if (circbuf_is_empty(&tx_buf))
      usart_disable_tx_interrupt(USART1);
  }
}

// Transfer a character to the TX buffer and enable the ISR to service it.
// This function is called by "xprintf" and "xputs"
void usart_putchar(unsigned char c)
{
  circbuf_add(&tx_buf, c);
  usart_enable_tx_interrupt(USART1);
}

int usart_getchar(void)
{
  return circbuf_remove(&rx_buf);
}
  


/*******************************************************************************
 * Configuration functions
 ******************************************************************************/

void usart_setup(void)
{
  // Configure the USART in asynchronous 8n1 mode
  usart_set_baudrate(USART1, USART_BAUD_RATE);
  usart_set_databits(USART1, 8);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_stopbits(USART1, 1);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART1, USART_MODE_TX_RX);
  usart_enable(USART1);

  usart_rx_sem = xSemaphoreCreateBinary();

  usart_enable_rx_interrupt(USART1);
  nvic_enable_irq(NVIC_USART1_IRQ);
}
