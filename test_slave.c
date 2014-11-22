#include <util/delay.h>
#include <util/atomic.h>

#include <arduino/pins.h>
#include <arduino/serial.h>


#define PIN_DE 7
#define PIN_RE 6
/* RO er 0, DI er 1. */


static volatile uint8_t last_char;

static void
led_off(void)
{
  pin_low(13);
}


static void
led_on(void)
{
  pin_high(13);
}


static void
serial_wait_for_tx_complete(void)
{
  while (!(UCSR0A & _BV(TXC0)))
    ;
}


static void
rs485_receive_mode(void)
{
  pin_low(PIN_DE);
  pin_low(PIN_RE);
}


static void
rs485_transmit_mode(void)
{
  pin_high(PIN_RE);
  pin_high(PIN_DE);
}


static uint8_t
serial_getc(void)
{
  while (!serial_readable())
    ;
  //led_on();
  return serial_read();
}


static void
serial_putc(uint8_t c)
{
  while (!serial_writeable())
    ;
  /*
    Make sure we can use TXC (transmit complete) to wait for the char to be
    completely transmittet before turning off RS485 transmit mode.

    The TXC flag must be manually cleared (unless it gets cleared by
    triggering an interrupt). And we need to avoid races when clearing it, so
    that we do not risk missing that it becomes set, nor risk seeing it become
    set by an earlier char that completes after clearing it.

    So clear the TXC flag immediately after writing the last char. And disable
    interrupts around it; this is necessary to avoid that an interrupt triggers
    just after writing the char, delaying the clear until the last char has
    completed, which would lose the completion event.
  */
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    serial_write(c);
    UCSR0A |= _BV(TXC0);
  }
}


static void
serial_puts(char *s)
{
  while (*s)
    serial_putc(*s++);
}


static void
test_slave(void)
{
  char buf[64];
  unsigned idx;

  serial_puts("Starting test...\r\n");

  for (;;)
  {
    unsigned i;

    rs485_receive_mode();
    idx = 0;

    for (;;)
    {
      uint8_t c = serial_getc();
      if (!idx && c != '?')
        continue;
      if (c == '\n')
        break;
      if (c == '\r')
        continue;
      if (idx >= sizeof(buf))
        continue;
      buf[idx++] = c;
    }

    /* Let's give the master a bit of time to get into receive mode. */
    _delay_ms(1);
    rs485_transmit_mode();
    _delay_ms(1);  // Todo: Only need to wait like 200 ns or so for enable proparation delay.
    /*
      Send a dummy byte of all one bits. This should ensure that the UART state
      machine can sync up to the byte boundary, as in prevents any new start bit
      being seen for one character's time.
    */
    serial_putc(0xff);
    serial_puts("!ECHO: ");
    for (i = 0; i < idx; ++i)
      serial_putc(buf[i]);
    serial_puts("\r\n");
    serial_wait_for_tx_complete();
    rs485_receive_mode();
  }
}


serial_interrupt_rx()
{
  uint8_t c;

  c = serial_read();
  led_on();
  last_char = c;
}


int
main(int argc, char *argv[])
{
  pin_mode_output(13);
  led_off();

  sei();

  serial_baud_2400();
  serial_mode_8n1();
  serial_transmitter_enable();
  serial_receiver_enable();
//  serial_interrupt_rx_enable();

  pin_mode_output(PIN_RE);
  pin_high(PIN_RE);
  pin_mode_output(PIN_DE);
  pin_low(PIN_DE);

  serial_puts("test_slave inited.\r\n");

  test_slave();
  return 0;
}
