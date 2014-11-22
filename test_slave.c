#include <util/delay.h>

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
  serial_write(c);
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
// serial_putc('['); serial_putc((c>>4)+'A'); serial_putc((c&0xf)+'A'); serial_putc(']');
      if (c == '\n')
        break;
      if (c == '\r')
        continue;
      if (idx >= sizeof(buf))
        continue;
      buf[idx++] = c;
    }

    _delay_ms(1);
    rs485_transmit_mode();
    serial_puts("ECHO: ");
    for (i = 0; i < idx; ++i)
      serial_putc(buf[i]);
    serial_puts("\r\n123");
    serial_wait_for_tx_complete();
    _delay_ms(1);
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

  cli();//sei();

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
