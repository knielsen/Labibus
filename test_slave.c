#include <util/delay.h>
#include <util/atomic.h>
#include <avr/pgmspace.h>

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


/* CRC-16. */
static const uint16_t crc16_tab[256] PROGMEM = {
  0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
  0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
  0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
  0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
  0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
  0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
  0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
  0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
  0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
  0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
  0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
  0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
  0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
  0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
  0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
  0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
  0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
  0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
  0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
  0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
  0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
  0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
  0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
  0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
  0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
  0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
  0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
  0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
  0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
  0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
  0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
  0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040
};


static uint32_t crc16(uint8_t byte, uint16_t crc_val)
{
  uint16_t tab_lookup = pgm_read_word(&crc16_tab[(uint8_t)crc_val ^ byte]);
  return tab_lookup ^ (crc_val >> 8);
}


static uint32_t crc16_buf(const uint8_t *buf, uint16_t len)
{
  uint16_t crc_val = 0;
  while (len > 0)
  {
    crc_val = crc16(*buf, crc_val);
    ++buf;
    --len;
  }
  return crc_val;
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
