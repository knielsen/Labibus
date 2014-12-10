#include <util/delay.h>
#include <util/atomic.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdlib.h>
#include <stdio.h>

#ifdef ARDUINO
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#else
#include <arduino/pins.h>
#endif

#include "Labibus.h"


#ifdef ARDUINO
#ifdef PERNITTENGRYNET
#if !defined(__AVR_ATmega168__) && !defined(__AVR_ATmega328P__)
#error this AVR device is not yet supported :-(
#endif
#endif
#endif


static struct {
  float sensor_value;
  uint16_t poll_interval;
  /* A NULL description value means an unused entry. */
  const char *description, *unit;
  uint8_t device_id;
  uint8_t have_value;
} rs485_devices[MAX_DEVICES];


static void
setup_rs485_pins(void)
{
#ifdef ARDUINO
  pinMode(PIN_RE, OUTPUT);
  pinMode(PIN_DE, OUTPUT);
#else
  pin_mode_output(PIN_RE);
  pin_mode_output(PIN_DE);
#endif
}


static void
rs485_receive_mode(void)
{
#ifdef ARDUINO
  digitalWrite(PIN_DE, 0);
  digitalWrite(PIN_RE, 0);
#else
  pin_low(PIN_DE);
  pin_low(PIN_RE);
#endif
}


static void
rs485_transmit_mode(void)
{
#ifdef ARDUINO
  digitalWrite(PIN_RE, 1);
  digitalWrite(PIN_DE, 1);
#else
  pin_high(PIN_RE);
  pin_high(PIN_DE);
#endif
}


static void
setup_serial(void)
{
//#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
#if F_CPU == 16000000UL
  /* serial_baud_115200() */
  UCSR0A = (UCSR0A & ~(_BV(FE0) | _BV(DOR0) | _BV(UPE0)))
    | _BV(U2X0);
  UBRR0 = 16;
#else
#error This CPU frequency is not yet supported :-(
#endif
  /* serial_mode_8n1() */
  UCSR0B &= ~(_BV(UCSZ02));
  UCSR0C = (UCSR0C & ~(_BV(UPM01) | _BV(UPM00) | _BV(USBS0)))
    | _BV(UCSZ01) | _BV(UCSZ00);
  /* serial_transmitter_enable() */
  UCSR0B |= _BV(TXEN0);
  /* serial_receiver_enable() */
  UCSR0B |= _BV(RXEN0);
  /* serial_interrupt_rx_enable() */
  UCSR0B |= _BV(RXCIE0);
//#endif
}


static inline void
serial_interrupt_rx_enable(void)
{
//#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  UCSR0B |= _BV(RXCIE0);
//#endif
}


static inline void
serial_interrupt_rx_disable(void)
{
//#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  UCSR0B &= ~(_BV(RXCIE0));
//#endif
}


static inline uint8_t
serial_writeable(void)
{
//#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  return UCSR0A & _BV(UDRE0);
//#endif
}


static inline void
serial_write(uint8_t c)
{
//#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  UDR0 = c;
//#endif
}


static inline uint8_t
serial_read(void)
{
//#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  return UDR0;
//#endif
}


static void
serial_wait_for_tx_complete(void)
{
  while (!(UCSR0A & _BV(TXC0)))
    ;
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


static uint8_t
hex2dec(uint8_t c)
{
  if (c >= '0' && c <= '9')
    return c - '0';
  else if (c >= 'A' && c <= 'F')
    return c - ('A'-10);
  else if (c >= 'a' && c <= 'f')
    return c - ('a'-10);
  else
    return 0;
}


static uint8_t
dec2hex(uint8_t x)
{
  if (x <= 9)
    return x + '0';
  else
    return x + ('a' - 10);
}



static uint8_t
append_char_to_buf(uint8_t *buf, uint8_t idx, uint8_t c)
{
  if (idx < MAX_REQ-1)
    buf[idx++] = c;
  return idx;
}


static uint8_t
quoted_append_char_to_buf(uint8_t *buf, uint8_t idx, uint8_t c)
{
  if (c < ' ' || c >= 127 || c == '!' || c == '?' || c == '|' || c == '\\')
  {
    idx = append_char_to_buf(buf, idx, '\\');
    idx = append_char_to_buf(buf, idx, dec2hex(c >> 4));
    idx = append_char_to_buf(buf, idx, dec2hex(c & 0xf));
  }
  else
    idx = append_char_to_buf(buf, idx, c);

  return idx;
}


static uint8_t
append_to_buf(uint8_t *buf, uint8_t idx, const char *s)
{
  char c;

  while ((c = *s))
  {
    idx = append_char_to_buf(buf, idx, c);
    ++s;
  }
  return idx;
}


static uint8_t
quoted_append_to_buf(uint8_t *buf, uint8_t idx, const char *s)
{
  char c;

  while ((c = *s))
  {
    idx = quoted_append_char_to_buf(buf, idx, c);
    ++s;
  }
  return idx;
}


static void
send_reply(uint8_t *buf, uint8_t len)
{
  uint8_t i;
  uint16_t crc = 0;

  /* Let's give the master a bit of time to get into receive mode. */
  _delay_ms(1);
  rs485_transmit_mode();
  /*
    The enable propagation delay of our RS485 driver is only 200 ns or so, so
    we only need a small delay before we can start to transmit.
  */
  _delay_us(1);
  /*
    Send a dummy byte of all one bits. This should ensure that the UART state
    machine can sync up to the byte boundary, as in prevents any new start bit
    being seen for one character's time.
  */
  serial_putc(0xff);
  for (i = 0; i < len; ++i)
  {
    uint8_t c = buf[i];
    crc = crc16(c, crc);
      serial_putc(c);
  }
  /* Send the CRC and request end marker. */
  serial_putc(dec2hex(crc >> 12));
  serial_putc(dec2hex((uint8_t)(crc >> 8) & 0xf));
  serial_putc(dec2hex((uint8_t)(crc >> 4) & 0xf));
  serial_putc(dec2hex((uint8_t)crc & 0xf));
  serial_putc('\r');
  serial_putc('\n');
  serial_wait_for_tx_complete();
  rs485_receive_mode();
}


static void
device_discover(uint8_t id, uint8_t *buf)
{
  uint8_t idx, i;
  char tmp[20];

  for (i = 0; i < MAX_DEVICES; ++i)
  {
    if (!rs485_devices[i].description)
      break;
    if (rs485_devices[i].device_id == id)
    {
      idx = 0;
      sprintf(tmp, "!%02x:D%u|", id, rs485_devices[i].poll_interval);
      idx = append_to_buf(buf, idx, tmp);
      idx = quoted_append_to_buf(buf, idx, rs485_devices[i].description);
      idx = append_char_to_buf(buf, idx, '|');
      idx = quoted_append_to_buf(buf, idx, rs485_devices[i].unit);
      idx = append_char_to_buf(buf, idx, '|');
      send_reply(buf, idx);
      break;
    }
  }
}


static void
device_poll(uint8_t id, uint8_t *buf)
{
  uint8_t idx, i;
  char tmp[20];

  for (i = 0; i < MAX_DEVICES; ++i)
  {
    if (!rs485_devices[i].description)
      break;
    if (rs485_devices[i].device_id == id)
    {
      if (!rs485_devices[i].have_value)
        break;
      idx = 0;
      sprintf(tmp, "!%02x:P", id);
      idx = append_to_buf(buf, idx, tmp);
      dtostrf((double)rs485_devices[i].sensor_value, 1, 6, tmp);
      idx = quoted_append_to_buf(buf, idx, tmp);
      idx = append_char_to_buf(buf, idx, '|');
      send_reply(buf, idx);
      rs485_devices[i].have_value = 0;
      break;
    }
  }
}


/*
  Process a request.
  Request format:
    ?ii:D|cccc                 # Discovery request
    ?ii:P|cccc                 # Poll request
  Here, ii is two hex digits to identify the device. Only the device owning
  that ID may reply.
  cccc is the CRC16 (in hex) of the request up to and including the '|'.
*/
static void
process_req(uint8_t *req, uint8_t len)
{
  uint16_t calc_crc, rcv_crc;
  uint8_t rcv_id;

  if (len != 10)
    return;
  if (req[0] != '?' || req[5] != '|' || (req[4] != 'D' && req[4] != 'P'))
    return;
  calc_crc = crc16_buf(req, 6);
  rcv_crc = ((uint16_t)hex2dec(req[6]) << 12) |
    ((uint16_t)hex2dec(req[7]) << 8) |
    ((uint16_t)hex2dec(req[8]) << 4) |
    (uint16_t)hex2dec(req[9]);
  if (calc_crc != rcv_crc)
    return;
  rcv_id = (hex2dec(req[1]) << 4) | hex2dec(req[2]);
  if (req[4] == 'D')
    device_discover(rcv_id, req);
  else
    device_poll(rcv_id, req);
}


static uint8_t rcv_buf[MAX_REQ];
static uint8_t rcv_idx;

static void
process_received_char(uint8_t c)
{
  /* Initially, wait for start-of-request marker '?'. */
  if (rcv_idx == 0 && c != '?')
    return;
  if (rcv_idx >= MAX_REQ)
  {
    /* Too long request. */
    rcv_idx = 0;
    return;
  }
  /* CR before LF is useful for serial debugging, but is otherwise ignored. */
  if (c == '\r')
    return;
  /* A LF marks the end of the request. */
  if (c == '\n')
  {
    /*
      We have received a request.
      Process it, with interrupts enabled (but serial reception interrupt
      disabled), so that we do not block other interrupt processing during
      long serial transmission.
      Then reset the buffer, ready for the next request.
    */
    serial_interrupt_rx_disable();
    sei();
    process_req(rcv_buf, rcv_idx);
    cli();
    serial_interrupt_rx_enable();
    rcv_idx = 0;
    return;
  }
  /* Save the received byte in the buffer for later processing. */
  rcv_buf[rcv_idx++] = c;
}

ISR(USART_RX_vect)
{
  uint8_t c;

  c = serial_read();
  process_received_char(c);
}


void
labibus_init(uint8_t device_id, uint16_t poll_interval,
             const char *description, const char *unit)
{
  uint8_t i;

  if (!description)
    return;
  /* Disable interrupts while changing the device table. */
  cli();
  for (i = 0; i < MAX_DEVICES; ++i)
  {
    if (!rs485_devices[i].description ||
        rs485_devices[i].device_id == device_id)
    {
      rs485_devices[i].sensor_value = 0.0f;
      rs485_devices[i].poll_interval = poll_interval;
      rs485_devices[i].description = description;
      rs485_devices[i].unit = unit;
      rs485_devices[i].device_id = device_id;
      rs485_devices[i].have_value = 0;
      break;
    }
  }

  if (i == 0)
  {
    /* Setup the serial port and interrupt on the first call. */
    setup_serial();

    setup_rs485_pins();
    rs485_receive_mode();
  }
  sei();
}


void
labibus_set_sensor_value(uint8_t device_id, float value)
{
  uint8_t i;
  for (i = 0; i < MAX_DEVICES; ++i)
  {
    if (!rs485_devices[i].description)
      break;
    if (rs485_devices[i].device_id != device_id)
      continue;
    rs485_devices[i].sensor_value = value;
    rs485_devices[i].have_value = 1;
    return;
  }
}
