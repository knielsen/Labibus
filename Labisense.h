/*
  Arduino library for Labisense RS485 network.
*/

#define PIN_DE 7
#define PIN_RE 6
/* RO er 0, DI er 1. */

#define MAX_DEVICES 3

#define MAX_DESCRIPTION 140
#define MAX_UNIT 20
#define MAX_REQ (20+MAX_DESCRIPTION+MAX_UNIT)


/*
  Configure a new device as an RS485 sensor.

  The device_id is from 0 to 127. It must be allocated to not conflict with
  any other sensor on the same network.

  The poll_interval is how often the sensor value should be requested by the
  master, in seconds.

  The description and unit must be non-NULL strings, they describe the meaning
  of the sensor and its unit (eg. "degree C" for a temperature sensor). The
  strings must be valid for the duration of the program, normally just a
  string litteral will be passed.

  Once configured, a serial receive interrupt will listen for requests from
  the master, and reply with latest data set with rs485_set_sensor_value(),
  if any.
*/
extern void rs485_init(uint8_t device_id, uint16_t poll_interval,
                       const char *description, const char *unit);

/*
  Supply a sensor value for the given device.

  The value will be sent to the master on the next poll request, the interval
  of which was set in rs485_init().

  This function can be called as often as desired; the latest value will be
  used when the master request the value. If it is called less often than the
  poll interval, then polls will be skipped (at most one value will be sent
  per call to rs485_set_sensor_value()).
*/
extern void rs485_set_sensor_value(uint8_t device_id, float value);
