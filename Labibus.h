#include <stdint.h>

/*
  Arduino library for Labisense RS485 network.
*/

#define PIN_DE 7
#define PIN_RE 6
/* RO er 0, DI er 1. */

#define MAX_DEVICES 10

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
  the master, and reply with latest data set with labibus_set_sensor_value(),
  if any.
*/
extern void labibus_init(uint8_t device_id, uint16_t poll_interval,
                         const char *description, const char *unit);


/*
  Supply a sensor value for the given device.

  The value will be sent to the master on the next poll request, the interval
  of which was set in labibus_init().

  This function can be called as often as desired; the latest value will be
  used when the master request the value. If it is called less often than the
  poll interval, then polls will be skipped (at most one value will be sent
  per call to labibus_set_sensor_value()).
*/
extern void labibus_set_sensor_value(uint8_t device_id, float value);

/*
  Wait for the master device to poll this slave device for its sensor value.

  This function will wait until the current sensor value, set by
  labibus_set_sensor_value(), has been sent to the master. If the last set
  value was already sent (or if no value was sent), then the function returns
  immediately.

  Can be used to work-around poorly designed sensor libraries that do not work
  well with other concurrent activity in interrupt routines. One such example
  is the DHT11/DHT22 Arduino library, which relies on exact timing of delay
  loops, and thus needs interrupts disabled while reading the sensor to not
  get errors due to incorrect timing.

  Calling this function before disabling the interrupts and reading the sensor
  can help avoid that the master poll arrives while interrupts are disabled
  (since the communication with the master is handled in the serial interrupt,
  a master poll request will get lost if it arrives while interrupts are
  disabled for long).

  Note that this function temporarily enables interrupts for the duration of
  the call, if they were disabled upon entry.
*/
extern void labibus_wait_for_poll(uint8_t device_id);

/*
  Check if the master device has polled the slave device for its sensor value.

  This function will return true if the current sensor value, set by
  labibus_set_sensor_value(), has been sent to the master (or if no value was
  ever set). It returns false if a sensor value, set by
  labibus_set_sensor_value(), is still waiting to be sent to the master.
*/
extern bool labibus_check_for_poll(uint8_t device_id);

/*
  Listen for activity from another device on the bus. After calling this
  function, labibus_check_data() and labibus_get_data() can be used to access
  values reported by another device.
*/
extern void labibus_listen(uint8_t device_id);

/*
  Check if any new data is available from a device that was previously
  configured for listening with labibus_listen(). The function will return
  true if new data has been seen on the bus from this device since last
  call to labibus_get_data(), false otherwise.
*/
extern bool labibus_check_data(uint8_t device_id);
/*
  Get the latest data seen from another device previously configured for
  listening to with labibus_listen().
*/
extern float labibus_get_data(uint8_t device_id);
