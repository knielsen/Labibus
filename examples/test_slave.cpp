#include <util/delay.h>

#include "Labibus.h"

int
main(int argc, char *argv[])
{
  float val1, val2;

  labibus_init( 9, 10, "Temperature room 2", "degree C");
  labibus_init(11, 60, "Humidity 2", "%rel");

  val1 = 0.0f;
  val2 = 10.0f;
  for (;;)
  {
    labibus_set_sensor_value(9, val1);
    labibus_set_sensor_value(11, val2);
    _delay_ms(5);
    val1 += 1.237f;
    val2 += 0.03f;
  }

  return 0;
}
