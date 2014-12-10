#include <Labibus.h>

#include "DHT.h"

#define DHTPIN 12     // what pin we're connected to

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11 
//#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  //Serial.begin(9600); 
  labibus_init(40, 10, "Temperature in room 4", "degree C");
 
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  digitalWrite(10,LOW); //GND PIN
  digitalWrite(11,HIGH); //VCC PIN
  
  dht.begin();
}

void loop() {
  delay(2000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit
  float f = dht.readTemperature(true);
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    //Serial.println("Failed to read from DHT sensor!");
    return;
  }

  //Serial.print("Humidity: "); 
  //Serial.print(h);
  //Serial.print(" %\t");
  //Serial.print("Temperature: "); 
  //Serial.print(t);
  //Serial.println(" *C ");
  labibus_set_sensor_value(40, t);
}

