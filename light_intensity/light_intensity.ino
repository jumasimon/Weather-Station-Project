/*
  Sample code for the BH1750 Light sensor

  Connection:
  VCC-5v
  GND-GND
  SCL-SCL(analog pin 5)
  SDA-SDA(analog pin 4)
  ADD-NC
*/
#include <OneWire.h>
#include <DallasTemperature.h>

#include <Wire.h> //BH1750 IIC Mode 
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

const int SENSOR_PIN = 13; // Arduino pin connected to DS18B20 sensor's DQ pin

float tempCelsius;    // temperature in Celsius
float tempFahrenheit; // temperature in Fahrenheit
int BH1750address = 0x23; //setting i2c address

byte buff[2];
OneWire oneWire(SENSOR_PIN);         // setup a oneWire instance
DallasTemperature tempSensor(&oneWire); // pass oneWire to DallasTemperature library
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

void displaySensorDetails(void)
{
  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
void setup()
{
  Wire.begin();
  Serial.begin(9600);//init Serial rate
  tempSensor.begin();    // initialize the sensor
  /* Initialise the sensor */
  if (!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();
}

void loop()
{
  int i;
  uint16_t val = 0;
  tempSensor.requestTemperatures();             // send the command to get temperatures
  tempCelsius = tempSensor.getTempCByIndex(0);  // read temperature in Celsius
  tempFahrenheit = tempCelsius * 9 / 5 + 32; // convert Celsius to Fahrenheit

  BH1750_Init(BH1750address);
  sensors_event_t event;
  bmp.getEvent(&event);
  delay(200);
  if (event.pressure)
  {
    /* Display atmospheric pressue in hPa */
    Serial.print("Pressure:    ");
    Serial.print(event.pressure);
    Serial.println(" hPa");
    float temperature;
    bmp.getTemperature(&temperature);
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    Serial.print("Altitude:    "); 
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                        event.pressure)); 
    Serial.println(" m");
    Serial.println("");
  }
  else
    {
    Serial.println("Sensor error");
  }
  if (2 == BH1750_Read(BH1750address))
  {
    val = ((buff[0] << 8) | buff[1]) / 1.2;
    Serial.print(val, DEC);
    Serial.println("[lx]");
  }
  Serial.print("Temperature: ");
  Serial.print(tempCelsius);    // print the temperature in Celsius
  Serial.print("°C");
  Serial.print("  ~  ");        // separator between Celsius and Fahrenheit
  Serial.print(tempFahrenheit); // print the temperature in Fahrenheit
  Serial.println("°F");
  delay(150);
}

int BH1750_Read(int address) //
{
  int i = 0;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 2);
  while (Wire.available()) //
  {
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  Wire.endTransmission();
  return i;
}

void BH1750_Init(int address)
{
  Wire.beginTransmission(address);
  Wire.write(0x10);//1lx reolution 120ms
  Wire.endTransmission();
}
