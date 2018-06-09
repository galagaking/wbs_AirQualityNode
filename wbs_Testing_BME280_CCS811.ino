/*
  Get basic environmental readings from the BME280
  By: Nathan Seidle
  SparkFun Electronics
  Date: March 9th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14348 - Qwiic Combo Board
  https://www.sparkfun.com/products/13676 - BME280 Breakout Board
  
  This example shows how to read humidity, pressure, and current temperature from the BME280 over I2C.

******************************************************************************
  BasicReadings.ino

  Marshall Taylor @ SparkFun Electronics
  Nathan Seidle @ SparkFun Electronics

  April 4, 2017

  https://github.com/sparkfun/CCS811_Air_Quality_Breakout
  https://github.com/sparkfun/SparkFun_CCS811_Arduino_Library

  Read the TVOC and CO2 values from the SparkFun CSS811 breakout board

  This is the simplest example.  It throws away most error information and
  runs at the default 1 sample per second.

  A new sensor requires at 48-burn in. Once burned in a sensor requires
  20 minutes of run in before readings are considered good.

  Hardware Connections (Breakoutboard to Arduino):
  3.3V to 3.3V pin
  GND to GND pin
  SDA to A4
  SCL to A5
  WAKE to 8 (or connect to ground)
  INT to 3 (Not used in this example)

  Development environment specifics:
  Arduino IDE 1.8.1

  This code is released under the [MIT License](http://opensource.org/licenses/MIT).

  Please review the LICENSE.md file included with this example. If you have any questions
  or concerns with licensing, please contact techsupport@sparkfun.com.

  Distributed as-is; no warranty is given.
******************************************************************************/
/* merged BME280 and CCS811 examples from SparkFun libraries for 'well being solution workshop'
 *  IoT Meetup Eindhoven June 2018
 *  Frank Beks
 */
#include "SparkFunBME280.h"
#include "SparkFunCCS811.h"

//#define CCS811_ADDR 0x5B //Default I2C Address
#define CCS811_ADDR 0x5A //Alternate I2C Address
#define PIN_NOT_WAKE 8    // must be 0 to Wake up the sensor
#define PIN_NOT_INT 3     // not used in this example

#define BME280_ADDR 0x76    //Sparkfun
//#define BME280_ADDR 0x77  //Alternate I2C Address (Adafruit board)

CCS811 Sensor811(CCS811_ADDR);
BME280 Sensor280; //Global sensor object

void setup()
{
  Serial.begin(9600);
  while(!Serial);
  Serial.println();
  Serial.println("Reading basic values from BME280 and CCS811");
  Serial.println();
  pinMode(PIN_NOT_INT, INPUT_PULLUP);
  pinMode(PIN_NOT_WAKE, OUTPUT);
  digitalWrite(PIN_NOT_WAKE, 0); //Start awake

  //It is recommended to check return status on .begin(), but it is not
  //required.
  CCS811Core::status returnCode = Sensor811.begin();
  if (returnCode != CCS811Core::SENSOR_SUCCESS)
  {
    Serial.println("CCS811 not found, check wiring and address of CCS811");
    while (1); //Hang if there was a problem.
  }

  Sensor280.settings.I2CAddress = BME280_ADDR;  // Adafruit BME280 Sensor uses 0x77

  if (Sensor280.beginI2C() == false) //Begin communication over I2C
  {
    Serial.println("BME280 not found, check wiring and I2C Address.");
    while(1); //Freeze
  }
}

void loop()
{
  if (Sensor811.dataAvailable())
  {
    //If so, have the sensor read and calculate the results.
    //Get them later
    Sensor811.readAlgorithmResults();

    Serial.print("CO2[");
    //Returns calculated CO2 reading
    Serial.print(Sensor811.getCO2());
    Serial.print("] tVOC[");
    //Returns calculated TVOC reading
    Serial.print(Sensor811.getTVOC());
    Serial.print("] Humidity[");
    Serial.print(Sensor280.readFloatHumidity(), 0);
    Serial.print("] Pressure [");
    Serial.print(Sensor280.readFloatPressure(), 0);
    Serial.print("] Alt [");
    Serial.print(Sensor280.readFloatAltitudeMeters(), 1); 
    Serial.print("] Temp [");
    Serial.print(Sensor280.readTempC(), 2);
    Serial.print("] Millis [");
    //Simply the time since program start
    Serial.print(millis());
    Serial.print("]");
    Serial.println();
  }
  delay(50);
}
