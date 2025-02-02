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

  todo
  example set local ref pressure
  fix repo description
  example of reading/writing cal factors

*/

#include "Wire.h"

#include "SparkFunBME280.h"
BME280 mySensor; //Global sensor object

void setup()
{
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Reading basic values from BME280");
  mySensor.settings.I2CAddress = 0x76;  // Adafruit Sensor uses 0x77
  Wire.begin();

  if (mySensor.beginI2C() == false) //Begin communication over I2C
  {
    Serial.println("The chip did not respond. Please check wiring.");
    while(1); //Freeze
  }
}

void loop()
{
  Serial.print("Humidity: ");
  Serial.print(mySensor.readFloatHumidity(), 0);

  Serial.print(" Pressure: ");
  Serial.print(mySensor.readFloatPressure(), 0);

  Serial.print(" Alt: ");
  Serial.print(mySensor.readFloatAltitudeMeters(), 1); 
  //Serial.print(mySensor.readFloatAltitudeFeet(), 1);

  Serial.print(" Temp: ");
  Serial.print(mySensor.readTempC(), 2);
  //Serial.print(mySensor.readTempF(), 2);

  Serial.println();

  delay(50);
}
