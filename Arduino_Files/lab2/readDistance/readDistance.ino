/*
  Reading distance from the laser based VL53L1X
  By: Nathan Seidle
  SparkFun Electronics
  Date: April 4th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14667

  This example prints the distance to an object.

  Are you getting weird readings? Be sure the vacuum tape has been removed from the sensor.
*/

#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN A2
#define INTERRUPT_PIN 3

// SFEVL53L1X distanceSensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
SFEVL53L1X distanceSensor;
SFEVL53L1X distanceSensor2(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

void setup(void)
{
  Wire.begin();

  Serial.begin(115200);
  pinMode(SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_PIN, LOW);
  Serial.println("VL53L1X Qwiic Test");
  Serial.print("Address 1 (start): ");
  Serial.println(distanceSensor.getI2CAddress());
  distanceSensor.setI2CAddress(80);

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor 1 online!");
  digitalWrite(SHUTDOWN_PIN, HIGH);
  Serial.print("Address 1 (end): ");
  Serial.println(distanceSensor.getI2CAddress());
  Serial.print("Address 2: ");
  Serial.println(distanceSensor2.getI2CAddress());
  if (distanceSensor2.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor 2 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  distanceSensor.setDistanceModeLong();
  Serial.println("Sensor 2 online!");
}

void loop(void)
{
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();

  float distanceInches= distance * 0.0393701;
  float distanceFeet = distanceInches / 12.0;
  // Serial.print("Distance1(ft): ");
  Serial.print(distanceFeet);
  // Serial.print("ft ");
  // Serial.print(distanceInches);
  // Serial.print("in");

  // distanceSensor2.startRanging(); //Write configuration bytes to initiate measurement
  // while (!distanceSensor2.checkForDataReady())
  // {
  //   delay(1);
  // }
  // float distance2 = distanceSensor2.getDistance(); //Get the result of the measurement from the sensor
  // distanceSensor2.clearInterrupt();
  // distanceSensor2.stopRanging();

  // float distanceInches2= distance2 * 0.0393701;
  // float distanceFeet2 = distanceInches2 / 12.0;
  // Serial.print("\tDistance2(ft): ");
  // Serial.print(distanceFeet2);
  // Serial.print("ft ");
  // Serial.print(distanceInches2);
  // Serial.print("in");

  Serial.println();
}
