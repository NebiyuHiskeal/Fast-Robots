/****************************************************************
 * Example1_Basics.ino
 * ICM 20948 Arduino Library Demo
 * Use the default configuration to stream 9-axis IMU data
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <math.h>

//#define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 0
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  
  Wire.begin();
  Wire.setClock(400000);
  bool initialized = false;
  while( !initialized )
  {
    myICM.begin( Wire, AD0_VAL );
    Serial.print( F("Initialization of the sensor returned: ") );
    Serial.println( myICM.statusString() );
    if( myICM.status != ICM_20948_Stat_Ok ){
      Serial.println( "Trying again..." );
      delay(500);
    }else{
      initialized = true;
    }

    // blinks twice to show that it is finished setting up
    digitalWrite(LED_BUILTIN, HIGH);  
    delay(1000);                      
    digitalWrite(LED_BUILTIN, LOW);   
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH);  
    delay(1000);                      
    digitalWrite(LED_BUILTIN, LOW);   
    delay(1000);
  }


}

void loop() {
  // put your main code here, to run repeatedly:
  float x=0, y=0, z=0, pitch=0, roll=0;

  while(1)
  {
    if(myICM.dataReady())
    {
      myICM.getAGMT();
      x = myICM.accX();
      y = myICM.accY();
      z = myICM.accZ();
      pitch = atan2(x,z)*180/M_PI;
      roll = atan2(y,z)*180/M_PI;
      // Serial.print("X: ");
      // Serial.print(x);
      // Serial.print(" | Y: ");
      // Serial.print(y);
      // Serial.print(" | Z: ");
      // Serial.println(z);
      Serial.print("Pitch: ");
      Serial.println(" | Roll: ");
      Serial.print(pitch);
      Serial.print(", ");
      Serial.println(roll);
    }
  }

}
