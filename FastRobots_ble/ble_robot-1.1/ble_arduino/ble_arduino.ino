
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include "SparkFun_VL53L1X.h"

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "ce469c59-68b8-434c-8476-075d53ab2a68"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
//////////// Global Variables ////////////

#define SHUTDOWN_PIN A2
#define INTERRUPT_PIN 3

//Uncomment the following line to use the optional shutdown and interrupt pins.
SFEVL53L1X distanceSensor;
SFEVL53L1X distanceSensor2(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
// SFEVL53L1X distanceSensor2;

#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include<math.h>

#define SERIAL_PORT Serial
#define AD0_VAL   0     // The value of the last bit of the I2C address, on the SparkFun 9DoF IMU breakout the default is 1
#define ARRAY_LEN 1500


ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object

unsigned long currentMillis = 0;
float pitch[ARRAY_LEN];
float roll[ARRAY_LEN];
float yaw[ARRAY_LEN];
float lp_pitch[ARRAY_LEN];
float lp_roll[ARRAY_LEN];
float stamps[ARRAY_LEN];
float gpitch[ARRAY_LEN];
float groll[ARRAY_LEN];
float gyaw[ARRAY_LEN];
int distances_a[51];
int distances_b[51];
int stored_times = 0;
float dt = 0;
float start = 0;
bool record = false;
int stored = 0;
int distance = 0;
///////////// Accelerometer //////////////

enum CommandTypes
{
    LP_ACCEL_READINGS,
    GYRO_READINGS,
    BOTH_READ,
    COMPLEMENT_READ,
    READ_DISTANCE,
    READ_DISTANCE_2,
    ASAP,
};

void
handle_command()
{   
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

    // Get robot command type (an integer)
    /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     * since it uses strtok internally (refer RobotCommand.h and 
     * https://www.cplusplus.com/reference/cstring/strtok/)
     */
    success = robot_cmd.get_command_type(cmd_type);

    // Check if the last tokenization was successful and return if failed
    if (!success) {
        return;
    }

    // Handle the command type accordingly
    switch (cmd_type) {
        /*
         * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
         */
        case LP_ACCEL_READINGS:
            stored_times=0;
            float a;

            while(stored_times<ARRAY_LEN){
              if(myICM.dataReady()){
                myICM.getAGMT();
                stamps[stored_times] = (float)millis()/1000.;
                if(stored_times==0){
                  pitch[stored_times] = atan2(myICM.accX(),myICM.accZ())*180/M_PI;
                  roll[stored_times] = atan2(myICM.accY(),myICM.accZ())*180/M_PI;
                  lp_pitch[stored_times] = atan2(myICM.accX(),myICM.accZ())*180/M_PI;
                  lp_roll[stored_times] = atan2(myICM.accY(),myICM.accZ())*180/M_PI;
                }
                else{
                  a = (stamps[stored_times]-stamps[stored_times-1]);
                  a = a/(a+(0.015915*2));
                  lp_pitch[stored_times] = (lp_pitch[stored_times-1]*(1-a)) + (a * (atan2(myICM.accX(),myICM.accZ())*180/M_PI));
                  lp_roll[stored_times] = (lp_roll[stored_times-1]*(1-a)) + (a * (atan2(myICM.accY(),myICM.accZ())*180/M_PI));
                  pitch[stored_times] = atan2(myICM.accX(),myICM.accZ())*180/M_PI;
                  roll[stored_times] = atan2(myICM.accY(),myICM.accZ())*180/M_PI;
                }
                stored_times += 1;
                delay(10);
              }
            }
            for(int i=0; i<ARRAY_LEN; i++){
                tx_estring_value.clear();
                tx_estring_value.append("LPAcc");
                tx_estring_value.append(pitch[i]);
                tx_estring_value.append(" ");
                tx_estring_value.append(roll[i]);
                tx_estring_value.append(" ");
                tx_estring_value.append(lp_pitch[i]);
                tx_estring_value.append(" ");
                tx_estring_value.append(lp_roll[i]);
                tx_estring_value.append(" ");
                tx_estring_value.append(stamps[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            break;

        case BOTH_READ:
            stored_times=0;

            while(stored_times<ARRAY_LEN){
              if(myICM.dataReady()){
                stamps[stored_times] = (float)millis()/1000.;
                myICM.getAGMT();
                if (stored_times > 0){
                  dt = stamps[stored_times] - stamps[stored_times-1];
                  gpitch[stored_times] = gpitch[stored_times-1] + myICM.gyrX()*dt;
                  groll[stored_times] = groll[stored_times-1] + myICM.gyrY()*dt;
                  gyaw[stored_times] = gyaw[stored_times-1] + myICM.gyrZ()*dt;

                  a = (stamps[stored_times]-stamps[stored_times-1]) / 1000;
                  a = a/(a+(0.015915*2));
                  lp_pitch[stored_times] = (lp_pitch[stored_times-1]*(1-a)) + (a * (atan2(myICM.accX(),myICM.accZ())*180/M_PI));
                  lp_roll[stored_times] = (lp_roll[stored_times-1]*(1-a)) + (a * (atan2(myICM.accY(),myICM.accZ())*180/M_PI));
                }
                else{
                  dt = stamps[stored_times];
                  gpitch[stored_times] = myICM.gyrX()*dt;
                  groll[stored_times] = myICM.gyrY()*dt;
                  gyaw[stored_times] = myICM.gyrZ()*dt;
                }
                stored_times += 1;
                delay(10);
              }
            }
            for(int i=0; i<ARRAY_LEN; i++){
                tx_estring_value.clear();
                tx_estring_value.append("both");
                tx_estring_value.append(gpitch[i]);
                tx_estring_value.append(" ");
                tx_estring_value.append(groll[i]);
                tx_estring_value.append(" ");
                tx_estring_value.append(gyaw[i]);
                tx_estring_value.append(" ");
                tx_estring_value.append(lp_pitch[i]);
                tx_estring_value.append(" ");
                tx_estring_value.append(lp_roll[i]);
                tx_estring_value.append(" ");
                tx_estring_value.append(stamps[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            break;


        case GYRO_READINGS:
            stored_times=0;

            while(stored_times<ARRAY_LEN){
              if(myICM.dataReady()){
                stamps[stored_times] = (float)millis()/1000.;
                myICM.getAGMT();
                if (stored_times > 0){
                  dt = stamps[stored_times] - stamps[stored_times-1];
                  gpitch[stored_times] = gpitch[stored_times-1] + myICM.gyrX()*dt;
                  groll[stored_times] = groll[stored_times-1] + myICM.gyrY()*dt;
                  gyaw[stored_times] = gyaw[stored_times-1] + myICM.gyrZ()*dt;
                }
                else{
                  dt = stamps[stored_times];
                  gpitch[stored_times] = myICM.gyrX()*dt;
                  groll[stored_times] = myICM.gyrY()*dt;
                  gyaw[stored_times] = myICM.gyrZ()*dt;
                }
                stored_times += 1;
                delay(10);
              }
            }
            for(int i=0; i<ARRAY_LEN; i++){
                tx_estring_value.clear();
                tx_estring_value.append("Gyr");
                tx_estring_value.append(gpitch[i]);
                tx_estring_value.append(" ");
                tx_estring_value.append(groll[i]);
                tx_estring_value.append(" ");
                tx_estring_value.append(gyaw[i]);
                tx_estring_value.append(" ");
                tx_estring_value.append(stamps[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            break;

        case COMPLEMENT_READ:
            stored_times=0;
            float alpha;
            alpha = 0.3;
            while(stored_times<ARRAY_LEN){
              if(myICM.dataReady()){
                stamps[stored_times] = (float)millis()/1000.;
                myICM.getAGMT();
                if (stored_times > 0){
                  dt = stamps[stored_times] - stamps[stored_times-1];
                  lp_pitch[stored_times] =  myICM.gyrX()*dt;
                  lp_roll[stored_times] = myICM.gyrY()*dt;
                  gyaw[stored_times] = myICM.gyrZ()*dt;
                  gpitch[stored_times] = pitch[stored_times-1] + lp_pitch[stored_times];
                  groll[stored_times] = roll[stored_times-1] + lp_roll[stored_times];
                  yaw[stored_times] = yaw[stored_times-1] + gyaw[stored_times];
                  lp_pitch[stored_times] += lp_pitch[stored_times-1];
                  lp_roll[stored_times] += lp_roll[stored_times-1];
                  gyaw[stored_times] += gyaw[stored_times-1];
                }
                else{
                  dt = .01;
                  lp_pitch[0] =  myICM.gyrX()*dt;
                  lp_roll[0] = myICM.gyrY()*dt;
                  gyaw[0] = myICM.gyrZ()*dt;
                  gpitch[0] = lp_pitch[0];
                  groll[0] = lp_roll[0];
                  yaw[0] = gyaw[0];
                }
                pitch[stored_times] =(atan2(myICM.accX(),myICM.accZ())*180/M_PI);
                roll[stored_times] = (atan2(myICM.accY(),myICM.accZ())*180/M_PI);
                pitch[stored_times] = ((1-alpha)*gpitch[stored_times]) + (alpha*pitch[stored_times]);
                roll[stored_times] = ((1-alpha)*groll[stored_times]) + (alpha*roll[stored_times]);
                stored_times += 1;
                delay(10);
              }
            }
            for(int i=0; i<ARRAY_LEN; i++){
                tx_estring_value.clear();
                tx_estring_value.append("Comp");
                tx_estring_value.append(pitch[i]);
                tx_estring_value.append(" ");
                tx_estring_value.append(roll[i]);
                tx_estring_value.append(" ");
                tx_estring_value.append(yaw[i]);
                tx_estring_value.append(" ");
                tx_estring_value.append(lp_pitch[i]);
                tx_estring_value.append(" ");
                tx_estring_value.append(lp_roll[i]);
                tx_estring_value.append(" ");
                tx_estring_value.append(gyaw[i]);
                tx_estring_value.append(" ");
                tx_estring_value.append(stamps[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            break;

        case READ_DISTANCE:
            distance = 0;
            for (int i = 0; i < 10; i++){
              distanceSensor2.startRanging(); //Write configuration bytes to initiate measurement
              while (!distanceSensor2.checkForDataReady())
              {
                delay(1);
              }
              distance = distanceSensor2.getDistance(); //Get the result of the measurement from the sensor
              distanceSensor2.clearInterrupt();
              distanceSensor2.stopRanging();

              tx_estring_value.clear();
              tx_estring_value.append("TOF");
              tx_estring_value.append(distance);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            break;

        case READ_DISTANCE_2:
            Serial.println("starting distance2");
            distance = 0;
            start = millis();
            stored = 0;
            distanceSensor2.startRanging(); //Write configuration bytes to initiate measurement
            distanceSensor.startRanging();
            while (stored < 51){
              if(distanceSensor2.checkForDataReady() && distanceSensor.checkForDataReady()) {
                distances_a[stored] = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
                distances_b[stored] = distanceSensor2.getDistance();
                stamps[stored] = millis();

                distanceSensor2.clearInterrupt();
                distanceSensor.clearInterrupt();
                // Serial.println(stored);
                stored += 1;
              }
            }
            distanceSensor2.stopRanging(); //Write configuration bytes to initiate measurement
            distanceSensor.stopRanging();
            for (int i = 0; i < stored; i++){
              tx_estring_value.clear();
              tx_estring_value.append("TOF2");
              tx_estring_value.append(distances_a[i]);
              tx_estring_value.append(" ");
              tx_estring_value.append(distances_b[i]);
              tx_estring_value.append(" ");
              tx_estring_value.append(stamps[i]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            Serial.println("finished distance2");
            break;

        case ASAP:
            distanceSensor.startRanging();
            distanceSensor2.startRanging();
            start = millis();
            while(millis() - start < 5000){
              if(distanceSensor2.checkForDataReady() && distanceSensor.checkForDataReady()){
                Serial.println(millis());
                // Serial.print("Distance1: ");
                // Serial.print(distanceSensor.getDistance());
                // Serial.println(" mm");
                // Serial.print("Distance2: ");
                // Serial.print(distanceSensor2.getDistance());
                // Serial.println(" mm");
                distanceSensor2.clearInterrupt();
                distanceSensor.clearInterrupt();
              }
            }
            break;

        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
    }
}

void
setup()
{
    Serial.begin(115200);
    record = true;

    BLE.begin();

    // Set advertised local name and service
    BLE.setDeviceName("Artemis BLE");
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);

    // Add BLE characteristics
    testService.addCharacteristic(tx_characteristic_float);
    testService.addCharacteristic(tx_characteristic_string);
    testService.addCharacteristic(rx_characteristic_string);

    // Add BLE service
    BLE.addService(testService);

    // Initial values for characteristics
    // Set initial values to prevent errors when reading for the first time on central devices
    tx_characteristic_float.writeValue(0.0);

    /*
     * An example using the EString
     */
    // Clear the contents of the EString before using it
    tx_estring_value.clear();

    // Append the string literal "[->"
    tx_estring_value.append("[->");

    // Append the float value
    tx_estring_value.append(9.0);

    // Append the string literal "<-]"
    tx_estring_value.append("<-]");

    // Write the value to the characteristic
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    // Output MAC Address
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    BLE.advertise();

    Wire.begin();
    Wire.setClock(400000);
    // bool initialized = false;
    // while( !initialized )
    // {
    //   myICM.begin( Wire, AD0_VAL );
    //   Serial.print( F("Initialization of the sensor returned: ") );
    //   Serial.println( myICM.statusString() );
    //   if( myICM.status != ICM_20948_Stat_Ok ){
    //     Serial.println( "Trying again..." );
    //     delay(500);
    //   }else{
    //     initialized = true;
    //   }
    // }
    pinMode(SHUTDOWN_PIN, OUTPUT);
    digitalWrite(SHUTDOWN_PIN, LOW);
    distanceSensor.setI2CAddress(80);
    if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
    {
      Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
      while (1)
        ;
    }
    Serial.println("Sensor 1 online!");
    digitalWrite(SHUTDOWN_PIN, HIGH);
    if (distanceSensor2.begin() != 0) //Begin returns 0 on a good init
    {
      Serial.println("Sensor 2 failed to begin. Please check wiring. Freezing...");
      while (1)
        ;
    }
    distanceSensor2.setDistanceModeShort();
    distanceSensor.setDistanceModeShort();
  Serial.println("Sensor 2 online!");
}

void
write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000) {
            tx_float_value = 0;
            
        }

        previousMillis = currentMillis;
    }

}

void
read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}

void
loop()
{
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());
        record = true;

        // While central is connected
        while (central.connected()) {
            // Send data
            write_data();

            // Read data
            read_data();
        }

        Serial.println("Disconnected");
    }
}
