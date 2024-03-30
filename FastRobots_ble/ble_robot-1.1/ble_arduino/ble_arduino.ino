
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
float previousMillis = 0;
//////////// Global Variables ////////////

#define SHUTDOWN_PIN A2
#define INTERRUPT_PIN 3
#define RIGHT_F_PIN A1
#define RIGHT_B_PIN A0
#define LEFT_F_PIN A14
#define LEFT_B_PIN A16


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

float currentMillis = 0;
float recordMillis = 0;
float stamps[ARRAY_LEN];
float gpitch[ARRAY_LEN];
float groll[ARRAY_LEN];
float gyaw[ARRAY_LEN];
float distances_a[ARRAY_LEN];
float distances_b[ARRAY_LEN];
float kps[ARRAY_LEN];
float kis[ARRAY_LEN];
float kds[ARRAY_LEN];
float fl[ARRAY_LEN];
float bl[ARRAY_LEN];
float fr[ARRAY_LEN];
float br[ARRAY_LEN];
float speeds[ARRAY_LEN];
int stored_times = 0;
float dt = 0;
float initial = 0;
float start;
float yaw = 0;
float alpha = 0.3;
float kp = 2;
float ki = .5;
float kd = .5;
float P = 0;
float I = 0;
float D = 0;
float u = 0;
int goal = 0;
float curr_err = 0;
float prev_err = 0;
float total_err = 0;
int stored = 0;
float distance = 0;
float prev_distance = 0;
bool distance1 = false;
bool distance2 = false;
bool icm = false;
bool icm_measured = false;
bool dist_measured = false;
bool record = false;
bool sending = false;
bool pid = false;
float ldrive = 0;
int rmin = 40;
int lmin = 40;
float dspeed = 0;
bool arrived = false;
///////////// Accelerometer //////////////

enum CommandTypes
{
    LOG,
    ICM,
    DISTANCE1,
    DISTANCE2,
    PID,
    STOP,
    GAINS,
    DRIVE,
    RLIMIT,
    LLIMIT,
    UNTIL,
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
        case LOG:
            sending = false;
            record = true;
            break;

        case ICM:
            icm = true;
            break;

        case DISTANCE1:
            distance1 = true;
            distanceSensor.startRanging();

            break;

        case DISTANCE2:
            distance2 = true;
            break;

        case PID:
            int temp;
            if(robot_cmd.get_next_value(temp)){
              goal = temp;
            }
            goal = goal % 360;
            pid = true;
            currentMillis = millis();
            curr_err = 0;
            u = 0;
            total_err = 0;
            icm_measured = false;
            dist_measured = false;
            break;

        case STOP:
            pid = false;
            icm_measured = false;
            dist_measured = false;
            analogWrite(RIGHT_B_PIN, 0);
            analogWrite(LEFT_B_PIN, 0);
            analogWrite(LEFT_F_PIN, 0);
            analogWrite(RIGHT_F_PIN, 0);
            distanceSensor.stopRanging();
            if(record){
              record = false;
              sending = true;
            }

            break;
        
        case GAINS:
            float gain;
            if (robot_cmd.get_next_value(gain)) {
              kp = gain;
            }
            if (robot_cmd.get_next_value(gain)) {
              ki = gain;
            }
            if (robot_cmd.get_next_value(gain)) {
              kd = gain;
            }
            break;

        case DRIVE:
            robot_cmd.get_next_value(dspeed);
            dspeed = dspeed/100.0;
            Serial.println(dspeed);
            if(dspeed<0){
              dspeed = (-235.0*dspeed) + 20.0;
              analogWrite(RIGHT_F_PIN, 0);
              analogWrite(LEFT_F_PIN, 0);
              analogWrite(RIGHT_B_PIN, dspeed*0.3);
              analogWrite(LEFT_B_PIN, dspeed);
            }
            else{
              dspeed = (165.0*dspeed) + 90.0;
              analogWrite(RIGHT_F_PIN, dspeed*0.5);
              analogWrite(LEFT_F_PIN, dspeed);
              analogWrite(RIGHT_B_PIN, 0);
              analogWrite(LEFT_B_PIN, 0);
            }
            start = (float)millis();
            while ((float)millis() - start < 2000) {
              delay(10);
            }
            analogWrite(RIGHT_F_PIN, 0);
            analogWrite(LEFT_F_PIN, 0);
            analogWrite(RIGHT_B_PIN, 0);
            analogWrite(LEFT_B_PIN, 0);
            break;
        
        case RLIMIT:
          robot_cmd.get_next_value(rmin);
          break;
        
        case LLIMIT:
          robot_cmd.get_next_value(lmin);
          break;
        
        case UNTIL:
            // Serial.println(distanceSensor.getDistanceMode());
            arrived = false;
            stored = 0;
            robot_cmd.get_next_value(goal);
            robot_cmd.get_next_value(dspeed);

            dspeed = dspeed / 100;
            dspeed = (215.0*dspeed) + 40.0;
            analogWrite(RIGHT_F_PIN, dspeed*0.5);
            analogWrite(LEFT_F_PIN, dspeed);
            analogWrite(RIGHT_B_PIN, 0);
            analogWrite(LEFT_B_PIN, 0);
            distanceSensor.startRanging();

            start = (float)millis();
            while (!arrived) {
              if (distanceSensor.checkForDataReady()){
                distance = distanceSensor.getDistance() * 0.0393701 / 12.0;
                distanceSensor.clearInterrupt();
                distances_a[stored] = distance;
                stamps[stored] = millis();
                fl[stored] = dspeed;
                fr[stored] = dspeed*0.5;
                stored += 1;
                if (distance - goal < .5) {
                  arrived = true;
                }
              }
            }
            analogWrite(RIGHT_F_PIN, 255);
            analogWrite(LEFT_F_PIN, 255);
            analogWrite(RIGHT_B_PIN, 255);
            analogWrite(LEFT_B_PIN, 255);
            delay(1000);
            analogWrite(RIGHT_F_PIN, 0);
            analogWrite(LEFT_F_PIN, 0);
            analogWrite(RIGHT_B_PIN, 0);
            analogWrite(LEFT_B_PIN, 0);
            while(stored > 0) {
              tx_estring_value.clear();
              tx_estring_value.append("M");
              tx_estring_value.append(distances_a[stored - 1]);
              tx_estring_value.append(" ");
              tx_estring_value.append(fl[stored - 1]);
              tx_estring_value.append(" ");
              tx_estring_value.append(fr[stored - 1]);
              tx_estring_value.append(" ");
              tx_estring_value.append(stamps[stored - 1]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
              stored = stored - 1;
            }

            break;

        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
    }
}

void
Log()
{
  // Serial.println("Recording");
  if (stored == 0 || (stored < ARRAY_LEN && currentMillis - recordMillis > 10)) {
    // Serial.print("Stored: ");
    // Serial.println(stored);
    // Serial.print("Current : ");
    // Serial.println(currentMillis);
    // Serial.print("Last: ");
    // Serial.println(recordMillis);
    stamps[stored] = currentMillis;
    recordMillis = currentMillis;
    // distanceSensor2.startRanging();
    // if (distance2){
    //   while(!distanceSensor2.checkForDataReady()) {
    //     delay(1);
    //   }
    //   distances_b[stored] = distanceSensor2.getDistance() * 0.0393701 / 12.0;
    //   distanceSensor2.clearInterrupt();
    // }
    // else {
    //   distances_b[stored] = 0;
    // }
    // distanceSensor2.stopRanging();
    if(distance1){
      distances_a[stored] = distance;
    }
    if(icm){
      gyaw[stored] = yaw;
      // pitch[stored] =(atan2(myICM.accX(),myICM.accZ())*180/M_PI);
      // roll[stored] = (atan2(myICM.accY(),myICM.accZ())*180/M_PI);
      // pitch[stored] = ((1-alpha)*gpitch[stored]) + (alpha*pitch[stored]);
      // roll[stored] = ((1-alpha)*groll[stored]) + (alpha*roll[stored]);
    }
    kps[stored] = P;
    kis[stored] = I;
    kds[stored] = D;
    speeds[stored] = ldrive;
    stored += 1;
  }
  if(stored >= ARRAY_LEN) {
    if(distance1) {
      distanceSensor.stopRanging();
    }
    record = false;
    sending = true;
  }
}

void
send()
{
  if(stored > 0) {
    // Serial.print("Sending: ");
    // Serial.println(stored);
    tx_estring_value.clear();
    tx_estring_value.append("S");
    // tx_estring_value.append(gpitch[i]);
    // tx_estring_value.append(" ");
    // tx_estring_value.append(groll[i]);
    // tx_estring_value.append(" ");
    tx_estring_value.append(gyaw[stored - 1]);
    tx_estring_value.append(" ");
    tx_estring_value.append(distances_a[stored - 1]);
    tx_estring_value.append(" ");
    // tx_estring_value.append(distances_b[i]);
    // tx_estring_value.append(" ");
    tx_estring_value.append(kps[stored - 1]);
    tx_estring_value.append(" ");
    tx_estring_value.append(kis[stored - 1]);
    tx_estring_value.append(" ");
    tx_estring_value.append(kds[stored - 1]);
    tx_estring_value.append(" ");
    tx_estring_value.append(speeds[stored - 1]);
    tx_estring_value.append(" ");
    tx_estring_value.append(stamps[stored - 1]);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    stored = stored - 1;
  }

  if(stored == 0) {
    // Serial.println("Finished sending");
    sending = false;
    icm = false;
    distance1 = false;
  }
}

void
setup()
{
    Serial.begin(115200);

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
    }
    ICM_20948_fss_t myFSS;
    myFSS.g = dps2000;
    myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS) ;
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
  distanceSensor2.setDistanceModeLong();
  distanceSensor.setDistanceModeLong();
  // distanceSensor.setTimingBudgetInMs(15);
  // Serial.println(distanceSensor.getTimingBudgetInMs());
  Serial.println("Sensor 2 online!");

  pinMode(RIGHT_F_PIN, OUTPUT);
  pinMode(RIGHT_B_PIN, OUTPUT);
  pinMode(LEFT_F_PIN, OUTPUT);
  pinMode(LEFT_B_PIN, OUTPUT);
  analogWrite(RIGHT_F_PIN, 0);
  analogWrite(RIGHT_B_PIN, 0);
  analogWrite(LEFT_F_PIN, 0);
  analogWrite(LEFT_B_PIN, 0);
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
Orient()
{
  if (myICM.dataReady()){
    myICM.getAGMT();
    previousMillis = currentMillis;
    currentMillis = millis();
    dt = (currentMillis - previousMillis) / 1000;

    if(!icm_measured){
      yaw = myICM.gyrZ()*dt;
      goal = yaw + goal;
      icm_measured = true;
    }
    else {
      yaw += myICM.gyrZ()*dt;
    }

    prev_err = curr_err;
    curr_err = yaw - goal;

    total_err += curr_err*dt;

    P = kp * curr_err;
    if (P > 200) {
      P = 200;
    }
    if (P < -200) {
      P = -200;
    }

    I = ki * total_err;
    if (I > 200) {
      I = 200;
    }
    if (I < -200) {
      I = -200;
    }

    D = kd * (curr_err - prev_err)/dt;
    if (D > 200) {
      D = 200;
    }
    if (D < -200) {
      D = -200;
    }
    u = P + I + D;
    if (u > 200) {
      u = 200;
    }
    if (u < -200) {
      u = -200;
    }
    if (record) {
      Log();
    }
  }


  ldrive = (int)((u/200)*(255 - lmin));
  if (u < 5 && u > -5) {
    analogWrite(RIGHT_B_PIN, 0);
    analogWrite(LEFT_B_PIN, 0);
    analogWrite(LEFT_F_PIN, 0);
    analogWrite(RIGHT_F_PIN, 0);
  }
  else if (u < 0) {
    analogWrite(RIGHT_B_PIN, 0);
    analogWrite(LEFT_B_PIN, -1*ldrive+lmin);
    // analogWrite(LEFT_B_PIN, 0);
    analogWrite(LEFT_F_PIN, 0);
    analogWrite(RIGHT_F_PIN, -1*(ldrive)+rmin);
  }
  else {
    analogWrite(RIGHT_F_PIN, 0);
    analogWrite(LEFT_F_PIN, ldrive+lmin);
    analogWrite(LEFT_B_PIN, 0);
    // analogWrite(RIGHT_B_PIN, 0);
    analogWrite(RIGHT_B_PIN, ldrive+lmin);
  }
}

void
Foot()
{
    
  if (distanceSensor.checkForDataReady()){
    previousMillis = currentMillis;
    currentMillis = millis();
    dt = (currentMillis - previousMillis) / 1000;
    prev_distance = distance;
    distance = distanceSensor.getDistance() * 0.0393701 / 12.0;
    distanceSensor.clearInterrupt();
  }
  else {
    int temp = (distance - prev_distance) / (dt*1000);
    prev_distance = distance;
    previousMillis = currentMillis;
    currentMillis = millis();
    dt = (currentMillis - previousMillis) / 1000;
    distance += temp*dt;
  }
  prev_err = curr_err;
  curr_err = distance - 1;

  total_err += curr_err*dt;

  P = kp * curr_err;
  if (P > 70) {
    P = 100;
  }
  if (P < -100) {
    P = -100;
  }

  I = ki * total_err;
  if (I > 100) {
    I = 100;
  }
  if (I < -100) {
    I = -100;
  }

  D = kd * (curr_err - prev_err)/dt;
  if (!dist_measured) {
    D = 0;
    dist_measured = true;
  }
  if (D > 100) {
    D = 100;
  }
  if (D < -100) {
    D = -100;
  }
  u = P + I + D;
  if (u > 100) {
    u = 100;
  }
  if (u < -100) {
    u = -100;
  }
  if (u < 0.25 && u > -0.25){
    analogWrite(RIGHT_B_PIN,0);;
    analogWrite(LEFT_B_PIN, 0);
    analogWrite(LEFT_F_PIN, 0);
    analogWrite(RIGHT_F_PIN, 0);
  }
  else if (u < 0) {
    ldrive = -115.0*(u/100.0) + 140.0;
    analogWrite(RIGHT_B_PIN, ldrive*0.3);
    analogWrite(LEFT_B_PIN, 1*ldrive);
    analogWrite(LEFT_F_PIN, 0);
    analogWrite(RIGHT_F_PIN, 0);
  }
  else {
    ldrive = 165.0*(u/100.0) + 90.0;
    analogWrite(RIGHT_F_PIN,  ldrive*0.5);
    analogWrite(LEFT_F_PIN,  ldrive);
    analogWrite(LEFT_B_PIN, 0);
    analogWrite(RIGHT_B_PIN, 0);
  }
  if (record) {
    Log();
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

        // While central is connected
        while (central.connected()) {

            // Read data
            read_data();
            if (pid){
              // execute pid
              if(icm){
                Orient();
              }
              else {
                Foot();
              }
            }
            // if(record){
            //   Log();
            // }
            if(sending){
              send();
            }
        }
        Serial.println("Disconnected");
        analogWrite(RIGHT_B_PIN, 0);
        analogWrite(LEFT_B_PIN, 0);
        analogWrite(LEFT_F_PIN, 0);
        analogWrite(RIGHT_F_PIN, 0);
    }
}
