
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

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
unsigned long currentMillis = 0;
float times[100];
float temps[100];
float pitch[200];
float roll[200];
float lp_pitch[200];
float lp_roll[200];
float stamps[200];
int stored_times = 0;
//////////// Global Variables ////////////

#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include<math.h>

#define SERIAL_PORT Serial
#define AD0_VAL   0     // The value of the last bit of the I2C address, on the SparkFun 9DoF IMU breakout the default is 1


ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object
///////////// Accelerometer //////////////

enum CommandTypes
{
    PING,
    SEND_TWO_INTS,
    SEND_THREE_FLOATS,
    ECHO,
    DANCE,
    SET_VEL,
    GET_TIME_MILLIS,
    SAMPLE_TIME,
    SEND_TIME_DATA,
    GET_TEMP_READINGS,
    GET_ACCEL_READINGS,
    LP_ACCEL_READINGS,
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
        case PING:
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        /*
         * Extract two integers from the command string
         */
        case SEND_TWO_INTS:
            int int_a, int_b;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_a);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_b);
            if (!success)
                return;

            Serial.print("Two Integers: ");
            Serial.print(int_a);
            Serial.print(", ");
            Serial.println(int_b);
            
            break;
        /*
         * Extract three floats from the command string
         */
        case SEND_THREE_FLOATS:
            /*
             * Your code goes here.
             */

            break;
        /*
         * Add a prefix and postfix to the string value extracted from the command string
         */
        case ECHO:

            char char_arr[MAX_MSG_SIZE];

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(char_arr);
            if (!success)
                return;

            /*
             * Your code goes here.
             */
            tx_estring_value.clear();
            tx_estring_value.append("Robot says -> ");
            tx_estring_value.append(char_arr);
            tx_estring_value.append(" :)");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            Serial.println(tx_estring_value.c_str());
            
            break;
        /*
         * DANCE
         */
        case DANCE:
            Serial.println("Look Ma, I'm Dancin'!");

            break;
        
        /*
         * SET_VEL
         */
        case SET_VEL:

            break;
        
        /* 
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */

        case GET_TIME_MILLIS:
            tx_estring_value.clear();
            tx_estring_value.append("T:");
            tx_estring_value.append((float)millis());
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            Serial.println(tx_estring_value.c_str());
            break;

        case SAMPLE_TIME:
            float start;
            start = (float)millis();
            while((millis() - start) < 5000) {
              tx_estring_value.clear();
              tx_estring_value.append("T:");
              tx_estring_value.append((float)millis());
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;

        case SEND_TIME_DATA:
            stored_times = 0;
            while (stored_times < 100) {
              times[stored_times] = (float)millis();
              stored_times += 1;
            }
            for(int i = 0; i < 100; i++){
              tx_estring_value.clear();
              tx_estring_value.append("T:");
              tx_estring_value.append(times[i]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;

        case GET_TEMP_READINGS:
            stored_times = 0;
            while (stored_times < 100) {
              times[stored_times] = (float)millis();
              temps[stored_times] = (float)getTempDegC();
              stored_times += 1;
            }
            for(int i = 0; i < 100; i++){
              tx_estring_value.clear();
              tx_estring_value.append("Temp: ");
              tx_estring_value.append(temps[i]);
              tx_estring_value.append("@Time: ");
              tx_estring_value.append(times[i]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;
        case GET_ACCEL_READINGS:
            stored_times=0;

            while(stored_times<200){
              if(myICM.dataReady()){
                myICM.getAGMT();
                pitch[stored_times] = atan2(myICM.accX(),myICM.accZ())*180/M_PI;
                roll[stored_times] = atan2(myICM.accY(),myICM.accZ())*180/M_PI;
                stamps[stored_times] = (float)millis();
                stored_times += 1;
                delay(50);
              }
            }
            for(int i=0; i<200; i++){
                tx_estring_value.clear();
                tx_estring_value.append("Acc");
                tx_estring_value.append(pitch[i]);
                tx_estring_value.append(" ");
                tx_estring_value.append(roll[i]);
                tx_estring_value.append(" ");
                tx_estring_value.append(stamps[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            break;

        case LP_ACCEL_READINGS:
            stored_times=0;
            float a;

            while(stored_times<200){
              if(myICM.dataReady()){
                myICM.getAGMT();
                stamps[stored_times] = (float)millis();
                if(stored_times==0){
                  pitch[stored_times] = atan2(myICM.accX(),myICM.accZ())*180/M_PI;
                  roll[stored_times] = atan2(myICM.accY(),myICM.accZ())*180/M_PI;
                  lp_pitch[stored_times] = atan2(myICM.accX(),myICM.accZ())*180/M_PI;
                  lp_roll[stored_times] = atan2(myICM.accY(),myICM.accZ())*180/M_PI;
                }
                else{
                  a = (stamps[stored_times]-stamps[stored_times-1]) / 1000;
                  a = a/(a+(0.015915*2));
                  pitch[stored_times] = atan2(myICM.accX(),myICM.accZ())*180/M_PI;
                  roll[stored_times] = atan2(myICM.accY(),myICM.accZ())*180/M_PI;
                  lp_pitch[stored_times] = (pitch[stored_times-1]*(1-a)) + (a * (atan2(myICM.accX(),myICM.accZ())*180/M_PI));
                  lp_roll[stored_times] = (roll[stored_times-1]*(1-a)) + (a * (atan2(myICM.accY(),myICM.accZ())*180/M_PI));
                }
                stored_times += 1;
                delay(50);
              }
            }
            for(int i=0; i<200; i++){
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
