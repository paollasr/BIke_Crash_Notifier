#include <HCSR04.h>

#include <HCSR04.h>

#include "SparkFunLSM6DS3.h"
#include "Wire.h"


#include <TheThingsNetwork.h>


#define DEBUG 1 // scaffold code: preprocessor variable of given value 1, to complie stuff during debugging/testing process and remove them later on production

// Setting AppEUI and AppKey for OTTA mode
const char *appEui = "70B3D57ED001139E";
const char *appKey = "D9BD1B83A358893A91B8598C671CF805";

#define loraSerial Serial1
#define debugSerial Serial
#define freqPlan TTN_FP_EU868

//TTN class sets already the values of spreading factor, freq plan and freq sub-band
TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);

//Sensors Configuration
//Free fall detector : Pins A4 (SDA) e A5 (SCL)
LSM6DS3 lsm6ds3( I2C_MODE, 0x6A );  //I2C device address 0x6A
uint16_t detectCount = 0;
uint8_t error = 0;  
uint8_t dataToWrite = 0; 

//configuration of free Fall detection - Sensor library
int config_free_fall_detect(void)
{  
 
  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
  
  error += lsm6ds3.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);
  error += lsm6ds3.writeRegister( LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x00 );
  error += lsm6ds3.writeRegister(LSM6DS3_ACC_GYRO_FREE_FALL, 0x33); //writes 33h into free fall
  error += lsm6ds3.writeRegister( LSM6DS3_ACC_GYRO_MD1_CFG, 0x10 );//writes 10h into MD1 and MD2
  error += lsm6ds3.writeRegister( LSM6DS3_ACC_GYRO_MD2_CFG, 0x10 );
  error += lsm6ds3.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x01);//latch interrupt
}


//Distance Sensor Configuration
UltraSonicDistanceSensor distanceSensor(7, 8);
bool range = false;
const long minRange = 3;      // When we have zero this is measurement error.
const long maxRange = 60;     // Below maxRange means unsafe distance



void setup() {
  loraSerial.begin(57600);
  debugSerial.begin(9600);

#ifdef DEBUG //#if def is checking whether the identifier(DEBUG) is defined by a #define
  // Wait a maximum of 5s for Serial Monitor
  while (!debugSerial && millis() < 5000);

  debugSerial.println("-- STATUS");
  ttn.showStatus(); //this method writes info about the Lora Radio


  debugSerial.println("-- JOIN");
#endif

  ttn.join(appEui, appKey); //confirming the OTAA

// Free fall Sensor Begin
  if( lsm6ds3.begin() != 0 ){
    Serial.println("Device error");
  }
  else{
      Serial.println("Device OK!");
  }
  
  if(0 != config_free_fall_detect())
    Serial.println("Fail to configure!");
  else
    Serial.println("Success to Configure!");
}

/*
   Measure distance once.
*/
uint8_t senseDistance(void) {
uint16_t distance = distanceSensor.measureDistanceCm();
#ifdef DEBUG
  Serial.print(F("Distance measured: "));
  Serial.print(distance);
#endif

  if ( distance < 2 || distance > 2000 )    // This is an error
    distance = 0;                            // Use 0 to indicate an error
  else if ( distance > 255 )
    distance = 255;                          // All bigger than 255 = 255 to fit within a byte

  if (distance > minRange || distance < maxRange ){

    range= true;
    Serial.print("UNSAFE dist (cm): ");
    Serial.println(distance);
 
    }
    
    else (distance > maxRange);
    {
      range= false;
    Serial.print("Safe distance");
  }
#ifdef DEBUG
  Serial.print(", pruned to: ");
  Serial.println(distance);
#endif

  return distance;
}

/*
   Measure free falls

uint16_t freeFallDetectCount = 0;

uint16_t senseFalls(void) {
  // TBD needs work....
  if (  millis() % 100 < 25 )  { // real sensor flag set or so?? Simulate a 25% fall every once in a while for now 
    freeFallDetectCount++;
    return freeFallDetectCount;
  } else {
    return 0;               // No fall detected
  }
}
*/


uint16_t Freefallcounter(void)
 {
  uint8_t readDataByte = 0;
  //Read the wake-up source register
  lsm6ds3.readRegister(&readDataByte, LSM6DS3_ACC_GYRO_WAKE_UP_SRC);
  //Mask off the FF_IA bit for free-fall detection
  readDataByte &= 0x20;
  //checking the free fall 
  if( readDataByte )
  {   
    detectCount ++;
    Serial.print("Rider has fallen!");        
    Serial.println(detectCount);
    return error;
     
}
  delay(10);
  }

/*
  Pressure Sensor - Check if the rider is present
*/
const int pressureSensor = A0;
int fsrReading;

bool isRiderPresent(void) {
  fsrReading = analogRead(pressureSensor);       //Read and save analog value from potentiometer
  fsrReading = map(fsrReading, 0, 1023, 0, 255); //Map value 0-1023 to 0-255 (PWM)  
  if ( fsrReading > 1 )  {
    Serial.println(fsrReading);
    return true;
  } else{
    return false;
  }
}

bool mustWeSendData(uint16_t f, uint8_t d ) {
#ifdef DEBUG
  Serial.print(F("Must evaluate falls="));
  Serial.print(f);
  Serial.print(F(" and distance="));
  Serial.println(d);
#endif

  detectCount ++;
  range = true; 
 
  return true;
}

void sendLoRaData(uint16_t f, uint8_t d ) {
  byte bytesToSend[3];      // We need 2 bytes for the fall counter and one for the distance;

  bytesToSend[0] = d;       // Distance goes in the first.
  bytesToSend[1] = f >> 8;  // High byte of the fall counter goes next.
  bytesToSend[2] = f;       // And low byte goes last
  ttn.sendBytes(bytesToSend, sizeof(bytesToSend), 2 /* port */);
}

void loop() {
  delay(10 * 1000);       //  Evaluate every 10 secs
  if ( isRiderPresent() )  {
    // Bike is in use, lets do the other measurements
#ifdef DEBUG
    Serial.println("Rider present");
#endif
    uint16_t falls = Freefallcounter();
    uint8_t distance = senseDistance();

    if ( mustWeSendData(falls, distance) )  {
      // Send to backend via LoRa
      sendLoRaData(falls, distance);



      
#ifdef DEBUG
      Serial.println("Data was sent.");
#endif
    } else {
      // Nothing to send
#ifdef DEBUG
      Serial.println("Nothing to send.");
#endif
    }
  } else {
#ifdef DEBUG
    Serial.println("No rider present.");
#endif
  }
}
