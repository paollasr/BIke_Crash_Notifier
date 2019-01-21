#include <TheThingsNetwork.h>
#include <HCSR04.h>
#include "SparkFunLSM6DS3.h"
#include "Wire.h"


// OTTA LoRa mode
const char *appEui = "70B3D57ED0016C3A";
const char *appKey = "992811CCD9FEBCDF79C5DD874A4BA251";

#define loraSerial Serial1
#define debugSerial Serial
#define freqPlan TTN_FP_EU868

//TTN class sets already the values of spreading factor, freq plan and freq sub-band
TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);

/*
 Free Fall detector preset
Pins (Arduino Uno A4 A5) (Arduino Leonardo SDA SCL)
 
*/

LSM6DS3 lsm6ds3( I2C_MODE, 0x6A );  //I2C device address 0x6A
uint16_t detectCount = 0;
uint8_t error = 0; // accumulation variable 
uint8_t dataToWrite = 0; 
int config_free_fall_detect(void)
{  
 
  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz; //Anti-aliasing filter bandwidth selected = 200hz
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g; //Accelerometer full-scale selected = 2g
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz; // Output data rate and power mode = 416hz high performance

  //Writing the accumulated data in the error accumulation variable
  error += lsm6ds3.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite); //REGISTER 1 = Linear acceleration sensor control 
  //Write 00h into WAKE_UP_DUR
  error += lsm6ds3.writeRegister( LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x00 );//(pag 80) Free-fall, wakeup, timestamp and sleep mode functions duration setting register 
  error += lsm6ds3.writeRegister(LSM6DS3_ACC_GYRO_FREE_FALL, 0x33); //writes 33h into free fall
  error += lsm6ds3.writeRegister( LSM6DS3_ACC_GYRO_MD1_CFG, 0x10 );//writes 10h into MD1 and MD2
  error += lsm6ds3.writeRegister( LSM6DS3_ACC_GYRO_MD2_CFG, 0x10 );
  error += lsm6ds3.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x01);//latch interrupt and writes 01h into TAP_CFG1 
}


/*
 Ultrasonic distance Sensor settings
*/
UltraSonicDistanceSensor distanceSensor(9, 8); //Trigger aand Echo Pins
bool range;
const long minRange = 3;      // starts from 3cm since zero is measurement error
const long maxRange = 70;     // Below maxRange means unsafe distance

/*
 Leds 
*/
#define LED_RED 10 //LED light up everytime fall is detected on a bike
#define LED_BLUE 12 //LED light up everytime a message is sent to LoRa backend
int buzzerFreq;

void setup() {
  loraSerial.begin(57600);
  debugSerial.begin(9600);

  while (!debugSerial && millis() < 5000); // 5s for Serial Monitor

  debugSerial.println("-- STATUS");
  ttn.showStatus(); //this method writes info about the Lora Radio
  debugSerial.println("-- JOIN");

  ttn.join(appEui, appKey); //confirming the OTAA

// Starting the accelerometer
  if( lsm6ds3.begin() != 0 ){
    Serial.println("Was NOT possible to initiate the Free Fall Sensor");
  }
  else{
      Serial.println("Free Fall Sensor OK to start!");
  }
  
  if(0 != config_free_fall_detect())
    Serial.println("Fail to configure FF detection!");
  else
    Serial.println("Success to Configure FF detection!");

 //TTN Downlink method
 ttn.onMessage(downlinkLED);

 //Setting LED Pins as output
 pinMode(LED_RED, OUTPUT);
 pinMode(LED_BLUE, OUTPUT);
 //buzzer Pin
 pinMode(11,OUTPUT);
}

/*
   Distance Sensor
*/
uint8_t senseDistance(void) {
uint16_t distance = distanceSensor.measureDistanceCm();

  if ( distance < 2 || distance > 2000 )      // Error Range
    distance = 0;                            // 0  indicates an error
  else if ( distance > 255 )
    distance = 255;                          // Measurements higher than 255cm  are pruned to 255 in order to fit in 1Byte

  if (distance > minRange || distance < maxRange ){

    range= true;
    Serial.print("ALERT! Object is too close to rear wheel (cm): ");
    Serial.println(distance);
    }  
    else (distance > maxRange);
    {
      range= false;
      Serial.println("Safe distance from rear side");
  }
  return distance;
}

/*
   Measure free falls
*/

//uint16_t Freefallcounter(void)
uint16_t Freefallcounter(bool){
  
  uint8_t readDataByte = 0;
  //Read the wake-up source register
  lsm6ds3.readRegister(&readDataByte, LSM6DS3_ACC_GYRO_WAKE_UP_SRC);
  //Mask off the FF_IA bit for free-fall detection
  readDataByte &= 0x20; //dataByte= dataByte & 0x20
  //checking the free fall 
  if( readDataByte && RiderPresent()==true) {   
    detectCount ++;
    Serial.print("FALL DETECTED!");      
    Serial.println(detectCount);
    digitalWrite(LED_RED, HIGH);  
    //return error; 
    return true;
    buzzerFreq=2000;

  //tone(pin, frequency, duration)
  tone(11,buzzerFreq, 100);

     
     }else{
    digitalWrite(LED_RED, LOW);
     noTone(11);
     return false; 
      }
       delay(10);
 }


/*
  Pressure Sensor - Is rider Present?
*/
const int pressureSensor = A3;
int fsrReading;

bool RiderPresent(void) {
  fsrReading = analogRead(pressureSensor);       //Read and save analog value from potentiometer
  fsrReading = map(fsrReading, 0, 1023, 0, 255); //Map value 0-1023 to 0-255 (PWM)  
  if ( fsrReading > 1 )  {
    Serial.println(fsrReading);
    return true;
  } else{
    return false;
  }
}

/*
Evaluating conditions when data must be sent to LoRa
*/
bool mustWeSendData(uint16_t f, uint8_t d, uint8_t p ) {
 /*
  evaluatiing sensor conditions
  FreeFall X Distance
  0           0       FALSE
  0           1       FALSE
  1           0       TRUE
  1           1       TRUE
*/
   if( Freefallcounter(true) || range == false){
      return true;
      }else{
      return false;
      }
  
  if( Freefallcounter(true) && range == true) {
      return true;
      }else{
      return false;
      }
 }
/*
  Bytes to be sent
*/
void sendLoRaData(uint16_t f, uint8_t d, uint8_t p ) {

// Wake RN2483
  ttn.wake();
  
  byte bytesToSend[4];      //  2 bytes for Accelerometer / 1 byte distance sensor;

  bytesToSend[0] = d;       // Distance Byte
  bytesToSend[1] = f >> 8;  // High byte Accelerometer
  bytesToSend[2] = f;       // Low Byte Accelerometer
  bytesToSend[3] = p;       //Rider is present
  
  ttn.sendBytes(bytesToSend, sizeof(bytesToSend), 2 ); /*bytes,size, port */



  // Set RN2483 to sleep mode
  ttn.sleep(60000);

  // This one is not optionnal, remove it
  // and say bye bye to RN2983 sleep mode
  delay(50);


}


/*
  Sending Data to LoRa Server
*/
      uint16_t falls = Freefallcounter(true);
      uint8_t distance = senseDistance();
      uint8_t presence = RiderPresent();
 
void loop() {
  
  if ( RiderPresent() == true )  {
      
    if ( mustWeSendData(falls, distance, presence) )  {
      // Send to backend via LoRa
      sendLoRaData(falls, distance, presence);
       buzzerFreq=4000;
       //tone(pin, frequency, duration)
      tone(11,buzzerFreq, 100);
      Serial.println("Oops, accident detected!");
     }else{
      Serial.println("All good in the hood :) No accident detected.");
      }
  }

void  sendLoRaData (uint16_t f, uint8_t d );
  
  delay(10 * 1000);       //  Evaluate every 10 secs
}

//DownlinkLED function - Light up the blue LED whenever data is sent to LoRa backend

void downlinkLED(const uint16_t *payload, size_t size, port_t port){
  
  if (payload [0] ==  mustWeSendData(falls, distance, presence) ){
    digitalWrite(LED_BLUE, HIGH);
    }else{
    digitalWrite(LED_BLUE, LOW);
      }

 debugSerial.print("Received " + String(size) + " bytes on port " + String(port) + ":");
}
