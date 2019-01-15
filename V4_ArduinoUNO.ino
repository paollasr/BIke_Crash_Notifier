#include <HCSR04.h>
#include "SparkFunLSM6DS3.h"
#include "Wire.h"

UltraSonicDistanceSensor distanceSensor(9, 8); //Trigger aand Echo Pins
bool range;
const long minRange = 3;      // starts from 3cm since zero is measurement error
const long maxRange = 70;     // Below maxRange means unsafe distance
int buzzerFreq;
/*
 Leds 
*/
#define LED_RED 10 //LED light up everytime fall is detected on a bike
#define LED_BLUE 12 //LED light up everytime a message is sent to LoRa backend

LSM6DS3 lsm6ds3( I2C_MODE, 0x6A );  //I2C device address 0x6A
uint16_t detectCount = 0;
uint8_t error = 0;  
uint8_t dataToWrite = 0; 
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



void setup() {
 Serial.begin(9600);

 //Setting LED Pins as output
 pinMode(LED_RED, OUTPUT);
 pinMode(LED_BLUE, OUTPUT);
//Buzzer Pin
 pinMode(11,OUTPUT);
 
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

}

/*
   Measure free falls
*/

//uint16_t Freefallcounter(void)
uint16_t Freefallcounter(bool)

 {
  uint8_t readDataByte = 0;
  //Read the wake-up source register
  lsm6ds3.readRegister(&readDataByte, LSM6DS3_ACC_GYRO_WAKE_UP_SRC);
  //Mask off the FF_IA bit for free-fall detection
  readDataByte &= 0x20;

  //checking the free fall AND riders presence
  if( readDataByte && RiderPresent()== true )
  {   
    detectCount ++;
    Serial.print("FALL DETECTED!");      
    Serial.println(detectCount);
   
    //return error;
    return true;  
  }else{
    
    return false;
    }  

  delay(1000);
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
    Serial.print("Rider is present ");
    Serial.println(fsrReading);
     digitalWrite(LED_RED, HIGH);  
    return true;
  } else{
    Serial.println("No rider");
    digitalWrite(LED_RED, LOW);
    return false;
  }
}

uint8_t senseDistance(void) {
uint16_t distance = distanceSensor.measureDistanceCm();

  if ( distance < 2 || distance > 2000 )      // Error Range
    distance = 0;                            // 0  indicates an error
  else if ( distance > 255 )
    distance = 255;                          // Measurements higher than 255cm  are pruned to 255 in order to fit in 1Byte

  if (distance > minRange && distance < maxRange ){

    range= true;
    Serial.print("ALERT! Object is too close to rear wheel (cm): ");
    Serial.println(distance);
    }  
    else
    {
      range = false;
      Serial.println("Safe distance from rear side");
  }
  return distance;
}

void loop() {
    
    RiderPresent();
    
    if(Freefallcounter(true)){
       senseDistance();
       buzzerFreq=4000;
       //tone(pin, frequency, duration)
  tone(11,buzzerFreq, 100);
  Serial.println("Oops, accident detected!");
  digitalWrite(LED_BLUE, HIGH);
      
      }else{

   digitalWrite(LED_BLUE, LOW);
   Serial.println("No accident detected");   
   noTone(11);
        
        }
  
  delay(1000);       //  Evaluate every 1 secs  
  }
      

      
  
