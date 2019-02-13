#include <TheThingsNetwork.h>
#include <HCSR04.h>
#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define DEBUG 1

// OTTA LoRa mode
const char *appEui = "70B3D57ED0016C3A";
const char *appKey = "992811CCD9FEBCDF79C5DD874A4BA251";

#define loraSerial Serial1
#define debugSerial Serial
#define freqPlan TTN_FP_EU868
#define CONFIG_INTERVAL ((uint32_t) 300)


//TTN class sets already the values of spreading factor, freq plan and freq sub-band
TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);

/*
 Free Fall detector preset
 (Arduino Leonardo SDA SCL)
 
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
  //Writes ooh into WAKE_UP_DUR
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
 Leds & Buzzer
*/
#define LED_RED 10 //LED light up everytime fall is detected on a bike
#define LED_BLUE 12 //LED light up everytime a message is sent to LoRa backend
volatile boolean ledOn = false;
int buzzerFreq;

void setup() {
  loraSerial.begin(57600);
  debugSerial.begin(9600);

  #ifdef DEBUG
  while (!debugSerial && millis() < 5000); // 5s for Serial Monitor
  debugSerial.println("-- STATUS");
  ttn.showStatus(); //this method writes info about the Lora Radio
  debugSerial.println("-- JOIN");
  #endif
  
  ttn.join(appEui, appKey); //confirming the OTAA

// Starting the accelerometer
  if( lsm6ds3.begin() != 0 ){
    
    #ifdef DEBUG
    Serial.println("Was NOT possible to initiate the Free Fall Sensor");
    #endif
  }
  else{
    
     #ifdef DEBUG
      Serial.println("Free Fall Sensor OK to start!");
     #endif
  }
  
  if(0 != config_free_fall_detect()){
    
   #ifdef DEBUG
    Serial.println("Fail to configure FF detection!");
   #endif
   
 }else{
  
   #ifdef DEBUG
    Serial.println("Success to Configure FF detection!");
  #endif
  
 }
 //Setting LED Pins as output
 pinMode(LED_RED, OUTPUT);
 pinMode(LED_BLUE, OUTPUT);

 // Interrupt INT0 - Leonardo
   pinMode(2,INPUT);
 //ISR function(digPIN2, function, SignalMode)
//Leonardo attachInterrupt0 - INT1 - Digital Pin 2
  attachInterrupt(1,PinInterrupt,RISING); 

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
    
     #ifdef DEBUG
    Serial.print("ALERT! Object is too close to rear wheel (cm): ");
    Serial.println(distance);
    #endif
    }  
    else (distance > maxRange);
    {
    range= false;
    
    #ifdef DEBUG
    Serial.println("Safe distance from rear side");
    #endif
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
    
    #ifdef DEBUG
    Serial.print("FALL DETECTED!");      
    Serial.println(detectCount);
    #endif
    
    digitalWrite(LED_RED, HIGH);  
    //return error; 
    return true;
    //sending a Tone
    buzzerFreq=4000;
    //tone(pin, frequency, duration)
      tone(11,buzzerFreq, 100);
     }
     else{
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
    #ifdef DEBUG
    Serial.println(fsrReading);
    #endif
    
    return true;
  } else{
    return false;
  }
}


/*
  Bytes to be sent
*/
      uint16_t falls = Freefallcounter(true);
      uint8_t distance = senseDistance();
      uint8_t presence = RiderPresent();

void doSend(uint16_t falls, uint8_t distance, uint8_t presence ) {
 
 
  if(Freefallcounter(true) && RiderPresent() == true){
  
  byte bytesToSend[4];                  

  bytesToSend[0] = distance;          // Distance Byte
  bytesToSend[1] = falls >> 8;       // High byte Accelerometer
  bytesToSend[2] = falls;           // Low Byte Accelerometer
  bytesToSend[3] = presence;       //Rider is present
  
  ttn.sendBytes(bytesToSend, sizeof(bytesToSend), 1 ); /*bytes,size, port */

  delay(3000);
  }else{
    #ifdef DEBUG
    Serial.println(F("NO DATA TO BE SENT"));
    #endif
    
    }

}

/*
Sleep Mode configuration
Leonardo attachInterrupt0 - INT0 - Digital Pin 3
Leonardo attachInterrupt0 - INT1 - Digital Pin 2
*/

void sleepRadio() {
    #ifdef DEBUG
        Serial.println("[MAIN] Sleeping the radio");
    #endif
   
    if (loraSerial.available()) loraSerial.read();
//    ttn.sleep();
}


void sleepSetupMCU() {
    #ifdef DEBUG
        Serial.println("MCU Sleeping ");
        delay(10);
    #endif
 
  sleep_enable(); 
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
 
  sleep_cpu(); //(SE)bit is enable. MCU goes sleep
//Stops to execute the function

    #ifdef DEBUG
        Serial.println("MCU/LoRa Radio Awake!");
    #endif
}

//Button to turn the device ON/OFF
void PinInterrupt(){

   sleep_disable();
 
  if(ledOn)
  {
    ledOn = false;
    digitalWrite(LED_BLUE,LOW);
  }else
  {
    ledOn = true;
    digitalWrite(LED_BLUE,HIGH); 
 }
  
  }


/*
  Sending Data to LoRa Server
*/

      
void loop() {


if (ledOn == false){
  Serial.println(F("going to sleep"));
   sleepRadio();
   sleepSetupMCU();
   
  
  }
  else {
    #ifdef DEBUG
    Serial.println (F("MCU and LoRa Radio awake"));
    #endif
    // Wake LoRa Modul RN2483
      ttn.awake();
      delay(3000);
    //sending LoRa data
    void doSend();
    //Returns the voltage in millivolt (mV)  LoRa module.  
    uint16_t getVDD();         
    }

}



 
