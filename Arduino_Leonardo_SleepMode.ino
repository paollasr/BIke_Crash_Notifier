// Routinen fuer Sleep-Mode
#include "Arduino.h"
#include <avr/interrupt.h>
#include <avr/sleep.h>


//completely lock the CPU (until a hardware reset), 
//interrupts need to be enabled before going to sleep.
void setup(){
  
  Serial.begin(9600);
 
}

void loop(){
  
  sleepPinInterrupt();
  enterSleepMode();
 
}


void sleepPinInterrupt(void)
{
 sleep_disable();

  detachInterrupt(1);
}

void enterSleepMode(void)
{ 

     //setting MCU to sleep mode idle
  set_sleep_mode(SLEEP_MODE_IDLE); 

    //if(Rider is not present)
  
  Serial.println(F("enabling sleep mode"));
  
  sleep_enable();


 //Makin sure we able to wake-up again (ignoring any interrupts before setting them bellow)
 noInterrupts();

 //setting an interrupt
  attachInterrupt(1, sleepPinInterrupt, LOW);

 
  sleep_cpu();

  sleep_disable();
  Serial.println(F("waking up"));
}
