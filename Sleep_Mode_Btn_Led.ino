#include <avr/sleep.h>


const int buttonPin = 2;  // int 0 pin, connected to button UNO
const int ledPin = 13;    // built-in LED on Uno board

bool buttonPressed = false;
bool ledState = LOW;
unsigned long tempo;

void setup() {
  Serial.begin(115200);
  
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  attachInterrupt(0, buttonISR, FALLING);
  
  tempo = millis();
}


void buttonISR() { 
  buttonPressed = true; 
  }

void loop() {
  if (buttonPressed) {
    
    while (!digitalRead(buttonPin)) {}    // wait for it to be released
   
    digitalWrite(ledPin, LOW);    // turn off LED
    ledState = LOW;

    Serial.println("Going to sleep...");
    Serial.flush();
    gotoSleep();    // function to put the processor to sleep; a button press will wake it up

    while (!digitalRead(buttonPin)) {}    // wait for it to be released
    buttonPressed = false;
    Serial.println("Awake!");
    Serial.flush();
  }
  
  if ((millis() - tempo) > 250) {    // when awake, toggle the LED every 250 ms
    tempo = millis();
    if (ledState == LOW) {
      digitalWrite(ledPin, HIGH);
      ledState = HIGH;
    } else {
      digitalWrite(ledPin, LOW);
      ledState = LOW;
    }
  }
}

void gotoSleep()
{
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  noInterrupts();           // timed sequence follows
  sleep_enable();
  interrupts();             // guarantees next instruction executed
  sleep_cpu();              // nighty-night!
  sleep_disable();          // awake again -- cancel sleep as a precaution
}
