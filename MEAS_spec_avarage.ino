const int PIEZO_PIN = A0; // Piezo output
 
unsigned long startMillis;
float accumulatedPiezoV;
unsigned numberOfMeasurements;
 
void setup()
{
  Serial.begin(9600);
  startMillis = millis();
  accumulatedPiezoV = 0.0;
  numberOfMeasurements = 0;
  pinMode(LED_BUILTIN, OUTPUT);
}
 
float readPiezo(void) {
  int piezoADC = analogRead(PIEZO_PIN);
  return (float)piezoADC / 1023.0 * 5.0;
}
 
void loop()
{
  float piezoV;
 
  if ( millis() - startMillis < 10000 ) {  // check accumulation time in miliseconds....
    // Not waited long enough. Just take another measurement.
    accumulatedPiezoV += readPiezo();
    numberOfMeasurements ++;
   
  } else {
    // Ok, show what we have got..
    piezoV = accumulatedPiezoV / numberOfMeasurements;
    Serial.print("Average piezo voltage is: ");
    Serial.println(piezoV);
    // and inititate another cycle
    startMillis = millis();
    accumulatedPiezoV = 0.0;
    numberOfMeasurements = 0;
  }
  if (piezoV > 2.0 ) {  // Just run it a while while the 
    //sensor is at ease. No rider present. 
    //Put the maximum rest-voltage measured here. 
    //And then try if it works.
    Serial.println("Rider is present!");
    //digitalWrite(LED_BUILTIN, HIGH);
  }
  else {
    Serial.println("Rider is NOT present. ");
    //digitalWrite (LED_BUILTIN, LOW);
  }
}
