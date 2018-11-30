int tiltSwitch = 3; // Tilt Switch Input
int tiltVal; // variable to store tilt input
boolean bIsTilted ;// define numeric variables val

void setup ()
{
  Serial.begin(9600);
  pinMode (tiltSwitch, INPUT) ;// define the mercury tilt switch sensor output interface
}
void loop ()
{
  tiltVal = digitalRead (tiltSwitch) ;// read the switch value
  if (tiltVal == HIGH) // Means we've tilted
  {
    if (!bIsTilted){
      bIsTilted = true;
      Serial.println("** - TILTED - **");
    } 
  }
  else
  {
    if (bIsTilted){
      bIsTilted = false;
      Serial.println("not tilted");
    }    
  }
}
