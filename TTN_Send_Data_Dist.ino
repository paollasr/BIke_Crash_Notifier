// LoRaWan code for sending nodes payloads to TTN

#include <TheThingsNetwork.h>
#include <Ultrasonic.h>

// Setting AppEUI and AppKey for OTTA mode
const char *appEui = "70B3D57ED0012353";
const char *appKey = "3E46B55864AC2081CDF4E88F1ED5676D";

#define loraSerial Serial1
#define debugSerial Serial
#define freqPlan TTN_FP_EU868

//TTN class sets already the values of spreading factor, freq plan and freq sub-band
TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);

//Sensor Config
Ultrasonic ultrassom(7,8); // digital pins for trig(7) and echo (8)
long distancia;
char inRange; // True or False (T/F)
const long minRange = 0;
const long maxRange = 60;


//Sensor Data
static uint8_t dataTX[1];

//test for sending 5 packets
//uint8_t testPayloads [ ] =
//{0x35, 0x31, 0xAB, 0x1A, 0x33 };


void setup() {
 loraSerial.begin(57600);
 debugSerial.begin(9600);

  // Wait a maximum of 10s for Serial Monitor
  while (!debugSerial && millis() < 10000)
    ;

  debugSerial.println("-- STATUS");
  ttn.showStatus(); //this method writes info about the Lora Radio

  debugSerial.println("-- JOIN");
  ttn.join(appEui, appKey); //confirming the OTAA

}



void do_sense() {

   distancia = ultrassom.Ranging(CM);
   
   // safety distance rear/front is >= 60cm
   if (distancia > minRange && distancia < maxRange) {
    inRange = 'T';
    Serial.print ("UNSAFE Distance from Object (cm): ");
    Serial.println(inRange);
  } else { 
    inRange = 'F';
    Serial.println ("SAFE Distance from Object");
    
  }
  delay(50); 

  return;
}
  
void send_data() {

  dataTX[0] = inRange;
  Serial.print(millis());
  Serial.print(F("Sending.. "))

  packet = (char*)dataTX;
  sendPacket(packet);
        
      }
  
  }  


void loop() {
  
 /* 
  //printing the example packets
   for (uint8_t i= 5; i>0; i--)
  { debogSerial.println (i);
     delay(1000);
    }

/*void sendMessage (String outgoing)
{
  LoRa.beginPacket(); //init of payload
  LoRa.write(msgCount); //payload counter added
  LoRa.write(outgoing.length();); //writes the size of the payloads in bytes
  LoRa.endPacket(); //end packets and sed
  msgCount++; //counting sended payloads
  
  }    */

}
