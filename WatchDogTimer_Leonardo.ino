//sleep mode for ATmega32u4 Leonardo
#define Led 4




void setup() {
  pinMode(Led, OUTPUT);

for (int i; i<20; i++){ //saving power setting all de IOs as output from 18mA to 13mA
  if (i != 2)
  pinMode(i, OUTPUT);
    }

    //interrupt to wake up
attachInterrupt (0, digitalInterrupt, FALLING);
}



void loop() {
 
  digitalWrite (Led, HIGH);
  delay(1000);
  digitalWrite (Led, LOW);
  delay(1000);


//DET UP WDT - Pag 55 WatchDog Timer
//WDE must be  clear - set to 0 in order to enable  
WDTCSR = (24); //CHANGE ENABLE AND WED
WDTCSR = (33); 
WDTCSR |= (1<<6); //ENABLE INTERRUPT MODE

//Sleep Mode Control Register – SMCR pag43

SMCR |= (1<<2); //setting power mode down by shifting bit 1
SMCR |= 1; // enable sleep

//using inline assembler to enable sleep mode
__asm__ __volatile__("sleep"); // getting a 0,380mA

//disble the ADC (pag 310)
ADCSRA &= ~(1<<7); //written 0 on the bit 7 from ADC
//current drop to 0,058mA after


//BOD Disable (current  drops 0.328 microAmps --Super Low!)
 MCUCR |= (3 << 5);
 MCUCR = ( MCUCR & ~(1 << 5)) |(1 << 6); 
  
}

void digitalInterrupt(){
  
   //digital pin 2  cant be an output anymore cause its an input of the function digital interrupt
}
 
 ISR (WTD_vect){
  //this interrupt from watch dog is called when it gets up 
  }
