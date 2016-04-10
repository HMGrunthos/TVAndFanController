#include "pins_arduino.h"

int RXPIN = A1;
int TXPIN = A2;

void setup()
{
  // Set up TIMER0 for PWM controlling the fan
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);

  TCCR2A = _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS20);
  OCR2A = 216; // The top value for the timer.  The modulation frequency will be SYSCLOCK / 2 / OCR2A.
  OCR2B = OCR2A / 3; // 33% duty cycle 

  Serial.begin(115200);

  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);

  pinMode(RXPIN, INPUT);
  pinMode(TXPIN, OUTPUT);
  digitalWrite(TXPIN, HIGH);
 
  pinMode(2, INPUT);
  
  pinMode(11, INPUT);

pinMode(13, OUTPUT); // LED
  digitalWrite(13, LOW);
  
    pinMode(6, OUTPUT); // Power on/off relay
  digitalWrite(6, LOW);

  setupSoftSerial();

  setupSoftTx();

  Serial.println("\n[memCheck]");
  Serial.println(freeRam());
}

void loop()
{
  byte ssAvailable = softSerialAvailable();
  while(ssAvailable--) {
    Serial.print((char)softSerialRead());
  }

  while(Serial.available()) {
    unsigned short int val = Serial.read();
    sendChar(val);
  }
}

// Pin change handler, records the state and time since last reading change of a ss input pin
ISR(PCINT1_vect)
{ 
  srvSSStateChange();
}

ISR(TIMER2_OVF_vect)
{
  srvTXTimer();
}

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
