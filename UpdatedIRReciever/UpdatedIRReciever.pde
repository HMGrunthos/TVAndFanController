/*
 * IRremote: IRrecvDump - dump details of IR codes with IRrecv
 * An IR detector/demodulator must be connected to the input RECV_PIN.
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * http://arcfn.com
 */

#include <IRremote.h>
// #include <PinChangeInt.h>

int RECV_PIN = 11;

// Using original timer polled code reader 511 RAM left 8760 code
// Using pin change reader via PinChangeInt 187 RAM left 9388 code
// Using pin change reader via manual ISR 263 RAM left 8898 code
// Using original IRRecv code 437 RAM left 8272 code

// Using updated timer polled code reader 573 RAM left 7980 code
// Using pin change reader via PinChangeInt 194 RAM left 9216 code
// Using pin change reader via manual ISR 270 RAM left 8720 code
// Using original IRRecv code 437 RAM left 8272 code

// IRrecv irrecv(RECV_PIN);
IRrecvPinChangeSvr irrecv(RECV_PIN);

// decode_results results;
void numpty();
void setup()
{
  // numpty();
  Serial.begin(115200);
  
  pinMode(2, INPUT);
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);
  
  // irrecv.blink13(true);
  irrecv.enableIRIn(); // Start the receiver
  Serial.println("\n[memCheck]");
  Serial.println(freeRam());
  // PCintPort::attachInterrupt(RECV_PIN, IRPinChangeFun, CHANGE);
  PCMSK0 |= (1<<PCINT3);
  PCICR |= (1<<PCIE0);
}

// based on Print::printNumber
void printHex64(unsigned long long n)
{
  unsigned char buf[sizeof(unsigned long long) * 2];
  unsigned long i = 0;

  if (n == 0) {
    Serial.print('0');
    return;
  } 

  while (n > 0) {
    buf[i++] = n & 0xf;
    n >>= 4;
  }

  for (; i > 0; i--) {
    Serial.print((char) (buf[i - 1] < 10 ?
      '0' + buf[i - 1] :
      'A' + buf[i - 1] - 10));
  }
}

// Dumps out the decode_results structure.
// Call this after IRrecv::decode()
// void * to work around compiler issue
//void dump(void *v) {
//  decode_results *results = (decode_results *)v
void dump(IRrecv::decode_results *results) {
  int count = results->rawlen;
  if (results->decode_type == IRrecv::UNKNOWN) {
    Serial.print("Unkenc: ");
  }
  else if (results->decode_type == IRrecv::NEC) {
    Serial.print("Decoded NEC: ");
  } 
  else if (results->decode_type == IRrecv::SONY) {
    Serial.print("Decoded SONY: ");
  } 
  else if (results->decode_type == IRrecv::RC5) {
    Serial.print("Decoded RC5: ");
  } 
  else if (results->decode_type == IRrecv::RC6) {
    Serial.print("DecRC6: ");
  }
  printHex64(results->value);
  Serial.print(" (");
  Serial.print(results->bits, DEC);
  Serial.println(" bits)");
  Serial.print("Raw (");
  Serial.print(count, DEC);
  Serial.print("): ");

  for (int i = 0; i < count; i++) {
    if ((i % 2) == 1) {
      // Serial.print(results->rawbuf[i]*USECPERTICK, DEC);
    } 
    else {
      // Serial.print(-(int)results->rawbuf[i]*USECPERTICK, DEC);
    }
    Serial.print(" ");
  }
  Serial.println("");
}

void loop() {
  static unsigned long long lastValue;
  static unsigned long long int lastTime;
  static boolean keyDown = false;
  
  if (irrecv.decode()) {
    dump(&irrecv.results);
    if(irrecv.results.decode_type == IRrecv::RC5) {
      if((irrecv.results.value & 0x7FF) == 0x7BD && lastValue != irrecv.results.value) {
        PINB |= (1<<PINB5);
      }
      if((irrecv.results.value & 0x7FF) == 0x7BB) {
        if(lastValue != irrecv.results.value) {
          digitalWrite(13, HIGH);
          delay(100);
          digitalWrite(13, LOW);
          lastTime = millis();
        } else {
          if(!keyDown && (millis() - lastTime) > 500) {
            digitalWrite(13, HIGH);
            keyDown = true;
            lastTime = millis();
          } else if(keyDown) {
            lastTime = millis();
          }
        }
      }
      lastValue = irrecv.results.value;
    }
    if(irrecv.results.decode_type == IRrecv::RC6) {
      if((irrecv.results.value & 0xFFFFF7FFFLL) == 0xC800F040CLL && lastValue != irrecv.results.value) {
        PINB |= (1<<PINB5);
        lastValue = irrecv.results.value;
      }
    }
    irrecv.resume(); // Receive the next value
  }
  if(keyDown && (millis() - lastTime) > 150) {
    digitalWrite(13, LOW);
    keyDown = false;
    lastTime = millis();
  }
}

ISR(PCINT0_vect)
{
  irrecv.servIRPinChange();
}

/*
void IRPinChangeFun()
{
  irrecv.servIRPinChange();
}
*/
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
