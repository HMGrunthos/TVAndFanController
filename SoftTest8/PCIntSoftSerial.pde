#define BITTIMESCALE 7

#define BITTIME (3333) // (26.0416 << BITTIMESCALE) // 9600
// #define BITTIME (1667) // (13.0208 << BITTIMESCALE) // 19200
// #define BITTIME (833) // (6.5104 << BITTIMESCALE) // 38400
// #define BITTIME (556) // (4.3403 << BITTIMESCALE) // 57600

struct {
  struct {
    uint8_t pinMask;
    volatile uint8_t pinStateCngDelt[128];
    volatile uint8_t pinStateHist[16];
    volatile uint8_t pcTop;
    volatile uint8_t *pinReg;
    volatile uint8_t *stateStore;
    unsigned long int lastTime;
  } pinData;
  struct {
    unsigned long int lastTime;
    signed short currentDelta;
    uint8_t pcBottom;
    byte bytePos;
    byte byteVal;
    byte lastState;
    byte availableChars[4];
    byte bufferHead;
    byte bufferTail;
    boolean frameError;
  } decState;
} pcSSParams;

void setupSoftSerial()
{
  uint8_t rxPort = digitalPinToPort(RXPIN);
  pcSSParams.pinData.pinMask = digitalPinToBitMask(RXPIN);
  pcSSParams.pinData.pinReg = portInputRegister(rxPort);
  pcSSParams.pinData.stateStore = pcSSParams.pinData.pinStateHist;
  *pcSSParams.pinData.stateStore = 0;
  pcSSParams.pinData.lastTime = 0;
  pcSSParams.pinData.pcTop = 0;

  pcSSParams.decState.pcBottom = 0;
  pcSSParams.decState.bytePos = 0;
  pcSSParams.decState.byteVal = 0;
  pcSSParams.decState.lastState = 0xFF;
  pcSSParams.decState.currentDelta = 0;
  pcSSParams.decState.lastTime = millis();
  pcSSParams.decState.bufferHead = 0;
  pcSSParams.decState.bufferTail = 0;
  pcSSParams.decState.frameError = false;
    
  PCMSK1 |= (1<<PCINT9);
  uint8_t oldSREG = SREG;
  cli();
    srvSSStateChange(); // Force an update
    PCICR |= (1<<PCIE1);
  SREG = oldSREG;
  pcSSParams.decState.pcBottom++;
}

unsigned short int softSerialRead()
{
  softSerialAvailable();
  char readVal;
  boolean bufferEmpty = pcSSParams.decState.bufferHead == pcSSParams.decState.bufferTail;
  if(!bufferEmpty) {
    readVal = pcSSParams.decState.availableChars[pcSSParams.decState.bufferTail];
    pcSSParams.decState.bufferTail = (pcSSParams.decState.bufferTail + 1) & 0x3;
    return readVal;
  } else {
    return -1;
  }
}

byte softSerialAvailable()
{
  uint8_t localPCTop = pcSSParams.pinData.pcTop;
  if(localPCTop != pcSSParams.decState.pcBottom) { // There's new data and space for a new character
    boolean bufferFull = ((pcSSParams.decState.bufferHead+1) & 0x3) == pcSSParams.decState.bufferTail;

    // Select the byte containing the next bits
    uint8_t curByte = pcSSParams.pinData.pinStateHist[pcSSParams.decState.pcBottom >> 3];
    for(; (pcSSParams.decState.pcBottom != localPCTop) && !bufferFull; pcSSParams.decState.pcBottom = (pcSSParams.decState.pcBottom+1) & 0x7F) { // For each new bit
      if((pcSSParams.decState.pcBottom & 0x7) == 0) { // Load a new set of bits if pcBottom is divisable by 8
        curByte = pcSSParams.pinData.pinStateHist[pcSSParams.decState.pcBottom >> 3];
      }
      byte pinState = (curByte << (~pcSSParams.decState.pcBottom & 0x7)) & 0x80; // Shift the next bit out
      byte pinDelta = pcSSParams.pinData.pinStateCngDelt[pcSSParams.decState.pcBottom];

      if(pcSSParams.decState.lastState != pinState) {
        pcSSParams.decState.currentDelta = -((BITTIME+1)>>1); // Jump halfway into the new bit
      }
      pcSSParams.decState.currentDelta = pcSSParams.decState.currentDelta + (((signed short int)pinDelta) << BITTIMESCALE);

      while(pcSSParams.decState.currentDelta > 0) {
        if(pcSSParams.decState.bytePos == 0) { // Search for the start bit if we're not already reading a byte
          if(pinState != 0) { // Looking for a start bit and the line's in the idle state
            pcSSParams.decState.currentDelta = 0; // Throw away the rest of the pin state/time
            break;
          } else { // Found the start bit
            pcSSParams.decState.bytePos = 1;
            pcSSParams.decState.byteVal = 0;
            pcSSParams.decState.currentDelta -= BITTIME;
            // PIND |= (1<<PIND3);
          }
        }

        while(pcSSParams.decState.currentDelta > 0) { // Consume the serial data
          pcSSParams.decState.currentDelta -= BITTIME;
          if(pcSSParams.decState.bytePos == 9) { // Skip the stop bit
            if(pinState == 0) { // Framing error!
              pcSSParams.decState.frameError = true;
            } else { // Store the bit
              // Serial.print(pcSSParams.decState.byteVal);
              pcSSParams.decState.availableChars[pcSSParams.decState.bufferHead] = pcSSParams.decState.byteVal;
              pcSSParams.decState.bufferHead = (pcSSParams.decState.bufferHead + 1) & 0x3;
              bufferFull = ((pcSSParams.decState.bufferHead+1) & 0x3) == pcSSParams.decState.bufferTail;
            }
            // PIND |= (1<<PIND3);
            pcSSParams.decState.bytePos = 0;
            break;
          }
          // Shift in bits
          pcSSParams.decState.byteVal = pcSSParams.decState.byteVal >> 1;
          pcSSParams.decState.byteVal |= pinState;
          pcSSParams.decState.bytePos++;
        }
      }

      pcSSParams.decState.lastState = pinState;
      pcSSParams.decState.lastTime = millis();
    }
  } else if(pcSSParams.decState.bytePos != 0 && (millis() - pcSSParams.decState.lastTime) > 0) {
    uint8_t oldSREG = SREG;
    cli();
      srvSSStateChange(); // Force an update
    SREG = oldSREG;
  }
  return (pcSSParams.decState.bufferHead + 4 - pcSSParams.decState.bufferTail) & 0x3;
}
  
void srvSSStateChange()
{
  extern volatile unsigned long int timer0_overflow_count;

  // Get the pin value as soon as possible (without using digital read)
  uint8_t pinValue = ((*pcSSParams.pinData.pinReg) & pcSSParams.pinData.pinMask) != 0 ? 0x1 : 0x0;
  // Then measure the current time (assumes a 16MHz clock and timer0 providing the millis functions)
  unsigned long int m;
  uint8_t t;
  m = timer0_overflow_count;
  t = TCNT0;
  if ((TIFR0 & _BV(TOV0)) && (t < 255)) {
    m++;
  }

  uint8_t localPCTop = pcSSParams.pinData.pcTop; // Store this volatile value
  
  unsigned long int tMeasured = (m << 8) + t; // Current time in units of 4us
  unsigned long int tTaken = tMeasured - pcSSParams.pinData.lastTime;
  *(pcSSParams.pinData.pinStateCngDelt + localPCTop) = (tTaken & 0xFFFFFF00) ? 0xFF : tTaken; // If (tTaken > 255) == (tTaken & 0xFFFFFF00)

  localPCTop = (localPCTop+1) & 0x7F; // Limit to 128 bit transitions
  pcSSParams.pinData.pcTop = localPCTop; // Signal that we've made a measurement, atomic?
  pcSSParams.pinData.lastTime = tMeasured; // Record the time of the last measurement
  if((localPCTop & 0x7) == 0x0) { // Move to the next packed byte if necessary
    pcSSParams.pinData.stateStore = pcSSParams.pinData.pinStateHist + (localPCTop>>3);
    *pcSSParams.pinData.stateStore = 0;
  }

  *pcSSParams.pinData.stateStore |= pinValue << (localPCTop & 0x7); // Store the pin reading
}
