struct {
  struct {
    uint8_t pinMask;
    volatile uint8_t *pinReg;
    byte iCount;
    byte rDir;
    byte target;
    signed short int tError;
  } intData;
  struct {
    volatile unsigned short aValu;
    volatile unsigned short nBits;
  } txData;  
} ssTx;

void setupSoftTx()
{
  uint8_t txPort = digitalPinToPort(TXPIN);
  ssTx.intData.pinMask = digitalPinToBitMask(TXPIN);
  ssTx.intData.pinReg = portOutputRegister(txPort);
  ssTx.intData.iCount = 0;
  ssTx.intData.rDir = 1;
  ssTx.intData.target = 63;
  ssTx.intData.tError = 369;
  ssTx.txData.nBits = 0;
}

void sendChar(char val)
{
  while(ssTx.txData.nBits) {} // wait for the next character to go
  ssTx.txData.aValu = 0b1000000000 | (val << 1); // blank
  ssTx.txData.nBits = 10;
  TIMSK2 |= (1<<TOIE2);
}

void srvTXTimer()
{
  // Ticks at about 37037Hz
  if(ssTx.intData.iCount++ == 3) {
    signed short int tNow = 0;
    byte tNew, tOld, cDir;

    if(ssTx.intData.rDir == 0) {
      do {
        tNew = TCNT2;
        cDir = tNew > tOld ? 0 : 1;
        tOld = tNew;
      } while ( !((tNew >= ssTx.intData.target) || cDir == 1) ); // Enough ticks have passed or we start down counting
    } else {
      do {
        tNew = TCNT2;
        cDir = tNew > tOld ? 0 : 1;
        tOld = tNew;
      } while (cDir == 0); // Wait until we're down counting
      do {
        tNew = TCNT2;
        cDir = tNew > tOld ? 0 : 1;
        tOld = tNew;
      } while ( !((tNew <= ssTx.intData.target) || cDir == 0) ); // Wait until enough ticks have passed or we start up counting
      if(cDir == 0) { // In case we roll over in the above loop
        tNow = 432;
      }
    }

    if(ssTx.txData.aValu & 0x1) {
      (*ssTx.intData.pinReg) |= ssTx.intData.pinMask;
    } else {
      (*ssTx.intData.pinReg) &= ~ssTx.intData.pinMask;
    }

    ssTx.intData.iCount = 0;

    // Release interrupts
    uint8_t oldSREG = SREG;
    cli();

    if(cDir) {
      tNow += 432 - (signed short int)tNew;
    } else {
      tNow += tNew;
    }
    ssTx.intData.tError = ssTx.intData.tError - tNow;
    signed short int tToGo = (371 + ssTx.intData.tError) - 432 + (signed short int)tNow;
    if(tToGo < 0) { // Skip an interrupt
      tToGo = 432 + tToGo;
      SREG = oldSREG; // turn interrupts back on
      ssTx.intData.iCount++;
      // Release interrupts
      uint8_t oldSREG = SREG;
      cli();
    }
    ssTx.intData.tError = tToGo;
    ssTx.intData.rDir = tToGo >= 216 ? 1 : 0;
    ssTx.intData.target = tToGo >= 216 ? 432 - tToGo : tToGo < 0 ? 0 : tToGo;

    ssTx.txData.aValu >>= 1;
    if(!--ssTx.txData.nBits) {
      TIMSK2 &= ~(1<<TOIE2);
    }

    // Return to pervious interrupt state
    SREG = oldSREG;
  }
}
