#include "pins_arduino.h"

const static byte PCPOWERPIN = 6;


IRCodeHandler::IRCodeHandler()
{
  pulsePinTimeOut = lastIRCodeTime = millis();
  currentKeyDown = 0;
  currPulsePin = 0;
  nKeyPress = 0;
}

void IRCodeHandler::updateKeyState()
{
  unsigned long int cTime = millis();
  if(currPulsePin && (cTime - pulsePinTimeOut) > IOPULSETIME) {
    digitalWrite(currPulsePin, LOW);
    currPulsePin = 0;
  }
  if(currentKeyDown && (cTime - lastIRCodeTime) > KEYUPTIMEOUT) {
    handleKeyState(currentKeyDown, UP, 0);
    currentKeyDown = 0;
  }
  if(nKeyPress && (cTime - lastIRCodeTime) > DBLTAPTIMEOUT) {
    nKeyPress = 0;
  }
}

void IRCodeHandler::handleIRSignal(unsigned long long int rawIRValue)
{
  boolean repeated = rawIRValue == lastKeyCode;
  unsigned short int irValue = rawIRValue & 0x7FF;
  
  if(!repeated) { // An unrepeated keypress
    if(currentKeyDown) {
      handleKeyState(currentKeyDown, UP, 0);
      currentKeyDown = 0;
    }
    // This key has been pressed more than once in succession
    if(irValue == (lastKeyCode & 0x7FF)) {
      nKeyPress++;
    } else {
      nKeyPress = 1;
    }
    if(nKeyPress == 1) {
      lastIRCodeTime = millis();
    }
    handleKeyState(irValue, PRESS, nKeyPress);
  } else {
    if(currentKeyDown) { // The key is down and we're getting key repeats
      handleKeyState(currentKeyDown, DOWN, 0);
      lastIRCodeTime = millis();
    } else if(!currentKeyDown && (millis() - lastIRCodeTime) > REPEATGUARD) { // Key was repeated and  it's been some time since the last key press
      nKeyPress = 0; // Key repeat off
      currentKeyDown = irValue;
      handleKeyState(currentKeyDown, DOWN, 0);
      lastIRCodeTime = millis();
    } else { // Key has not been held down long enough
    }
  }
  lastKeyCode = rawIRValue;
}

void IRCodeHandler::togglePin(byte pin)
{
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  *portInputRegister(port) |= bit;
}

void IRCodeHandler::pulsePin(byte pin)
{
  currPulsePin = pin;
  pulsePinTimeOut = millis();
  digitalWrite(pin, HIGH);
}

void IRCodeHandler::handleKeyState(unsigned short int irValue, KeyStateType keyState, unsigned short int nKeyPress)
{
  switch(keyState) {
    case PRESS:
      if(standAloneMode) {
        if(serveStandAloneOnlyPRESS(irValue)) { // We processed the key
          break;
        }
      }
      switch(irValue) { // Haven't processed the key yet do these in any mode
        case 0x7BD: // TV power
          disableIRInterrupts();
          decodeAndSendIRCommand(13);
          break;
        case 0x79C: // TV Aspect
          disableIRInterrupts();
          decodeAndSendIRCommand(29);
          break;
        case 0x79B: // TV Input
          disableIRInterrupts();
          decodeAndSendIRCommand(3);
          break;
        case 0x78B: // TV Multi PIP
          disableIRInterrupts();
          decodeAndSendIRCommand(32);
          break;
        case 0x7AE: // TV Swap
          disableIRInterrupts();
          decodeAndSendIRCommand(22);
          break;
        case 0x7B8: // TV Select
          disableIRInterrupts();
          decodeAndSendIRCommand(30);
          break;
        case 0x7A9: // TV Move/Zoom
          disableIRInterrupts();
          decodeAndSendIRCommand(23);
          break;
        case 0x78A: // TV Component input
          disableIRInterrupts();
          decodeAndSendIRCommand(24);
          break;
        case 0x78E: // TV Digital input
          disableIRInterrupts();
          decodeAndSendIRCommand(1);
          break;
        case 0x7BB: // PC power
          if(nKeyPress == 2) { // Only on a double tap
            pulsePin(PCPOWERPIN);
          }
          break;
      }
      break;
    case DOWN:
      if(standAloneMode) {
        if(serveStandAloneOnlyDOWN(irValue)) { // We processed the key
          break;
        }
      }
      switch(irValue) {
        case 0x7BB: // PC power
          digitalWrite(PCPOWERPIN, HIGH);
          break;
      }
      break;
    case UP:
      switch(irValue) {
        case 0x7BB: // PC power
          digitalWrite(PCPOWERPIN, LOW);
          break;
      }
      break;
  }
}

boolean IRCodeHandler::serveStandAloneOnlyPRESS(unsigned short int irValue)
{
  switch(irValue) {
    case 0x7A5: // TV Enter
      disableIRInterrupts();
      decodeAndSendIRCommand(16);
      break;
    case 0x794: // TV Up
      disableIRInterrupts();
      decodeAndSendIRCommand(17);
      break;
    case 0x795: // TV Down
      disableIRInterrupts();
      decodeAndSendIRCommand(18);
      break;
    case 0x796: // TV Left
      disableIRInterrupts();
      decodeAndSendIRCommand(19);
      break;
    case 0x797: // TV Right
      disableIRInterrupts();
      decodeAndSendIRCommand(20);
      break;
    case 0x79F: // TV Return/Back
      disableIRInterrupts();
      decodeAndSendIRCommand(28);
      break;
    case 0x790: // TV Vol up
      disableIRInterrupts();
      decodeAndSendIRCommand(8);
      break;
    case 0x791: // TV Vol down
      disableIRInterrupts();
      decodeAndSendIRCommand(9);
      break;
    case 0x78D: // TV Setup
      disableIRInterrupts();
      decodeAndSendIRCommand(21);
      break;
    case 0x792: // TV Normalise
      disableIRInterrupts();
      decodeAndSendIRCommand(6);
      break;
    case 0x78F: // TV Mute
      disableIRInterrupts();
      decodeAndSendIRCommand(11);
      break;
    case 0x79A: // TV Picture
      disableIRInterrupts();
      decodeAndSendIRCommand(4);
      break;
    case 0x799: // TV Sound
      disableIRInterrupts();
      decodeAndSendIRCommand(5);
      break;
    case 0x78C: // TV Surround
      disableIRInterrupts();
      decodeAndSendIRCommand(10);
      break;
    case 0x798: // TV Pos/Size
      disableIRInterrupts();
      decodeAndSendIRCommand(31);
      break;
    case 0x781: // TV Display source/channel
      disableIRInterrupts();
      decodeAndSendIRCommand(12);
      break;
    default:
      return false;
  }
  return true;
}

boolean IRCodeHandler::serveStandAloneOnlyDOWN(unsigned short int irValue)
{
  switch(irValue) {
    case 0x7A5: // TV Enter
      disableIRInterrupts();
      decodeAndSendIRCommand(16);
      break;
    case 0x794: // TV Up
      disableIRInterrupts();
      decodeAndSendIRCommand(17);
      break;
    case 0x795: // TV Down
      disableIRInterrupts();
      decodeAndSendIRCommand(18);
      break;
    case 0x796: // TV Left
      disableIRInterrupts();
      decodeAndSendIRCommand(19);
      break;
    case 0x797: // TV Right
      disableIRInterrupts();
      decodeAndSendIRCommand(20);
      break;
    case 0x79F: // TV Return
      disableIRInterrupts();
      decodeAndSendIRCommand(28);
      break;
    case 0x790: // TV Vol up
      disableIRInterrupts();
      decodeAndSendIRCommand(8);
      break;
    case 0x791: // TV Vol down
      disableIRInterrupts();
      decodeAndSendIRCommand(9);
      break;
    case 0x78F: // TV Mute
      disableIRInterrupts();
      decodeAndSendIRCommand(11);
      break;
    case 0x78C: // TV Surround
      disableIRInterrupts();
      decodeAndSendIRCommand(10);
      break;
    default:
      return false;
  }
  return true;
}
