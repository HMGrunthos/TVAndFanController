#include <avr/pgmspace.h>
#include <CmdMessenger.h>
#include <Streaming.h>

// Must not conflict / collide with our message payload data. Fine if we use base64 library ^^ above
const char FIELDSEPARATOR = ',';
const char COMMANDSEPARATOR = ';';

const unsigned char MASTERPIDCOUNTDOWN = 61; // about 16 Hz
const unsigned char TEMPPIDCOUNTDOWN = 8; // about 2 Hz

const unsigned short int PWMSCALE = 7;
const unsigned short int RPMSCALE = 2;
const unsigned short int TEMPSCALE = 5;

const unsigned short int INITIALPWM = (183<<PWMSCALE);
const unsigned short int INITIALRPM = (1500<<RPMSCALE);

const unsigned short int TIMER1TOP = 340;
const unsigned short int MAXPWM = (TIMER1TOP<<PWMSCALE);
const unsigned short int MINPWM = (30<<PWMSCALE);

const unsigned short int MAXRPM = (2150<<RPMSCALE);
const unsigned short int MINRPM = (400<<RPMSCALE);

const signed short int INITIALTARGETTEMP = (35<<TEMPSCALE);
const signed short int INITIALCURRENTTEMP = (20<<TEMPSCALE);

volatile unsigned char rpmCount;

volatile byte masterPIDCount;
volatile byte tempPIDCount;
volatile boolean pwmPIDUpdate;
volatile boolean tempPIDUpdate;

unsigned long int lastTempUpdate = 0;
signed short int currentTemp = INITIALCURRENTTEMP;
signed short int tempTarget = INITIALTARGETTEMP;

// Attach a new CmdMessenger object to the default Serial port
CmdMessenger cmdMessenger = CmdMessenger(Serial, FIELDSEPARATOR, COMMANDSEPARATOR);

// Commands we send from the Arduino to be received on the PC
enum
{
  kCOMM_ERROR    = 000, // Lets Arduino report serial port comm error back to the PC (only works for some comm errors)
  kACK           = 001, // Arduino acknowledges cmd was received
  kARDUINO_READY = 002, // After opening the comm port, send this cmd 02 from PC to check arduino is ready
  kERR           = 003, // Arduino reports badly formatted cmd, or cmd not recognised
  // Now we can define many more 'send' commands, coming from the arduino -> the PC, eg
  // kICE_CREAM_READY,
  // kICE_CREAM_PRICE,
  // For the above commands, we just call cmdMessenger.sendCmd() anywhere we want in our Arduino program.
  kSEND_CMDS_END, // Mustnt delete this line
};

// Commands we send from the PC and want to recieve on the Arduino.
// We must define a callback function in our Arduino program for each entry in the list below vv.
// They start at the address kSEND_CMDS_END defined ^^ above as 004
messengerCallbackFunction messengerCallbacks[] = 
{
  serialMessage,            // 004 in this example
  currentTemperatureMessage,  // 005
  sendIRCommandMessage,  // 006
  NULL
};
// Its also possible (above ^^) to implement some symetric commands, when both the Arduino and
// PC / host are using each other's same command numbers. However we recommend only to do this if you
// really have the exact same messages going in both directions. Then specify the integers (with '=')

void setup()
{
  // Set up TIMER0 for PWM controlling the fan
  TCCR1A = (1<<COM1A1) | (1<<COM1A0);
  TCCR1B = (1<<WGM13) | (1<<CS10);
  ICR1 = TIMER1TOP;
  OCR1A = INITIALPWM >> PWMSCALE;
  pinMode(9, OUTPUT);
  // DDRB |= (1<<DDB1);

  // RPM monitoring variable
  rpmCount = 0;

  // Timer countdowns
  masterPIDCount = MASTERPIDCOUNTDOWN;
  pwmPIDUpdate = false;
  tempPIDCount = TEMPPIDCOUNTDOWN;
  tempPIDUpdate = false;

  Serial.begin(115200);

  Serial.println("\n[memCheck]");
  Serial.println(freeRam());

  cmdMessenger.print_LF_CR();   // Make output more readable whilst debugging in Arduino Serial Monitor

  // Attach default / generic callback methods
  cmdMessenger.attach(kARDUINO_READY, arduino_ready);
  cmdMessenger.attach(unknownCmd);

  // Attach my application's user-defined callback methods
  attach_callbacks(messengerCallbacks);
  
  arduino_ready();

  setupIRTransmitter();

  pinMode(2, INPUT);

  attachInterrupt(0, rpmTick, RISING);
  OCR0A = 128;
  TIMSK0 |= (1<<OCIE0A);
}

void loop()
{
  static unsigned short int rpm = 0;
  static unsigned short int pwmVal = INITIALPWM;
  static unsigned short rpmTarget = INITIALRPM;
  static unsigned long int timeCalc;

  if (rpmCount >= 20 || ((millis() - timeCalc) > 3000)) {
    if((millis() - timeCalc) > 3000) {
      Serial.println("Speed too low");
    }
    static unsigned char rpmFilterIndex = 0;
    static unsigned short int rpmFilter[4] = {0, 0, 0, 0};

    // Update RPM every 20 counts, increase this for better RPM resolution,
    timeCalc = millis() - timeCalc;
    unsigned short int localRPM = rpmCount*(((unsigned short int)30000)/timeCalc);
    
    rpm -= rpmFilter[rpmFilterIndex%4];
    rpmFilter[rpmFilterIndex++%4] = localRPM;
    rpm += localRPM;

    rpmCount = 0;
    timeCalc = millis();
  }

  if(pwmPIDUpdate == true) {
    static unsigned short int lastRPM;

    signed short int errorRPM;
    unsigned short int pdPWM;

    errorRPM = (rpmTarget - (signed short int)rpm);

    pdPWM = pwmVal + ((errorRPM + 4) >> 3) - ((errorRPM + 16) >> 5); // + PTerm
    pdPWM = pdPWM - (((signed short int)rpm - (signed short int)lastRPM + 2) >> 2); // + DTerm

    lastRPM = rpm;

    if(pdPWM < MINPWM) {
      pdPWM = MINPWM;
    } else if(pdPWM > MAXPWM) {
      pdPWM = MAXPWM;
    }

    pwmVal = pdPWM;
    OCR1A = (pdPWM + (1<<(PWMSCALE-1))) >> PWMSCALE;

    pwmPIDUpdate = false;
  }

  if(tempPIDUpdate == true) {
    static signed long int tempErrorInteg = ((signed long int)INITIALRPM) << 1;

    signed short int errorTemp;
    signed short int piRPM;

    if(millis() - lastTempUpdate > 2000) {
      lastTempUpdate = millis();
      currentTemp += currentTemp <= (38 << TEMPSCALE) ? (1 << TEMPSCALE) : 0;
    }

    errorTemp = currentTemp - tempTarget;
    tempErrorInteg += errorTemp;

    Serial.print("RPM : ");
    Serial.print(rpm>>RPMSCALE, DEC);
    Serial.print(" RPM Target : ");
    Serial.print(rpmTarget>>RPMSCALE);
    Serial.print(" PWM value : ");
    Serial.print(pwmVal);
    Serial.print("\\");
    Serial.print((pwmVal + (1<<(PWMSCALE-1))) >> PWMSCALE);
    Serial.print(" Temp target : ");
    Serial.print(tempTarget/(float)(1<<TEMPSCALE));
    Serial.print(" Current temp : ");
    Serial.println(currentTemp/(float)(1<<TEMPSCALE));

    Serial.print("Error temp : ");
    Serial.print(errorTemp);
    Serial.print(" Integ error temp : ");
    Serial.print(tempErrorInteg);

    piRPM = errorTemp << 3; // + PTerm
    Serial.print(" PTerm : ");
    Serial.print(piRPM);
    piRPM = piRPM + ((tempErrorInteg + 1) >> 1);
    Serial.print(" ITerm : ");
    Serial.print(((tempErrorInteg + 4) >> 1));
    Serial.print(" Total : ");
    Serial.println(piRPM);

    if(piRPM < MINRPM) {
      piRPM = MINRPM;
      tempErrorInteg -= errorTemp;
    } else if(piRPM > MAXRPM) {
      piRPM = MAXRPM;
      tempErrorInteg -= errorTemp;
    }

    rpmTarget = piRPM;

    tempPIDUpdate = false;
  }
  
  // Process incoming serial data, if any
  cmdMessenger.feedinSerialData();
}

// ------------------ C A L L B A C K  M E T H O D S -------------------------
void serialMessage()
{
  // Message data is any ASCII bytes (0-255 value). But can't contain the field
  // separator, command separator chars you decide (eg ',' and ';')
  cmdMessenger.sendCmd(kACK, "Serial message received.");
  while(cmdMessenger.available()) {
    char buf[50] = { '\0' };
    cmdMessenger.copyString(buf, sizeof(buf));
    if(buf[0])
      cmdMessenger.sendCmd(kACK, buf);
  }
}

void currentTemperatureMessage()
{
  unsigned char fieldNo = 0;

  // Message data is any ASCII bytes (0-255 value). But can't contain the field
  // separator, command separator chars you decide (eg ',' and ';')
  // cmdMessenger.sendCmd(kACK, "Current temperature message message received");
  while(cmdMessenger.available()) {
    char buf[50] = { '\0' };
    char *bufPtr = buf;

    cmdMessenger.copyString(buf, sizeof(buf));

    while(*bufPtr != '\0') {
      if(isdigit(*bufPtr)) {
        signed short int tempRead = 0;

        tempRead = readDigits(bufPtr, TEMPSCALE);

        if(fieldNo == 0) {
          lastTempUpdate = millis();
          currentTemp = tempRead;
        } else if(fieldNo == 1) {
          tempTarget = tempRead;
        }

        fieldNo++;

        break;
      }
      bufPtr++;
    }
  }
}

void sendIRCommandMessage()
{
  unsigned char fieldNo = 0;

  // Message data is any ASCII bytes (0-255 value). But can't contain the field
  // separator, command separator chars you decide (eg ',' and ';')
  cmdMessenger.sendCmd(kACK, "IR Command temperature message message received");
  while(cmdMessenger.available()) {
    char buf[50] = { '\0' };
    char *bufPtr = buf;

    cmdMessenger.copyString(buf, sizeof(buf));

    while(*bufPtr != '\0') {
      if(isdigit(*bufPtr)) {
        signed short int commandRead = 0;

        commandRead = readDigits(bufPtr, 0);

        if(!decodeAndSendIRCommand(commandRead)) {
          cmdMessenger.sendCmd(kERR, "Invalid IR command");
        }

        break;
      }
      bufPtr++;
    }
  }
}

// ------------------ D E F A U L T  C A L L B A C K S -----------------------
void arduino_ready()
{
  // In response to ping. We just send a throw-away Acknowledgement to say "im alive"
  cmdMessenger.sendCmd(kACK, "Arduino ready");
}

void unknownCmd()
{
  // Default response for unknown commands and corrupt messages
  cmdMessenger.sendCmd(kERR, "Unknown command");
}

// ------------------ S E T U P ----------------------------------------------
void attach_callbacks(messengerCallbackFunction *callbacks)
{
  int i = 0;
  int offset = kSEND_CMDS_END;
  while(callbacks[i]) {
    cmdMessenger.attach(offset + i, callbacks[i]);
    i++;
  }
}

// ---------------- Interrupt handlers --------------
void rpmTick()
{
  // Each rotation, this interrupt function is run twice
 rpmCount++;
}

ISR(TIMER0_COMPA_vect)
{
  // Ticks at 976.56Hz by default
  if(--masterPIDCount == 0) { // Will hit zero at 15Hz
    pwmPIDUpdate = true;
    masterPIDCount = MASTERPIDCOUNTDOWN;
    if(--tempPIDCount == 0) {
      tempPIDUpdate = true;
      tempPIDCount = TEMPPIDCOUNTDOWN;
    }
  }
}

// ---------------- Utility functions --------------
signed short int readDigits(char *bufPtr, byte fpScale)
{
  signed short int valRead = 0;
  unsigned char nDecPlaces = 0;

  while(isdigit(*bufPtr) || *bufPtr == '.') {
    if(isdigit(*bufPtr)) {
      valRead = valRead*10 + *bufPtr - 48;
      nDecPlaces *= 10;
    } else {
      nDecPlaces = 1;
    }
    bufPtr++;
  }
  
  valRead = (valRead << fpScale) / (nDecPlaces==0?1:nDecPlaces);

  return valRead;
}

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
