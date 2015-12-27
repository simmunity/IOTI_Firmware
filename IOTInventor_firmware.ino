// IOT Inventor Arduino firmware for Uno and Mega 2560 boards
// Released under the permissive GNU 2.0 license by IOT Inventor / Shannon Bailey / Copyright Dec 2015
//
// This is an early prototype of the Arduino firmware for the cloud/mobile/desktop based IOT Inventor product/service currently in development.
// It is currently a pre-release development version with lots of warts and a memory corruption on Arduino Uno (if globals are moved around).
// It supports a variety of sensors including the Seeed Studio Grove ultrasonic ranger, liquid flow rate sensor, R/C servos
// and the DS2482-100 i2c to 1-wire bridge and DS1820 temperature sensors as well as standard analog input, digital I/O
// and pwm outputs and a timer function.
// The current usage model has the firmware functioning as a simple client to a smart gateway device server over a wired serial link
// or a USB simulated serial link or unreliable sub 1ghz radio link.  It uses brace framing of commands and mainly functions as a
// mode and value getter/setter that is typically setup and then polled every two seconds by the gateway.
//
// To facilitate ongoing development and manual operation of the firmware from the serial console, human typeable and readable
// commands were chosen instead of a more compact machine only readable/writable command langauge.
//
// Please forgive the code inconsistency as it has goen through many partial rewrites as functionality was added and my intention
// was to provide a highly refactored first version for IOT Inventor customers to use, but to assist in debugging the Uno memory
// corruption issue, I've decided to release it prior to commercial release so other developers can help make it better and
// hopefully help me find the corruption bug which has been elusive.  By moving some of the global data around in a different order
// it is possible on an Uno to get the {reset} and {info} commands to not be recognized, indicating their strings in memory
// have been corrupted.  Other corruption can result in command responses being corrupted due to the sprintf format strings being whacked.
//
// 
// Version 0.8.2   Fixed bug in _sprintf for floats that would add characters from the stale buffer to the end
//                 of a formatted float.  Added optional int digits "%f8" and optional int and frac digits "%f7.2" support.
// Version 0.8.1 - Fixed MatchCommand which was not accepting reliable mode commands
// Version 0.8.0 - Wrote 16 and 32 bit numeric formatters and alternative _sprintf to avoid sprintf which causes memory corruption
//                 also fixed current formatted number responses of time and interrupts from a 16 bit values to 32 bit values
//                 Also stopped setting the fraction of time on {s 2 n} or {s 3 n} during an {s t 0} due to possible negative time offset being returned
// Version 0.7.9 - For Ultrasonic sensor, set to INPUT mode during mode set, not OUTPUT mode and added doc
// Version 0.7.8 - Rewrite of code atempting to avoid usage of library code that is causing memory overwrites on Uno.
//                 The problem persists and is due to problems with Arduino libraries.  Decided to switch analog offset to 70 +
// Version 0.7.6 - Adding I2C interface capabilities for DS2482-100 and connected DS18b20's of which a maximum of 8 are supported
// Version 0.7.5 - Refactored how commands are matched and responses are composed
// Version 0.7.4 - Added pin mode 'c' for analog maximmum value over ~60ms time frame measurement and also fixed {reset}
// Version 0.7.3 - To avoid collision between Analog Input 0-15 and Digital I/O 0-15 shift analog pins to 70 + pin #
//                 and error if 0-69 attempts to be analog input and error if 70-99 attempts to not be analog input.
// Version 0.7.2 - Fixed info and reg functions which di not deal with reliableMode properly
// Version 0.7.1 - added Tone output to firmware
// Version 0.7 - added unique board UUID registration capability
// Version 0.6 - adding servo and get value on output pin
// Version 0.5 - base first firmware release
//
// The "{reset}" command and "{s t 0}" commands resets the time counter since last reset back to 0
//   NCS recommends having the the IOT Inventor application (NCS Engine IOT app) reset the time
//   preiodically with "{s t 0}" to avoid problems

// Pin modes:  'i' = digital input, 'o' = digital output, 'a' = analog value from A/D converter in a range of 0 - 1023
//             'c' = analog waveform maximum for current measurement that runs over ~ 60ms
//             'p' = pulse width modulation (PWM) output with duty cycle from 0% to 100% using values from 0 - 255 
//             'n' = interrupt counter and time stamp on pins 2 and 3, for interrupt 0 and 1
//             's' = servo mode with position from 0 - 255
//             't' = tone mode with tone value and duration in milliseconds
//             'w' = 1-wire temperature sensors through a DS2482-100 I2C to 1-wire bridge and DS18b20 temperature sensors
//             'u' = Grove ultrasonic ranging device
//
// Commands:
//   to reset the firmware serial input command buffer use: '~' which responds "{~}"
//   to reset the firmware state and hardware use: "{reset}" or "{reset:4d7a}" which responds "{reset}"
//   to reset the time and interrupt time tracking variables use: "{s t 0}" which responds "{s t 0}"
//   to get the time use: "{g t}" which responds "{g t 48100}" indicating the time since reset is 48.1 seconds
//   to get the firmware version use: "{g v}"  which responds "{'name':'IOT Inventor'; 'board':'Arduino Mega2560'; 'firmware':'0.8.2'}"
//
//   to set pin mode use: "{s pin mode}" which responds "{s pin mode}" 
//   to set pin value use: "{s 123 i}"   which responds "{illegal pin}"
//   to set pin value use: "{s 13 x}"    which responds "{illegal mode}"
//   to set pin mode use: "{s 10 i}"     which responds "{s 10 i}" setting pin 10 to digital input mode
//   to set pin mode use: "{s 70 a}"     which responds "{s 70 a}" setting analog pin 0 to A/D input mode (NOTE  the 70 + offset)
//   to set pin mode use: "{s 70 c}"     which responds "{s 70 c}" setting analog pin 0 to A/D input mode which does 500 samples finding the maximum (NOTE  the 70 + offset)
//   to set pin mode use: "{s 13 o}"     which responds "{s 13 o}" setting pin 13 to digital output mode
//   to set pin mode use: "{s 9 p}"      which responds "{s 3 p}" setting pin 9 to pulse width modulation
//   to set pin mode use: "{s 2 n}"      which responds "{s 2 n}" setting pin 2 to input mode with RISING edge counting interrupt handler
//   to set pin mode use: "{s 3 t}"      which responds "{s 3 t}" setting pin 3 to tone mode
//   to set pin mode use: "{s 74 w}"     which responds "{s 74 w}" setting analog pins 4 and 5 to I2C interface for DS2482-100 bridge to DS18b20s (for Mega it is pins 20 and 21)
//   to set pin mode use: "{s 8 u}"      which responds "{s 8 u}" setting pin 8 digital to function using the Grove UltraSonic Ranging device.

//   to set pin value use: "{s pin value}" which responds "{s pin value}"
//   to set pin value use: "{s 123 0}"     which responds "{illegal pin}"
//   to set pin value use: "{s 0 1}"       which responds "{illegal mode}" if pin 0 was set to digital input or analog input mode
//   to set pin value use: "{s 13 1}"      which responds "{s 13 1}" and sets pin 13 digital output high
//   to set pin value use: "{s 9 123}"     which responds "{s 3 123} and sets the hardware pin 9's PWM hardware to 123
//     note: pwm values are limited to 0 to 255 and the firmware will limit the value to 255: "{s 3 500}" responds "{s 3 255}"
//   to set pin tone and duration use: "{s 3 200}"  which reponds "{s 3 200} and plays the tone for half a second off a timer asynchronously
//   to set pin tone and duration use: "{s 3 200 1000}"  which reponds "{s 3 200 1000} and plays the tone for 1000 milliseconds off a timer asynchronously
//   to set pins I2C_SCL and I2C_SDA use: "{s 74 w}"  which if three DS18B20's are connected responds
// "{s 74 w 0 28b8bcf4200e6}"   // the serial numbers are specific to idividual devices
// "{s 74 w 1 2876c6f4200ba}"
// "{s 74 w 2 28e1a8f42003d}"
//   note: if the board is a Mega2560 use "{s 90 w} and {g 90} to use the I2C to 1-wire temperature sensors

//   to get pin value: "{g pin}"  which responds  "{g pin value}"
//   to get pin value: "{g 123}"  which responds "{illegal pin}"
//   to get pin value: "{g 10}"   which responds "{g 10 1}"  if the pins input state is high, "{g 10 0}" if low
//   to get pin value: "{g 70}"   which responds "{g 70 468}"  if the A/D converter produced a value of 468 from a range of 0 - 1023
//   to get pin value: "{g 70}"   which responds "{g 70 468}" 
//   to get 1-Wire temperatures sensor values from I2C:  "{g 74}  which if 3 sensors are connectd responds
// "{g 74 0  21.25}"
// "{g 74 1  21.06}"
// "{g 74 2  20.56}"
//   if a 1-wire temperature sensor that was discovered during setup {s 74 w} has an error or is
//   disconnected, the response for the sensor is error as shown below for sensor 1:
// "{g 74 0  21.25}"
// "{g 74 1 -99}"
// "{g 74 2  20.56}"
//
//   if pin 2 or 3 are set to mode 'n' for interrupt, the response when command {g 2} is issues is:
//     "{g pin interruptCount interrupTime interruptCountAtLastRequest, interruptTimeAtLastRequest}"
//   to get pin value: "{g 2}"  which responds "{g 2 35 34567 25 25431}"  indicating 35 interrupts at current time 34567, 25 at 25432 last time we checked

// The firmware includes a commands to set a crc16 validated string of 36 characters to store a system designated UUID in EEPROM
//   To register a new UUID, issue a reg(ister) command within 10 minutes of turning the device on
//   If not in reliable mode issue =>  {reg 123456789012345678901234567890123456}
//   If in reliable mode issue =====>  {reg 123456789012345678901234567890123456:67B5}
//   and if it is done within 10 minutes of power on, should reply with
//   {reg set} if not in reliable mode, and {reg set:7F66}
//   and if issued after 10 minutes, will respond with {reg unable} if not in reliable mode and {reg unable:1D87} if in reliable mode.
//   note: the 10 minute registration timer lockout code was removed but should be recoded in a future version.
//
//   To retrieve the registered UUID, issue the command {info}, or in reliable mode {info:7B23} and it will reply
//   {info 123456789012345678901234567890123456} in non reliable mode, or {info 123456789012345678901234567890123456:0F1D} in reliable mode

// The firmware provides reliable communications across RF serial links that don't provide error free, guaranteed delivery
// We provide reliable communications support using CCITT CRC-16 with 0xffff starting value as part of the framing
// validation system by adding a colon and 4 hex values to each command.
// Reliable mode is enabled with {s r 1:427e} and disabled with {s r 0:cb6f} or {reset:4d7a}
// The format used for reliable commands and responses is {command:crc16} for example the command 's 9 p'
// encodes to the full frame {s 9 p:1f02} where '1f02' is the crc16 value, providing an error detection rate
// that is 99.9974 percent, worse case.
//
// NOTES:  Future versions should be bidirectional, asyncronously interacting client/server where this firmware
// is the server.  The firmware can be set to report on specific peripheral states and interrupt events on a schedule
// in the future it can take simple actions based on scheduled logical activities, such as when an input pin is
// in a certain state, set another pin to a state, or report back or both, once every 1/10th second for example.
// Another future improvement beyond reliable mode with CRC16 might be optional and command sequence numbering
// to ensure communication syncronization.
//
// WARNING millis() rolls over after 49 days so do not allow this to occur during normal interaction, reset time every 90 seconds preferably
// to keep the time value returned limited to a few characters.
//

#include <stdio.h>
#include <stdarg.h>
//#include <stdlib.h>
//#include <MemoryFree.h>

#include <Servo.h>
#include <EEPROM.h>
#include <DS2482.h>
#include <Wire.h>

long MeasureInCentimeters(unsigned char pin);

//#define ARDUINO_UNO
#define ARDUINO_MEGA

#ifdef ARDUINO_UNO
#define FIRMWARE_VERSION "'name':'IOT Inventor'; 'board':'Arduino Uno'; 'firmware':'0.8.2'"
#define I2C_SCL 74  // 70 is analog offset plus 4 as pin A4 is SCL
#define I2C_SDA 75  // 70 is analog offset plus 5 as pin A5 is SDA
#endif

#ifdef ARDUINO_MEGA
#define FIRMWARE_VERSION "'name':'IOT Inventor'; 'board':'Arduino Mega 2560'; 'firmware':'0.8.2'"
#define I2C_SCL 90  // 70 is special offset plus 20 as pin D20 is SCL
#define I2C_SDA 91  // 70 is special offset plus 21 as pin D21 is SDA
#endif

// New _sprintf and number formatters
uint8_t * format_u16(uint16_t value, uint8_t * buffer);
uint8_t * format_u32(uint32_t value, uint8_t * buffer);
uint8_t * format_s16(int16_t value, uint8_t * buffer);
uint8_t * format_s32(int32_t value, uint8_t * buffer);
int _sprintf(uint8_t * output, const char * format, ...);

extern volatile unsigned long timer0_millis;
volatile unsigned long currentTime;
volatile unsigned long timeInterval;
#define EEPROM_UUID_ADDRESS 207 // starting byte in EEPROM of registration UUID characters

volatile unsigned long pin2;
volatile unsigned long pin2Last;  // last value of pin2 as seen by main loop code
volatile unsigned long pin2Time;
volatile unsigned long pin2LastTime;
volatile unsigned long pin3;
volatile unsigned long pin3Last;  // last value of pin3 as seen by main loop code
volatile unsigned long pin3Time;
volatile unsigned long pin3LastTime;

#define MAX_PINS 94
unsigned char pinLimit = MAX_PINS;
unsigned char pinModes[MAX_PINS + 2];

#define CLEAR 0
#define STARTED 1

#define SYNC_CHARACTER '{'
#define TERMINATOR_CHARACTER '}'

unsigned char incomingByte;   // for incoming serial data
unsigned char reliableLength;
unsigned char commandIndex = 0;
unsigned char commandState = CLEAR;
unsigned char reliableMode = false;

#define INPUT_LENGTH 60
unsigned char commandBuffer[INPUT_LENGTH + 2];

#define OUTPUT_LENGTH 60
uint8_t outputBuffer[OUTPUT_LENGTH + 2];

#ifdef FLASH_STRING
const char hexDigitLookup[] PROGMEM = "0123456789ABCDEF";
#else
const char hexDigitLookup[] = "0123456789ABCDEF";
#endif

char sUU[] = "s %u %u";
char gUU[] = "g %u %u";
char guUUUU[] = "g %u %ul %ul %ul %ul";

#define MAX_SERVOS 6
Servo servo[MAX_SERVOS];
unsigned char servoValues[MAX_SERVOS];

#define MAX_1WIRE 4
unsigned char addr [MAX_1WIRE] [8];
unsigned char deviceIndex;
unsigned long wireConversionTime;

DS2482 ds(0);


//---------------------------------------------------------------------
void setup() {
  Serial.begin(19200);
  
  commandIndex = 0;
  commandState = CLEAR;
  reliableMode = false;

  resetTime();
}

//---------------------------------------------------------------------
void pin_two () {
  pin2Time = currentTime;
  pin2++;
} 

void pin_three () {
  pin3Time = currentTime;
  pin3++;
}

//---------------------------------------------------------------------
void resetTime(void) {
uint8_t oldSREG = SREG;
  // reset the millis counter register value
  cli();
  timer0_millis = 0;
  SREG = oldSREG;

  if ('n' == pinModes[2]) {  // Set counts and current time to the fraction of time since counts and time were last checked
    pin2 = pin2 - pin2Last;
    pin2Last = 0;
    pin2Time = 0; // pin2Time - pin2LastTime;  // do not set the fraction of time due to possible negative time offset being returned
    pin2LastTime = 0;
  }

  if ('n' == pinModes[3]) {
    pin3 = pin3 - pin3Last;
    pin3Last = 0;
    pin3Time = 0; // pin3Time - pin3LastTime;  // do not set the fraction of time due to possible negative time offset being returned
    pin3LastTime = 0;
  }
  
  timeInterval = currentTime = 0;

  sei();

  wireConversionTime = currentTime;
}  

//---------------------------------------------------------------------
void resetEverything(void) {
  if ('n' == pinModes[2]) {
    detachInterrupt(0);
    cli();
    pin2 = 0;   pin2Last = 0;
    sei();
  }
  if ('n' == pinModes[3]) {
    detachInterrupt(1);
    cli();
    pin3 = 0;   pin3Last = 0;
    sei();
  }
  for (int pinNumber = 0; pinNumber < pinLimit; pinNumber++) {
    pinMode(pinNumber, INPUT);
    pinModes[pinNumber] = 0;
  }
  for (int index = 0; index < MAX_SERVOS; index++)
  {
    // Need to detach servos NTR!
    if (-1 != servoValues[index])
    {
      servo[index].detach();
    }
    servoValues[index] = -1;
  }
  
  resetTime();
  
  commandIndex = 0;
  commandState = CLEAR;
  reliableMode = false;
}

//---------------------------------------------------------------------
void loop() {

// Servo's need to be refreshed every 20ms or so with a 1 to 2ms pulse
// that in the Arduino case is triggered by the PWM in the Servo library  
  if (timeInterval < (currentTime = timer0_millis) ) {
     // this gets run every 15 milliseconds
    timeInterval = currentTime + 15;
    for (int index = 0; index < MAX_SERVOS ; index++) {
      if (-1 != servoValues[index] ) {
        servo[index].write(servoValues[index]);
      }
    } 
  }
  
  while (Serial.available() > 0) {  // read and process the incoming character

    incomingByte = Serial.read();

    if ('~' == incomingByte) {  // reset the serial input stream buffer
        Serial.println("{~}");
        commandIndex = 0;
        commandState = CLEAR;
        continue;
      }

    if (CLEAR == commandState) {
        if (SYNC_CHARACTER == incomingByte) {  // look for start of command sync character
          commandState = STARTED;
          continue;
        }
      }

    if (STARTED == commandState) {      
      if (TERMINATOR_CHARACTER == incomingByte) {  // look for end of command terminator character
        commandBuffer[commandIndex] = 0;  // null terminate the command buffer
        if (false == reliableMode) {
          dispatchCommand (commandIndex);
        }
        else {
          if (true == checkCommandCRC(commandBuffer, commandIndex, &reliableLength) ) {
            dispatchCommand (reliableLength);
          }
        }
        commandIndex = 0;
        commandState = CLEAR;
        return;
      }
      else {
        commandBuffer[commandIndex] = incomingByte;  // store a command character in buffer
        commandIndex++;
        if (INPUT_LENGTH < commandIndex) {
          Serial.println("{overflow}");
          commandIndex = 0;
          commandState = CLEAR;
          return;
        }
      }
    }
  }
}

//---------------------------------------------------------------------
// accepts the command string and total length, computes the CRC16 value for the value
// up to the ':' and returns the actual command length up to the ':'
bool checkCommandCRC(unsigned char * commandString, unsigned char totalLength, unsigned char * commandLength) {
  unsigned char index;
  unsigned int crcValue, crcCheck;
  
  *commandLength = 0;
  for (index = 0; index < totalLength; index++) {
    if (':' == commandString[index]) {
      *commandLength = index;
      
      if(true == hexToInt(&crcCheck, commandString + (index + 1), totalLength - (index + 1)) ) {
        crcValue = crc16(commandString, index);
        if (crcValue == crcCheck) {
          return (true);
        }
      }
    }
  }
  return (false);
}

//---------------------------------------------------------------------
void printFrame(const char * outputString)
{
  unsigned int value;
  unsigned char index;
  unsigned char hexDigit;
  unsigned char hexChar;
    
  Serial.write(SYNC_CHARACTER);  
  Serial.print((const char *)outputString);
  if (true == reliableMode)
  {
    Serial.write(':');
    
    value = crc16((unsigned char *)outputString, strlen((const char *)outputString) );
    for (index = 0; index < 4; index++) {
      hexDigit = (value & 0xf000) >> 12;
      hexChar = hexDigitLookup[hexDigit];
      Serial.write(hexChar);
      value = value << 4;
    }
  }
  Serial.write(TERMINATOR_CHARACTER);
  Serial.write('\n');
}

//---------------------------------------------------------------------
// Match lower and upper case commands
bool MatchCommand(char * command, char * match)
{
  char * pCommand = command;
  char * pMatch = match;
  while(((*pCommand) == (*pMatch)) || ((*pCommand) == (32 + *pMatch)) )
  {
    pCommand++;
    pMatch++;
    if (((0 == *pCommand) || (':' == *pCommand)) && (0 == *pMatch) )
      return true;
  }
  return false;
}

//---------------------------------------------------------------------
void dispatchCommand (unsigned char commandLength) {
  int pinNumber;
  unsigned int pinValue;
  unsigned char pinModeValue;
  unsigned char index;
  
  unsigned long intCount; // for interrupt pins
  unsigned long pinTime;
  
  unsigned int infoCRC; // for eeprom info
  
  int duration;  // for tone function and sensorMax (500 interations looking for maximum analog value)
  unsigned int sensorMax;
  
  uint8_t byteIndex;
  uint8_t displayIndex;
  uint8_t type_s;
  uint8_t present = 0;
  uint8_t data[16];
  uint8_t i;
    
  pinNumber = 0;
  pinValue = 0;
  pinModeValue = 'u';
  index = 2;  // set to first character of pin number or command modifier
      
// call function to actually set a pins output value, based on output mode, digital output or pwm output
  if (('s' == commandBuffer[0]) || ('S' == commandBuffer[0]) ) {  // set pint value commands: "{s 13 1}"  or "{s 3 35}"
    if (('0' <= commandBuffer[index]) && ('9' >= commandBuffer[index]) ) {
      pinNumber = decToInt(&index);
      if (pinNumber > pinLimit) {
        printFrame("illegal pin");
        return;
      }
    }
    else {  // check if it is a "{s t 0}"  to reset time back to zero including interrupt handler time tracking variables
      if ( (('t' == commandBuffer[index]) || ('T' == commandBuffer[index])) && (' ' == commandBuffer[index + 1])
        && ('0' == commandBuffer[index + 2]) && (5 == commandLength) ) {
          resetTime();
          printFrame("s t 0");
          return;
      } else { // check if it is a "{s r 1}" or "{s r 0} to set or unset reliable mode with CRC16 values
        if ( (('r' == commandBuffer[index]) || ('R' == commandBuffer[index]) ) && (' ' == commandBuffer[index + 1])
          && ( (5 == commandLength) || ( (10 == commandLength) &&  (':' == commandBuffer[5])
          && ('4' == commandBuffer[6]) && ('2' == commandBuffer[7]) && ('7' == commandBuffer[8]) && ( ('E' == commandBuffer[9]) || ('e' == commandBuffer[9]) ) ) ) ) {
          if ('1' == commandBuffer[index + 2]) {
            reliableMode = true;
            printFrame("s r 1");
            return;
          }
          else {
            if ('0' == commandBuffer[index + 2]) {
              printFrame("s r 0");
              reliableMode = false;
              return;
            }
          }             
        } else {
          printFrame("illegal set command");
          return;
        }
      }
    }

    if (('0' <= commandBuffer[index]) && ('9' >= commandBuffer[index]) ) {
      pinValue  = decToInt(&index);
      if (pinValue >= MAX_PINS) {
        printFrame("illegal pin value");
        return;
      }
     
    // call function to actually set a pin to a value here
      switch (pinModes[pinNumber]) {
        case 'o':
          pinValue = 0 != pinValue;
          digitalWrite (pinNumber, 0 != pinValue);
          _sprintf(outputBuffer, sUU, pinNumber, pinValue);
          
          printFrame((const char *)outputBuffer);
          break;

        case 'p':
          if (255 < pinValue)
            pinValue = 255;
          analogWrite (pinNumber, pinValue);
          _sprintf(outputBuffer, sUU, pinNumber, pinValue);
          printFrame((const char *)outputBuffer);
          break;

        case 's':
          if (179 < pinValue)
            pinValue = 179;
          servoValues[pinNumber] = pinValue;
          servo[pinNumber].write(pinValue);              
          _sprintf(outputBuffer, sUU, pinNumber, pinValue);
          printFrame((const char *)outputBuffer);
          break;
  
        case 't':
          duration = decToInt(&index);
          tone(pinNumber, pinValue, (duration != -1) ? duration : 500 );              
          if (-1 == duration)
            _sprintf(outputBuffer, sUU, pinNumber, pinValue);
          else
            _sprintf(outputBuffer, "s %u %u %u", pinNumber, pinValue, duration);          
          printFrame((const char *)outputBuffer);
          break;
          
        default:
          printFrame("illegal mode");
      }
    } 
    else {
// call function to actually set a pins operating mode, input, analog value, digital output, pwm output or interrupt input
      pinModeValue = commandBuffer[index];
     
      if ((2 == pinValue) && ('n' == pinModes[pinValue]) ) {
        detachInterrupt(0);
        pin2 = 0;  pin2Last = 0;
        pin2LastTime = 0;  pin2Time = 0;
        pinModes[pinValue] = 0;
      }
      else {
        if ((3 == pinValue) && ('n' == pinModes[pinValue]) ) {
          detachInterrupt(1);
          pin3 = 0;  pin3Last = 0;
          pin3LastTime = 0;  pin3Time = 0;
          pinModes[pinValue] = 0;
        }
      }

      switch (pinModeValue)
      {
        case 'i': case 'I':
          if (70 <= pinNumber)
            goto illegal;
          pinMode(pinNumber, INPUT);
          pinModes[pinNumber] = 'i';     
          _sprintf(outputBuffer, "s %u i", pinNumber);
          printFrame((const char *)outputBuffer);
          break;

        case 'a': case 'A':
          if (70 > pinNumber)
            goto illegal;
          if ('w' == pinModes[pinNumber]) // once the I2C interface is setup it cannot be reassigned back to analog
            goto illegal;
          pinMode(pinNumber, INPUT);
          pinModes[pinNumber] = 'a';
          _sprintf(outputBuffer, "s %u a", pinNumber);
          printFrame((const char *)outputBuffer);
          break;

        case 'c': case 'C':
          if (70 > pinNumber)
            goto illegal;
          if ('w' == pinModes[pinNumber]) // once the I2C interface is setup it cannot be reassigned back to analog
            goto illegal;
          pinMode(pinNumber, INPUT);
          pinModes[pinNumber] = 'c';
          _sprintf(outputBuffer, "s %u c", pinNumber);
          printFrame((const char *)outputBuffer);
          break;
          
        case 'o': case 'O':
          if (70 <= pinNumber)
            goto illegal;
          pinMode(pinNumber, OUTPUT);
          pinModes[pinNumber] = 'o';
          _sprintf(outputBuffer, "s %u o", pinNumber);
          printFrame((const char *)outputBuffer);
          break;

        case 'p': case 'P':
          if (70 <= pinNumber)
            goto illegal;
          pinMode(pinNumber, OUTPUT);
          pinModes[pinNumber] = 'p';
          _sprintf(outputBuffer, "s %u p", pinNumber);
          printFrame((const char *)outputBuffer);
          break;

        case 'n': case 'N':  // setup interrupt mode using digital input pin mode and attach an interrupt handler
          if (2 == pinNumber) {
            pin2 = 0;  pin2Last = 0;
            pin2Time = (pin2LastTime = currentTime);
            pinMode(pinNumber, INPUT);       
            attachInterrupt(0, pin_two, RISING);
          }
          else {
            if (3 == pinNumber) {
              pin3 = 0;  pin3Last = 0;
              pin3Time = (pin3LastTime = currentTime);
              pinMode(pinNumber, INPUT);       
              attachInterrupt(1, pin_three, FALLING);
            }
            else {
              goto illegal;
            }
          }
          pinModes[pinNumber] = 'n';
          _sprintf(outputBuffer, "s %u n", pinNumber);
          printFrame((const char *)outputBuffer);
          break;

        case 's': case 'S':
          if (70 <= pinNumber)
            goto illegal;
// Scan for an existing setup to avoid duplication
          if (pinModes[pinNumber] != 's')
          {
            servoValues[pinNumber] = 0;
            servo[pinNumber].attach(pinNumber, 1000, 2000);
          
            pinModes[pinNumber] = 's';
          }
          _sprintf(outputBuffer, "s %u s", pinNumber);
          printFrame((const char *)outputBuffer);
          break;

        case 't': case 'T':
          if (70 <= pinNumber)
            goto illegal;
          pinMode(pinNumber, OUTPUT);
          pinModes[pinNumber] = 't';
          _sprintf(outputBuffer, "s %u t", pinNumber);
          printFrame((const char *)outputBuffer);
          break;

// NTR need to add pin usage lockouts
        case 'w': case 'W':
          if ((I2C_SCL == pinNumber) || (I2C_SDA == pinNumber) )
          {
            if ('w' == pinModes[pinNumber]) // once the I2C interface is setup, just run the directory
              goto getDevices;

            Wire.begin();
            ds.reset();
            ds.wireReset();
  //configure DS2482 to use active pull-up instead of pull-up resistor 
  //configure returns 0 if it cannot find DS2482 connected 
            if (!ds.configure(DS2482_CONFIG_APU)) 
            {
              goto illegal; 
            }
            pinModes[I2C_SCL] = 'w';
            pinModes[I2C_SDA] = 'w';
getDevices:
            deviceIndex = 0;
            ds.wireResetSearch();
// Build a directory of one-wire devices
            while(ds.wireSearch(addr[deviceIndex]) )
            {
              if (MAX_1WIRE > deviceIndex) {
//                sprintf((char *)outputBuffer, "s %d w %d %02x%02x%02x%02x%02x%02x%02x%02x", pinNumber, deviceIndex, addr[deviceIndex][0], addr[deviceIndex][1], addr[deviceIndex][2], addr[deviceIndex][3], addr[deviceIndex][4], addr[deviceIndex][5], addr[deviceIndex][6], addr[deviceIndex][7]);
//                printFrame((const char *)outputBuffer);
                _sprintf(outputBuffer, "s %u w %u %xh%xh%xh%xh%xh%xh%xh%xh", pinNumber, deviceIndex, addr[deviceIndex][0], addr[deviceIndex][1], addr[deviceIndex][2], addr[deviceIndex][3], addr[deviceIndex][4], addr[deviceIndex][5], addr[deviceIndex][6], addr[deviceIndex][7]);
                printFrame((const char *)outputBuffer);
                 deviceIndex++;
              } else {
                goto maxDevices;
              }
              ds.wireReset();
            }
maxDevices:  // return number of devices found as additional value

            ds.wireReset();
            ds.wireWriteByte(0xCC);  // address all temperature sensors
            ds.wireWriteByte(0x44);  // start conversion
            wireConversionTime = 750 + currentTime; // set a time value 750ms from now when the conversions will be complete
            
          } else {
            goto illegal;
          }
          break;

        case 'u': case 'U':   // Set pin to Grove Ultrasonic Ranging device
          if (70 <= pinNumber)
            goto illegal;
          pinMode(pinNumber, INPUT);
          pinModes[pinNumber] = 'u';
          _sprintf(outputBuffer, "s %u u", pinNumber);
          printFrame((const char *)outputBuffer);
          break;

        default:
illegal:
          printFrame("illegal mode");
          break;       
      }
      return;
    }
  }

// call function to actually set a pins operating mode, input, output etc. based on letter i, o, a  etc.
  if (('g' == commandBuffer[0]) || ('G' == commandBuffer[0]) ) {
    if (('0' <= commandBuffer[index]) && ('9' >= commandBuffer[index]) ) {
      pinNumber = decToInt(&index);
      if (pinNumber > pinLimit) {
          printFrame("illegal pin");
        return;
      }

    // call function to actually get a pin to a value here and print it to the console
      switch (pinModes[pinNumber]) {
        case 'i':
          if (HIGH == digitalRead(pinNumber)) {
            pinValue = 1;
          }
          else {
            pinValue = 0;  
          }
          _sprintf(outputBuffer, gUU, pinNumber, pinValue);          
         printFrame((const char *)outputBuffer);
          break;

        case 'a':
// Analog pins are in a bank starting at pin 70
          pinValue = analogRead(pinNumber - 70);
          _sprintf(outputBuffer, gUU, pinNumber, pinValue);
          printFrame((const char *)outputBuffer);
          break;

        case 'c':
// Analog pins are in a bank starting at pin 70
          pinNumber -= 70;
          sensorMax = 0;
          duration = 500;
          while(duration--)
          {
            pinValue = analogRead(pinNumber);
            if (pinValue > sensorMax) 
            {
              sensorMax = pinValue;
            }
          }
          _sprintf(outputBuffer, gUU, pinNumber + 70, sensorMax);
          printFrame((const char *)outputBuffer);
          break;
  
        case 'n':
          if (2 == pinNumber) {
            cli();      //Disable interrupts
            intCount = pin2;
            pinTime  = pin2Time;
            sei();
            _sprintf(outputBuffer, guUUUU, pinNumber, intCount, pinTime, pin2Last, pin2LastTime);            
            pin2Last = intCount ;
            pin2LastTime = pinTime;
            printFrame((const char *)outputBuffer);            
          }
          else {
            if (3 == pinNumber) {
              cli();      //Disable interrupts
              intCount = pin3;
              pinTime  = pin3Time;
              sei();
              _sprintf(outputBuffer, guUUUU, pinNumber, intCount, pinTime, pin3Last, pin3LastTime);  
              pin3Last = intCount;
              pin3LastTime = pinTime;
              printFrame((const char *)outputBuffer);   
            }
            else {
              printFrame("illegal pin");
            }
          }
          break;

        case 'w':
// 1-Wire temperature conversion will not respond until a conversion has been completed through setup or a previous request
// I2C pins are on analog pins and are at pin 74 and 75 (pins 4 and 5 on Uno)
          if (wireConversionTime < currentTime) {

            for (displayIndex = 0; displayIndex < deviceIndex; displayIndex++)
            {
              if (ds.crc8(addr[displayIndex], 7) != addr[displayIndex][7]) {
                goto wireError;
              }


// the first ROM byte indicates which chip
              switch (addr[displayIndex][0]) {
                case 0x10:  // old DS1820
                  type_s = 1;
                  break;
                case 0x28:  // DS18B20
                  type_s = 0;
                  break;
                case 0x22:  // DS1822
                  type_s = 0;
                  break;
                default:    // Device is not a DS18x20 family device
                  goto wireError;
              }
            
              present = ds.wireReset();
              ds.wireSelect(addr[displayIndex]);
              ds.wireWriteByte(0xBE);

              for ( i = 0; i < 9; i++) {           // we need 9 bytes
                data[i] = 0;
                data[i] = ds.wireReadByte();
              }

              if (ds.crc8(data, 8) != data[8]) {
                goto wireError;
              }
            
// Convert the data to actual temperature
// because the result is a 16 bit signed integer, it should
// be stored to an "int16_t" type, which is always 16 b
// even when compiled on a 32 bit processor.
              int16_t raw;
              raw = (data[1] << 8) | data[0];
              if (type_s)
              {
                raw = raw << 3;          // 9 bit resolution default
                if (data[7] == 0x10) {   // "count remain" gives full 12 bit resolution
                  raw = (raw & 0xFFF0) + 12 - data[6];
                  }
              } else {
                byte cfg = (data[4] & 0x60);  // at lower res, the low bits are undefined, so let's zero them
                if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
                else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
                  else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
//// default is 12 bit resolution, 750 ms conversion time
              }
              
              float celsius;
              celsius = ((float)raw) / 16.0;    
displayTemp:        
              _sprintf(outputBuffer, "g %u %u %f6.2", pinNumber, displayIndex, celsius);
              goto printIt;
wireError:
              celsius = -99.0; // error value is -99
              goto displayTemp;
printIt:
              printFrame((const char *)outputBuffer);
            }
            
// Initiate another request for the sensors to refresh their readings            
            ds.wireReset();
            ds.wireWriteByte(0xCC);  // address all temperature sensors
            ds.wireWriteByte(0x44);  // start conversion
            wireConversionTime = 750 + currentTime; // set a time value 750ms from now when the conversions will be complete
          }
          break;

        case 'u':
// Perform Ultra Sonic Ranger function
          _sprintf(outputBuffer, "g %u %dl", pinNumber, MeasureInCentimeters(pinNumber));
          printFrame((const char *)outputBuffer);
          break;

        default:
          printFrame("illegal mode");
          return;
      }
    }
    else {
      if ('v' == commandBuffer[index]) {
        printFrame(FIRMWARE_VERSION);
      }
      else {
        if ('t' == commandBuffer[index]) {
          cli();
          unsigned long ct = currentTime;
          sei();
          _sprintf(outputBuffer, "g t %ul", ct);  
          printFrame((const char *)outputBuffer);   
        } else {
          printFrame("illegal get command");
        }
      }
    }
  }

  // 'reset' the firmware, pinModes and other variables, and hardware to reset state
  if (MatchCommand((char *)commandBuffer, "reset") )
  {
    resetEverything();
    Serial.println("{reset}");
    return;
  }
  
  // get device registration 'info' if it has been set
  if (MatchCommand((char *)commandBuffer, "info") )
  {
    outputBuffer[0] = 'i'; outputBuffer[1] = 'n';  outputBuffer[2] = 'f';  outputBuffer[3] = 'o';  outputBuffer[4] = ' ';
    for (index = 0; index < 36; index++)
    {
      outputBuffer[index + 5] = EEPROM.read(index + EEPROM_UUID_ADDRESS);
    }
    outputBuffer[index + 5] = 0;  // null terminate
    EEPROM.get(EEPROM_UUID_ADDRESS + 36, infoCRC);
    if (infoCRC != crc16((unsigned char *)&(outputBuffer[5]), 36))
    {
      printFrame("info undefined");
      return;
    }
    printFrame((const char *)outputBuffer);
    return;
  }
  
// 'reg'(ister) device info if requested within registration time after startup (normally 10 minutes)
  if ((40 == commandLength) && ('r' == commandBuffer[0]) && ('e' == commandBuffer[1]) && ('g' == commandBuffer[2]) && (' ' == commandBuffer[3]) ) 
  {
    infoCRC = crc16(&(commandBuffer[4]), 36);
    for (index = 0; index < 36; index++)
    {
      EEPROM.write(index + EEPROM_UUID_ADDRESS, commandBuffer[index + 4]);
    }
    EEPROM.put(EEPROM_UUID_ADDRESS + 36, infoCRC);
    printFrame("reg set");
    return;
  }

}

//---------------------------------------------------------------------
// For Grove Ultrasonic Distance measurement device by Seeed Studio
long MeasureInCentimeters(unsigned char pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin,LOW);
  pinMode(pin,INPUT);
  long duration;
  duration = pulseIn(pin,HIGH);
  long RangeInCentimeters;
  RangeInCentimeters = duration / 58ul;
  return RangeInCentimeters;
}

//---------------------------------------------------------------------
// returns true if a correct 4 digit hex value is provided, false otherwise
bool hexToInt(unsigned int * outputValue, unsigned char * hexString, unsigned char stringLength) {
  unsigned char index, letter;
  unsigned int value;
  unsigned int multipliers[] = {4096, 256, 16, 1, 0};
  
  *outputValue = 0;
  
  if (4 != stringLength) {
    return (false);
  }
// convert four ASCII hex digits to an unsigned 16 bit int, or return false
  value = 0;
  for (index = 0; index < stringLength; index++) {
    letter = hexString[index];
    if (('0' <= letter) && ('9' >= letter) ) {
      value += multipliers[index] * (letter - '0');
    } else {
      if (('a' <= letter) && ('f' >= letter) ) {
        value += multipliers[index] * (10 + (letter - 'a'));
      } else {
        if (('A' <= letter) && ('F' >= letter) ) {
          value += multipliers[index] * (10 + (letter - 'A'));
        } else {
          return (false);
        }
      }
    }
  }
  *outputValue = value;
  return (true);
}

//---------------------------------------------------------------------
// Convert ASCII "0" to "9999" into an integer, -1 is value if no valid number is found, advances **pText
int decToInt(unsigned char * index) {
  char letter;
  int returnValue;
  int count;

  returnValue = 0;
  letter = commandBuffer[*index];

  for (count = 0; count < 4; count++)
  {
    if (('0' <= letter) && ('9' >= letter) )
    {
      returnValue = returnValue * 10;
      returnValue += letter - '0';
      (*index)++;
      letter = commandBuffer[*index];
    }
    else {
      if ((' ' == letter) || (':' == letter) ) {  // if the scanner hits a space, colon advance index and exit successfully
        (*index)++;
        goto exit;
      } else {
        if (0 == letter) {    // if the scanner hits a null just exit successfully
          goto exit;
        } else {
          returnValue = -1;
          goto exit;
        }
      }
    }
  }
exit:
  return (returnValue);
}

//---------------------------------------------------------------------
// Binary to decimal numeric format routines for 16 and 32 bit signed and unsigned values
uint8_t * format_s16(int16_t value, uint8_t * buffer)
{
  if (value < 0) {
    *buffer++ = '-';
    value = -value;
  }
  return format_u16((uint16_t)value, buffer);
}

uint8_t * format_s32(int32_t value, uint8_t * buffer)
{
  if (value < 0) {
    *buffer++ = '-';
    value = -value;
  }
  return format_u32((uint32_t)value, buffer);
}

uint8_t * format_u16(uint16_t value, uint8_t * buffer)
{
  uint8_t digit;

  if (value >= 10000u) goto ten_k;
  if (value >= 1000u) goto one_k;
  if (value >= 100u) goto hundred;
  if (((uint8_t)value) >= 10u) goto ten;
  goto one;

ten_k:
  digit  = value / 10000u;
  value -= digit * 10000u;
  *buffer++ = digit + '0';
one_k:
  digit  = value / 1000u;
  value -= digit * 1000u;
  *buffer++ = digit + '0';
hundred:
  digit  = value / 100u;
  value -= digit * 100u;
  *buffer++ = digit + '0';
ten:
  digit  = ((uint8_t)value) / 10u;
  value -= digit * 10u;
  *buffer++ = digit + '0';
one:
  digit = value;
  *buffer++ = digit + '0';
  *buffer = 0;  // Null terminate

  return buffer;
}

uint8_t * format_u32(uint32_t value, uint8_t * buffer)
{
  uint8_t digit;

  if (value >= 1000000000ul) goto one_b;
  if (value >= 100000000ul) goto hundred_m;
  if (value >= 10000000ul) goto ten_m;
  if (value >= 1000000ul) goto one_m;
  if (value >= 100000ul) goto hundred_k;
  if (value >= 10000ul) goto ten_k;
  if (((uint16_t)value) >= 1000u) goto one_k;
  if (((uint16_t)value) >= 100u) goto hundred;
  if (((uint8_t)value) >= 10u) goto ten;
  goto one;

one_b:
  digit  = value / 1000000000ul;
  value -= digit * 1000000000ul;
  *buffer++ = digit + '0';
hundred_m:
  digit  = value / 100000000ul;
  value -= digit * 100000000ul;
  *buffer++ = digit + '0';
ten_m:
  digit  = value / 10000000ul;
  value -= digit * 10000000ul;
  *buffer++ = digit + '0';
one_m:
  digit  = value / 1000000ul;
  value -= digit * 1000000ul;
  *buffer++ = digit + '0';
hundred_k:
  digit  = value / 100000ul;
  value -= digit * 100000ul;
  *buffer++ = digit + '0';
ten_k:
  digit  = value / 10000ul;
  value -= digit * 10000ul;
  *buffer++ = digit + '0';
one_k:
  digit  = ((uint16_t)value) / 1000u;
  value -= digit * 1000u;
  *buffer++ = digit + '0';
hundred:
  digit  = ((uint16_t)value) / 100u;
  value -= digit * 100u;
  *buffer++ = digit + '0';
ten:
  digit  = ((uint8_t)value) / 10u;
  value -= digit * 10u;
  *buffer++ = digit + '0';
one:
  digit = value;
  *buffer++ = digit + '0';
  *buffer = 0;  // Null terminate

  return buffer;
}

uint8_t * formatHexValue(uint32_t value, uint8_t * buffer, uint8_t digits) {
  uint8_t digit;
  int8_t x;
  for (x = digits - 1; x >= 0; x -= 2 )
  {
    digit = value & 255;
    buffer[x] = hexDigitLookup[digit & 0x0f];
    digit = digit >> 4;
    buffer[x - 1] = hexDigitLookup[digit & 0x0f];
    value = value >> 8;
  }
  return (buffer + digits);
}
  
int _sprintf(uint8_t * output, const char * format, ...)
{
  int i;
  int count = 0;
  uint8_t * pOutput = output;
  uint8_t * pString;
  uint8_t character;
  int length;

  for (i = 0; format[i] != '\0'; i++)
    if (format[i] == '%')
      count++;

  va_list argv;
  va_start(argv, format);   // set the start to [format} the argument immediately preceding the variable arguments

  for(i=0; format[i] != '\0'; i++) {
    if(format[i] == '%') {
      switch(format[++i])
      {
        case 'u':
          if (format[i+1] == 'l') {
            i++;
            pOutput = format_u32(va_arg(argv, uint32_t), pOutput);    
          } else {
            pOutput = format_u16(va_arg(argv, uint16_t), pOutput);
          }
          break;
        case 'd': case 'i':
          if (format[i+1] == 'l') {
            i++;              
            pOutput = format_s32(va_arg(argv, int32_t), pOutput);    
          } else {
            pOutput = format_s16(va_arg(argv, uint16_t), pOutput);
          }
          break;
        case 'x':
          if (format[i+1] == 'h') {
            i++;
            pOutput = formatHexValue(va_arg(argv, uint16_t), pOutput, 2);
          } else {
            if (format[i+1] == 'l') {
              i++;
              pOutput = formatHexValue(va_arg(argv, uint32_t), pOutput, 8);
            } else {
            pOutput = formatHexValue(va_arg(argv, uint16_t), pOutput, 4);
            }
          }
          break;
        case 'f':
          {
            char intDigits = 9;
            char fracDigits = 0;
            i++;
            char formatChar = format[i];
            if ((formatChar >= '0') && (formatChar <= '9') )
            {
              i++;
              intDigits = formatChar - '0';
              if (format[i] == '.')
              {
                i++;
                formatChar = format[i];
                if ((formatChar >= '0') && (formatChar <= '9') )
                {
                  i++;
                  fracDigits = formatChar - '0';
                }
              }
            }
            dtostrf(va_arg(argv, double), intDigits, fracDigits, (char *)pOutput);
            pOutput += intDigits;
            i--;
          }
          break;
        case 'c': *pOutput++ = va_arg(argv, uint8_t);
          break;
        case 's': pString = va_arg(argv, uint8_t *);
          length = strlen((char *)pString);
          while (length--) *pOutput++ = *pString++;
          break;
        case '%': *pOutput++ = '%';
          break;
        default:  ;
          break;
      }
    } else {
      character = format[i];
      *pOutput++ = character;
    }
  }
  *pOutput = '\0';
  va_end(argv);
  return pOutput - output;
}

/**************************************************************************
//
// crc16.c - generate a CCITT 16 bit cyclic redundancy check (crc)
//
//      The code in this module generates the crc for a block of data.
//
**************************************************************************/
/*
//                                16  12  5
// The CCITT CRC 16 polynomial is X + X + X + 1.
// In binary, this is the bit pattern 1 0001 0000 0010 0001, and in hex it
//  is 0x11021.
// A 17 bit register is simulated by testing the MSB before shifting
//  the data, which affords us the luxury of specifiy the polynomial as a
//  16 bit value, 0x1021.
// Due to the way in which we process the CRC, the bits of the polynomial
//  are stored in reverse order. This makes the polynomial 0x7408.
*/
 
/*
// note: when the crc is included in the message, the valid crc is:
//      0xF0B8, before the compliment and byte swap,
//      0x0F47, after compliment, before the byte swap,
//      0x470F, after the compliment and the byte swap.
*/
 
//extern  crc_ok;
//int     crc_ok = 0x470F;
 
/**************************************************************************
//
// crc16() - generate a 16 bit crc
//
//
// PURPOSE
//      This routine generates the 16 bit remainder of a block of
//      data using the ccitt polynomial generator.
//
// CALLING SEQUENCE
//      crc = crc16(data, len);
//
// PARAMETERS
//      data    <-- address of start of data block
//      len     <-- length of data block
//
// RETURNED VALUE
//      crc16 value. data is calcuated using the 16 bit ccitt polynomial.
//
// NOTES
//      The CRC is preset to all 1's to detect errors involving a loss
//        of leading zero's.
//      The CRC (a 16 bit value) is generated in LSB MSB order.
//      Two ways to verify the integrity of a received message
//        or block of data:
//        1) Calculate the crc on the data, and compare it to the crc
//           calculated previously. The location of the saved crc must be
//           known.
/         2) Append the calculated crc to the end of the data. Now calculate
//           the crc of the data and its crc. If the new crc equals the
//           value in "crc_ok", the data is valid.
//
// PSEUDO CODE:
//      initialize crc (-1)
//      DO WHILE count NE zero
//        DO FOR each bit in the data byte, from LSB to MSB
//          IF (LSB of crc) EOR (LSB of data)
//            crc := (crc / 2) EOR polynomial
//          ELSE
//            crc := (crc / 2)
//          FI
//        OD
//      OD
//      1's compliment and swap bytes in crc
//      RETURN crc
//
*/

#define POLY 0x8408

unsigned int crc16(unsigned char *data_p, unsigned char length)
{
  unsigned char i;
  unsigned int data;
  unsigned int crc = 0xffff;

  if (length == 0)
    return (~crc);

  do
  {
    for (i = 0, data = (unsigned int)0xff & *data_p++; i < 8; i++, data >>= 1)
    {
      if ((crc & 0x0001) ^ (data & 0x0001))
        crc = (crc >> 1) ^ POLY;
      else
        crc >>= 1;
    }
  }
  while (--length);

  crc = ~crc;

  data = crc;
  crc = (crc << 8) | (data >> 8 & 0xff);
  
  return (crc);
}

/**************************************************************************/

