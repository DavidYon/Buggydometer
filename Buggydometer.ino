#include <SoftwareSerial.h>
#include <serLCD.h>

#include <TimerOne.h>

#include <EEPROM.h>
#include <stdarg.h>
#include <pgmspace.h>

//
// Absolute min and max eeprom addresses.
// Actual values are hardware-dependent.
//
// These values can be changed e.g. to protect
// eeprom cells outside this range.
//
const int EEPROM_MIN_ADDR = 0;
const int EEPROM_MAX_ADDR = 1023;
const int EEPROM_ODOMETER_START = 512;

// EEPROM utility functions
boolean eeprom_is_addr_ok(int addr);
boolean eeprom_write_int(int addr, int value);
boolean eeprom_read_int(int addr, int* value);
boolean eeprom_write_bytes(int startAddr, const byte* array, int numBytes);
boolean eeprom_read_bytes(int startAddr, byte array[], int numBytes);
void eeprom_serial_dump_table(int bytesPerRow = 16);

// Format of Odometer readings stored in EEPROM
typedef struct {
  uint8_t length;             // Length of structure, which is how we detect that we changed
                              // (usually expanded) the structure
                              
  uint8_t sentinel;           // Lower 8 bits of the offset of the structure in the EEPROM, 
                              // used to sanity-check that this is actually a valid structure
                              // and not some random data in the EEPROM that we are looking at
                              
  unsigned long pulseCount;   // Last pulse count reading from the odometer
  
  unsigned long tripAdelta;   // Last trip odometer setting (expressed as a difference betweeen
                              // the total pulse counts and the start of the trip)
} OdometerData;

// Odometer/EEPROM functions
int findOdometerReadingOffset(OdometerData &data);
int WriteOdometerDataAt(OdometerData &data, int offset);
int PrintOdometerInfo(int offset, OdometerData &currentData);

// Saved odometer data
OdometerData odometer;
int odometerEEPROMOffset = 0;
 
// Tracks pulse edges in ISR, with latestMillis being read by loop()
unsigned long leadingEdgeMillis = 0;
volatile unsigned long latestMillis = 0;
volatile int latestMillisWidth = 0;
volatile int bounceLength = 150;
volatile uint8_t waitingForCycle = 1;

// Units based on pulse count
volatile unsigned long pulseCount = 0;

// Input pins
int IRpin = 8;
int ForwardPin = 12;
int MiddlePin = 11;
int BackwardPin = 10;
bool readButtonPin(byte &pin, bool &pressed);

// Strings
const prog_char ResetTrip[] PROGMEM       = "Reset Trip?     ";
const prog_char OKSelected[] PROGMEM      = "*OK* cancel     ";
const prog_char CancelSelected[] PROGMEM  = "ok *CANCEL*     ";

// Display Modes
enum _displayMode
{
  dispNormal,
  dispTrip,
  dispResetTripOK,
  dispResetTripCancel
} DisplayMode = dispNormal;

// Parallel display
serLCD lcd(2);

// 3 rotations of pulse times for speed averaging
unsigned long pulseTimes[3];
volatile unsigned long avgPulseWidth = 0;

volatile int oldPinValue = LOW;
volatile uint8_t loopStart = 0;
volatile uint8_t writeToEEPROM = 0;

//#define DEBUG
#ifdef DEBUG
// debugging
uint8_t debugOutputted = 0;
volatile int arrayIdx = 0;
unsigned long lastDebugMillis = 0;
uint8_t pinValues[256];
uint16_t deltas[256];
#endif

void setup() 
{
  Serial.begin(9600);

  // Initialize the I/O pins and the LCD
//  lcd.setType(3);

  lcd.clear();
  pinMode(ForwardPin,INPUT);
  pinMode(BackwardPin,INPUT);
  pinMode(MiddlePin,INPUT);
  pinMode(IRpin,INPUT);
    
  // Write the welcome message
  lcd.selectLine(1); 
  lcd.print(" Buggydometer! ");
  lcd.selectLine(2); 
  lcd.print("  Version 2.1  ");
 
#if false 
  // Reset the odometer readings
  for (int addr = EEPROM_ODOMETER_START; addr <= EEPROM_MAX_ADDR; addr++)
  {
    EEPROM.write(addr,0);
  }
#endif

  // Find any saved odometer readings
  odometerEEPROMOffset = findOdometerReadingOffset(odometer);

  // Set the initial pulse counts to those values
  pulseCount = odometer.pulseCount;
  pulseTimes[0] = pulseCount;
  pulseTimes[1] = pulseCount;
  pulseTimes[2] = pulseCount;

#if false 
  Serial.print("Offset "); Serial.println(odometerEEPROMOffset);
  Serial.print("Odometer = "); Serial.println(odometer.pulseCount);
  Serial.print("Trip = "); Serial.println(odometer.tripAdelta);
  eeprom_serial_dump_table();
#endif

  // Start the timer interrupt
  Timer1.initialize(1000); // set a timer of length 1000 microseconds (or 0.001 sec - or 1000Hz)
  Timer1.attachInterrupt( timerIsr ); // attach the service routine here

  // Keep the message on the screen for 3 sec
  delay(3000);
}
 

void loop()
{
  char printfBuf[32];

  if (!loopStart)
  {
    delay(750);
    loopStart = 1;
  }
  
  // Write to EEPROM if requested
  if (writeToEEPROM != 0)
  {
    writeToEEPROM = 0;
    odometer.pulseCount = pulseCount;  
    WriteOdometerDataAt(odometer,odometerEEPROMOffset);
  }
 
  // Grab the pulse stats
  unsigned long lPulseCount = pulseCount;
  unsigned long lAvgPulseWidth = avgPulseWidth;
  unsigned long lLatestMillisWidth = latestMillisWidth;  
  
  // Read the button press
  byte buttonPress = 0;
  bool state = false;
  if (readButtonPin(buttonPress,state))
  {
    if ((buttonPress == MiddlePin) && !state)
    {
      if (DisplayMode == dispResetTripOK) 
      {
        odometer.pulseCount = pulseCount;
        odometer.tripAdelta = odometer.pulseCount;
        DisplayMode = dispTrip;
        WriteOdometerDataAt(odometer,odometerEEPROMOffset);
      }
      else
      {
        DisplayMode = ((DisplayMode == dispNormal) || (DisplayMode == dispTrip)) ? dispResetTripCancel : dispNormal;
      }
    }
    else if (!state)
    {
      if ((buttonPress == BackwardPin) && (DisplayMode == dispResetTripCancel))
      {
        DisplayMode = dispResetTripOK;
      }
      else if ((buttonPress == ForwardPin) && (DisplayMode == dispResetTripOK))
      {
        DisplayMode = dispResetTripCancel;
      }
      else if ((buttonPress == BackwardPin) && (DisplayMode == dispTrip))
      {
        DisplayMode = dispNormal;
      }
      else if ((buttonPress == ForwardPin) && (DisplayMode == dispNormal))
      {
        DisplayMode = dispTrip;
      }
    }
  }


  // If displaying Trip, adjust the pulse count
  if (DisplayMode == dispTrip)
  {
    lPulseCount -= odometer.tripAdelta;
  }
  
  // Convert to miles
  double miles = lPulseCount * 0.000390419947;
  
  // Convert to mph
  double mph = (lAvgPulseWidth > 0) ? (1405.512 / lAvgPulseWidth) : 0;
    
  // move cursor to beginning of first line
  if ((DisplayMode == dispNormal) || (DisplayMode == dispTrip))
  {
    if ((millis() % 750) == 0)
    {
      lcd.selectLine(1);
  
      // Format and print the lines
      dtostrf(mph,6,2,printfBuf);
      lcd.print(printfBuf);
      lcd.print(" mph      ");
      lcd.selectLine(2);
      dtostrf(miles,6,2,printfBuf);
      lcd.print(printfBuf);  
      if (DisplayMode == dispNormal)
      {
        lcd.print(" miles    ");
      }
      else
      {
        lcd.print(" miles (T)");
      }
    }
  }
  else
  {
    if ((millis() % 100) == 0)
    {  
      lcd.selectLine(1);
      strcpy_P(printfBuf,ResetTrip);
      lcd.print(printfBuf);
      lcd.selectLine(2);
      strcpy_P(printfBuf,(DisplayMode == dispResetTripCancel) ? CancelSelected : OKSelected);
      lcd.print(printfBuf);
    }
  }

#ifdef DEBUG  
  if ((arrayIdx == 256) && (debugOutputted == 0))
  {
    debugOutputted = 1;
    for (int i = 0; i < 256; i++)
    {
      snprintf(printfBuf,32,"pin=%d delta=%d",(int)pinValues[i],(int)deltas[i]);
      Serial.println(printfBuf);
    }
  }
#endif
  
}

int pulsed = 0;
unsigned long lastHigh = 0;

/// --------------------------
/// Custom ISR Timer Routine
/// --------------------------
void timerIsr()
{    
    // Read the photo-interruptor
    int newPinValue = digitalRead(IRpin);

    // Let pin settle until main loop starts
    if (!loopStart)
    {
      oldPinValue = newPinValue;
      return;
    }

#ifdef DEBUG
    // build up a debug array
    if (arrayIdx < 256)
    {
      unsigned long debugNow = millis();
      if (arrayIdx == 0) lastDebugMillis = debugNow;
      if ((arrayIdx == 0) || (newPinValue != pinValues[arrayIdx-1]))
      {
        pinValues[arrayIdx] = newPinValue;
        deltas[arrayIdx] = debugNow - lastDebugMillis;
        lastDebugMillis = debugNow;
        arrayIdx++;
      }
    }
#endif

    // We track the first time we've seen a low-to-high transition,
    // the last time a low-to-high occurred in the bounce period,
    // then the first time we've seen a high-to-low transition during the cycle time
    static unsigned long firstHigh = 0;
    static unsigned long lastHigh = 0;
    static unsigned long firstLow = 0;
   
    // Read the current number of milliseconds since power-up
    unsigned long now = millis();
    
    // Only proceed if the pin changed value 
    if (oldPinValue != newPinValue) 
    {
      oldPinValue = newPinValue;
      
      // LOW -> HIGH
      if (newPinValue == HIGH)
      {
        if (waitingForCycle == 1)
        {
          // If this is not the first time around, we need to be waiting 
          // for the trailing edge bounce period to expire
          if ((firstLow == 0) || ((now - firstLow) > bounceLength))
          {
            // If this is not the first time around, and we are past 
            // the trailing-edge bounce, mark a cycle
            if (firstLow != 0)
            {
              // Mark the latest pulse width
              latestMillisWidth = now - firstHigh;
              
              // Compute the average
              avgPulseWidth = (now - pulseTimes[0]) / 3;
    
              // Update the pulse counts for averaging
              pulseTimes[0] = pulseTimes[1];
              pulseTimes[1] = pulseTimes[2];
              pulseTimes[2] = now;
              
              // Increment the pulse count
              pulseCount++;
            }
             
            // Mark this as our first high trigger
            firstHigh = now;
            
            // Premptively mark the next high trigger, with a minimum distance
            // of 1ms, which handles the case where no bounce occurs
            lastHigh = now+1; 

            // Do the same for the first low trigger
            firstLow = lastHigh + 1;

            // We are not waiting for the leading edge of the cycle anymore
            waitingForCycle = 0;
          }          
        }
        
        // If we aren't waiting for the leading edge anymore, then
        // a LOW->HIGH means a bounce, so mark where that occured
        else
        {
          lastHigh = now+1; // minimum 1ms, see above
          firstLow = lastHigh + 1; // keep the trailing edge ahead of the leading edge
        }        
      }
      
      // HIGH -> LOW
      else
      {
        
        // If we have seen the leading edge, look for the trailing edge
        if (waitingForCycle == 0)
        {
          // Mark the transition time, can't hurt
          firstLow = now;
          
          // If the ratio of the cycle time to bounce time is over (ratio of 20:1),
          // then we've seen the true trailing edge occur
          if (((firstLow - firstHigh) > 200) || ((firstLow - firstHigh) / (lastHigh - firstHigh)) > 20)
          {
            // Calculate the bounce window
            bounceLength = (now - firstHigh) / 20;
            if (bounceLength < 7) 
              bounceLength = 7;
            else if (bounceLength > 150)
              bounceLength = 150;
                        
            // Mark that we are waiting for the cycle again
            waitingForCycle = 1;
          }
        }        
      }
      
    }   
              
    // Zero out mph on slow or stopped
    if ((avgPulseWidth > 0) && ((now - firstHigh) > 2200))
    {
      pulseTimes[0] = 0;
      pulseTimes[1] = 0;
      pulseTimes[2] = 0;
      avgPulseWidth = 0;

      // If pulseCount has changed since last save, save it again to EEPROM.
      if (pulseCount != odometer.pulseCount)  
      {
        writeToEEPROM = 1;
      }     
    }
}

#if false
int PrintOdometerInfo(int offset, OdometerData &currentData)
{
      Serial.print("Offset: "); Serial.print(offset);
      Serial.print(" Length: "); Serial.print(currentData.length);
      Serial.print(" Sentinel: "); Serial.print(currentData.sentinel);
      Serial.print(" Pulse Count: "); Serial.print(currentData.pulseCount);
      Serial.print(" Trip: "); Serial.println(currentData.tripAdelta);
      Serial.println("");
}
#endif

//
// Finds the offset of the highest odometer reading 
// in the EEPROM.  If cannot be found, the readings
// are reset and the first available location returned.
// If found, the last readings are returned and the 
// *next* location to write new readings is returned.
//
int findOdometerReadingOffset(OdometerData &data)
{
  int offset = EEPROM_ODOMETER_START;
  int lastValidOffset = -1;
  unsigned long maxPulses = 0;
  OdometerData currentData;
    
  // Go until end of EEPROM or invalid structure
  while ((offset < EEPROM_MAX_ADDR) && (offset != -1))
  {
    if (eeprom_read_bytes(offset, (byte*)&currentData, sizeof(currentData)))
    {
      if ((currentData.length == sizeof(data)) && (currentData.sentinel == ((offset + 1) & 0xFF)))
      {
        offset += currentData.length;
        if (currentData.pulseCount >= maxPulses)
        {
          lastValidOffset = offset;
          maxPulses = currentData.pulseCount;
          
          // Copy this to the caller's copy
          data.length = currentData.length;
          data.sentinel = currentData.sentinel;
          data.pulseCount = currentData.pulseCount;
          data.tripAdelta = currentData.tripAdelta;
        }
        else
        {
          offset = -1;
        }
      }
      
      // not valid header
      else
      {
        offset = -1;
      }
   }
  }
  
  // If we had a valid record, return with that
  if (lastValidOffset != -1)
  {
    // Wrap around to first offset if necessary
    if ((offset + sizeof(currentData)) >= EEPROM_MAX_ADDR)
    {
      offset = EEPROM_ODOMETER_START;
    }
    
    // If not, next offset is last valid + size of structure
    else
    {
      offset = lastValidOffset + currentData.length;
    }    
  }
  
  // Otherwise reset to zero and return first offset
  else
  {
    offset = EEPROM_ODOMETER_START;
    data.length = sizeof(data);
    data.sentinel = (offset + 1) & 0xFF;
    data.pulseCount = 0;
    data.tripAdelta = 0; 
  }
  
  return offset;  
}


//
// Writes the odometer structure at the specified offset.  If failed, returns
// the same offset.  If succeeded, returns the offset for the next write.
//
int WriteOdometerDataAt(OdometerData &data, int offset)
{
   // Set the header 
   data.length = sizeof(data);
   data.sentinel = (offset + 1) & 0xFF;
   
   // Write the data
   offset = eeprom_write_bytes(offset, (const byte*)&data, sizeof(data)) ? offset : offset + data.length;
   return (offset > EEPROM_MAX_ADDR) ? EEPROM_ODOMETER_START : offset;
}

//
// Returns true if the address is between the
// minimum and maximum allowed values,
// false otherwise.
//
// This function is used by the other, higher-level functions
// to prevent bugs and runtime errors due to invalid addresses.
//
boolean eeprom_is_addr_ok(int addr) 
{
  return ((addr >= EEPROM_MIN_ADDR) && (addr <= EEPROM_MAX_ADDR));
}


//
// Writes a sequence of bytes to eeprom starting at the specified address.
// Returns true if the whole array is successfully written.
// Returns false if the start or end addresses aren't between
// the minimum and maximum allowed values.
// When returning false, nothing gets written to eeprom.
//
boolean eeprom_write_bytes(int startAddr, const byte* array, int numBytes)
{
  // counter
  int i;

  // both first byte and last byte addresses must fall within
  // the allowed range  
  if (!eeprom_is_addr_ok(startAddr) || !eeprom_is_addr_ok(startAddr + numBytes)) {
    return false;
  }

  for (i = 0; i < numBytes; i++) {
    EEPROM.write(startAddr + i, array[i]);
  }

  return true;
}


//
// Reads the specified number of bytes from the specified address into the provided buffer.
// Returns true if all the bytes are successfully read.
// Returns false if the star or end addresses aren't between
// the minimum and maximum allowed values.
// When returning false, the provided array is untouched.
//
// Note: the caller must ensure that array[] has enough space
// to store at most numBytes bytes.
//
boolean eeprom_read_bytes(int startAddr, byte array[], int numBytes) 
{
  int i;

  // both first byte and last byte addresses must fall within
  // the allowed range  
  if (!eeprom_is_addr_ok(startAddr) || !eeprom_is_addr_ok(startAddr + numBytes)) {
    return false;
  }

  for (i = 0; i < numBytes; i++) {
    array[i] = EEPROM.read(startAddr + i);
  }

  return true;
}


//
// Dump eeprom memory contents over serial port in tabular form.
// Each printed row shows the value of bytesPerRow bytes
// (by default 16).
//
void eeprom_serial_dump_table(int bytesPerRow) {
  // address counter
  int i;

  // row bytes counter
  int j;

  // byte read from eeprom
  byte b;

  // temporary buffer for sprintf
  char buf[10];


  // initialize row counter
  j = 0;

  // go from first to last eeprom address
  for (i = EEPROM_ODOMETER_START; i <= EEPROM_MAX_ADDR; i++) {

    // if this is the first byte of the row,
    // start row by printing the byte address
    if (j == 0) {
      sprintf(buf, "%03X: ", i);
      Serial.print(buf);
    }

    // read current byte from eeprom
    b = EEPROM.read(i);

    // write byte in hex form
    sprintf(buf, "%02X ", b);

    // increment row counter
    j++;

    // if this is the last byte of the row,
    // reset row counter and use println()
    // to start a new line
    if (j == bytesPerRow) {
      j = 0;
      Serial.println(buf);
    }
    // else just print the hex value with print()
    else {
      Serial.print(buf);
    }
  }
}

bool readButtonPin(byte &pin, bool &pressed)
{
  static byte lastPin = 0;
  static byte debouncingPin = 0;
  static int debounceLastMillis = 0;
  static int debounceTime = 50;
  byte currentPin = 0;
  
  if (digitalRead(ForwardPin) == LOW)
  {
    currentPin = ForwardPin;
  }
  else if (digitalRead(BackwardPin) == LOW)
  {
    currentPin = BackwardPin;
  }
  else if (digitalRead(MiddlePin) == LOW)
  {
    currentPin = MiddlePin;
  }
  
  // We have no button presses yet
  if (debouncingPin == 0)
  {
    // If we detect a new pin or a pin change state, start the debounce phase
    if (currentPin != lastPin)
    {
      if (currentPin != 0)
      {
        debouncingPin = currentPin;
        debounceLastMillis = millis();
      }
      else if (lastPin != 0) 
      {
        debouncingPin = lastPin;
        debounceLastMillis = millis();
      }
    }
  }
  else
  {
     // Wait for debounce period to elapse before checking
     if ((millis() - debounceLastMillis) > debounceTime)
     {
        // If a debounced press occurs, return true and report the pin
        if (currentPin == debouncingPin)
        {
          debouncingPin = 0;
          lastPin = currentPin;
          pin = lastPin;
          pressed = true;
          return true;
        }
        
        // If a debounced unpress occurs, return true and report the pin
        else if (lastPin == debouncingPin)
        {
          debouncingPin = 0;
          pin = lastPin;
          pressed = false;
          lastPin = 0;
          return true;          
        }
        
     }
  }
  
  // Return no activity
  return false;
  
}


