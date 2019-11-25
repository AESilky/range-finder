/**
 * Test the LV-MaxSonar-EZ sonar range finder output methods with an Arduino.
 * Additionally test using the less expensive HC-SR04 device to compare the results 
 * and ease of use.
 *
 * This uses the three different data output methods of the LV-MaxSonar-EZ to see 
 * if there is any noticable difference in the values read by the Arduino and to 
 * illustrate the code required to read the values using the different methods. 
 * It also uses the more common (for hobby use) HC-SR04 device.
 *
 * The methods are (pin is on MaxSonar device):
 * 1. Serial Data on pin 5. Format:9600,8,n,1 Data:"Rxxx\r" xxx=0-255in.
 *    Enabled when BW (pin 1) is open/low. If BW is pulled high this becomes a
 *    trigger output pulse for multiple device chaining.
 * 2. Analog Voltage on pin 3. Value is Vcc/512 per inch. Supply of 5V:~9.8mV/in,
 *    3.3V:~6.4mV/in), buffered to provide continuous output corresponding to the 
 *    most recent range reading.
 * 3. Pulse high on pin 2 at 147uS per inch
 *
 * Control (pin is on MaxSonar device):
 * 1. BW on pin 1 controls the function of TX on pin 5. When open or pulled low serial
 * data is sent on TX. When pulled high TX sends a pulse to trigger additional chained
 * devices to avoid cross device noise interference.
 * 2. RX on pin 4 controls the measurement mode. When open or high the device continuously
 * measures distance. Then held low the device stops taking range measurements. When
 * pulsed high for >=20uS the device will take a range measurement and, if BW is high,
 * will send a pulse to trigger the next device in a chain.
 *
 * Per the Timing Diagram and Timing Description in the datasheet the pulse width value
 * will be available first and then the values of other two methods are available.
 * 
 * In continuous mode (not used in this sketch) no trigger is necessary and the PW 
 * and Serial data values will be generated and the analog value updated approximately 
 * once every 50mS.
 *
 * Datasheet: https://www.maxbotix.com/documents/LV-MaxSonar-EZ_Datasheet.pdf
 *
 * The distance values are sent in tab separated form to the main serial so
 * they can be displayed in the monitor or plotted.
 * They are sent in the format:
 * <pw distance in><tab><serial distance in><tab><analog distance in><cr-lf>
 * Every 20 rows a label row is sent in a format compatable with the Serial Plotter tool.
 * 
 * HC-SR04 Range Finder use:
 * A switch (or jumper) enables reading the distance using a HC-SR04 device and
 * including the value in the data sent. If enabled the data format sent is:
 * <pw distance in><tab><serial distance in><tab><analog distance in><tab><HC-SR04 distance in><cr-lf>
 *
 * HC-SR04 user guide: https://elecfreaks.com/estore/download/EF03085-HC-SR04_Ultrasonic_Module_User_Guide.pdf
 *  datasheet: https://www.elecrow.com/download/HC_SR04%20Datasheet.pdf
 */

// Libraries
#include <stdlib.h>
#include <SoftwareSerial.h>

// ///////////////////////////////////////////////////////////////////////////////////////////////////////
// Debugging help -
//  Provide debug serial print macros that can be easily and dynamically enabled/disabled.
//  set 'DEBUG_OUTPUT' to 'true' to print debug output, set to 'false' to supress print debug output.
//
//  A boolean is used so it can be changed by code.
bool _DEBUG_OUTPUT = false;
#define DB_PRINT(s) if(_DEBUG_OUTPUT) {Serial.print(s);}
#define DB_PRINTNB(n,b) if(_DEBUG_OUTPUT) {Serial.print(n,b);}
#define DB_PRINTLN(s) if(_DEBUG_OUTPUT) {Serial.println(s);}
#define DB_PRINTNBLN(n,b) if(_DEBUG_OUTPUT) {Serial.println(n,b);}
// ///////////////////////////////////////////////////////////////////////////////////////////////////////

// Constants
//  GPIO Pins (digital)
//   MaxSonar-EZ pins
#define MSE_PW_PIN 5
#define MSE_READ_MODE_PIN 4
#define MSE_SDATA_PIN 2
//   HC-SR04 pins
#define HCSR04_TRIGGER_PIN 6
#define HCSR04_ECHO_PIN 7
#define HCSR04_INCLUDE_PIN 13
//  GPIO Pins (analog)
#define MSE_AN_PIN A0

// MaxSonar to Arduino control/adjustments
#define MSE_READ_CONTINUOUS HIGH
#define MSE_READ_IDLE LOW
#define MSE_READ_TRIGGER HIGH
#define MSE_READ_REQUIRED_DURATION_mS 50
#define MSE_PW_START_DELAY_MSE_uS 3000
#define MSE_PW_uS_PER_INCH 147
#define MSE_ANALOG_DIVISOR 2
// HC-SR04 control/constants
#define HCSR04_INCLUDED LOW
//  Speed of Sound @ 22°C (71.6°F) = 344.5 m/S = 34450.0 cm/S
//  0.0000290275 S/cm = 29.0275762 uS/cm = 73.73 uS/in
#define SOUND_SPEED_uSpIN 73.73

// Globals
SoftwareSerial mseSerial(MSE_SDATA_PIN, 3, true); // RX,TX,inverse_logic (TX is not used)
int _loopCount = 0; // Used to track the times through loop
String _inputBuffer;
const String MT_STRING = String(""); 


void setup() {
  Serial.begin(38400); // Open serial communications
  mseSerial.begin(9600); // Set the rate for MaxSonar communications
  mseSerial.listen();
  // Wait for the serial port and allow time for the MaxSonar to initialize (>500mS from power-up)
  while (!Serial || millis() < 500) {
    ; // wait...
  }

  _inputBuffer.reserve(82); // reserve string space for an input line

  // Output a header, but don't use ' ' ',' or '\t' so it doesn't get picked up as a Plot Label when using Serial Plotter
  //  Use Unicode 'En-Space (0x2002)' [ ] instead. Be careful when editing this header - copy the 'space' between the 
  //  brackets and paste where needed.
  Serial.println();
  Serial.println("<<<==================================================================================================>>>");
  Serial.println("<<< LV-MaxSonar-EZ and HC-SR04 measurements. Distance data sent: Pulse | Serial | Analog | [HC-SR04] >>>");
  Serial.println("<<<==================================================================================================>>>");
  //
  // Output a row of 0's and 100's (~8') to set the scale of the Serial Plotter
  Serial.println("0\t0\t0\t0");
  Serial.println("100\t100\t100\t100");

  Serial.println();
  delay(1000);

  // Configure the rest of the pins used for the MaxSonar and HC-SR04
  pinMode(MSE_PW_PIN, INPUT);
  pinMode(MSE_AN_PIN, INPUT);
  digitalWrite(MSE_READ_MODE_PIN, MSE_READ_IDLE); // Use 'triggered' mode
  pinMode(MSE_READ_MODE_PIN, OUTPUT);
  // and HC-SR04
  pinMode(HCSR04_INCLUDE_PIN, INPUT_PULLUP); // Will be LOW if HC-SR04 should be included in operation
  digitalWrite(HCSR04_TRIGGER_PIN, HIGH);
  pinMode(HCSR04_TRIGGER_PIN, OUTPUT);
  pinMode(HCSR04_ECHO_PIN, INPUT);
}

void loop() {
  DB_PRINTLN("\n<<< ---------------------------------------------------------------------------------------------- >>>");
  DB_PRINT("Loop("); DB_PRINT(_loopCount); DB_PRINTLN(")");
  // Trigger a measurement and read the distance using the pulse-width method
  // then the serial and analog methods
  int pwDistance = tiggerAndReadDistanceFromPulse();
  int serialDistance = readDistanceFromSerial();
  int analogDistance = readDistanceFromAnalog();
  // Send the values to the monitor/plotter
  String dataValues = String(pwDistance) + '\t' + String(serialDistance) + '\t' + String(analogDistance) + '\t';
  if (includeHCSR04()) {
    int sr04Distance = sr04ReadDistance();
    dataValues += sr04Distance;
  }
  // Print the graph header at start and then every 20 lines of output
  DB_PRINTLN("\nPulse:\tSerial:\tAnalog:\tHC-SR04:");
  if (_loopCount++ % 20 == 0) {
    Serial.println("Pulse:\tSerial:\tAnalog:\tHC-SR04:");
  }
  Serial.println(dataValues);

  // check for input to enable/disable debug mode
  String inputLine = serialReadLine();
  // long enough for "debug on/off true/false"?
  if (inputLine.length() > 7) {
    if (!setDebugOutputMode(inputLine)) {
      Serial.print("Input line doesn't match required format to enable/disable debug mode. Received: "); Serial.println(inputLine);
    }
  }
  
  // wait for 1/4 second total to pass
  delay(250 - MSE_READ_REQUIRED_DURATION_mS);
}

/**
 * Read the MaxSonar analog distance value.
 *
 * MaxSonar value is 0-255 while Arduino value is 0-1023 so the
 * value read needs to be divided.
 *
 * triggerAndReadDistanceFromPulse() must be called before this
 * to get an updated distance measurement.
 *
 * Return: distance in inches
 */
int readDistanceFromAnalog() {
  int rawValue = analogRead(MSE_AN_PIN);
  int distance = rawValue / MSE_ANALOG_DIVISOR;
  DB_PRINT("\nA="); DB_PRINT(rawValue); DB_PRINT(" D="); DB_PRINTLN(distance);

  return distance;
}

/**
 * Read the MaxSonar serial distance value.
 * If a value can't be read 0 is returned.
 *
 * triggerAndReadDistanceFromPulse() must be called before this
 * to get an updated distance measurement.
 *
 * Return: distance in inches
 */
int readDistanceFromSerial() {
  int distance = 0;
  char text[6];
  text[0] = '\0';
  
  // Wait for a character to become available (or the maximum time for a measurement)
  DB_PRINT("\nS ("); DB_PRINT(mseSerial.available()); DB_PRINT(")...");
  int timeout = MSE_READ_REQUIRED_DURATION_mS;
  for (; timeout > 0 && mseSerial.available() < 5; timeout--) {
    delay(1);
  }
  DB_PRINT("\n t="); DB_PRINT(timeout); DB_PRINT(" cc="); DB_PRINT(mseSerial.available());
  if (timeout > 0) { // didn't time out
    DB_PRINT(" data=[");
    // Build up the string looking for a carriage-return ('\r') or a maximum of
    // 5 characters. MaxSonar format is "Rxxx\r".
    // 
    // Wait up to the maximum MaxSonar measurement time to receive 5 characters...
    for(long start=millis(); mseSerial.available() < 5 && millis()-start < MSE_READ_REQUIRED_DURATION_mS; ) {
      delay(1); // short delay so we don't slam cpu
    }

    int i = 0;
    if (mseSerial.available() >= 5) {
      for(; i<5; i++) {
        char c = (char)mseSerial.read();
        DB_PRINT("'0x"); DB_PRINTNB(c,HEX); DB_PRINT("'");
        text[i] = (c != '\r' ? c : '\0'); // terminate with null when RETURN is received
      }
      text[i] = '\0'; // Add null terminator
    }
    // Convert the string to an integer value
    distance = atoi(&text[1]);
  }
  DB_PRINT("] text='"); DB_PRINT(text); DB_PRINT("' distance="); DB_PRINTLN(distance);

  return distance;
}

/**
 * To assure that the distance pulse is detected going from low to high,
 * this triggers the MaxSonar to read and then measures the pulse.
 *
 * The analog input is held after a reading and the serial will be
 * held in the serial input buffer since it is only 6 characters.
 *
 * Refer to the Timing Diagram and Description in the datasheet for timing details.
 */
int tiggerAndReadDistanceFromPulse() {
  DB_PRINTLN("LV-MaxSonar-EZ Triggering...");
  // Clear out the mseSerial read buffer...
  DB_PRINT(" serial-buffer clear("); DB_PRINT(mseSerial.available()); DB_PRINT(")-");
  mseSerial.stopListening();
  mseSerial.listen();
  DB_PRINT("("); DB_PRINT(mseSerial.available()); DB_PRINTLN(")");
  // Assure that the trigger is set to IDLE (HOLD)
  digitalWrite(MSE_READ_MODE_PIN, MSE_READ_IDLE);
  delayMicroseconds(10);
  //
  // Minimum time for readings (start of reading to start of reading)
  // is 49mS.
  //
  // To guarentee readings are not taken too quickly this method makes
  // sure that it takes at least this minimum time before returning by
  // recording the start time and waiting for the minimum time to have elapsed.
  //
  unsigned long startMillis = millis();

  // Trigger and measure
  digitalWrite(MSE_READ_MODE_PIN, MSE_READ_TRIGGER);
  delayMicroseconds(10);
  unsigned long pulseWidth = pulseIn(MSE_PW_PIN, HIGH, (MSE_PW_START_DELAY_MSE_uS + (MSE_READ_REQUIRED_DURATION_mS * 1000L)));
  int distance = (int)(pulseWidth / MSE_PW_uS_PER_INCH);
  digitalWrite(MSE_READ_MODE_PIN, MSE_READ_IDLE);
 
  // Wait for the minimum time to pass (first check for overflow)
  if (millis() < startMillis) {
    startMillis = millis();
  }
  while (millis() - startMillis < MSE_READ_REQUIRED_DURATION_mS) {
    delay(1);
  }
  // Analog and Serial can now be read as well
  DB_PRINT(" pw="); DB_PRINT(pulseWidth); DB_PRINT(" distance="); DB_PRINT(distance); DB_PRINT(" f(n)T="); DB_PRINTLN(millis() - startMillis);
  
  return distance;
}


//////////////////////////////////////////////////////////////////////////////
// HC-SR04 Functions
//////////////////////////////////////////////////////////////////////////////

/**
 * Check whether the HC-SR04 device should be included in the measurements.
 *
 * Return: TRUE if it should be included
 */
bool includeHCSR04() {
  return (digitalRead(HCSR04_INCLUDE_PIN) == LOW);
}

/**
 * Trigger measurement and calculate distance (in inches)
 * Range 1" to 157" (13')
 * 
 * @return Distance in inches
 */
int sr04ReadDistance()
{
  long duration;
  int distance;
  
  DB_PRINT("\nHC-SR04 Trigger...");
  digitalWrite(HCSR04_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(HCSR04_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(HCSR04_TRIGGER_PIN, LOW);
  // Wait for the echo with timeout of < 18,000μs (just over 10' (20' round trip) just less than datasheet max)
  duration = pulseIn(HCSR04_ECHO_PIN, HIGH, 18000);
  duration = (duration >= 18000 ? 0 : duration);
  distance = (int)(((float)duration / SOUND_SPEED_uSpIN) / 2.0); // divide by 2 due to round trip (out and back)
  digitalWrite(HCSR04_TRIGGER_PIN, HIGH);

  DB_PRINT(" duration="); DB_PRINT(duration); DB_PRINT(" distance="); DB_PRINTLN(distance);
  
  return distance;
}


//////////////////////////////////////////////////////////////////////////////
// Utility Functions
//////////////////////////////////////////////////////////////////////////////

/**
 * Read a 'line' from the serial input if available.
 * 
 * This function builds a line from the available serial input characters 
 * until a newline '\n' is read. It uses a global input string to 
 * accumulate input until the newline is found (to a maximum of 200 characters).
 * 
 * When called, if a newline terminated string has not been collected an empty string  
 * is returned.
 * 
 * The length of the line is limited to the last 80 characters received. 
 * (back in the day, CRT terminals were 80 characters by 24 lines)
 * 
 */
String serialReadLine() {
  while(Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      // Found a 'line' of text. Put the string together and return it.
      Serial.println(_inputBuffer);
      String retVal = _inputBuffer;
      _inputBuffer = String();
      _inputBuffer.reserve(82);
      return retVal;
    }
    else {
      _inputBuffer.concat(c);
      // limit length to the last 80 chars
      if (_inputBuffer.length() > 80) {
        _inputBuffer.remove(0, (_inputBuffer.length() - 80));
      }
    }
  }

  return MT_STRING;
}

/**
 * Enable/disable DEBUG OUTPUT mode (set _DEBUG_OUTPUT true or false)
 * 
 * This takes a string in the form:
 *  debug = 'true'/'false | 'on'/'off'
 * and enables/disables DEBUG mode.
 * 
 * If the input string doesn't conform to the syntax of enabling or disabling 
 * the DEBUG mode, the function returns false.
 * 
 */
bool setDebugOutputMode(String s) {
  bool validInput = false;

  // Trim and convert to lowercase to test for arguments
  s.trim();
  s.toLowerCase();
  if (s.startsWith("debug ")) {
    s = s.substring(6); // remove 'debug ' and continue...
    if (s.equals("true") || s.equals("on")) {
      _DEBUG_OUTPUT = true;
      validInput = true;
    }
    else if (s.equals("false") || s.equals("off")) {
      _DEBUG_OUTPUT = false;
      validInput = true;
    }
  }

  return validInput;
}
