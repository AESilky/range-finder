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
 * 1. Serial Data on pin 5. Format:9600,8,n,1 Data:"Rxxx\n" xxx=0-255in.
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
 * will be available the quickest of the three methods.
 *
 * Datasheet: https://www.maxbotix.com/documents/LV-MaxSonar-EZ_Datasheet.pdf
 *
 * The distance values are sent in tab separated form to the main serial (USB) so
 * they can be displayed in the monitor or plotted.
 * They are sent once a second in the format:
 * <pw distance in><tab><serial distance in><tab><analog distance in><cr-lf>
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
//  Provide debug serial print macros that can be easily enabled/disabled.
//  set 'DEBUG_OUTPUT' to 'true' to print debug output, set to 'false' to not print debug output
#define DEBUG_OUTPUT true
#if DEBUG_OUTPUT
 #define DB_PRINT(s) Serial.print(s)
 #define DB_PRINTNB(n,b) Serial.print(n,b)
 #define DB_PRINTLN(s) Serial.println(s)
 #define DB_PRINTNBLN(n,b) Serial.println(n,b)
#else
 #define DB_PRINT(s)
 #define DB_PRINTNB(n,b)
 #define DB_PRINTLN(s)
 #define DB_PRINTNBLN(n,b)
#endif
// ///////////////////////////////////////////////////////////////////////////////////////////////////////

// Constants
//  GPIO Pins (digital)
//   MaxSonar pins
#define MAX_PW_PIN 5
#define MAX_READ_MODE_PIN 4
#define MAX_SDATA_PIN 2
//   HC-SR04 pins
#define SR04_TRIGGER_PIN 6
#define SR04_ECHO_PIN 7
#define SR04_INCLUDE_PIN 13
//  GPIO Pins (analog)
#define MAX_AN_PIN A0

// MaxSonar to Arduino control/adjustments
#define MAX_READ_CONTINUOUS HIGH
#define MAX_READ_IDLE LOW
#define MAX_READ_TRIGGER HIGH
#define MAX_READ_REQUIRED_DURATION_mS 50
#define MAX_PW_START_DELAY_MAX_uS 3000
#define MAX_PW_uS_PER_INCH 147
#define MAX_ANALOG_DIVISOR 2
// HC-SR04 control/constants
#define SR04_INCLUDED LOW
//  Speed of Sound @ 22°C (71.6°F) = 344.5 m/S = 34450.0 cm/S
//  0.0000290275 S/cm = 29.0275762 uS/cm = 73.73 uS/in
#define SOUND_SPEED_uSpIN 73.73

// Globals
SoftwareSerial maxSerial(MAX_SDATA_PIN, 3, true); // RX,TX,inverse_logic (TX is not used)
int loopCount = 0; // Used to track the times through loop

void setup() {
  Serial.begin(38400); // Open serial communications
  maxSerial.begin(9600); // Set the rate for MaxSonar communications
  maxSerial.listen();
  // Wait for the serial port and allow time for the MaxSonar to initialize (>500mS from power-up)
  while (!Serial || millis() < 500) {
    ; // wait...
  }

  Serial.println("");
  Serial.println("LV-MaxSonar-EZ and HC-SR04 measurements test. Distance data sent: Pulse, Serial, Analog, [HC-SR04]");
  Serial.println("");
  Serial.println();
  delay(1000);

  // Configure the rest of the pins used for the MaxSonar and HC-SR04
  pinMode(MAX_PW_PIN, INPUT);
  pinMode(MAX_AN_PIN, INPUT);
  digitalWrite(MAX_READ_MODE_PIN, MAX_READ_IDLE); // Use 'triggered' mode
  pinMode(MAX_READ_MODE_PIN, OUTPUT);
  // and HC-SR04
  pinMode(SR04_INCLUDE_PIN, INPUT_PULLUP); // Will be LOW if HC-SR04 should be included in operation
  digitalWrite(SR04_TRIGGER_PIN, HIGH);
  pinMode(SR04_TRIGGER_PIN, OUTPUT);
  pinMode(SR04_ECHO_PIN, INPUT);
}

void loop() {
  DB_PRINTLN("loop()");
  // Print the graph header at start and then every 20 lines of output
  if (loopCount++ % 20 == 0) {
    Serial.println("Pulse\tSerial\tAnalog\tHC-SR04");
  }
  // Trigger a measurement and read the distance using the pulse-width method
  // then the serial and analog methods
  int pwDistance = tiggerAndReadDistanceFromPulse();
  int serialDistance = readDistanceFromSerial();
  int analogDistance = readDistanceFromAnalog();
  // Send the values to the monitor/plotter
  String dataValues = String(pwDistance) + '\t' + String(serialDistance) + '\t' + String(analogDistance) + '\t';
  if (includeSR04()) {
    int sr04Distance = sr04ReadDistance();
    dataValues += sr04Distance;
  }
  Serial.println(dataValues);
  // wait for 1/4 second total to pass
  delay(250 - MAX_READ_REQUIRED_DURATION_mS);
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
  int rawValue = analogRead(MAX_AN_PIN);
  int distance = rawValue / MAX_ANALOG_DIVISOR;
  DB_PRINT("\nA:"); DB_PRINT(rawValue); DB_PRINT(" D:"); DB_PRINTLN(distance);

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
  DB_PRINT("\nS ("); DB_PRINT(maxSerial.available()); DB_PRINT(")...");
  int timeout = MAX_READ_REQUIRED_DURATION_mS;
  for (; timeout > 0 && maxSerial.available() < 5; timeout--) {
    delay(1);
  }
  DB_PRINT("\n t:"); DB_PRINT(timeout); DB_PRINT(" cc:"); DB_PRINT(maxSerial.available());
  if (timeout > 0) { // didn't time out
    DB_PRINT(" data:[");
    // Build up the string looking for a carriage-return ('\r') or a maximum of
    // 5 characters. MaxSonar format is "Rxxx\r".
    // 
    // Wait up to the maximum MaxSonar measurement time to receive 5 characters...
    for(long start=millis(); maxSerial.available() < 5 && millis()-start < MAX_READ_REQUIRED_DURATION_mS; ) {
      delay(1); // short delay so we don't slam cpu
    }

    int i = 0;
    if (maxSerial.available() >= 5) {
      for(; i<5; i++) {
        char c = (char)maxSerial.read();
        DB_PRINT("'0x"); DB_PRINTNB(c,HEX); DB_PRINT("'");
        text[i] = (c != '\r' ? c : '\0'); // terminate with null when RETURN is received
      }
      text[i] = '\0'; // Add null terminator
    }
    // Convert the string to an integer value
    distance = atoi(&text[1]);
  }
  DB_PRINT("] text:'"); DB_PRINT(text); DB_PRINT("' distance:"); DB_PRINTLN(distance);

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
  DB_PRINT("\nMax Trigger...");
  // Clear out the maxSerial read buffer...
  DB_PRINT(" clear("); DB_PRINT(maxSerial.available()); DB_PRINT(")-");
  maxSerial.stopListening();
  maxSerial.listen();
  DB_PRINT("("); DB_PRINT(maxSerial.available()); DB_PRINT("):");
  // Assure that the trigger is set to IDLE (HOLD)
  digitalWrite(MAX_READ_MODE_PIN, MAX_READ_IDLE);
  delayMicroseconds(10);
  //
  // Minimum time for readings (start of reading to start of reading)
  // is 49mS.
  //
  // To guarentee readings are not taken to quickly this method makes
  // sure that it takes at least this minimum time refore returning by
  // recording the start time and waiting for the minimum time to have elapsed.
  //
  unsigned long startMillis = millis();

  // Trigger and measure
  digitalWrite(MAX_READ_MODE_PIN, MAX_READ_TRIGGER);
  delayMicroseconds(10);
  unsigned long pulseWidth = pulseIn(MAX_PW_PIN, HIGH, (MAX_PW_START_DELAY_MAX_uS + (MAX_READ_REQUIRED_DURATION_mS * 1000L)));
  int distance = (int)(pulseWidth / MAX_PW_uS_PER_INCH);
  digitalWrite(MAX_READ_MODE_PIN, MAX_READ_IDLE);
 
  // Wait for the minimum time to pass (first check for overflow)
  if (millis() < startMillis) {
    startMillis = millis();
  }
  while (millis() - startMillis < MAX_READ_REQUIRED_DURATION_mS) {
    delay(1);
  }
  // Analog and Serial can now be read as well
  DB_PRINT(" pw:"); DB_PRINT(pulseWidth); DB_PRINT(" distance:"); DB_PRINT(distance); DB_PRINT(" fnT:"); DB_PRINTLN(millis() - startMillis);
  
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
bool includeSR04() {
  return (digitalRead(SR04_INCLUDE_PIN) == LOW);
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
  
  DB_PRINT("\nSR04 Trigger...");
  digitalWrite(SR04_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SR04_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SR04_TRIGGER_PIN, LOW);
  duration = pulseIn(SR04_ECHO_PIN, HIGH, 38000); // Wait for the echo with a timeout of 38m (from datasheet for no object detected)
  distance = (int)(((float)duration / SOUND_SPEED_uSpIN) / 2.0); // divide by 2 due to round trip (out and back)
  digitalWrite(SR04_TRIGGER_PIN, HIGH);

  DB_PRINT(" duration:"); DB_PRINT(duration); DB_PRINT(" distance:"); DB_PRINTLN(distance);
  
  return distance;
}
