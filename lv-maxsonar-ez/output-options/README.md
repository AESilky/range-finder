# Test Serial, Voltage, and Pulse-Width Output of the LV-MaxSonar Range Finder

The LV-MaxSonar provides three methods to retrieve the distance 
measurement:  
1. Serial (Pin 5: UART @ 9600 BAUD)
2. Voltage (Pin 3: Proportional to the distance)
3. Pulse-width (Pin 2: Proportional to the distance)

The Ardino UNO only has a single Serial port, so this test will 
use the software serial library to keep the serial port open for 
other needs (output to the IDE Serial Monitor and Plotter).

## Pulse Width Output (pin 2)

Pin 2-PW- This pin outputs a pulse width representation of range. 
The distance can be calculated using the scale factor of 147uS per inch.

## Voltage Output (pin 3)

Pin 3-AN- Outputs analog voltage with a scaling factor of (Vcc/512) per inch. 
A supply of 5V yields ~9.8mV/in. and 3.3V yields ~6.4mV/in. The output is 
buffered and corresponds to the most recent range data.

## Serial Data Output (pin 5)

Pin 5-TX- Outputs serial data at 9600,8,N in the format: Rnnn\r 
Where 'nnn' is three ASCII digits representing the distance in inches.

## Power

Power is 3.3 to 5.0 volts on pin 6 and GND on pin 7. Current is 3mA@5V.

# Optional Comparison to HC-SR04 Sonar Range Finder (common 'bugeye' device)

The Arduino code includes the ability to optionally measure using the more 
common HC-SR04 device and add the distance value to the data sent.

This allows comparing the measured distance and the sensitivity of the 
two devices.

A switch (or jumper) in the circuit allows for taking and including the 
HC-SR04 measurement.

