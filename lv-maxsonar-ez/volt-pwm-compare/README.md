# Compare Voltage and Pulse-Width output for distance measurement

The LV-MaxSonar provides three methods to retrieve the distance 
measurement:  
1. Serial (Pin 5: UART @ 9600 BAUD)
2. Voltage (Pin 3: Proportional to the distance)
3. Pulse-width (Pin 2: Proportional to the distance)

The Ardino UNO only has a single Serial port, so this test will 
not use the serial output method to keep the serial open for other 
needs.

The voltage and pulse-width methods will be used to compare the 
output and the ability for the Arduino to use it.

## Voltage Output (pin 3)

Pin 3-AN- Outputs analog voltage with a scaling factor of (Vcc/512) per inch. 
A supply of 5V yields ~9.8mV/in. and 3.3V yields ~6.4mV/in. The output is 
buffered and corresponds to the most recent range data.

## Pulse Width Output (pin 2)

Pin 2-PW- This pin outputs a pulse width representation of range. 
The distance can be calculated using the scale factor of 147uS per inch.

## Power

Power is 3.3 to 5.0 volts on pin 6 and GND on pin 7.

