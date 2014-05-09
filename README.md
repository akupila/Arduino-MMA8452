# MMA8452Q accelerometer Arduino/AVR library
======

This library allows you to use the MMA8452Q accelerometer from Freescale, a kickass little device with a lot of features

[Datasheet](http://www.freescale.com/files/sensors/doc/data_sheet/MMA8452Q.pdf)

Accelerometer features:
  
	* 1.95V to 3.6V supply voltage
	* 1.6V to 3.6V interface voltage
	* ±2g/±4g/±8g dynamically selectable full-scale
	* Output Data Rates (ODR) from 1.56 Hz to 800 Hz
	* 99 μg/√Hz noise
	* 12-bit and 8-bit digital output
	* I2C digital output interface
	* Two programmable interrupt pins for six interrupt sources
	* Three embedded channels of motion detection
		* Freefall or Motion Detection: 1 channel
		* Pulse Detection: 1 channel
		* Transient Detection: 1 channel
			* Orientation (Portrait/Landscape) detection with set hysteresis
			* Automatic ODR change for Auto-WAKE and return to SLEEP
			* High-Pass Filter Data available real-time
			* Self-Test
			* RoHS compliant
			* Current consumption: 6 μA to 165 μA
 
Library features:

	* Abstracts all registers to easy-to-use methods
	* Can be used with Arduino (Wire.h etc) or without
	
Documentation and examples are not complete, have a look at [MMA8452.h](MMA8452.h) for usage