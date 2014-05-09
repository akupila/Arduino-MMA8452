#include <Wire.h>
#include <MMA8452.h>

#include <SoftwareSerial.h>

MMA8452 accelerometer;

SoftwareSerial serial(13, 12);

void setup()
{
	serial.begin(57600);
	serial.print(F("Initializing MMA8452Q: "));

	Wire.begin();

	bool initialized = accelerometer.init(); 
	
	if (initialized)
	{
		serial.println(F("ok"));

		accelerometer.setDataRate(MMA_400hz);
		accelerometer.setRange(MMA_RANGE_2G);

		accelerometer.setMotionDetectionMode(MMA_MOTION, MMA_ALL_AXIS);
		accelerometer.setMotionTreshold(0x11);
	}
	else
	{
		Serial.println(F("failed. Check connections."));
		while (true) {};
	}
}

void loop()
{
	bool motion = accelerometer.motionDetected();
	if (motion)
	{
		serial.print(F("Motion @ "));
		serial.println(millis());
	}
}