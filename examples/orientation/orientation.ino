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

		accelerometer.setDataRate(MMA_1_56hz);
		accelerometer.setRange(MMA_RANGE_2G);
		accelerometer.enableOrientationChange(true);
	}
	else
	{
		Serial.println(F("failed. Check connections."));
		while (true) {};
	}
}

void loop()
{
	bool orientationChanged;
	bool zTiltLockout;
	mma8452_orientation_t orientation;
	bool back;

	accelerometer.getPortaitLandscapeStatus(&orientationChanged, &zTiltLockout, &orientation, &back);

	if (orientationChanged)
	{
		serial.print("Orientation is now ");
		switch (orientation)
		{
			case MMA_PORTRAIT_UP:
				serial.print(F("Portrait up"));
				break;
			case MMA_PORTRAIT_DOWN:
				serial.print(F("Portrait down"));
				break;
			case MMA_LANDSCAPE_RIGHT:
				serial.print(F("Landscape right"));
				break;
			case MMA_LANDSCAPE_LEFT:
				serial.print(F("Landscape left"));
				break;
		}
		serial.print(F(" (back: "));
		serial.print(back ? F("yep )") : F("nope)"));
		serial.print(F(" - Z Tilt lockout: "));
		serial.println(zTiltLockout);
	}
}