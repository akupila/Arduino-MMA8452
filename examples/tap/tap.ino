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

		accelerometer.setDataRate(MMA_800hz);	// we need a quick sampling rate
		accelerometer.setRange(MMA_RANGE_2G);

		accelerometer.enableSingleTapDetector(MMA_X);
		accelerometer.enableDoubleTapDetector(MMA_X, 0x22, 0xCC);
		accelerometer.setTapThreshold(0x55, 0x55, 0x33);
	}
	else
	{
		Serial.println(F("failed. Check connections."));
		while (true) {};
	}
}

void loop()
{
	bool singleTap;
	bool doubleTap;
	bool x;
	bool y;
	bool z;
	bool negX;
	bool negY;
	bool negZ;
	accelerometer.getTapDetails(&singleTap, &doubleTap, &x, &y, &z, &negX, &negY, &negZ);

	if (singleTap || doubleTap)
	{

		serial.print(millis());
		serial.print(F(": "));
		if (doubleTap) serial.print(F("Double"));
		serial.print(F("Tap on "));
		if (x)
		{
			serial.print(F("X "));
			serial.print(negX ? F("left ") : F("right "));
		}
		if (y)
		{
			serial.print(F("Y "));
			serial.print(negY ? F("down ") : F("up "));
		}
		if (z)
		{
			serial.print(F("Z "));
			serial.print(negZ ? F("out ") : F("in "));
		}
		serial.println();
	}
}