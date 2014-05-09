#include <avr/power.h>
#include <avr/sleep.h>

#include <Wire.h>
#include <MMA8452.h>

#include <SoftwareSerial.h>

MMA8452 accelerometer;

SoftwareSerial serial(13, 12);

volatile bool interrupt;
volatile bool sleeping;

void setup()
{
	serial.begin(57600);
	serial.print(F("Initializing MMA8452Q: "));

	Wire.begin();

	pinMode(5, OUTPUT);
	pinMode(9, OUTPUT);

	bool initialized = accelerometer.init(); 
	
	if (initialized)
	{
		serial.println(F("ok"));

		accelerometer.setPowerMode(MMA_HIGH_RESOLUTION);
		accelerometer.setDataRate(MMA_800hz);
		accelerometer.setRange(MMA_RANGE_2G);

		accelerometer.setAutoSleep(true, 0x11, MMA_SLEEP_1_56hz);
		accelerometer.setWakeOnInterrupt(true);

		accelerometer.setMotionDetectionMode(MMA_MOTION, MMA_ALL_AXIS);
		accelerometer.setMotionTreshold(0x11);

		accelerometer.setInterruptsEnabled(MMA_AUTO_SLEEP | MMA_FREEFALL_MOTION);
		accelerometer.configureInterrupts(false, false);
		accelerometer.setInterruptPins(true, true, true, true, true, true);

		attachInterrupt(0, accelerometerInterruptHandler, FALLING);
	}
	else
	{
		Serial.println(F("failed. Check connections."));
		while (true) {};
	}
}

void loop()
{
	if (interrupt)
	{
		noInterrupts();
		interrupt = false;
		interrupts();

		bool wakeStateChanged;
		bool transient;
		bool landscapePortrait;
		bool tap;
		bool freefallMotion;
		bool dataReady;
		accelerometer.getInterruptEvent(&wakeStateChanged, &transient, &landscapePortrait, &tap, &freefallMotion, &dataReady);

		if (wakeStateChanged) {
			serial.print(F("Wake state changed. Now: "));
			mma8452_mode_t mode = accelerometer.getMode();
			serial.println(mode);

			set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
			sleep_enable();
			sleep_mode();
			sleeping = true;
			// nap time.. ZZzz..

			// wait for interrupt to fire //

			// woke up from sleep
			sleeping = false;
			serial.println(F("Woke up"));
			sleep_disable();
		}

		if (transient)
		{
			serial.println(F("Transient"));
		}

		if (freefallMotion)
		{
			serial.println(F("Motion"));
		}
	}
}

void accelerometerInterruptHandler()
{
	if (sleeping)
	{
		sleep_disable();
	}
	interrupt = true;
}