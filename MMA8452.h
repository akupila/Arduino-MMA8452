#ifndef MMA8452_H_
#define MMA8452_H_

#include <Arduino.h>
#include <Wire.h>

#include "MMA8452Reg.h"
#include "SoftwareSerial.h"

// I2C address set in hardware (tied high or low)
#define SA0 0
#if SA0
#define MMA8452_ADDRESS 0x1D
#else
#define MMA8452_ADDRESS 0x1C
#endif

typedef enum {
	MMA_RANGE_2G = 0,
	MMA_RANGE_4G,
	MMA_RANGE_8G
} mma8452_range_t;

typedef enum {
	MMA_STANDBY = 0,
	MMA_WAKE,
	MMA_SLEEP
} mma8452_mode_t;

// See table on page 23 in datasheet (http://www.freescale.com/files/sensors/doc/data_sheet/MMA8452Q.pdf)
typedef enum {
	MMA_HP1 = 0,
	MMA_HP2,
	MMA_HP3,
	MMA_HP4
} mma8452_highpass_mode_t;

typedef enum {
	MMA_PORTRAIT_UP = 0,
	MMA_PORTRAIT_DOWN,
	MMA_LANDSCAPE_RIGHT,
	MMA_LANDSCAPE_LEFT
} mma8452_orientation_t;

typedef enum {
	MMA_FREEFALL = 0,
	MMA_MOTION
} mma8452_motion_type_t;

// sleep sampling mode
typedef enum {
	MMA_SLEEP_50hz = 0,
	MMA_SLEEP_12_5hz,
	MMA_SLEEP_6_25hz,
	MMA_SLEEP_1_56hz
} mma8452_sleep_frequency_t;

// normal running mode
typedef enum {
	MMA_800hz = 0,
	MMA_400hz,
	MMA_200hz,
	MMA_100hz,
	MMA_50hz,
	MMA_12_5hz,
	MMA_6_25hz,
	MMA_1_56hz
} mma_datarate_t;

// power mode
typedef enum {
	MMA_NORMAL = 0,
	MMA_LOW_NOISE_LOW_POWER,
	MMA_HIGH_RESOLUTION,
	MMA_LOW_POWER
} mma_power_mode_t;

typedef enum {
	MMA_AUTO_SLEEP = 0x80,
	MMA_TRANSIENT = 0x20,
	MMA_ORIENTATION_CHANGE = 0x10,
	MMA_TAP = 0x08,
	MMA_FREEFALL_MOTION = 0x04,
	MMA_DATA_READY = 0x01
} mma_interrupt_types_t;

class MMA8452
{

	public:
		bool init();

		void setRange(mma8452_range_t range);
		mma8452_range_t getRange();

		void getRawData(uint16_t *x, uint16_t *y, uint16_t *z);
		void getAcceleration(float *x, float *y, float *z);

		mma8452_mode_t getMode();
		void getInterruptEvent(bool *wakeStateChanged = NULL, bool *movementOccurred = NULL, bool *landscapePortrait = NULL, bool *pulseEvent = NULL, bool *freefall = NULL, bool *dataReady = NULL);

		// todo: implement Pulse_LPF_EN
		void setHighPassFilter(bool enabled, mma8452_highpass_mode_t mode = MMA_HP1);
		bool getHighPassFilter(mma8452_highpass_mode_t *mode = NULL);

		void getPortaitLandscapeStatus(bool *orientationChanged, bool *zTiltLockoutDetected, mma8452_orientation_t *orientation, bool *back);

		void configureLandscapePortraitDetection(bool enableDetection, uint8_t debounceCount = 0, bool debounceTimeout = true);
		void getLandscapePortraitConfig(bool *enabled, uint8_t *debounceCount, bool *debounceTimeout);

		void setMotionDetectionMode(mma8452_motion_type_t motion, bool xAxis, bool yAxis, bool zAxis, bool latchMotion = false);
		bool motionDetected(bool *x, bool *y, bool *z, bool *negativeX = NULL, bool *negativeY = NULL, bool *negativeZ = NULL);
		// threshold: 8G/127
		void setMotionTreshold(uint8_t threshold, uint8_t debounceCount = 0, bool resetDebounceOnNoMotion = 0);

		void setTransientDetection(bool xAxis, bool yAxis, bool zAxis, bool latchMotion = false, bool bypassHighPass = false);
		bool transientDetected(bool *x, bool *y, bool *z, bool *negativeX = NULL, bool *negativeY = NULL, bool *negativeZ = NULL);
		// threshold: 8G/127
		void setTransientTreshold(uint8_t threshold, uint8_t debounceCount = 0, bool resetDebounceOnNoMotion = 0);

		void enableSingleTapDetector(bool xAxis, bool yAxis, bool zAxis);
		void enableDoubleTapDetector(bool xAxis, bool yAxis, bool zAxis, uint8_t minDuration, uint8_t maxDuration, bool abortOnQuickDoubleTap);
		bool tapDetected(bool *doubleTap, bool *x, bool *y, bool *z, bool *negativeX, bool *negativeY, bool *negativeZ);
		void setTapThreshold(uint8_t x, uint8_t y, uint8_t z);
		void setMaxTapDuration(uint8_t maxDuration);

		void setAutoSleep(bool enabled, uint8_t time, mma8452_sleep_frequency_t sleepFrequencySampling = MMA_SLEEP_1_56hz, mma_power_mode_t sleepPowerMode = MMA_LOW_POWER);
		void setWakeOnInterrupt(bool transient, bool landscapePortraitChange, bool tap, bool freefall_motion);

		void setDataRate(mma_datarate_t dataRate);
		void setLowNoiseMode(bool enabled);
		void set8BitMode(bool enabled);

		void reset(); // todo: might not be working

		void setPowerMode(mma_power_mode_t powerMode);

		void setInterruptsEnabled(uint8_t interruptMask);
		void configureInterrrupts(bool activeHigh, bool openDrain);
		// true: pin1, false: pin2
		void setInterruptPins(bool autoSleepWake, bool transient, bool landscapePortraitChange, bool tap, bool freefall_motion, bool dataReady);

		// 2mG/LSB
		void setOffsets(int8_t x, int8_t y, int8_t z);

		void setActive(bool active = true);

		SoftwareSerial *debug;

	private:
		bool active;

		mma8452_range_t range;

		void standby(bool standby);
		byte read(byte reg);
		void readMultiple(byte reg, byte *buffer, uint8_t numBytes);
		void write(byte reg, byte value);

		bool singleTapEnabled;
		bool doubleTapEnabled;

		float convertGCounts(uint16_t data);
		int8_t convertTo2sComplement(int8_t value);
};

#endif // MMA8452_H_