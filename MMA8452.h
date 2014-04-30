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

class MMA8452
{

	public:
		bool init();

		void setRange(mma8452_range_t range);
		mma8452_range_t getRange();

		void getRawData(uint16_t *x, uint16_t *y, uint16_t *z);
		void getAcceleration(float *x, float *y, float *z);

		mma8452_mode_t getMode();
		void getInterruptEvent(bool *wakeStateChanged, bool *movementOccurred, bool *landscapePortrait, bool *pulseEvent, bool *freefall, bool *dataReady);

		// todo: implement Pulse_LPF_EN
		void setHighPassFilter(bool enabled, mma8452_highpass_mode_t mode);
		bool getHighPassFilter(mma8452_highpass_mode_t *mode = NULL);

		void getPortaitLandscapeStatus(bool *orientationChanged, bool *zTiltLockoutDetected, mma8452_orientation_t *orientation, bool *back);

		void configureLandscapePortraitDetection(bool enableDetection, uint8_t debounceCount = 0, bool debounceTimeout = true);
		void getLandscapePortraitConfig(bool *enabled, uint8_t *debounceCount, bool *debounceTimeout);

		// threshold: 8G/127
		void setMotionTreshold(uint8_t threshold, uint8_t debounceCount = 0, bool resetDebounceOnNoMotion = 0);

		SoftwareSerial *debug;

	private:
		mma8452_range_t range;

		void setActive(bool standby);
		byte read(byte reg);
		void readMultiple(byte reg, byte *buffer, uint8_t numBytes);
		void write(byte reg, byte value);

		float convertGCounts(uint16_t data);
};

#endif // MMA8452_H_