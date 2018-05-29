#include "MMA8452.h"

#define BIT_0 0x01
#define BIT_1 0x02
#define BIT_2 0x04
#define BIT_3 0x08
#define BIT_4 0x10
#define BIT_5 0x20
#define BIT_6 0x40
#define BIT_7 0x80

bool MMA8452::init()
{
	// Check who-am-i register
	uint8_t reg = read(WHO_AM_I);
	if (reg != 0x2A)
	{
		// this will always return 0x2A, connections probably wrong
		return false;
	}

	// range on startup is always 2g, we'll need to know the range for G calculation
	range = MMA_RANGE_2G;
	active = true; // active by default
	return true;
}

void MMA8452::getRawData(uint16_t *x, uint16_t *y, uint16_t *z)
{
	// the x, y, z registers are consecutive so read them in one go
	// data: x [msb, lsb]   y [msb, lsb]   z [msb, lsb]
	uint8_t data[6];
	readMultiple(OUT_X_MSB, data, 6);
	*x = (data[0] << 8) | data[1];
	*y = (data[2] << 8) | data[3];
	*z = (data[4] << 8) | data[5];
}

void MMA8452::getAcceleration(float *x, float *y, float *z)
{
	uint16_t rawX;
	uint16_t rawY;
	uint16_t rawZ;
	getRawData(&rawX, &rawY, &rawZ);
	*x = convertGCounts(rawX);
	*y = convertGCounts(rawY);
	*z = convertGCounts(rawZ);
}

mma8452_mode_t MMA8452::getMode()
{
	uint8_t sysmode = read(SYSMOD);
	return (mma8452_mode_t)(sysmode & 0x3);
}

void MMA8452::getInterruptEvent(bool *wakeStateChanged, bool *transient, bool *landscapePortrait, bool *tap, bool *freefallMotion, bool *dataReady)
{
	uint8_t int_source = read(INT_SOURCE);
	if (wakeStateChanged) *wakeStateChanged = int_source & BIT_7;
	if (transient) *transient = int_source & BIT_5;
	if (landscapePortrait) *landscapePortrait = int_source & BIT_4;
	if (tap) *tap = int_source & BIT_3;
	if (freefallMotion) *freefallMotion = int_source & BIT_2;
	if (dataReady) *dataReady = int_source & BIT_0;
}

void MMA8452::setRange(mma8452_range_t newRange)
{
	uint8_t reg = read(XYZ_DATA_CFG);
	reg &= ~0x3;
	reg |= newRange;
	write(XYZ_DATA_CFG, reg);
	range = newRange;
}

mma8452_range_t MMA8452::getRange()
{
	uint8_t reg = read(XYZ_DATA_CFG);
	return (mma8452_range_t)(reg & 0x3);
}

void MMA8452::setHighPassFilter(bool enabled, mma8452_highpass_mode_t mode)
{
	uint8_t reg = read(XYZ_DATA_CFG);
	reg &= ~BIT_4;
	if (enabled) reg |= BIT_4;
	write(XYZ_DATA_CFG, reg);

	reg = read(HP_FILTER_CUTOFF);
	reg &= ~(0x03);
	reg |= mode;
	write(HP_FILTER_CUTOFF, reg);
}

void MMA8452::enableOrientationChange(bool enabled, bool clearCounterWhenInvalid)
{
	uint8_t reg = read(PL_CFG);
	reg &= ~BIT_7;
	if (clearCounterWhenInvalid) reg |= BIT_7;
	reg &= ~BIT_6;
	if (enabled) reg |= BIT_6;
	write(PL_CFG, reg);
}

void MMA8452::getPortaitLandscapeStatus(bool *orientationChanged, bool *zTiltLockoutDetected, mma8452_orientation_t *orientation, bool *back)
{
	uint8_t pl_status = read(PL_STATUS);
	*orientationChanged = pl_status & BIT_7;
	*zTiltLockoutDetected = pl_status & BIT_6;
	*orientation = (mma8452_orientation_t)((pl_status >> 1) & 0x3);
	*back = pl_status & BIT_0;
}

bool MMA8452::isFlat()
{
	return read(PL_STATUS) & BIT_6;
}

void MMA8452::configureLandscapePortraitDetection(bool enableDetection, uint8_t debounceCount, bool debounceTimeout)
{
	uint8_t pl_cfg = read(PL_CFG);
	pl_cfg &= ~BIT_7;
	if (debounceTimeout) pl_cfg |= BIT_7;
	pl_cfg &= ~BIT_6;
	if (enableDetection) pl_cfg |= BIT_6;
	write(PL_CFG, pl_cfg);

	write(PL_COUNT, debounceCount);
}

void MMA8452::setMotionDetectionMode(mma8452_motion_type_t motion, uint8_t axis, bool latchMotion)
{
	bool x = axis & MMA_X;
	bool y = axis & MMA_Y;
	bool z = axis & MMA_Z;
	uint8_t reg =  (latchMotion << 7) |
				(motion << 6) |
				(z << 5) |
				(y << 4) |
				(x << 3);
	write(FF_MT_CFG, reg);
}

void MMA8452::setMotionTreshold(uint8_t threshold, uint8_t debounceCount, bool resetDebounceOnNoMotion)
{
	if (threshold > 127) threshold = 127;
	write(FF_MT_THS, (resetDebounceOnNoMotion << 7) | threshold);
	write(FF_MT_COUNT, debounceCount);
}

bool MMA8452::motionDetected(bool *x, bool *y, bool *z, bool *negativeX, bool *negativeY, bool *negativeZ)
{
	uint8_t ff_mt_src = read(FF_MT_SRC);
	bool motionDetected = ff_mt_src & BIT_7;
	if (z) *z = ff_mt_src & BIT_5;
	if (y) *y = ff_mt_src & BIT_3;
	if (x) *x = ff_mt_src & BIT_1;
	if (negativeZ) *negativeZ = ff_mt_src & BIT_4;
	if (negativeY) *negativeY = ff_mt_src & BIT_2;
	if (negativeX) *negativeX = ff_mt_src & BIT_0;

	return motionDetected;
}

void MMA8452::setTransientDetection(uint8_t axis, bool latchMotion, bool bypassHighPass)
{
	bool x = axis & MMA_X;
	bool y = axis & MMA_Y;
	bool z = axis & MMA_Z;
	uint8_t reg =  (latchMotion << 4) |
				(z << 3) |
				(y << 2) |
				(x << 1) |
				bypassHighPass;

	write(TRANSIENT_CFG, reg);
}

bool MMA8452::transientDetected(bool *x, bool *y, bool *z, bool *negativeX, bool *negativeY, bool *negativeZ)
{
	uint8_t ff_transient_src = read(TRANSIENT_SRC);
	bool transientDetected = ff_transient_src & BIT_6;
	*z = ff_transient_src & BIT_5;
	*y = ff_transient_src & BIT_3;
	*x = ff_transient_src & BIT_1;
	if (negativeZ) *negativeZ = ff_transient_src & BIT_4;
	if (negativeY) *negativeY = ff_transient_src & BIT_2;
	if (negativeX) *negativeX = ff_transient_src & BIT_0;

	return transientDetected;
}

void MMA8452::setTransientTreshold(uint8_t threshold, uint8_t debounceCount, bool resetDebounceOnNoMotion)
{
	if (threshold > 127) threshold = 127;
	write(TRANSIENT_THS, (resetDebounceOnNoMotion << 7) | threshold);
	write(TRANSIENT_COUNT, debounceCount);
}

void MMA8452::enableSingleTapDetector(uint8_t axis, bool latch)
{
	uint8_t pulse_cfg = read(PULSE_CFG);

	bool x = (axis & MMA_X);
	bool y = (axis & MMA_Y);
	bool z = (axis & MMA_Z);

	// for configuring double tap settings
	singleTapEnabled = axis > 0;

	pulse_cfg &= ~BIT_6;
	if (latch) pulse_cfg |= BIT_6;

	// set single tap on z axis
	pulse_cfg &= ~BIT_4;
	if (z) pulse_cfg |= BIT_4;

	// set single tap on y axis
	pulse_cfg &= ~BIT_2;
	if (y) pulse_cfg |= BIT_2;

	// set single tap on x axis
	pulse_cfg &= ~BIT_0;
	if (x) pulse_cfg |= BIT_0;

	// write pulse (tap) config
	write(PULSE_CFG, pulse_cfg);

	if (!doubleTapEnabled) write(PULSE_LTCY, 0x4);
}

void MMA8452::enableDoubleTapDetector(uint8_t axis, uint8_t minDuration, uint8_t maxDuration, bool latch, bool abortOnQuickDoubleTap)
{
	uint8_t pulse_cfg = read(PULSE_CFG);

	bool x = (axis & MMA_X);
	bool y = (axis & MMA_Y);
	bool z = (axis & MMA_Z);

	// for configuring double tap settings
	singleTapEnabled = axis > 0;

	pulse_cfg &= ~BIT_6;
	if (latch) pulse_cfg |= BIT_6;

	// set double tap on z axis
	pulse_cfg &= ~BIT_5;
	if (z) pulse_cfg |= BIT_5;

	// set double tap on y axis
	pulse_cfg &= ~BIT_3;
	if (y) pulse_cfg |= BIT_3;

	// set double tap on x axis
	pulse_cfg &= ~BIT_1;
	if (x) pulse_cfg |= BIT_1;

	// write pulse (tap) config
	write(PULSE_CFG, pulse_cfg);

	write(PULSE_LTCY, minDuration);
	write(PULSE_WIND, maxDuration);
}

bool MMA8452::getTapDetails(bool *singleTap, bool *doubleTap, bool *x, bool *y, bool *z, bool *negativeX, bool *negativeY, bool *negativeZ)
{
	uint8_t pulse_src = read(PULSE_SRC);

	bool active = pulse_src & BIT_7;
	if (z) *z = pulse_src & BIT_6;
	if (y) *y = pulse_src & BIT_5;
	if (x) *x = pulse_src & BIT_4;

	if (singleTap) *singleTap = active && !(pulse_src & BIT_3);
	if (doubleTap) *doubleTap = active &&  (pulse_src & BIT_3);

	if (negativeZ) *negativeZ = pulse_src & BIT_2;
	if (negativeY) *negativeY = pulse_src & BIT_1;
	if (negativeX) *negativeX = pulse_src & BIT_0;

	return active;
}

void MMA8452::setTapThreshold(uint8_t x, uint8_t y, uint8_t z)
{
	if (x > 127) x = 127;
	if (y > 127) y = 127;
	if (z > 127) z = 127;
	write(PULSE_THSX, x);
	write(PULSE_THSY, y);
	write(PULSE_THSZ, z);
}

void MMA8452::setMaxTapDuration(uint8_t maxDuration)
{
	write(PULSE_TMLT, maxDuration);
}

void MMA8452::setAutoSleep(bool enabled, uint8_t time, mma8452_sleep_frequency_t sleepFrequencySampling, mma_power_mode_t sleepPowerMode)
{
	uint8_t ctrl_reg2 = read(CTRL_REG2);
	ctrl_reg2 &= ~(0x3 << 3);
	ctrl_reg2 |= (sleepPowerMode << 3);
	ctrl_reg2 &= ~(0x1 << 2);
	ctrl_reg2 |= (enabled << 2);
	write(CTRL_REG2, ctrl_reg2);

	if (enabled)
	{
		uint8_t ctrl_reg1 = read(CTRL_REG1);
		ctrl_reg1 &= ~(0x3 << 6);
		ctrl_reg1 |= (sleepFrequencySampling << 6);
		write(CTRL_REG1, ctrl_reg1);
		write(ASLP_COUNT, time);
	}
}

void MMA8452::setWakeOnInterrupt(bool transient, bool landscapePortraitChange, bool tap, bool freefall_motion)
{
	uint8_t ctrl_reg3 = read(CTRL_REG3);
	bool iPol = ctrl_reg3 & BIT_1;
	bool pp_od = ctrl_reg3 & BIT_0;
	ctrl_reg3 = (transient << 6) |
				(landscapePortraitChange << 5) |
				(tap << 4) |
				(freefall_motion << 3) |
				(iPol << 1) |
				(pp_od << 0);
	write(CTRL_REG3, ctrl_reg3);
}

void MMA8452::setDataRate(mma_datarate_t dataRate)
{
	uint8_t ctrl_reg1 = read(CTRL_REG1);
	ctrl_reg1 &= ~(0x7 << 3);
	ctrl_reg1 |= (dataRate << 3);
	write(CTRL_REG1, ctrl_reg1);
}

void MMA8452::setLowNoiseMode(bool enabled)
{
	uint8_t ctrl_reg1 = read(CTRL_REG1);
	ctrl_reg1 &= ~BIT_2;
	if (enabled) ctrl_reg1 |= BIT_2;
	write(CTRL_REG1, ctrl_reg1);
}

void MMA8452::set8BitMode(bool enabled)
{
	uint8_t ctrl_reg1 = read(CTRL_REG1);
	ctrl_reg1 &= ~BIT_1;
	if (enabled) ctrl_reg1 |= BIT_1;
	write(CTRL_REG1, ctrl_reg1);
}

void MMA8452::reset()
{
	uint8_t reg2 = read(CTRL_REG2);
	reg2 |= BIT_6;
	write(CTRL_REG2, reg2);
}

void MMA8452::setPowerMode(mma_power_mode_t powerMode)
{
	uint8_t ctrl_reg2 = read(CTRL_REG2);
	ctrl_reg2 &= ~0x3;
	ctrl_reg2 |= powerMode;
	write(CTRL_REG2, ctrl_reg2);
}

void MMA8452::configureInterrupts(bool activeHigh, bool openDrain)
{
	uint8_t ctrl_reg3 = read(CTRL_REG3);
	ctrl_reg3 &= ~0x3;
	ctrl_reg3 |= (activeHigh << 1) | openDrain;
	write(CTRL_REG3, ctrl_reg3);
}

void MMA8452::setInterruptsEnabled(uint8_t interruptMask)
{
	// clear bits that shouldn't be set, in case they were
	interruptMask &= ~(1 << 6);
	interruptMask &= ~(1 << 1);
	write(CTRL_REG4, interruptMask);
}

void MMA8452::setInterruptPins(bool autoSleepWake, bool transient, bool landscapePortraitChange, bool tap, bool freefall_motion, bool dataReady)
{
	uint8_t ctrl_reg5 = (autoSleepWake << 7) |
					 (transient << 5) |
					 (landscapePortraitChange << 4) |
					 (tap << 3) |
					 (freefall_motion << 2) |
					 (dataReady << 0);
	write(CTRL_REG5, ctrl_reg5);
}

void MMA8452::setOffsets(int8_t x, int8_t y, int8_t z)
{
	x = convertTo2sComplement(x);
	y = convertTo2sComplement(y);
	z = convertTo2sComplement(z);
	write(OFF_X, x);
	write(OFF_Y, y);
	write(OFF_Z, z);
}

void MMA8452::setActive(bool newActive)
{
	active = newActive;
	standby(!active);
}

// -- private --

void MMA8452::standby(bool standby)
{
	uint8_t reg_ctrl1 = read(CTRL_REG1);
	reg_ctrl1 &= ~BIT_0;
	reg_ctrl1 |= !standby;
#ifdef ARDUINO
	Wire.beginTransmission(MMA8452_ADDRESS);
	Wire.write(CTRL_REG1);
	Wire.write(reg_ctrl1);
	Wire.endTransmission();
#else
	i2c_start_wait((MMA8452_ADDRESS << 1));
	i2c_write(CTRL_REG1);
	i2c_write(reg_ctrl1);
	i2c_stop();
#endif
}

uint8_t MMA8452::read(uint8_t reg)
{
	uint8_t buf = 0;
	readMultiple(reg, &buf, 1);
	return buf;
}

void MMA8452::write(uint8_t reg, uint8_t value)
{
	// need to go to standby mode to modify registers
	// except CTRL_REG1[STANDBY] and CTRL_REG2[RST]
	// no need to go to standby if we're not active anyway
	bool needsStandby = (!(
							(reg == CTRL_REG1 && (value & 0x1)) || 
						  	(reg == CTRL_REG2 && ((value >> 6) & 0x1))
						) && active);
	needsStandby = true;
	if (needsStandby) standby(true);
#ifdef ARDUINO
	Wire.beginTransmission(MMA8452_ADDRESS);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
#else
	i2c_start_wait((MMA8452_ADDRESS << 1));
	i2c_write(reg);
	i2c_write(value);
	i2c_stop();
#endif
	if (needsStandby) standby(false);
}

void MMA8452::readMultiple(uint8_t reg, uint8_t *buffer, uint8_t numBytes)
{
#ifdef ARDUINO
	Wire.beginTransmission(MMA8452_ADDRESS);
	Wire.write(reg);
	Wire.endTransmission(false);
	Wire.requestFrom((uint8_t)MMA8452_ADDRESS, numBytes);
	while (Wire.available() < numBytes) {};
	while (numBytes--)
	{
		*buffer++ = Wire.read();
	}
#else
	i2c_start_wait((MMA8452_ADDRESS << 1));
	i2c_write(reg);
	i2c_rep_start((MMA8452_ADDRESS << 1) + 1);
	while (numBytes-- > 1)
	{
		*buffer++ = i2c_readAck();
	}
	*buffer++ = i2c_readNak();
	i2c_stop();
#endif
}

float MMA8452::convertGCounts(uint16_t data)
{
	int16_t gCount = (data >> 4); // data is 12bit

	// first bit is sign
	if (gCount > 0x7FF)
	{
		gCount = (0xFFF & ~gCount) + 1;	// data is 2’s complement, flip and add one
		gCount *= -1;					// we're negative so add sign
	}

	float countsPerG[3] = {
		1024.0f, 512.0f, 256.0f
	};
	float divider = countsPerG[range];

	// todo: would be nice to do this without floating point math
	float output = gCount;
	output /= divider;

	return output;
}

int8_t MMA8452::convertTo2sComplement(int8_t value)
{
	if (value >= 0) return value;
	value = (0xFF && ~value) + 1;
	return value;
}
