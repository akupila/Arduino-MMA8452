#include "MMA8452.h"

bool MMA8452::init()
{
	// Check who-am-i register
	byte reg = read(REG_WHO_AM_I);
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
	byte data[6];
	readMultiple(REG_OUT_X_MSB, data, 6);
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
	byte sysmode = read(REG_SYSMOD);
	return (mma8452_mode_t)(sysmode & 0x3);
}

void MMA8452::getInterruptEvent(bool *wakeStateChanged, bool *movementOccurred, bool *landscapePortrait, bool *tap, bool *freefall, bool *dataReady)
{
	byte int_source = read(REG_INT_SOURCE);
	if (wakeStateChanged) *wakeStateChanged = (int_source >> 7) & 0x1;
	if (movementOccurred) *movementOccurred = (int_source >> 5) & 0x1;
	if (landscapePortrait) *landscapePortrait = (int_source >> 4) & 0x1;
	if (tap) *tap = (int_source >> 3) & 0x1;
	if (freefall) *freefall = (int_source >> 2) & 0x1;
	if (dataReady) *dataReady = (int_source >> 0) & 0x1;
}

void MMA8452::setRange(mma8452_range_t newRange)
{
	byte reg = read(REG_XYZ_DATA_CFG);
	reg &= ~0x3;
	reg |= newRange;
	write(REG_XYZ_DATA_CFG, reg);
	range = newRange;
}

mma8452_range_t MMA8452::getRange()
{
	byte reg = read(REG_XYZ_DATA_CFG);
	return (mma8452_range_t)(reg & 0x3);
}

void MMA8452::setHighPassFilter(bool enabled, mma8452_highpass_mode_t mode)
{
	byte reg = read(REG_XYZ_DATA_CFG);
	reg &= ~(1 << 4);
	reg |= (enabled << 4);
	write(REG_XYZ_DATA_CFG, reg);

	reg = read(REG_HP_FILTER_CUTOFF);
	reg &= ~(0x03);
	reg |= mode;
	write(REG_HP_FILTER_CUTOFF, reg);

}

bool MMA8452::getHighPassFilter(mma8452_highpass_mode_t *mode)
{
	byte reg = read(REG_XYZ_DATA_CFG);
	if (mode != NULL)
	{
		reg = read(REG_HP_FILTER_CUTOFF);
		*mode = (mma8452_highpass_mode_t)(reg & 0x3);
	}
	return (reg >> 4) & 1;
}

void MMA8452::getPortaitLandscapeStatus(bool *orientationChanged, bool *zTiltLockoutDetected, mma8452_orientation_t *orientation, bool *back)
{
	byte reg = read(REG_PL_STATUS);
	*orientationChanged = (reg >> 7) & 0x1;
	*zTiltLockoutDetected = (reg >> 6) & 0x1;
	*orientation = (mma8452_orientation_t)((reg >> 1) & 0x3);
	*back = (reg >> 0) & 0x1;
}

void MMA8452::configureLandscapePortraitDetection(bool enableDetection, uint8_t debounceCount, bool debounceTimeout)
{
	byte reg = read(REG_PL_CFG);
	reg &= ~(1 << 7);
	reg |= (debounceTimeout << 7);
	reg |= (enableDetection << 6);
	write(REG_PL_CFG, reg);

	write(REG_PL_COUNT, debounceCount);
}

void MMA8452::getLandscapePortraitConfig(bool *enabled, uint8_t *debounceCount, bool *debounceTimeout)
{
	byte reg = read(REG_PL_CFG);
	*debounceTimeout = (reg >> 7) & 0x1;
	*enabled = (reg >> 6) & 0x1;
	*debounceCount = read(REG_PL_COUNT);
}

void MMA8452::setMotionDetectionMode(mma8452_motion_type_t motion, bool xAxis, bool yAxis, bool zAxis, bool latchMotion)
{
	byte reg =  (latchMotion << 7) |
				(motion << 6) |
				(zAxis << 5) |
				(yAxis << 4) |
				(xAxis << 3);
	write(REG_FF_MT_CFG, reg);
}

void MMA8452::setMotionTreshold(uint8_t threshold, uint8_t debounceCount, bool resetDebounceOnNoMotion)
{
	if (threshold > 127) threshold = 127;
	write(REG_FF_MT_THS, (resetDebounceOnNoMotion << 7) | threshold);
	write(REG_FF_MT_COUNT, debounceCount);
}

bool MMA8452::motionDetected(bool *x, bool *y, bool *z, bool *negativeX, bool *negativeY, bool *negativeZ)
{
	byte ff_mt_src = read(REG_FF_MT_SRC);
	bool motionDetected = (ff_mt_src >> 7) & 0x1;
	*z = (ff_mt_src >> 5) & 0x1;
	*y = (ff_mt_src >> 3) & 0x1;
	*x = (ff_mt_src >> 1) & 0x1;
	if (negativeZ) *negativeZ = (ff_mt_src >> 4) & 0x1;
	if (negativeY) *negativeY = (ff_mt_src >> 2) & 0x1;
	if (negativeX) *negativeX = (ff_mt_src >> 0) & 0x1;

	return motionDetected;
}

void MMA8452::setTransientDetection(bool xAxis, bool yAxis, bool zAxis, bool latchMotion, bool bypassHighPass)
{
	byte reg =  (latchMotion << 4) |
				(zAxis << 3) |
				(yAxis << 2) |
				(xAxis << 1) |
				bypassHighPass;

	write(REG_TRANSIENT_CFG, reg);
}

bool MMA8452::transientDetected(bool *x, bool *y, bool *z, bool *negativeX, bool *negativeY, bool *negativeZ)
{
	byte ff_mt_src = read(REG_TRANSIENT_SRC);
	bool transientDetected = (ff_mt_src >> 6) & 0x1;
	*z = (ff_mt_src >> 5) & 0x1;
	*y = (ff_mt_src >> 3) & 0x1;
	*x = (ff_mt_src >> 1) & 0x1;
	if (negativeZ) *negativeZ = (ff_mt_src >> 4) & 0x1;
	if (negativeY) *negativeY = (ff_mt_src >> 2) & 0x1;
	if (negativeX) *negativeX = (ff_mt_src >> 0) & 0x1;

	return transientDetected;
}

void MMA8452::setTransientTreshold(uint8_t threshold, uint8_t debounceCount, bool resetDebounceOnNoMotion)
{
	if (threshold > 127) threshold = 127;
	write(REG_TRANSIENT_THS, (resetDebounceOnNoMotion << 7) | threshold);
	write(REG_TRANSIENT_COUNT, debounceCount);
}

void MMA8452::enableSingleTapDetector(bool xAxis, bool yAxis, bool zAxis)
{
	byte pulse_cfg = read(REG_PULSE_CFG);

	// for configuring double tap settings
	singleTapEnabled = xAxis || yAxis || zAxis;

	pulse_cfg &= ~(1 << 6);
	pulse_cfg |= (singleTapEnabled || doubleTapEnabled);
	pulse_cfg &= ~(1 << 4);
	pulse_cfg |= (zAxis << 4);
	pulse_cfg &= ~(1 << 2);
	pulse_cfg |= (yAxis << 2);
	pulse_cfg &= ~(1 << 0);
	pulse_cfg |= (yAxis << 0);

	write(REG_PULSE_CFG, pulse_cfg);
}

void MMA8452::enableDoubleTapDetector(bool xAxis, bool yAxis, bool zAxis, uint8_t minDuration, uint8_t maxDuration, bool abortOnQuickDoubleTap)
{
	byte pulse_cfg = read(REG_PULSE_CFG);

	// for configuring double tap settings
	doubleTapEnabled = xAxis || yAxis || zAxis;

	pulse_cfg &= ~(1 << 7);
	pulse_cfg |= (abortOnQuickDoubleTap << 7);
	pulse_cfg &= ~(1 << 6);
	pulse_cfg |= (singleTapEnabled || doubleTapEnabled);
	pulse_cfg &= ~(1 << 5);
	pulse_cfg |= (zAxis << 5);
	pulse_cfg &= ~(1 << 3);
	pulse_cfg |= (yAxis << 3);
	pulse_cfg &= ~(1 << 1);
	pulse_cfg |= (yAxis << 1);

	write(REG_PULSE_CFG, pulse_cfg);

	write(REG_PULSE_LTCY, minDuration);
	write(REG_PULSE_WIND, maxDuration);
}

bool MMA8452::tapDetected(bool *doubleTap, bool *x, bool *y, bool *z, bool *negativeX, bool *negativeY, bool *negativeZ)
{
	byte pulse_src = read(REG_PULSE_SRC);
	bool tap = (pulse_src >> 7) & 0x1;
	*z = (pulse_src >> 6) & 0x1;
	*y = (pulse_src >> 5) & 0x1;
	*x = (pulse_src >> 4) & 0x1;
	*doubleTap = (pulse_src >> 3) & 0x1;
	if (negativeZ) *negativeZ = (pulse_src >> 2) & 0x1;
	if (negativeY) *negativeY = (pulse_src >> 1) & 0x1;
	if (negativeX) *negativeX = (pulse_src >> 0) & 0x1;

	return tap;
}

void MMA8452::setTapThreshold(uint8_t x, uint8_t y, uint8_t z)
{
	if (x > 127) x = 127;
	if (y > 127) y = 127;
	if (z > 127) z = 127;
	write(REG_PULSE_THSX, x);
	write(REG_PULSE_THSY, y);
	write(REG_PULSE_THSZ, z);
}

void MMA8452::setMaxTapDuration(uint8_t maxDuration)
{
	write(REG_PULSE_TMLT, maxDuration);
}

void MMA8452::setAutoSleep(bool enabled, uint8_t time, mma8452_sleep_frequency_t sleepFrequencySampling, mma_power_mode_t sleepPowerMode)
{
	byte ctrl_reg2 = read(REG_CTRL_REG2);
	ctrl_reg2 &= ~(0x3 << 3);
	ctrl_reg2 |= (sleepPowerMode << 3);
	ctrl_reg2 &= ~(0x1 << 2);
	ctrl_reg2 |= (enabled << 2);
	write(REG_CTRL_REG2, ctrl_reg2);

	if (enabled)
	{
		byte ctrl_reg1 = read(REG_CTRL_REG1);
		ctrl_reg1 &= ~(0x3 << 6);
		ctrl_reg1 |= (sleepFrequencySampling << 6);
		write(REG_CTRL_REG1, ctrl_reg1);
		write(REG_ASLP_COUNT, time);
	}
}

void MMA8452::setWakeOnInterrupt(bool transient, bool landscapePortraitChange, bool tap, bool freefall_motion)
{
	byte ctrl_reg3 = read(REG_CTRL_REG3);
	bool iPol = (ctrl_reg3 >> 1) & 0x1;
	bool pp_od = (ctrl_reg3 >> 0) & 0x1;
	ctrl_reg3 = (transient << 6) |
				(landscapePortraitChange << 5) |
				(tap << 4) |
				(freefall_motion << 3) |
				(iPol << 1) |
				(pp_od << 0);
	write(REG_CTRL_REG3, ctrl_reg3);
}

void MMA8452::setDataRate(mma_datarate_t dataRate)
{
	byte ctrl_reg1 = read(REG_CTRL_REG1);
	ctrl_reg1 &= ~(0x7 << 3);
	ctrl_reg1 |= (dataRate << 3);
	write(REG_CTRL_REG1, ctrl_reg1);
}

void MMA8452::setLowNoiseMode(bool enabled)
{
	byte ctrl_reg1 = read(REG_CTRL_REG1);
	ctrl_reg1 &= ~(0x1 << 2);
	ctrl_reg1 |= (enabled << 2);
	write(REG_CTRL_REG1, ctrl_reg1);
}

void MMA8452::set8BitMode(bool enabled)
{
	byte ctrl_reg1 = read(REG_CTRL_REG1);
	ctrl_reg1 &= ~(0x1 << 1);
	ctrl_reg1 |= (enabled << 1);
	write(REG_CTRL_REG1, ctrl_reg1);
}

void MMA8452::reset()
{
	byte reg2 = read(REG_CTRL_REG2);
	reg2 |= (1 << 6);
	write(REG_CTRL_REG2, reg2);
	// write(REG_CTRL_REG2, ~(1 << 6));
}

void MMA8452::setPowerMode(mma_power_mode_t powerMode)
{
	byte ctrl_reg2 = read(REG_CTRL_REG2);
	ctrl_reg2 &= ~0x3;
	ctrl_reg2 |= powerMode;
	write(REG_CTRL_REG2, ctrl_reg2);
}

void MMA8452::configureInterrrupts(bool activeHigh, bool openDrain)
{
	byte ctrl_reg3 = read(REG_CTRL_REG3);
	ctrl_reg3 &= ~0x3;
	ctrl_reg3 |= (activeHigh << 1) | openDrain;
	write(REG_CTRL_REG3, ctrl_reg3);
}

void MMA8452::setInterruptsEnabled(uint8_t interruptMask)
{
	// clear bits that shouldn't be set, in case they were
	interruptMask &= ~(1 << 6);
	interruptMask &= ~(1 << 1);
	write(REG_CTRL_REG4, interruptMask);
}

void MMA8452::setInterruptPins(bool autoSleepWake, bool transient, bool landscapePortraitChange, bool tap, bool freefall_motion, bool dataReady)
{
	byte ctrl_reg5 = (autoSleepWake << 7) |
					 (transient << 5) |
					 (landscapePortraitChange << 4) |
					 (tap << 3) |
					 (freefall_motion << 2) |
					 (dataReady << 0);
	write(REG_CTRL_REG5, ctrl_reg5);
}

void MMA8452::setOffsets(int8_t x, int8_t y, int8_t z)
{
	x = convertTo2sComplement(x);
	y = convertTo2sComplement(y);
	z = convertTo2sComplement(z);
	write(REG_OFF_X, x);
	write(REG_OFF_Y, y);
	write(REG_OFF_Z, z);
}

void MMA8452::setActive(bool newActive)
{
	active = newActive;
	standby(!active);
}

// -- private --

void MMA8452::standby(bool standby)
{
	byte reg_ctrl1 = read(REG_CTRL_REG1);
	reg_ctrl1 &= ~0x1;
	reg_ctrl1 |= !standby;
	Wire.beginTransmission(MMA8452_ADDRESS);
	Wire.write(REG_CTRL_REG1);
	Wire.write(reg_ctrl1);
	Wire.endTransmission();
}

byte MMA8452::read(byte reg)
{
	byte *buf;
	readMultiple(reg, buf, 1);
	return *buf;
}

void MMA8452::write(byte reg, byte value)
{
	// need to go to standby mode to modify registers
	// except CTRL_REG1[STANDBY] and CTRL_REG2[RST]
	// no need to go to standby if we're not active anyway
	bool needsStandby = (!(
							(reg == REG_CTRL_REG1 && (value & 0x1)) || 
						  	(reg == REG_CTRL_REG2 && ((value >> 6) & 0x1))
						) && active);
	needsStandby = true;
	if (needsStandby) standby(true);
	Wire.beginTransmission(MMA8452_ADDRESS);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
	if (needsStandby) standby(false);
}

void MMA8452::readMultiple(byte reg, byte *buffer, uint8_t numBytes)
{
	Wire.beginTransmission(MMA8452_ADDRESS);
	Wire.write(reg);
	Wire.endTransmission(false);
	Wire.requestFrom((uint8_t)MMA8452_ADDRESS, numBytes);
	// while (Wire.available() < numBytes) {};
	while (numBytes--)
	{
		*buffer++ = Wire.read();
	}
}

float MMA8452::convertGCounts(uint16_t data)
{
	int16_t gCount = (data >> 4); // data is 12bit

	// first bit is sign
	if (gCount > 0x7FF)
	{
		gCount = (0xFFF & ~gCount) + 1;	// data is 2’s complement, flip byte and add one
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
