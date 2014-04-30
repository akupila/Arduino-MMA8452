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

mma8452_mode_t MMA8452::getMode()
{
	byte sysmode = read(REG_SYSMOD);
	return (mma8452_mode_t)(sysmode & 0x3);
}

void MMA8452::getInterruptEvent(bool *wakeStateChanged, bool *movementOccurred, bool *landscapePortrait, bool *pulseEvent, bool *freefall, bool *dataReady)
{
	byte int_source = read(REG_INT_SOURCE);
	*wakeStateChanged = (int_source >> 7) & 0x1;
	*movementOccurred = (int_source >> 5) & 0x1;
	*landscapePortrait = (int_source >> 4) & 0x1;
	*pulseEvent = (int_source >> 3) & 0x1;
	*freefall = (int_source >> 2) & 0x1;
	*dataReady = (int_source >> 0) & 0x1;
}

void MMA8452::setRange(mma8452_range_t newRange)
{
	setActive(false);
	byte reg = read(REG_XYZ_DATA_CFG);
	reg &= ~0x3;
	reg |= newRange;
	write(REG_XYZ_DATA_CFG, reg);
	setActive(true);
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

// -- private --

void MMA8452::setActive(bool active)
{
	byte reg = read(REG_CTRL_REG1);
	reg &= ~1;
	reg |= active;
	write(REG_CTRL_REG1, reg);
}

byte MMA8452::read(byte reg)
{
	byte *buf;
	readMultiple(reg, buf, 1);
	return *buf;
}

void MMA8452::write(byte reg, byte value)
{
	Wire.beginTransmission(MMA8452_ADDRESS);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
}

void MMA8452::readMultiple(byte reg, byte *buffer, uint8_t numBytes)
{
	Wire.beginTransmission(MMA8452_ADDRESS);
	Wire.write(reg);
	Wire.endTransmission(false);
	Wire.requestFrom((uint8_t)MMA8452_ADDRESS, numBytes);
	while (Wire.available() < numBytes) {};
	while (numBytes--)
	{
		*buffer++ = Wire.read();
	}
}