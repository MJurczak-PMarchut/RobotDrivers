/*
 * Adafruit_I2CDevice.h
 *
 * Not the real Adafruit_BusIO library - a minimal, from-scratch replacement
 * implementing only the methods Adafruit_GrayOLED.cpp/Adafruit_SSD1327.cpp actually
 * call (constructor, begin(), the two write() forms, setSpeed(), maxBufferSize()),
 * backed directly by this project's CommManager instead of a real Arduino Wire bus.
 *
 * Uses the same async-push-then-block-on-semaphore pattern as
 * RobotDrivers/LSM6DSO/lsm6dso.cpp's platform_write() (SPI), adapted for I2C.
 */
#ifndef ROBOTDRIVERS_OLEDDRIVERS_ARDUINOCOMPAT_ADAFRUIT_I2CDEVICE_H_
#define ROBOTDRIVERS_OLEDDRIVERS_ARDUINOCOMPAT_ADAFRUIT_I2CDEVICE_H_

#include <cstddef>
#include <cstdint>

#include "Wire.h"

class CommManager;

class Adafruit_I2CDevice
{
public:
	Adafruit_I2CDevice(uint8_t addr, TwoWire *theWire = &Wire);
	~Adafruit_I2CDevice();

	bool begin(bool addr_detect = true);
	uint8_t address(void) const { return _addr; }

	// prefix_buffer/prefix_len is used by Adafruit_GrayOLED for the SSD130x-style
	// I2C "control byte" (0x00=command, 0x40=data) that must precede the payload in
	// the same transaction; here it's just concatenated into one contiguous transfer.
	bool write(const uint8_t *buffer, size_t len, bool stop = true,
			   const uint8_t *prefix_buffer = nullptr, size_t prefix_len = 0);

	bool setSpeed(uint32_t desiredclk);
	size_t maxBufferSize() const { return kMaxBufferSize; }

private:
	static constexpr size_t kMaxBufferSize = 512;

	uint8_t _addr;
	CommManager *_commManager;
	I2C_HandleTypeDef *_hi2c;
	uint8_t _txBuf[kMaxBufferSize];
};

#endif /* ROBOTDRIVERS_OLEDDRIVERS_ARDUINOCOMPAT_ADAFRUIT_I2CDEVICE_H_ */
