/*
 * Adafruit_SPIDevice.h
 *
 * Not the real Adafruit_BusIO library - a minimal, from-scratch replacement
 * implementing only what Adafruit_GrayOLED.cpp calls (both constructor overloads,
 * begin(), the 2-arg write()), backed by this project's CommManager instead of a
 * real Arduino SPI bus. Mirrors RobotDrivers/LSM6DSO/lsm6dso.cpp's SPI usage
 * (CS pin fields on MessageInfoTypeDef, semaphore-blocking on a completion callback).
 *
 * Only the hardware-SPI constructor is functional. This project has no known use of
 * Adafruit_GrayOLED's bit-banged soft-SPI mode (no GPIO wiring for it exists in
 * CubeMX), so that constructor just records its pins and begin()/write() report
 * failure - add a bit-bang implementation here if a soft-SPI display is ever wired up.
 */
#ifndef ROBOTDRIVERS_OLEDDRIVERS_ARDUINOCOMPAT_ADAFRUIT_SPIDEVICE_H_
#define ROBOTDRIVERS_OLEDDRIVERS_ARDUINOCOMPAT_ADAFRUIT_SPIDEVICE_H_

#include <cstddef>
#include <cstdint>

#include "SPI.h"

class CommManager;

class Adafruit_SPIDevice
{
public:
	// Bit-banged soft SPI - not implemented, see class comment.
	Adafruit_SPIDevice(int8_t cspin, int8_t clkpin, int8_t misopin, int8_t mosipin,
					   uint32_t freq = 1000000);
	// Hardware SPI - functional, routed through CommManager.
	Adafruit_SPIDevice(int8_t cspin, uint32_t freq, SPIBitOrder dataOrder,
					   SPIDataMode dataMode, SPIClass *theSPI);
	~Adafruit_SPIDevice();

	bool begin(void);
	bool write(const uint8_t *buffer, size_t len);

private:
	bool _isHardware;
	int8_t _csPin;
	CommManager *_commManager;
	SPI_HandleTypeDef *_hspi;

	static constexpr size_t kMaxBufferSize = 512;
	uint8_t _txBuf[kMaxBufferSize];
};

#endif /* ROBOTDRIVERS_OLEDDRIVERS_ARDUINOCOMPAT_ADAFRUIT_SPIDEVICE_H_ */
