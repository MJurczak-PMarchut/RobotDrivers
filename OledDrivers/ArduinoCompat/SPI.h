/*
 * SPI.h
 *
 * Thin binding, same rationale as TwoWire (see Wire.h): Adafruit_GrayOLED only ever
 * passes an SPIClass* through to Adafruit_SPIDevice's constructor, never calls
 * transfer methods on it directly, so none are implemented here.
 *
 * Deliberately named SPIClass, not SPI, and deliberately has no global default
 * instance: RobotDrivers/CommManager/inc/CommManager.hpp does `#define SPI
 * SPI_HandleTypeDef` project-wide, so a global object literally named `SPI` would
 * silently get macro-replaced into `SPI_HandleTypeDef` in any translation unit that
 * includes both this header and CommManager.hpp. Unlike Wire, nothing in the vendored
 * Adafruit sources defaults an argument to `&SPI`, so no global instance is needed.
 */
#ifndef ROBOTDRIVERS_OLEDDRIVERS_ARDUINOCOMPAT_SPI_H_
#define ROBOTDRIVERS_OLEDDRIVERS_ARDUINOCOMPAT_SPI_H_

#include "main.h"

class CommManager;

class SPIClass
{
public:
	SPIClass(CommManager *commManager, SPI_HandleTypeDef *hspi) : _commManager(commManager), _hspi(hspi) {}
	CommManager *commManager() const { return _commManager; }
	SPI_HandleTypeDef *handle() const { return _hspi; }

private:
	CommManager *_commManager;
	SPI_HandleTypeDef *_hspi;
};

// Literal values used by Adafruit_GrayOLED.cpp's hw-SPI Adafruit_SPIDevice
// constructor call; only MSB-first/mode-0 are ever referenced in these libraries.
enum SPIBitOrder
{
	SPI_BITORDER_MSBFIRST = 0,
	SPI_BITORDER_LSBFIRST = 1
};

enum SPIDataMode
{
	SPI_MODE0 = 0,
	SPI_MODE1 = 1,
	SPI_MODE2 = 2,
	SPI_MODE3 = 3
};

#endif /* ROBOTDRIVERS_OLEDDRIVERS_ARDUINOCOMPAT_SPI_H_ */
