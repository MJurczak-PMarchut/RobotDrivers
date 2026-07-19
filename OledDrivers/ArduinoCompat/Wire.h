/*
 * Wire.h
 *
 * Thin binding, not a real Arduino TwoWire implementation. Adafruit_GrayOLED only
 * ever stores this pointer and forwards it into Adafruit_I2CDevice (see
 * Adafruit_GrayOLED.cpp: `_theWire = twi;` then `new Adafruit_I2CDevice(addr, _theWire)`)
 * - no Wire::begin()/beginTransmission()/write()/... calls happen anywhere in the
 * vendored sources, so none are implemented here. Actual I2C traffic goes through
 * this project's CommManager (see Adafruit_I2CDevice.cpp), not through this class.
 */
#ifndef ROBOTDRIVERS_OLEDDRIVERS_ARDUINOCOMPAT_WIRE_H_
#define ROBOTDRIVERS_OLEDDRIVERS_ARDUINOCOMPAT_WIRE_H_

#include "main.h"

class CommManager;

class TwoWire
{
public:
	TwoWire(CommManager *commManager, I2C_HandleTypeDef *hi2c) : _commManager(commManager), _hi2c(hi2c) {}
	CommManager *commManager() const { return _commManager; }
	I2C_HandleTypeDef *handle() const { return _hi2c; }

private:
	CommManager *_commManager;
	I2C_HandleTypeDef *_hi2c;
};

// Global default instance - required because Adafruit_GrayOLED's/Adafruit_SSD1327's
// I2C constructors default their `TwoWire *twi` parameter to `&Wire`.
extern TwoWire Wire;

#endif /* ROBOTDRIVERS_OLEDDRIVERS_ARDUINOCOMPAT_WIRE_H_ */
