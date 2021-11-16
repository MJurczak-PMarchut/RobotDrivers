/*
 * vl53l1x.cpp
 *
 *  Created on: Nov 16, 2021
 *      Author: paulina
 */

#include "vl53l1x.h"

VL53L1X::VL53L1X(I2C_HandleTypeDef *hi2c)
{
	_MessageInfo.uCommInt = hi2c;
}

VL53L1X_ERROR VL53L1X::InitSensor(uint8_t sensor)
{
	VL53L1X_ERROR status = 0;
	status = VL53L1X_SetI2CAddress(TOF_DEFAULT_ADDRESS, sensor);
	status |= VL53L1X_SensorInit(sensor);
	return status;
}

VL53L1X_ERROR VL53L1X::InitAllSensors(void)
{
	VL53L1X_ERROR status = 0;
#if define(TOF0) or define(TOF1) or define(TOF2) or define(TOF3) or define(TOF4) or define(TOF5)
#ifdef TOF0
	status |= (this->InitSensor(TOF0));
#endif
#ifdef TOF1
	status |= (this->InitSensor(TOF1)) << 1;
#endif
#ifdef TOF2
	status |= (this->InitSensor(TOF2)) << 2;
#endif
#ifdef TOF3
	status |= (this->InitSensor(TOF3)) << 3;
#endif
#ifdef TOF4
	status |= (this->InitSensor(TOF4)) << 4;
#endif
#ifdef TOF5
	status |= (this->InitSensor(TOF5)) << 5;
#endif
#else
	status = 0xff;
#endif
	return status;
}
