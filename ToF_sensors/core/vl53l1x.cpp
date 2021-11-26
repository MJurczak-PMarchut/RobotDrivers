/*
 * vl53l1x.cpp
 *
 *  Created on: Nov 16, 2021
 *      Author: paulina
 */

#include "vl53l1x.hpp"

VL53L1X::VL53L1X(I2C_HandleTypeDef *hi2c, CommManager *CommunicationManager)
{
	this->_CommunicationManager = CommunicationManager;
	this->_hi2c= hi2c;
	for(uint8_t i = 0; i < MAX_TOF_NUMBER; i++) this->_Devices[i].Active = false;
#ifdef TOF0
	this->_Devices[TOF0].Address = TOF0_Addr;
	this->_Devices[TOF0].GPIO_GPIOx = TOF_GPIO_6_GPIO_Port;
	this->_Devices[TOF0].GPIO_PIN = TOF_GPIO_6_Pin;
	this->_Devices[TOF0].XSHUT_GPIOx = XSHUT_6_GPIO_Port;
	this->_Devices[TOF0].XSHUT_PIN = XSHUT_6_Pin;
#else
	this->_Devices[TOF0].Address = TOF_DEFAULT_ADDRESS;
#endif
#ifdef TOF1
	this->_Devices[TOF1].Address = TOF1_Addr;
	this->_Devices[TOF1].GPIO_GPIOx = TOF_GPIO_1_GPIO_Port;
	this->_Devices[TOF1].GPIO_PIN = TOF_GPIO_1_Pin;
	this->_Devices[TOF1].XSHUT_GPIOx = XSHUT_1_GPIO_Port;
	this->_Devices[TOF1].XSHUT_PIN = XSHUT_1_Pin;
#else
	this->_Devices[TOF1].Address = TOF_DEFAULT_ADDRESS;
#endif
#ifdef TOF2
	this->_Devices[TOF2].Address = TOF2_Addr;
	this->_Devices[TOF2].GPIO_GPIOx = TOF_GPIO_2_GPIO_Port;
	this->_Devices[TOF2].GPIO_PIN = TOF_GPIO_2_Pin;
	this->_Devices[TOF2].XSHUT_GPIOx = XSHUT_2_GPIO_Port;
	this->_Devices[TOF2].XSHUT_PIN = XSHUT_2_Pin;
#else
	this->_Devices[TOF2].Address = TOF_DEFAULT_ADDRESS;
#endif
#ifdef TOF3
	this->_Devices[TOF3].Address = TOF3_Addr;
	this->_Devices[TOF3].GPIO_GPIOx = TOF_GPIO_3_GPIO_Port;
	this->_Devices[TOF3].GPIO_PIN = TOF_GPIO_3_Pin;
	this->_Devices[TOF3].XSHUT_GPIOx = XSHUT_3_GPIO_Port;
	this->_Devices[TOF3].XSHUT_PIN = XSHUT_3_Pin;
#else
	this->_Devices[TOF3].Address = TOF_DEFAULT_ADDRESS;
#endif
#ifdef TOF4
	this->_Devices[TOF4].Address = TOF4_Addr;
	this->_Devices[TOF4].GPIO_GPIOx = TOF_GPIO_4_GPIO_Port;
	this->_Devices[TOF4].GPIO_PIN = TOF_GPIO_4_Pin;
	this->_Devices[TOF4].XSHUT_GPIOx = XSHUT_4_GPIO_Port;
	this->_Devices[TOF4].XSHUT_PIN = XSHUT_4_Pin;
#else
	this->_Devices[TOF4].Address = TOF_DEFAULT_ADDRESS;
#endif
#ifdef TOF5
	this->_Devices[TOF5].Address = TOF5_Addr;
//	this->_Devices[TOF5].GPIO_GPIOx = TOF_GPIO_5_GPIO_Port;
//	this->_Devices[TOF5].GPIO_PIN = TOF_GPIO_5_Pin;
	this->_Devices[TOF5].XSHUT_GPIOx = XSHUT_5_GPIO_Port;
	this->_Devices[TOF5].XSHUT_PIN = XSHUT_5_Pin;
#else
	this->_Devices[TOF5].Address = TOF_DEFAULT_ADDRESS;
#endif
}

VL53L1X_ERROR VL53L1X::InitSensor(uint8_t sensor)
{
	VL53L1X_ERROR status = 0;
	if(this->_Devices[sensor].Address == TOF_DEFAULT_ADDRESS)
	{
		status = 0xFE;
		this->_Devices[sensor].Active = false;
	}
	else status = this->SetSensorAddress(sensor);
	status |= VL53L1X_SensorInit(this->_Devices[sensor].Address);
	if(status == 0)
	{
		this->_Devices[sensor].Active = true;
	}
	return status;
}

VL53L1X_ERROR VL53L1X::InitAllSensors(void)
{
	VL53L1X_ERROR status = 0;
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
	return status;
}

VL53L1X_ERROR VL53L1X::SetSensorAddress(uint8_t sensor)
{
	VL53L1X_ERROR status = 0;

	status = VL53L1X_SetI2CAddress(TOF_DEFAULT_ADDRESS, this->_Devices[sensor].Address >> 1);
	return status;
}

VL53L1X_ERROR VL53L1X::SetSensorPins(uint8_t sensor, uint16_t GPIO_PIN, GPIO_TypeDef *GPIO_GPIOx, uint16_t XSHUT_PIN, GPIO_TypeDef *XSHUT_GPIOx)
{
	this->_Devices[sensor].GPIO_GPIOx = GPIO_GPIOx;
	this->_Devices[sensor].GPIO_PIN = GPIO_PIN;
	this->_Devices[sensor].XSHUT_GPIOx = XSHUT_GPIOx;
	this->_Devices[sensor].XSHUT_PIN = XSHUT_PIN;
	return 0;
}
