/*
 * vl53l1x.cpp
 *
 *  Created on: Nov 16, 2021
 *      Author: Paulina
 */

#include "vl53l1x.hpp"
#include "RobotSpecificDefines.hpp"
#include "vl53l1_platform.hpp"

const static uint8_t __ToFAddr[] = { 0x54, 0x56, 0x58, 0x60, 0x62, 0x64 };

const static uint16_t __ToFX_SHUT_Pin[] = XSHUT_PINS;
const static uint16_t __ToFX_GPIO_Pin[] = TOFx_GPIO_PINS;
static GPIO_TypeDef *__ToFX_SHUT_Port[] = TOFx_XSHUT_PORTS;

VL53L1X::VL53L1X(e_ToF_Position position, CommManager *comm, I2C_HandleTypeDef *hi2c1) :
		ToF_Sensor(vl53l1, position, comm), __hi2c1{hi2c1}, __wait_until_tick{0}, __Status {TOF_INIT_NOT_DONE}, __data_count{0} {

	this->__sensor_index = __no_of_sensors - 1;
	HAL_GPIO_WritePin(__ToFX_SHUT_Port[this->__sensor_index],
			__ToFX_SHUT_Pin[this->__sensor_index], GPIO_PIN_RESET);
	last_update_tick = HAL_GetTick();
}

HAL_StatusTypeDef VL53L1X::SensorInit(void){

	uint8_t ret;

//	ret = VL53L1X_WrMulti(this->__address, );
//	ret |= vl53l5cx_set_ranging_mode(&this->__sensor_conf, VL53L5CX_RANGING_MODE_CONTINUOUS);
//	ret |= vl53l5cx_set_ranging_frequency_hz(&this->__sensor_conf, 60);
//	ret |= vl53l5cx_start_ranging(&this->__sensor_conf);
//	this->__Status = (ret)?TOF_STATE_ERROR:TOF_STATE_OK;
//	this->last_update_tick = HAL_GetTick();
//	return (ret)?HAL_OK:HAL_ERROR;
}

HAL_StatusTypeDef VL53L1X::SetI2CAddress() {
	HAL_GPIO_WritePin(__ToFX_SHUT_Port[this->__sensor_index],
			__ToFX_SHUT_Pin[this->__sensor_index], GPIO_PIN_SET);
	uint8_t ret = 0;
	ret = VL53L1X_WrByte(this->__address, VL53L1_I2C_SLAVE__DEVICE_ADDRESS, __ToFAddr[this->__sensor_index]);
	this->__address = __ToFAddr[this->__sensor_index];
	return (ret == 0) ? HAL_OK : HAL_ERROR;
}

ToF_Status_t VL53L1X::CheckSensorStatus(void) {
	switch (this->__Status) {
	case TOF_STATE_INIT_WAIT:

		break;
	case TOF_STATE_OK:
	case TOF_STATE_DATA_RDY:
		if(HAL_GetTick() - last_update_tick > 100)//no update since 100ms;
		{
			if(GetRangingData() == HAL_ERROR)
			{
				this->__Status = TOF_STATE_ERROR;
			}
		}
		break;
	default:
		break;
	}
	return this->__Status;
}
//
//VL53L1X::VL53L1X(I2C_HandleTypeDef *hi2c, CommManager *CommunicationManager)
//{
//	this->_CommunicationManager = CommunicationManager;
//	this->_hi2c= hi2c;
//	for(uint8_t i = 0; i < MAX_TOF_NUMBER; i++) this->_Devices[i].Active = false;
//#ifdef TOF0
//	this->_Devices[TOF0].Address = TOF0_Addr;
//	this->SetSensorPins(TOF0, TOF_GPIO_6_Pin, TOF_GPIO_6_GPIO_Port, XSHUT_6_Pin, XSHUT_6_GPIO_Port);
//#else
//	this->_Devices[TOF0].Address = TOF_DEFAULT_ADDRESS;
//#endif
//#ifdef TOF1
//	this->_Devices[TOF1].Address = TOF1_Addr;
//	this->SetSensorPins(TOF1, TOF_GPIO_1_Pin, TOF_GPIO_1_GPIO_Port, XSHUT_1_Pin, XSHUT_1_GPIO_Port);
//#else
////	this->_Devices[TOF1].Address = TOF_DEFAULT_ADDRESS;
//#endif
//#ifdef TOF2
//	this->_Devices[TOF2].Address = TOF2_Addr;
//	this->SetSensorPins(TOF2, TOF_GPIO_2_Pin, TOF_GPIO_2_GPIO_Port, XSHUT_2_Pin, XSHUT_2_GPIO_Port);
//#else
////	this->_Devices[TOF2].Address = TOF_DEFAULT_ADDRESS;
//#endif
//#ifdef TOF3
//	this->_Devices[TOF3].Address = TOF3_Addr;
//	this->SetSensorPins(TOF3, TOF_GPIO_3_Pin, TOF_GPIO_3_GPIO_Port, XSHUT_3_Pin, XSHUT_3_GPIO_Port);
//#else
////	this->_Devices[TOF3].Address = TOF_DEFAULT_ADDRESS;
//#endif
//#ifdef TOF4
//	this->_Devices[TOF4].Address = TOF4_Addr;
//	this->SetSensorPins(TOF4, TOF_GPIO_4_Pin, TOF_GPIO_4_GPIO_Port, XSHUT_4_Pin, XSHUT_4_GPIO_Port);
//#else
////	this->_Devices[TOF4].Address = TOF_DEFAULT_ADDRESS;
//#endif
//#ifdef TOF5
//	this->_Devices[TOF5].Address = TOF5_Addr;
//	this->SetSensorPins(TOF5, TOF_GPIO_5_Pin, TOF_GPIO_5_GPIO_Port, XSHUT_5_Pin, XSHUT_5_GPIO_Port);
//#else
//	this->_Devices[TOF5].Address = TOF_DEFAULT_ADDRESS;
//#endif
//}
//
//VL53L1X_ERROR VL53L1X::InitSensor(uint8_t sensor)
//{
//	VL53L1X_ERROR status = 0;
//	MessageInfoTypeDef MsgInfo;
//
//	HAL_GPIO_WritePin(this->_Devices[sensor].XSHUT_GPIOx, this->_Devices[sensor].XSHUT_PIN, GPIO_PIN_SET);
//	if(this->_Devices[sensor].Address == TOF_DEFAULT_ADDRESS)
//	{
//		status = 0xFE;
//		this->_Devices[sensor].Active = false;
//	}
//	else status = this->SetSensorAddress(sensor);
////	status |= VL53L1X_SensorInit(this->_Devices[sensor].Address, this->_CommunicationManager, &MsgInfo);
//	if(status == 0)
//	{
//		this->_Devices[sensor].Active = true;
//	}
//	return status;
//}
//
//VL53L1X_ERROR VL53L1X::InitAllSensors(void)
//{
//	VL53L1X_ERROR status = 0;
//#ifdef TOF0
//	status |= (this->InitSensor(TOF0)) << TOF0;
//#endif
//#ifdef TOF1
//	status |= (this->InitSensor(TOF1)) << TOF1;
//#endif
//#ifdef TOF2
//	status |= (this->InitSensor(TOF2)) << TOF2;
//#endif
//#ifdef TOF3
//	status |= (this->InitSensor(TOF3)) << TOF3;
//#endif
//#ifdef TOF4
//	status |= (this->InitSensor(TOF4)) << TOF4;
//#endif
//#ifdef TOF5
//	status |= (this->InitSensor(TOF5)) << TOF5;
//#endif
//	return status;
//}
//
//VL53L1X_ERROR VL53L1X::SetSensorAddress(uint8_t sensor)
//{
//	VL53L1X_ERROR status = 0;
//	MessageInfoTypeDef MsgInfo;
//	MsgInfo.uCommInt.hi2c = this->_hi2c;
//	status = VL53L1X_SetI2CAddress(TOF_DEFAULT_ADDRESS, this->_Devices[sensor].Address, this->_CommunicationManager, &MsgInfo);
//	return status;
//}
//
//VL53L1X_ERROR VL53L1X::SetSensorPins(uint8_t sensor, uint16_t GPIO_PIN, GPIO_TypeDef *GPIO_GPIOx, uint16_t XSHUT_PIN, GPIO_TypeDef *XSHUT_GPIOx)
//{
//	this->_Devices[sensor].GPIO_GPIOx = GPIO_GPIOx;
//	this->_Devices[sensor].GPIO_PIN = GPIO_PIN;
//	this->_Devices[sensor].XSHUT_GPIOx = XSHUT_GPIOx;
//	this->_Devices[sensor].XSHUT_PIN = XSHUT_PIN;
//	return 0;
//}
//
//VL53L1X_ERROR VL53L1X::StartRanging(uint8_t sensor)
//{
//	MessageInfoTypeDef MsgInfo = {0};
//	MsgInfo.uCommInt.hi2c = this->_hi2c;
//	MsgInfo.eCommType = COMM_INT_I2C_TX;
//	MsgInfo.context = sensor << 4;
////	MsgInfo.pTxCompletedCB = this->MsgSent();
//	return VL53L1X_StartRanging(this->_Devices[sensor].Address, this->_CommunicationManager, &MsgInfo);
//}
//
//VL53L1X_ERROR VL53L1X::GetDistance(uint8_t sensor)
//{
//	uint16_t tmp;
//	VL53L1X_ERROR status = 0;
//	MessageInfoTypeDef MsgInfo = {0};
//	MsgInfo.uCommInt.hi2c = this->_hi2c;
//	MsgInfo.eCommType = COMM_INT_I2C_RX;
//	MsgInfo.context = sensor << 4;
////	MsgInfo.pTxCompletedCB = this->MsgSent();
//	status = VL53L1X_GetDistance(this->_Devices[sensor].Address, &tmp, this->_CommunicationManager, &MsgInfo);
//	return status;
//}
//
//VL53L1X_ERROR VL53L1X::ClearInterrupt(uint8_t sensor)
//{
////	uint16_t tmp;
////	VL53L1X_ERROR status = 0;
////	MessageInfoTypeDef MsgInfo = {0};
////	MsgInfo.uCommInt.hi2c = this->_hi2c;
////	MsgInfo.eCommType = COMM_INT_I2C_RX;
////	MsgInfo.context = sensor << 4;
//////	MsgInfo.pTxCompletedCB = this->MsgSent();
////	status = VL53L1X_GetDistance(this->_Devices[sensor].Address, &tmp, this->_CommunicationManager, &MsgInfo);
////	return status;
//}
//
//void VL53L1X::MsgSent(void)
//{
//
//}
//
//VL53L1X::~VL53L1X(void)
//{
//
//}
//
