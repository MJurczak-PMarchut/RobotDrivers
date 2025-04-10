/*
 * vl53l1x.cpp
 *
 *  Created on: Nov 16, 2021
 *      Author: Paulina
 */

#include "vl53l1x.hpp"
#include "RobotSpecificDefines.hpp"
#include "vl53l1_platform.hpp"
#include "VL53L1X_api.hpp"

const static uint8_t __ToFAddr[] = { 0x54, 0x56, 0x58, 0x60, 0x62, 0x64 };
static uint8_t start_ranging_const = 0x40;
const static uint16_t __ToFX_SHUT_Pin[] = XSHUT_PINS;
const static uint16_t __ToFX_GPIO_Pin[] = TOFx_GPIO_PINS;
static GPIO_TypeDef *__ToFX_SHUT_Port[] = TOFx_XSHUT_PORTS;

uint8_t VL51L1X_DEFAULT_CONFIGURATION[] = {
0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
0x00, /* 0x32 : not user-modifiable */
0x02, /* 0x33 : not user-modifiable */
0x08, /* 0x34 : not user-modifiable */
0x00, /* 0x35 : not user-modifiable */
0x08, /* 0x36 : not user-modifiable */
0x10, /* 0x37 : not user-modifiable */
0x01, /* 0x38 : not user-modifiable */
0x01, /* 0x39 : not user-modifiable */
0x00, /* 0x3a : not user-modifiable */
0x00, /* 0x3b : not user-modifiable */
0x00, /* 0x3c : not user-modifiable */
0x00, /* 0x3d : not user-modifiable */
0xff, /* 0x3e : not user-modifiable */
0x00, /* 0x3f : not user-modifiable */
0x0F, /* 0x40 : not user-modifiable */
0x00, /* 0x41 : not user-modifiable */
0x00, /* 0x42 : not user-modifiable */
0x00, /* 0x43 : not user-modifiable */
0x00, /* 0x44 : not user-modifiable */
0x00, /* 0x45 : not user-modifiable */
0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
0x0b, /* 0x47 : not user-modifiable */
0x00, /* 0x48 : not user-modifiable */
0x00, /* 0x49 : not user-modifiable */
0x02, /* 0x4a : not user-modifiable */
0x0a, /* 0x4b : not user-modifiable */
0x21, /* 0x4c : not user-modifiable */
0x00, /* 0x4d : not user-modifiable */
0x00, /* 0x4e : not user-modifiable */
0x05, /* 0x4f : not user-modifiable */
0x00, /* 0x50 : not user-modifiable */
0x00, /* 0x51 : not user-modifiable */
0x00, /* 0x52 : not user-modifiable */
0x00, /* 0x53 : not user-modifiable */
0xc8, /* 0x54 : not user-modifiable */
0x00, /* 0x55 : not user-modifiable */
0x00, /* 0x56 : not user-modifiable */
0x38, /* 0x57 : not user-modifiable */
0xff, /* 0x58 : not user-modifiable */
0x01, /* 0x59 : not user-modifiable */
0x00, /* 0x5a : not user-modifiable */
0x08, /* 0x5b : not user-modifiable */
0x00, /* 0x5c : not user-modifiable */
0x00, /* 0x5d : not user-modifiable */
0x01, /* 0x5e : not user-modifiable */
0xcc, /* 0x5f : not user-modifiable */
0x0f, /* 0x60 : not user-modifiable */
0x01, /* 0x61 : not user-modifiable */
0xf1, /* 0x62 : not user-modifiable */
0x0d, /* 0x63 : not user-modifiable */
0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm  */
0x68, /* 0x65 : Sigma threshold LSB */
0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
0x80, /* 0x67 : Min count Rate LSB */
0x08, /* 0x68 : not user-modifiable */
0xb8, /* 0x69 : not user-modifiable */
0x00, /* 0x6a : not user-modifiable */
0x00, /* 0x6b : not user-modifiable */
0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
0x00, /* 0x6d : Intermeasurement period */
0x0f, /* 0x6e : Intermeasurement period */
0x89, /* 0x6f : Intermeasurement period LSB */
0x00, /* 0x70 : not user-modifiable */
0x00, /* 0x71 : not user-modifiable */
0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold() */
0x00, /* 0x73 : distance threshold high LSB */
0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold() */
0x00, /* 0x75 : distance threshold low LSB */
0x00, /* 0x76 : not user-modifiable */
0x01, /* 0x77 : not user-modifiable */
0x0f, /* 0x78 : not user-modifiable */
0x0d, /* 0x79 : not user-modifiable */
0x0e, /* 0x7a : not user-modifiable */
0x0e, /* 0x7b : not user-modifiable */
0x00, /* 0x7c : not user-modifiable */
0x00, /* 0x7d : not user-modifiable */
0x02, /* 0x7e : not user-modifiable */
0xc7, /* 0x7f : ROI center, use SetROI() */
0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
0x9B, /* 0x81 : not user-modifiable */
0x00, /* 0x82 : not user-modifiable */
0x00, /* 0x83 : not user-modifiable */
0x00, /* 0x84 : not user-modifiable */
0x01, /* 0x85 : not user-modifiable */
0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87 */
};

static uint8_t clr_interrupt = 0x01;

VL53L1X::VL53L1X(e_ToF_Position position, CommManager *comm, I2C_HandleTypeDef *hi2c1) :
		ToF_Sensor(vl53l1, position, comm), __hi2c1{hi2c1}, __wait_until_tick{0}, __Status {TOF_INIT_NOT_DONE}, __data_count{0} {

	this->__sensor_index = __no_of_sensors - 1;
	HAL_GPIO_WritePin(__ToFX_SHUT_Port[this->__sensor_index],
			__ToFX_SHUT_Pin[this->__sensor_index], GPIO_PIN_RESET);
	last_update_tick = HAL_GetTick();
	this->_CallbackFunc = std::bind(&VL53L1X::DataReceived, this, std::placeholders::_1);
}

HAL_StatusTypeDef VL53L1X::SensorInit(void){

	uint8_t status = 0;
	uint8_t tmp;
	UNUSED(tmp);
	status |= VL53L1X_WrMulti(this->__address, 0x2D, VL51L1X_DEFAULT_CONFIGURATION, 91);
	status |= this->StartRanging();
	tmp  = 0;
	while(tmp==0){
			status |= CheckForDataReady( &tmp);
	}
	status |= this->ClearInterrupt();
	return (status)? HAL_ERROR:HAL_OK;
}

uint8_t VL53L1X::ClearInterrupt()
{
	HAL_StatusTypeDef ret = HAL_OK;
	MessageInfoTypeDef<I2C_HandleTypeDef> MsgInfoToSend = { 0 };
	MsgInfoToSend.context = 10;
	MsgInfoToSend.eCommType = COMM_INT_MEM_TX;
	MsgInfoToSend.I2C_Addr = this->__address;
	MsgInfoToSend.I2C_MemAddr = SYSTEM__INTERRUPT_CLEAR;
	MsgInfoToSend.len = 1;
	MsgInfoToSend.pTxData = &clr_interrupt;
	MsgInfoToSend.IntHandle = this->__hi2c1;
	ret = this->__CommunicationManager->PushCommRequestIntoQueue(&MsgInfoToSend);
	return (uint8_t)ret;
}

HAL_StatusTypeDef VL53L1X::SetI2CAddress() {
	HAL_GPIO_WritePin(__ToFX_SHUT_Port[this->__sensor_index],
			__ToFX_SHUT_Pin[this->__sensor_index], GPIO_PIN_SET);
	vTaskDelay(2);
	uint8_t pval=0;

	HAL_StatusTypeDef ret = HAL_OK;
	VL53L1X_RdByte(__address, VL53L1_I2C_SLAVE__DEVICE_ADDRESS, &pval);
	ret = VL53L1X_WrByte(this->__address, VL53L1_I2C_SLAVE__DEVICE_ADDRESS, (__ToFAddr[this->__sensor_index]>>1));
	this->__address = (ret)? TOF_DEFAULT_ADDRESS: __ToFAddr[this->__sensor_index];
	this->__address = __ToFAddr[this->__sensor_index];
	this->__Status = (ret)?TOF_STATE_ERROR:TOF_STATE_OK;
	return ret;
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
}

HAL_StatusTypeDef  VL53L1X::StartRanging(){
	uint8_t status = HAL_OK;
	status |= VL53L1X_WrByte(this->__address, SYSTEM__MODE_START, start_ranging_const);	/* Enable VL53L1X */
	return (status)? HAL_ERROR:HAL_OK;
}

uint8_t VL53L1X::CheckForDataReady(uint8_t *isDataReady)
{
	uint8_t Temp;
	uint8_t IntPol;
	uint8_t status = HAL_OK;
	status |= this->GetInterruptPolarity(this->__address, &IntPol);
	status |= VL53L1X_RdByte(this->__address, GPIO__TIO_HV_STATUS, &Temp);
	/* Read in the register to check if a new value is available */
	if (status == 0){
		if ((Temp & 1) == IntPol)
			*isDataReady = 1;
		else
			*isDataReady = 0;
	}
	return status;
}

uint8_t  VL53L1X::GetInterruptPolarity(uint16_t dev, uint8_t *pInterruptPolarity)
{
	uint8_t Temp;
	uint8_t status = 0;

	status |= VL53L1X_RdByte(dev, GPIO_HV_MUX__CTRL, &Temp);
	Temp = Temp & 0x10;
	*pInterruptPolarity = !(Temp>>4);
	return status;
}

HAL_StatusTypeDef VL53L1X::GetRangingData(void){
	HAL_StatusTypeDef ret = HAL_OK;
	MessageInfoTypeDef<I2C_HandleTypeDef> MsgInfoToSend = { 0 };
	MsgInfoToSend.context = 10;
	MsgInfoToSend.eCommType = COMM_INT_MEM_RX;
	MsgInfoToSend.I2C_Addr = this->__address;
	MsgInfoToSend.I2C_MemAddr = VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0;
	MsgInfoToSend.len = 2;
	MsgInfoToSend.pRxData = (uint8_t*)this->__comm_buffer;
	MsgInfoToSend.IntHandle = this->__hi2c1;
	MsgInfoToSend.pCB = &this->_CallbackFunc;
	ret = this->__CommunicationManager->PushCommRequestIntoQueue(&MsgInfoToSend);
	this->__Status = TOF_STATE_DATA_RDY;
	this->ClearInterrupt();
	return ret;
}

void VL53L1X::DataReceived(MessageInfoTypeDef<I2C_HandleTypeDef>* MsgInfo){
	this->__distance = this->__comm_buffer[1] | (this->__comm_buffer[0] << 8);
	xEventGroupSetBitsFromISR(EventGroupHandle, (1<<this->__sensor_index), NULL);
}

HAL_StatusTypeDef VL53L1X::__GetData(void)
{
	if(this->__Status == TOF_STATE_OK){
		this->__Status = TOF_STATE_DATA_RDY;
	}
	return HAL_OK;
}

uint16_t VL53L1X::GetSensorITPin(void)
{
	return __ToFX_GPIO_Pin[this->__sensor_index];
}

HAL_StatusTypeDef VL53L1X::DisableSensorComm(void)
{
	HAL_GPIO_WritePin(__ToFX_SHUT_Port[this->__sensor_index],
			__ToFX_SHUT_Pin[this->__sensor_index], GPIO_PIN_RESET);
	return HAL_OK;
}

void VL53L1X::SetMutex(osapi::Mutex *pmutex)
{
	PlatformSetMutex(pmutex);
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
