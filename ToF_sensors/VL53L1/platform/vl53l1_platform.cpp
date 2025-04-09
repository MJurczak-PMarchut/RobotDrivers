
/* 
* This file is part of VL53L1 Platform 
* 
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
* 
* License terms: BSD 3-clause "New" or "Revised" License. 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution. 
* 
* 3. Neither the name of the copyright holder nor the names of its contributors 
* may be used to endorse or promote products derived from this software 
* without specific prior written permission. 
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
* 
*/

#include <string.h>
#include <time.h>
#include <math.h>
#include "vl53l1_platform.hpp"

extern I2C_HandleTypeDef hi2c1;



#ifdef USES_RTOS
static osapi::Mutex *_pmutex = NULL;


static void lock_interface()
{
	if(_pmutex != NULL){
		_pmutex->lock(-1);
	}
}

static void unlock_interface()
{
	if(_pmutex != NULL){
		_pmutex->unlock();
	}
}

#else
void lock_interface()
{
}

void unlock_interface()
{
}
#endif

static uint8_t _data[MESSAGE_LENGTH];
uint8_t _data_receiveL[MESSAGE_LENGTH];
uint8_t _data_receiveR[MESSAGE_LENGTH];

uint8_t VL53L1X_RdByte(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
	uint8_t status = 0;
	lock_interface();
	status = HAL_I2C_Mem_Read(&hi2c1, dev, RegisterAdress, 2, p_value, 1, 150);
	unlock_interface();
	return status;

}

HAL_StatusTypeDef VL53L1X_WrByte(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint8_t value)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t value_stat = value;
	lock_interface();
	status = HAL_I2C_Mem_Write(&hi2c1, dev, RegisterAdress, 2, &value_stat, 1, 150);
	unlock_interface();
	return status;

}

uint8_t VL53L1X_WrMulti(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	uint8_t status = 0;
	lock_interface();
	status = HAL_I2C_Mem_Write(&hi2c1, dev, RegisterAdress, 2, p_values, size, 20000);
	unlock_interface();
	return status;

}

uint8_t VL53L1X_RdMulti(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	uint8_t status = 0;
	lock_interface();
	status = HAL_I2C_Mem_Read(&hi2c1, dev, RegisterAdress, 2, p_values, size, 250);
	unlock_interface();
	return status;

}


void VL53L1X_SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size)
{
	uint32_t i, tmp;

	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4)
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);

		memcpy(&(buffer[i]), &tmp, 4);
	}
}

uint8_t VL53L1X_WaitMs(
		uint32_t TimeMs)
{
	uint8_t status = 0;

#ifdef USES_RTOS
//Use vTaskDelay when RTOS is in use
	vTaskDelay(TimeMs);
#else
	HAL_Delay(TimeMs);
#endif


	return status;
}


//int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint16_t count, CommManager *CommunicationManager, MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo) {
//	int8_t status = 0;
//	MsgInfo->I2C_Addr = dev;
//	MsgInfo->eCommType = COMM_INT_MEM_TX;
//	MsgInfo->I2C_MemAddr = index;
//	MsgInfo->pTxData = pdata;
//	MsgInfo->len = count;
//	do
//	{
//		status = CommunicationManager->PushCommRequestIntoQueue(MsgInfo);
//	} while(status!=0);
//	return status;
//}
//
//int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count){
//	return 0; // to be implemented
//}
//
//int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t *data, CommManager *CommunicationManager, MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo) {
//	int8_t status = 0;
//	MsgInfo->I2C_Addr = dev;
//	MsgInfo->eCommType = COMM_INT_MEM_TX;
//	MsgInfo->I2C_MemAddr = index;
//	MsgInfo->pTxData = data;
//	MsgInfo->len = 1;
//
//	do
//	{
//		status = CommunicationManager->PushCommRequestIntoQueue(MsgInfo);
//	} while(status!=0);
////	return HAL_I2C_Master_Transmit(MsgInfo->uCommInt.hi2c, MsgInfo->I2C_Addr, _data, MsgInfo->len, 100);
//	return status;
//}
//
//int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
//	return 0; // to be implemented
//}
//
//int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
//	return 0; // to be implemented
//}
//
//int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data, CommManager *CommunicationManager, MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo) {
//	int8_t status = 0;
//	_data[0] = (uint8_t) index >> 8;
//	_data[1] = (uint8_t) index & 0xff;
//	_data[2] = 0;
//
//	MsgInfo->I2C_Addr = dev;
//	MsgInfo->I2C_MemAddr = index;
//	MsgInfo->eCommType = COMM_INT_MEM_RX;
//	MsgInfo->len = 1;
//	if(dev == TOF5_Addr)
//	{
//		_data_receiveL[0] = 0;
//		_data_receiveL[1] = 0;
//		_data_receiveL[2] = 0;
////		MsgInfo->pRxData = _data_receiveL;
//	}
//	else
//	{
//		_data_receiveR[0] = 0;
//		_data_receiveR[1] = 0;
//		_data_receiveR[2] = 0;
////		MsgInfo->pRxData = _data_receiveR;
//	}
//	MsgInfo->pRxData = data;
//	do
//	{
//		status = CommunicationManager->PushCommRequestIntoQueue(MsgInfo);
//	} while(status!=0);
//	return status;
//}
//
//int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data, CommManager *CommunicationManager, MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo) {
//	int8_t status = 0;
//	_data[0] = (uint8_t) index >> 8;
//	_data[1] = (uint8_t) index & 0xff;
//	_data[2] = 0;
//
//	MsgInfo->I2C_Addr = dev;
//	MsgInfo->I2C_MemAddr = index;
//	MsgInfo->pTxData = _data;
//	MsgInfo->len = 2;
//	if(dev == TOF5_Addr)
//	{
//		_data_receiveL[0] = 0;
//		_data_receiveL[1] = 0;
//		_data_receiveL[2] = 0;
//		MsgInfo->pRxData = _data_receiveL;
//	}
//	else
//	{
//		_data_receiveR[0] = 0;
//		_data_receiveR[1] = 0;
//		_data_receiveR[2] = 0;
//		MsgInfo->pRxData = _data_receiveR;
//	}
//
//	do
//	{
//		status = CommunicationManager->PushCommRequestIntoQueue(MsgInfo);
//	} while(status!=0);
//
////	status |= HAL_I2C_Master_Transmit_IT(MsgInfo->uCommInt.hi2c, MsgInfo->I2C_Addr, MsgInfo->pTxData, MsgInfo->len);
////	status |= HAL_I2C_Master_Receive_IT(MsgInfo->uCommInt.hi2c, MsgInfo->I2C_Addr, MsgInfo->pRxData, MsgInfo->len);
//
//	// it worked previously
////	status |= HAL_I2C_Master_Transmit(MsgInfo->uCommInt.hi2c, MsgInfo->I2C_Addr, MsgInfo->pTxData, MsgInfo->len, 100);
////	status |= HAL_I2C_Master_Receive(MsgInfo->uCommInt.hi2c, MsgInfo->I2C_Addr, MsgInfo->pRxData, MsgInfo->len, 100);
//	return status;
//}
//
//int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
//	return 0; // to be implemented
//}
//
//int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
//	return 0; // to be implemented
//}
