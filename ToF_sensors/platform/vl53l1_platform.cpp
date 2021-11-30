
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
#include <platform/vl53l1_platform.hpp>

static uint8_t _data[MESSAGE_LENGTH];
uint8_t _data_receiveL[MESSAGE_LENGTH];
uint8_t _data_receiveR[MESSAGE_LENGTH];

int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	return 0; // to be implemented
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count){
	return 0; // to be implemented
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data, CommManager *CommunicationManager, MessageInfoTypeDef *MsgInfo) {
	static uint8_t _data[MESSAGE_LENGTH];
	_data[0] = (uint8_t) index >> 8;
	_data[1] = (uint8_t) index & 0xff;
	_data[2] = data;
	MsgInfo->I2C_Addr = dev;
	MsgInfo->pTxData = _data;
	MsgInfo->len = MESSAGE_LENGTH;

//	return CommunicationManager->PushCommRequestIntoQueue(MsgInfo);
	return HAL_I2C_Master_Transmit(MsgInfo->uCommInt.hi2c, MsgInfo->I2C_Addr, _data, MsgInfo->len, 100);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
	return 0; // to be implemented
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
	return 0; // to be implemented
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data, CommManager *CommunicationManager, MessageInfoTypeDef *MsgInfo) {
	return 0; // to be implemented
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data, CommManager *CommunicationManager, MessageInfoTypeDef *MsgInfo) {
	int8_t status = 0;
	_data[0] = (uint8_t) index >> 8;
	_data[1] = (uint8_t) index & 0xff;
	_data[2] = 0;

	MsgInfo->I2C_Addr = dev;
	MsgInfo->pTxData = _data;
	MsgInfo->len = 2;
	if(dev == TOF5_Addr)
	{
		_data_receiveL[0] = 0;
		_data_receiveL[1] = 0;
		_data_receiveL[2] = 0;
		MsgInfo->pRxData = _data_receiveL;
	}
	else
	{
		_data_receiveR[0] = 0;
		_data_receiveR[1] = 0;
		_data_receiveR[2] = 0;
		MsgInfo->pRxData = _data_receiveR;
	}

//	return CommunicationManager->PushCommRequestIntoQueue(MsgInfo);
//	status |= HAL_I2C_Master_Transmit_IT(MsgInfo->uCommInt.hi2c, MsgInfo->I2C_Addr, MsgInfo->pTxData, MsgInfo->len);
//	status |= HAL_I2C_Master_Receive_IT(MsgInfo->uCommInt.hi2c, MsgInfo->I2C_Addr, MsgInfo->pRxData, MsgInfo->len);
	status |= HAL_I2C_Master_Transmit(MsgInfo->uCommInt.hi2c, MsgInfo->I2C_Addr, MsgInfo->pTxData, MsgInfo->len, 100);
	status |= HAL_I2C_Master_Receive(MsgInfo->uCommInt.hi2c, MsgInfo->I2C_Addr, MsgInfo->pRxData, MsgInfo->len, 100);
	return status;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
	return 0; // to be implemented
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	return 0; // to be implemented
}
