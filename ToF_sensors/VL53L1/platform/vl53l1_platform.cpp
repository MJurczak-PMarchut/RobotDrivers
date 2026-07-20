
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
#include "FreeRTOS.h"
#include "semphr.h"

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

uint8_t _data_receiveL[MESSAGE_LENGTH];
uint8_t _data_receiveR[MESSAGE_LENGTH];


static CommManager *_pCommManager = NULL;
static SemaphoreHandle_t _CommDoneSemaphore = NULL;

static void _I2CTransferCompletedCB(MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo)
{
	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(_CommDoneSemaphore, &pxHigherPriorityTaskWoken);
}

static std::function<void(MessageInfoTypeDef<I2C_HandleTypeDef>*)> _CompletionCallback = _I2CTransferCompletedCB;

void PlatformSetCommManager(CommManager *comm)
{
	_pCommManager = comm;
	if(_CommDoneSemaphore == NULL)
	{
		_CommDoneSemaphore = xSemaphoreCreateBinary();
	}
}

static HAL_StatusTypeDef _PushCommRequestAndWait(MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo, TickType_t timeoutTicks)
{
	if(_pCommManager == NULL)
	{
		return HAL_ERROR;
	}
	_pCommManager->PushCommRequestIntoQueue(MsgInfo);
	if(xSemaphoreTake(_CommDoneSemaphore, timeoutTicks) == pdFALSE)
	{
		_pCommManager->MsgReceivedCB(MsgInfo->IntHandle);
		xSemaphoreTake(_CommDoneSemaphore, 1);
		_pCommManager->PushCommRequestIntoQueue(MsgInfo);
		if(xSemaphoreTake(_CommDoneSemaphore, timeoutTicks) == pdFALSE)
		{
			_pCommManager->MsgReceivedCB(MsgInfo->IntHandle);
			return HAL_ERROR;
		}
	}
	return HAL_OK;
}

uint8_t VL53L1X_RdByte(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
	HAL_StatusTypeDef status;
	MessageInfoTypeDef<I2C_HandleTypeDef> MsgInfo = { 0 };
	lock_interface();
	MsgInfo.IntHandle = &hi2c1;
	MsgInfo.eCommType = COMM_INT_MEM_RX;
	MsgInfo.I2C_Addr = dev;
	MsgInfo.I2C_MemAddr = RegisterAdress;
	MsgInfo.len = 1;
	MsgInfo.pRxData = p_value;
	MsgInfo.pCB = &_CompletionCallback;
	status = _PushCommRequestAndWait(&MsgInfo, pdMS_TO_TICKS(150));
	unlock_interface();
	return status;

}

void PlatformSetMutex(osapi::Mutex *pMutex)
{

	_pmutex = pMutex;
}

HAL_StatusTypeDef VL53L1X_WrByte(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint8_t value)
{
	HAL_StatusTypeDef status;
	uint8_t value_stat = value;
	MessageInfoTypeDef<I2C_HandleTypeDef> MsgInfo = { 0 };
	lock_interface();
	MsgInfo.IntHandle = &hi2c1;
	MsgInfo.eCommType = COMM_INT_MEM_TX;
	MsgInfo.I2C_Addr = dev;
	MsgInfo.I2C_MemAddr = RegisterAdress;
	MsgInfo.len = 1;
	MsgInfo.pTxData = &value_stat;
	MsgInfo.pCB = &_CompletionCallback;
	status = _PushCommRequestAndWait(&MsgInfo, pdMS_TO_TICKS(150));
	unlock_interface();
	return status;

}

HAL_StatusTypeDef VL53L1X_WrWord(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint16_t value)
{
	HAL_StatusTypeDef status;
	uint8_t temp8[2] = {0, 0};
	MessageInfoTypeDef<I2C_HandleTypeDef> MsgInfo = { 0 };
	temp8[0] = uint8_t(value >> 8);
	temp8[1] = uint8_t(value & 0x00FF);
	lock_interface();
	MsgInfo.IntHandle = &hi2c1;
	MsgInfo.eCommType = COMM_INT_MEM_TX;
	MsgInfo.I2C_Addr = dev;
	MsgInfo.I2C_MemAddr = RegisterAdress;
	MsgInfo.len = 2;
	MsgInfo.pTxData = temp8;
	MsgInfo.pCB = &_CompletionCallback;
	status = _PushCommRequestAndWait(&MsgInfo, pdMS_TO_TICKS(150));
	unlock_interface();
	return status;

}

uint8_t VL53L1X_WrMulti(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	HAL_StatusTypeDef status;
	MessageInfoTypeDef<I2C_HandleTypeDef> MsgInfo = { 0 };
	lock_interface();
	MsgInfo.IntHandle = &hi2c1;
	MsgInfo.eCommType = COMM_INT_MEM_TX;
	MsgInfo.I2C_Addr = dev;
	MsgInfo.I2C_MemAddr = RegisterAdress;
	MsgInfo.len = (uint16_t)size;
	MsgInfo.pTxData = p_values;
	MsgInfo.pCB = &_CompletionCallback;
	status = _PushCommRequestAndWait(&MsgInfo, pdMS_TO_TICKS(20000));
	unlock_interface();
	return status;

}

uint8_t VL53L1X_RdMulti(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	HAL_StatusTypeDef status;
	MessageInfoTypeDef<I2C_HandleTypeDef> MsgInfo = { 0 };
	lock_interface();
	MsgInfo.IntHandle = &hi2c1;
	MsgInfo.eCommType = COMM_INT_MEM_RX;
	MsgInfo.I2C_Addr = dev;
	MsgInfo.I2C_MemAddr = RegisterAdress;
	MsgInfo.len = (uint16_t)size;
	MsgInfo.pRxData = p_values;
	MsgInfo.pCB = &_CompletionCallback;
	status = _PushCommRequestAndWait(&MsgInfo, pdMS_TO_TICKS(250));
	unlock_interface();
	return status;

}

HAL_StatusTypeDef VL53L1X_RdWord(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint16_t *p_value)
{
	HAL_StatusTypeDef status;
	uint8_t temp[2] = {0, 0};
	MessageInfoTypeDef<I2C_HandleTypeDef> MsgInfo = { 0 };
	lock_interface();
	MsgInfo.IntHandle = &hi2c1;
	MsgInfo.eCommType = COMM_INT_MEM_RX;
	MsgInfo.I2C_Addr = dev;
	MsgInfo.I2C_MemAddr = RegisterAdress;
	MsgInfo.len = 2;
	MsgInfo.pRxData = temp;
	MsgInfo.pCB = &_CompletionCallback;
	status = _PushCommRequestAndWait(&MsgInfo, pdMS_TO_TICKS(150));
	*p_value = (temp[0] << 8) + temp[1];
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
