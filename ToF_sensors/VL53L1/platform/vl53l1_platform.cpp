
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

/*
 * Request tagging (B2): every request gets a fresh context. The completion callback only
 * wakes the caller whose context matches, so a late completion of an earlier (timed-out)
 * request, possibly from another init worker, can no longer release the current wait.
 * Platform contexts have bit 15 set, so they never match ranging (10) or VL53L5 (__pos).
 */
#define PLATFORM_CTX_FLAG  0x8000u
static uint16_t _seq = 0;
static volatile uint16_t _waiting_ctx = 0;
static volatile HAL_StatusTypeDef _result = HAL_ERROR;
// TransactionStatus target; static because the ISR may write it after the caller timed out.
static HAL_StatusTypeDef _xfer_status = HAL_OK;

/*
 * Static transfer buffers (B3): a timed-out message can still run later, so it must never
 * point at a stack frame. Calls are serialized by the platform mutex. Separate RX/TX
 * buffers: a stale RX runs before newer messages (FIFO), so it cannot corrupt their data.
 */
#define PLATFORM_BUF_SIZE  128u   // largest transfer today: 91-byte default config
static uint8_t _rx_buf[PLATFORM_BUF_SIZE];
static uint8_t _tx_buf[PLATFORM_BUF_SIZE];

static void _I2CTransferCompletedCB(MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo)
{
	if(MsgInfo->context != _waiting_ctx)
	{
		return;   // completion of an abandoned request
	}
	_result = (MsgInfo->TransactionStatus != NULL)? *MsgInfo->TransactionStatus : HAL_ERROR;
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

// Up to two attempts on timeout. A completed transfer (OK or NACK) returns its real result.
static HAL_StatusTypeDef _PushCommRequestAndWait(MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo, TickType_t timeoutTicks)
{
	if(_pCommManager == NULL)
	{
		return HAL_ERROR;
	}
	for(uint8_t attempt = 0; attempt < 2; attempt++)
	{
		uint16_t ctx = (uint16_t)(PLATFORM_CTX_FLAG | (++_seq & 0x7FFFu));
		MsgInfo->context = ctx;
		MsgInfo->TransactionStatus = &_xfer_status;
		MsgInfo->pCB = &_CompletionCallback;
		xSemaphoreTake(_CommDoneSemaphore, 0);   // drop a token left by an abandoned request
		_waiting_ctx = ctx;
		_pCommManager->PushCommRequestIntoQueue(MsgInfo);
		bool done = (xSemaphoreTake(_CommDoneSemaphore, timeoutTicks) == pdTRUE);
		_waiting_ctx = 0;
		if(done)
		{
			return (_result == HAL_OK)? HAL_OK : HAL_ERROR;
		}
		// Timed out: drop our message only if it is at the head. If it is still queued
		// further back it runs later, harmlessly (static buffers, ignored completion).
		_pCommManager->RemoveHeadIfContext(MsgInfo->IntHandle, ctx);
	}
	return HAL_ERROR;
}

static HAL_StatusTypeDef _Read(uint16_t dev, uint16_t reg, uint8_t *p_out, uint32_t size, TickType_t timeoutTicks)
{
	if(size > PLATFORM_BUF_SIZE)
	{
		return HAL_ERROR;
	}
	MessageInfoTypeDef<I2C_HandleTypeDef> MsgInfo = { 0 };
	lock_interface();
	MsgInfo.IntHandle = &hi2c1;
	MsgInfo.eCommType = COMM_INT_MEM_RX;
	MsgInfo.I2C_Addr = dev;
	MsgInfo.I2C_MemAddr = reg;
	MsgInfo.len = (uint16_t)size;
	MsgInfo.pRxData = _rx_buf;
	HAL_StatusTypeDef status = _PushCommRequestAndWait(&MsgInfo, timeoutTicks);
	if(status == HAL_OK)
	{
		memcpy(p_out, _rx_buf, size);   // copy out only a confirmed result
	}
	unlock_interface();
	return status;
}

static HAL_StatusTypeDef _Write(uint16_t dev, uint16_t reg, const uint8_t *p_in, uint32_t size, TickType_t timeoutTicks)
{
	if(size > PLATFORM_BUF_SIZE)
	{
		return HAL_ERROR;
	}
	MessageInfoTypeDef<I2C_HandleTypeDef> MsgInfo = { 0 };
	lock_interface();
	memcpy(_tx_buf, p_in, size);
	MsgInfo.IntHandle = &hi2c1;
	MsgInfo.eCommType = COMM_INT_MEM_TX;
	MsgInfo.I2C_Addr = dev;
	MsgInfo.I2C_MemAddr = reg;
	MsgInfo.len = (uint16_t)size;
	MsgInfo.pTxData = _tx_buf;
	HAL_StatusTypeDef status = _PushCommRequestAndWait(&MsgInfo, timeoutTicks);
	unlock_interface();
	return status;
}

uint8_t VL53L1X_RdByte(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
	return _Read(dev, RegisterAdress, p_value, 1, pdMS_TO_TICKS(150));
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
	return _Write(dev, RegisterAdress, &value, 1, pdMS_TO_TICKS(150));
}

HAL_StatusTypeDef VL53L1X_WrWord(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint16_t value)
{
	uint8_t temp8[2] = {uint8_t(value >> 8), uint8_t(value & 0x00FF)};
	return _Write(dev, RegisterAdress, temp8, 2, pdMS_TO_TICKS(150));
}

uint8_t VL53L1X_WrMulti(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	return _Write(dev, RegisterAdress, p_values, size, pdMS_TO_TICKS(20000));
}

uint8_t VL53L1X_RdMulti(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	return _Read(dev, RegisterAdress, p_values, size, pdMS_TO_TICKS(250));
}

HAL_StatusTypeDef VL53L1X_RdWord(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint16_t *p_value)
{
	uint8_t temp[2] = {0, 0};
	HAL_StatusTypeDef status = _Read(dev, RegisterAdress, temp, 2, pdMS_TO_TICKS(150));
	if(status == HAL_OK)
	{
		*p_value = (temp[0] << 8) + temp[1];
	}
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
