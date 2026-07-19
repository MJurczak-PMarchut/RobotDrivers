/*
 * Adafruit_I2CDevice.cpp
 */
#include "Adafruit_I2CDevice.h"

#include <cstring>

#include "FreeRTOS.h"
#include "semphr.h"
#include "CommManager.hpp"

namespace
{
// Shared across every Adafruit_I2CDevice instance, matching lsm6dso.cpp's static
// SPIMutex - this project has exactly one physical I2C bus (hi2c1) and one OLED, so
// serializing all instances behind one mutex is sufficient; it would need to become
// per-instance if multiple concurrent I2C devices ever route through this class.
SemaphoreHandle_t I2CDeviceMutex = nullptr;

void TxCallback(MessageInfoTypeDef<I2C_HandleTypeDef> * /*msg*/)
{
	BaseType_t higherPrioTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(I2CDeviceMutex, &higherPrioTaskWoken);
}
} // namespace

Adafruit_I2CDevice::Adafruit_I2CDevice(uint8_t addr, TwoWire *theWire)
	: _addr(addr), _commManager(theWire ? theWire->commManager() : nullptr),
	  _hi2c(theWire ? theWire->handle() : nullptr)
{
	if (I2CDeviceMutex == nullptr)
	{
		I2CDeviceMutex = xSemaphoreCreateBinary();
	}
}

Adafruit_I2CDevice::~Adafruit_I2CDevice()
{
}

bool Adafruit_I2CDevice::begin(bool /*addr_detect*/)
{
	// Bus already initialized/attached to CommManager in Core/Src/DebugEntryCPP.cpp's
	// main_cpp() (MainCommManager.AttachCommInt(&hi2c1, COMM_INTERRUPT)).
	if (_commManager == nullptr || _hi2c == nullptr)
	{
		return false;
	}
	// Probes the address and reports failure if nothing acks, matching real
	// Adafruit_I2CDevice::begin() - Adafruit_GrayOLED::_init() depends on this to
	// catch a wrong address/absent device via `if (!i2c_dev->begin()) return false;`.
	// One-off blocking HAL probe (bypassing CommManager) since it only runs once at
	// begin() time, before any other I2C traffic is queued.
	return HAL_I2C_IsDeviceReady(_hi2c, (uint16_t)(_addr << 1), 3, 50) == HAL_OK;
}

bool Adafruit_I2CDevice::write(const uint8_t *buffer, size_t len, bool /*stop*/,
								const uint8_t *prefix_buffer, size_t prefix_len)
{
	if (_commManager == nullptr || _hi2c == nullptr)
	{
		return false;
	}
	size_t total = prefix_len + len;
	if (total == 0 || total > kMaxBufferSize)
	{
		return false;
	}
	if (prefix_len > 0)
	{
		memcpy(_txBuf, prefix_buffer, prefix_len);
	}
	if (len > 0)
	{
		memcpy(_txBuf + prefix_len, buffer, len);
	}

	MessageInfoTypeDef<I2C_HandleTypeDef> msg = {0};
	msg.IntHandle = _hi2c;
	msg.eCommType = COMM_INT_TX;
	msg.pTxData = _txBuf;
	msg.len = (uint16_t)total;
	// This project's I2C convention is the pre-shifted 8-bit address (see
	// RobotDrivers/ssd1306_conf.h: `(0x3C << 1)`), while _addr is the 7-bit address
	// Adafruit_SSD1327::begin() passes in - shift it here.
	msg.I2C_Addr = (uint16_t)(_addr << 1);
	msg.pTxCompletedCB = &TxCallback;

	// PushCommRequestIntoQueue() returning non-HAL_OK does not mean the message
	// failed to enqueue: CommBaseClass::__CheckForNextCommRequestAndStart() always
	// enqueues the message first, and only afterwards tries to start the transfer
	// immediately if the peripheral is free - if the peripheral is currently mid-
	// transfer (e.g. a ToF ranging read on the same hi2c1 bus), that immediate-start
	// attempt returns HAL_BUSY even though the message is already queued and will be
	// serviced once the current transfer's completion ISR chains to it. So: push
	// once, and wait on the completion semaphore regardless of the return value - a
	// message that was genuinely never enqueued at all (the rare true-failure paths
	// in PushMessageIntoQueue) will just never signal the semaphore, and this times
	// out and reports failure correctly. Do not retry the push on a non-OK status -
	// that re-enqueues a duplicate of the same message.
	_commManager->PushCommRequestIntoQueue(&msg);
	return xSemaphoreTake(I2CDeviceMutex, pdMS_TO_TICKS(100)) == pdTRUE;
}

bool Adafruit_I2CDevice::setSpeed(uint32_t /*desiredclk*/)
{
	// Bus speed is fixed by CubeMX's MX_I2C1_Init(); nothing to change at runtime.
	return true;
}
