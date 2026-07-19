/*
 * Adafruit_SPIDevice.cpp
 */
#include "Adafruit_SPIDevice.h"

#include <cstring>

#include "FreeRTOS.h"
#include "semphr.h"
#include "CommManager.hpp"
#include "Arduino.h"

namespace
{
// Shared across instances, same rationale as Adafruit_I2CDevice's I2CDeviceMutex.
SemaphoreHandle_t SPIDeviceMutex = nullptr;

void TxCallback(MessageInfoTypeDef<SPI_HandleTypeDef> * /*msg*/)
{
	BaseType_t higherPrioTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(SPIDeviceMutex, &higherPrioTaskWoken);
}
} // namespace

Adafruit_SPIDevice::Adafruit_SPIDevice(int8_t cspin, int8_t /*clkpin*/, int8_t /*misopin*/,
										int8_t /*mosipin*/, uint32_t /*freq*/)
	: _isHardware(false), _csPin(cspin), _commManager(nullptr), _hspi(nullptr)
{
}

Adafruit_SPIDevice::Adafruit_SPIDevice(int8_t cspin, uint32_t /*freq*/, SPIBitOrder /*dataOrder*/,
										SPIDataMode /*dataMode*/, SPIClass *theSPI)
	: _isHardware(true), _csPin(cspin),
	  _commManager(theSPI ? theSPI->commManager() : nullptr),
	  _hspi(theSPI ? theSPI->handle() : nullptr)
{
	if (SPIDeviceMutex == nullptr)
	{
		SPIDeviceMutex = xSemaphoreCreateBinary();
	}
}

Adafruit_SPIDevice::~Adafruit_SPIDevice()
{
}

bool Adafruit_SPIDevice::begin(void)
{
	if (!_isHardware)
	{
		return false;
	}
	pinMode(_csPin, OUTPUT);
	digitalWrite(_csPin, HIGH);
	return (_commManager != nullptr) && (_hspi != nullptr);
}

bool Adafruit_SPIDevice::write(const uint8_t *buffer, size_t len)
{
	if (!_isHardware || _commManager == nullptr || _hspi == nullptr)
	{
		return false;
	}
	if (len == 0 || len > kMaxBufferSize)
	{
		return false;
	}
	memcpy(_txBuf, buffer, len);

	GPIO_TypeDef *csPort = nullptr;
	uint16_t csMask = 0;
	ArduinoPinDecode(_csPin, &csPort, &csMask);

	MessageInfoTypeDef<SPI_HandleTypeDef> msg = {0};
	msg.IntHandle = _hspi;
	msg.eCommType = COMM_INT_TX;
	msg.pTxData = _txBuf;
	msg.len = (uint16_t)len;
	msg.GPIOx = csPort;
	msg.GPIO_PIN = csMask;
	msg.pTxCompletedCB = &TxCallback;

	// See Adafruit_I2CDevice::write() - PushCommRequestIntoQueue() returning
	// non-HAL_OK (e.g. HAL_BUSY) does not mean the message failed to enqueue; it's
	// already in CommBaseClass's queue by that point and will be serviced once the
	// peripheral frees up. Retrying the push here would enqueue a duplicate. Push
	// once, then wait on the completion semaphore regardless of the return value.
	_commManager->PushCommRequestIntoQueue(&msg);
	return xSemaphoreTake(SPIDeviceMutex, pdMS_TO_TICKS(100)) == pdTRUE;
}
