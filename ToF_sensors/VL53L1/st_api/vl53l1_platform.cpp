/*
 * vl53l1_platform.cpp
 *
 * Platform layer for the vendored ST VL53L1 full API: delegates register access
 * to the project's blocking, mutex-guarded CommManager primitives implemented in
 * ../platform/vl53l1_platform.cpp.
 *
 * The primitives are declared by hand below instead of including
 * ../platform/vl53l1_platform.hpp, because that header defines a VL53L1_Dev_t
 * that conflicts with the full API's one. HAL_Delay/HAL_GetTick are likewise
 * declared directly so no HAL header (with its I2C_HandleTypeDef) enters this
 * translation unit. This layer is only exercised during sensor initialization;
 * the runtime data path does not use the full API.
 */

#include "vl53l1_platform.h"

extern "C" {
/* Blocking primitives from ../platform/vl53l1_platform.cpp (dev = 8-bit I2C address).
 * Return types are int-sized status codes (HAL_StatusTypeDef / uint8_t promoted). */
uint8_t VL53L1X_WrMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);
uint8_t VL53L1X_RdMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);
int VL53L1X_WrByte(uint16_t dev, uint16_t index, uint8_t data);
int VL53L1X_WrWord(uint16_t dev, uint16_t index, uint16_t data);
uint8_t VL53L1X_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata);
int VL53L1X_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata);

/* STM32 HAL tick services (C symbols; declared to avoid pulling in HAL headers). */
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t Delay);
}

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
	return (VL53L1X_WrMulti(Dev->I2cDevAddr, index, pdata, count) == 0) ?
			VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
	return (VL53L1X_RdMulti(Dev->I2cDevAddr, index, pdata, count) == 0) ?
			VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data)
{
	return (VL53L1X_WrByte(Dev->I2cDevAddr, index, data) == 0) ?
			VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data)
{
	return (VL53L1X_WrWord(Dev->I2cDevAddr, index, data) == 0) ?
			VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data)
{
	uint8_t buffer[4];
	buffer[0] = (uint8_t)(data >> 24);
	buffer[1] = (uint8_t)((data >> 16) & 0xFF);
	buffer[2] = (uint8_t)((data >> 8) & 0xFF);
	buffer[3] = (uint8_t)(data & 0xFF);
	return VL53L1_WriteMulti(Dev, index, buffer, 4);
}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata)
{
	return (VL53L1X_RdByte(Dev->I2cDevAddr, index, pdata) == 0) ?
			VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *pdata)
{
	return (VL53L1X_RdWord(Dev->I2cDevAddr, index, pdata) == 0) ?
			VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *pdata)
{
	uint8_t buffer[4] = {0, 0, 0, 0};
	VL53L1_Error status = VL53L1_ReadMulti(Dev, index, buffer, 4);
	*pdata = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) |
			((uint32_t)buffer[2] << 8) | (uint32_t)buffer[3];
	return status;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_DEV Dev, int32_t wait_us)
{
	(void)Dev;
	/* Millisecond granularity is the finest available; round up. Init-time only. */
	HAL_Delay((uint32_t)((wait_us + 999) / 1000));
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_DEV Dev, int32_t wait_ms)
{
	(void)Dev;
	HAL_Delay((uint32_t)wait_ms);
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_GetTickCount(uint32_t *ptick_count_ms)
{
	*ptick_count_ms = HAL_GetTick();
	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitValueMaskEx(VL53L1_DEV Dev, uint32_t timeout_ms, uint16_t index,
		uint8_t value, uint8_t mask, uint32_t poll_delay_ms)
{
	uint32_t start_ms = HAL_GetTick();
	uint8_t reg_value = 0;
	VL53L1_Error status = VL53L1_ERROR_NONE;

	while ((HAL_GetTick() - start_ms) < timeout_ms)
	{
		status = VL53L1_RdByte(Dev, index, &reg_value);
		if (status != VL53L1_ERROR_NONE)
		{
			return status;
		}
		if ((reg_value & mask) == value)
		{
			return VL53L1_ERROR_NONE;
		}
		HAL_Delay(poll_delay_ms);
	}
	return VL53L1_ERROR_TIME_OUT;
}