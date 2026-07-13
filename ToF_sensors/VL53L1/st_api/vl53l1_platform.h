/*
 * vl53l1_platform.h
 *
 * Platform layer declarations for the vendored ST VL53L1 full API (st_api/).
 * The implementation (vl53l1_platform.cpp in this directory) bridges these onto
 * the project's blocking CommManager-based register access primitives from
 * ../platform/vl53l1_platform.cpp.
 *
 * NOTE: this is distinct from ../platform/vl53l1_platform.hpp (the ULD platform
 * header) - the two define conflicting VL53L1_Dev_t types and must never be
 * included in the same translation unit.
 */

#ifndef _VL53L1_PLATFORM_H_
#define _VL53L1_PLATFORM_H_

#include "vl53l1_ll_def.h"
#include "vl53l1_platform_log.h"
#include "vl53l1_platform_user_data.h"

#ifdef __cplusplus
extern "C"
{
#endif

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count);
VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count);
VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data);
VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data);
VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data);
VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata);
VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *pdata);
VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *pdata);
VL53L1_Error VL53L1_WaitUs(VL53L1_DEV Dev, int32_t wait_us);
VL53L1_Error VL53L1_WaitMs(VL53L1_DEV Dev, int32_t wait_ms);
VL53L1_Error VL53L1_GetTickCount(uint32_t *ptick_count_ms);
VL53L1_Error VL53L1_WaitValueMaskEx(VL53L1_DEV Dev, uint32_t timeout_ms, uint16_t index,
		uint8_t value, uint8_t mask, uint32_t poll_delay_ms);

#ifdef __cplusplus
}
#endif

#endif /* _VL53L1_PLATFORM_H_ */