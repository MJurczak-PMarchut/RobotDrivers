/**
 * @file  vl53l1_platform.h
 * @brief Those platform functions are platform dependent and have to be implemented by the user
 */
 
#ifndef _VL53L1_PLATFORM_H_
#define _VL53L1_PLATFORM_H_

#include "vl53l1_types.hpp"
#include "Configuration.h"
#include "CommManager.hpp"
#include "vl53l1x.hpp"
#include "osapi.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define MESSAGE_LENGTH 3

typedef struct {
	uint32_t dummy;
} VL53L1_Dev_t;


typedef VL53L1_Dev_t *VL53L1_DEV;

/** @brief VL53L1_WriteMulti() definition.\n
 * To be implemented by the developer
 */
uint8_t VL53L1X_WrMulti(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size);
/** @brief VL53L1_ReadMulti() definition.\n
 * To be implemented by the developer
 */
uint8_t VL53L1X_ReadMulti(
		uint16_t 			dev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);
/** @brief VL53L1_WrByte() definition.\n
 * To be implemented by the developer
 */
HAL_StatusTypeDef VL53L1X_WrByte(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint8_t value);
/** @brief VL53L1_WrWord() definition.\n
 * To be implemented by the developer
 */
HAL_StatusTypeDef VL53L1X_WrWord(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint16_t value);
/** @brief VL53L1_RdByte() definition.\n
 * To be implemented by the developer
 */
uint8_t VL53L1X_RdByte(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint8_t *p_value);
/** @brief VL53L1_RdWord() definition.\n
 * To be implemented by the developer
 */
HAL_StatusTypeDef VL53L1X_RdWord(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint16_t *p_value);
/** @brief VL53L1_WaitMs() definition.\n
 * To be implemented by the developer
 */
uint8_t VL53L1_WaitMs(
		uint16_t dev,
		int32_t       wait_ms);


void PlatformSetMutex(osapi::Mutex *pMutex);

#ifdef __cplusplus
}
#endif

#endif
