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
#include "vl53l1_platform_user_data.hpp"
#include "vl53l1_platform_log.hpp"


#ifdef __cplusplus
extern "C"
{
#endif

#define MESSAGE_LENGTH 3


//typedef struct {
//	uint32_t dummy;
//} VL53L1_Dev_t;

//
//typedef VL53L1_Dev_t *VL53L1_DEV;

VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us);

VL53L1_Error VL53L1_CommsInitialise(
	VL53L1_Dev_t *pdev,
	uint8_t       comms_type,
	uint16_t      comms_speed_khz);

/** @brief VL53L1_WriteMulti() definition.\n
 * To be implemented by the developer
 */
uint8_t VL53L1X_WrMulti(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size);

/**
 * @brief Writes the supplied byte buffer to the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   pdata     : pointer to uint8_t (byte) buffer containing the data to be written
 * @param[in]   count     : number of bytes in the supplied byte buffer
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_WriteMulti(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);


/** @brief VL53L1_ReadMulti() definition.\n
 * To be implemented by the developer
 */
uint8_t VL53L1X_ReadMulti(
		uint16_t 			dev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);


/**
 * @brief  Reads the requested number of bytes from the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[out]  pdata     : pointer to the uint8_t (byte) buffer to store read data
 * @param[in]   count     : number of bytes to read
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_ReadMulti(
		VL53L1_Dev_t *pdev,
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

/**
 * @brief  Writes a single byte to the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   data      : uint8_t data value to write
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_WrByte(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint8_t       data);
/** @brief VL53L1_WrWord() definition.\n
 * To be implemented by the developer
 */
HAL_StatusTypeDef VL53L1X_WrWord(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint16_t value);

/**
 * @brief  Writes a single word (16-bit unsigned) to the device
 *
 * Manages the big-endian nature of the device register map
 * (first byte written is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   data      : uin16_t data value write
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_WrWord(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint16_t      data);

/** @brief VL53L1_RdByte() definition.\n
 * To be implemented by the developer
 */
uint8_t VL53L1X_RdByte(
		uint16_t dev,
		uint16_t RegisterAdress,
		uint8_t *p_value);



/**
 * @brief  Reads a single byte from the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index
 * @param[out]  pdata     : pointer to uint8_t data value
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 *
 */

VL53L1_Error VL53L1_RdByte(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint8_t      *pdata);

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

/**
 * @brief  Reads a single word (16-bit unsigned) from the device
 *
 * Manages the big-endian nature of the device (first byte read is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[out]  pdata     : pointer to uint16_t data value
 *
 * @return   VL53L1_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53L1_Error
 */


VL53L1_Error VL53L1_RdWord(
		VL53L1_Dev_t *pdev,
		uint16_t      index,
		uint16_t     *pdata);

uint8_t VL53L1_WaitMs(
		uint16_t dev,
		int32_t       wait_ms);

/*
 * @brief Gets current system tick count in [ms]
 *
 * @return  time_ms : current time in [ms]
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_GetTickCount(
		VL53L1_Dev_t *pdev,
	uint32_t *ptime_ms);

/**
 * @brief Register "wait for value" polling routine
 *
 * Port of the V2WReg Script function  WaitValueMaskEx()
 *
 * @param[in]   pdev          : pointer to device structure (device handle)
 * @param[in]   timeout_ms    : timeout in [ms]
 * @param[in]   index         : uint16_t register index value
 * @param[in]   value         : value to wait for
 * @param[in]   mask          : mask to be applied before comparison with value
 * @param[in]   poll_delay_ms : polling delay been each read transaction in [ms]
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_WaitValueMaskEx(
		VL53L1_Dev_t *pdev,
		uint32_t      timeout_ms,
		uint16_t      index,
		uint8_t       value,
		uint8_t       mask,
		uint32_t      poll_delay_ms);

void PlatformSetMutex(osapi::Mutex *pMutex);

#ifdef __cplusplus
}
#endif

#endif
