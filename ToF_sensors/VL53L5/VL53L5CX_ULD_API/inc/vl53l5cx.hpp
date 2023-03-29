/*
 * vl53l5cx.hpp
 *
 *  Created on: 29 paź 2022
 *      Author: Paulina
 */

#ifndef TOF_SENSORS_VL53L5_VL53L5CX_ULD_API_INC_VL53L5CX_HPP_
#define TOF_SENSORS_VL53L5_VL53L5CX_ULD_API_INC_VL53L5CX_HPP_

#include "Configuration.h"
#include "CommManager.hpp"
#include "ToFSensor.hpp"
#include "vl53l5cx_api.h"

#define DEFAULT_ADDR 0x52
#define CHANGED_ADDRESS_OFFSET 0x10

typedef enum {ROTATE_0, ROTATE_90, ROTATE_180, ROTATE_270} SensorSpatialOrientation;

class VL53L5CX: public ToF_Sensor {
public:
	VL53L5CX_ResultsData result;
	VL53L5CX(e_ToF_Position position, CommManager *comm, I2C_HandleTypeDef *hi2c1);
	HAL_StatusTypeDef CheckDataReady(void);
	HAL_StatusTypeDef GetRangingData(void);
	ToF_Status_t CheckSensorStatus(void);
	uint16_t GetDataFromSensor(uint8_t x, uint8_t y);
	uint8_t GetStatusFromSensor(uint8_t x, uint8_t y);
	void DataReceived(MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo);
	void SetMutex(osapi::Mutex *pmutex);
	~VL53L5CX(void) {
	}
	;

protected:
	HAL_StatusTypeDef StopRanging(void);
//	HAL_StatusTypeDef SensorInit(MessageInfoTypeDef* MsgInfo);
	uint16_t GetSensorITPin(void);
	HAL_StatusTypeDef __GetData(void);
	HAL_StatusTypeDef SensorInit(void);
	HAL_StatusTypeDef SetI2CAddress(void);
	HAL_StatusTypeDef IsAlive(uint8_t *is_alive);
	HAL_StatusTypeDef SetPowerMode(void);
	HAL_StatusTypeDef StartRanging(void);
	HAL_StatusTypeDef DisableSensorComm(void);
	HAL_StatusTypeDef EnableSensorComm(void);
	HAL_StatusTypeDef SetResolution(void);
	HAL_StatusTypeDef SetRotation(SensorSpatialOrientation Orientation);
	HAL_StatusTypeDef SetRangingFrequency(void);

private:
	void __waitInit(uint32_t waitms);
	HAL_StatusTypeDef __CheckInitPollingMessage(
			uint8_t					size,
			uint8_t					pos,
			uint8_t					mask,
			uint8_t					expected_value,
			MessageInfoTypeDef<I2C_HandleTypeDef>* 	MsgInfo,
			MessageInfoTypeDef<I2C_HandleTypeDef>*		MsgInfoToSend);
	uint8_t __vl53l5cx_poll_for_mcu_boot(MessageInfoTypeDef<I2C_HandleTypeDef>* MsgInfoToSend);
	uint8_t __vl53l5cx_send_offset_data(uint8_t resolution);
	uint8_t __vl53l5cx_send_xtalk_data(uint8_t resolution);
	uint8_t __vl53l5cx_start_ranging(MessageInfoTypeDef<I2C_HandleTypeDef>* MsgInfoToSend);
	uint16_t __vl53l5cx_dci_write_data(uint8_t *data, uint32_t index, uint16_t data_size);
	uint8_t __vl53l5cx_dci_read_data(
			uint8_t 			*_cmd,
			uint32_t			index,
			uint16_t			data_size);
	I2C_HandleTypeDef *__hi2c1;
	uint16_t __InitSequenceID;
	TickType_t __wait_until_tick;
	static uint8_t __sensor_nb;
	static uint8_t __sensor_init_tbd;
	uint8_t __sensor_index;
	VL53L5CX_Configuration __sensor_conf;
	ToF_Status_t __Status;
	uint32_t __data_count;
	uint16_t __address;
	static uint8_t null_data_sink;
	uint8_t __comm_buffer[VL53L5CX_TEMPORARY_BUFFER_SIZE];
	uint8_t __offset_buffer[VL53L5CX_OFFSET_BUFFER_SIZE];
	uint32_t last_update_tick;
	SensorSpatialOrientation eOrientation;
};

#endif /* TOF_SENSORS_VL53L5_VL53L5CX_ULD_API_INC_VL53L5CX_HPP_ */
