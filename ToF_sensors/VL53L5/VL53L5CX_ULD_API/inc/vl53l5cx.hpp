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

class VL53L5CX: public ToF_Sensor
{
private:
	static uint8_t __sensor_nb;
	static uint8_t __sensor_init_tbd;
	uint8_t __sensor_index;
	VL53L5CX_Configuration __sensor_conf;
public:
	VL53L5CX_ResultsData result;
	VL53L5CX(e_ToF_Position position, CommManager *comm);
	virtual HAL_StatusTypeDef SensorInit(void);
	HAL_StatusTypeDef DisableSensorComm(void);
	HAL_StatusTypeDef SetI2CAddress(void);
	HAL_StatusTypeDef IsAlive(uint8_t *is_alive);
	HAL_StatusTypeDef SetPowerMode(void);
	HAL_StatusTypeDef StartRanging(void);
	HAL_StatusTypeDef StopRanging(void);
	HAL_StatusTypeDef CheckDataReady(void);
	HAL_StatusTypeDef GetRangingData(void);
	HAL_StatusTypeDef SetResolution(void);
	HAL_StatusTypeDef SetRangingFrequency(void);
	~VL53L5CX(void) {};
};





#endif /* TOF_SENSORS_VL53L5_VL53L5CX_ULD_API_INC_VL53L5CX_HPP_ */
