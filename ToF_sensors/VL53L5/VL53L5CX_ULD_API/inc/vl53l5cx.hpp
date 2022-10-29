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

class Sensor_vl53l5cx: public ToF_Sensor
{
private:
	static uint8_t __sensor_nb;
	VL53L5CX_Configuration __sensor_conf;
public:
	Sensor_vl53l5cx(e_ToF_Position position, CommManager *comm);
	virtual HAL_StatusTypeDef SensorInit(void);
	HAL_StatusTypeDef SetI2CAddress(uint8_t new_addr);
	~Sensor_vl53l5cx(void) {};
};





#endif /* TOF_SENSORS_VL53L5_VL53L5CX_ULD_API_INC_VL53L5CX_HPP_ */
