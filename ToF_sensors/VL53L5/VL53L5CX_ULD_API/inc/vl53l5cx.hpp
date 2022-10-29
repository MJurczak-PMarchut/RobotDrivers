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

class Sensor_vl53l5cx: public ToF_Sensor
{
private:
	static uint8_t __sensor_nb;
public:
	Sensor_vl53l5cx(uint8_t position, CommManager *comm);
	~Sensor_vl53l5cx(void) {};
	virtual HAL_StatusTypeDef SensorInit(void);
};





#endif /* TOF_SENSORS_VL53L5_VL53L5CX_ULD_API_INC_VL53L5CX_HPP_ */
