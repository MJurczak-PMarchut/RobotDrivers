/*
 * vl53l5cx.cpp
 *
 *  Created on: 29 paź 2022
 *      Author: Paulina
 */

#include "vl53l5cx.hpp"


Sensor_vl53l5cx::Sensor_vl53l5cx(e_ToF_Position position, CommManager *comm):
	ToF_Sensor(vl53l5, position, comm)
{
	__sensor_conf.platform.__CommunicationManager = comm;
}

HAL_StatusTypeDef Sensor_vl53l5cx::SensorInit(void)
{
	HAL_StatusTypeDef ret = HAL_OK;
	ret = HAL_StatusTypeDef(vl53l5cx_init(&(this->__sensor_conf)));
	return ret;
}

