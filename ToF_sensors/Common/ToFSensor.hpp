/*
 * ToFSensor.h
 *
 *  Created on: 29 paź 2022
 *      Author: Paulina
 */

#ifndef TOF_SENSORS_COMMON_TOFSENSOR_HPP_
#define TOF_SENSORS_COMMON_TOFSENSOR_HPP_

#include "CommManager.hpp"
#include "Configuration.h"

typedef enum  {vl53l5, vl53l1}e_ToF_Type;

typedef enum  {FRONT_LEFT, FRONT_RIGHT}e_ToF_Position;

class ToF_Sensor {
protected:
	e_ToF_Type ToF_Type;
	CommManager *__CommunicationManager;
	e_ToF_Position __pos;
public:
	ToF_Sensor(e_ToF_Type type, e_ToF_Position position, CommManager *comm);
	virtual HAL_StatusTypeDef SensorInit(void) = 0;
	e_ToF_Position getPosition(void) {return __pos;};
	virtual ~ToF_Sensor();
};

#endif /* TOF_SENSORS_COMMON_TOFSENSOR_HPP_ */
