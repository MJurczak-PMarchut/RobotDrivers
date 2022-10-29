/*
 * ToFSensor.cpp
 *
 *  Created on: 29 paź 2022
 *      Author: Paulina
 */

#include "ToFSensor.hpp"

ToF_Sensor::ToF_Sensor(e_ToF_Type type, e_ToF_Position position, CommManager *comm):
	ToF_Type{type}, __CommunicationManager{comm}, __pos{position}
{

}

ToF_Sensor::~ToF_Sensor() {
	// TODO Auto-generated destructor stub
}

