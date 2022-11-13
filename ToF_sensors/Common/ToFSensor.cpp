/*
 * ToFSensor.cpp
 *
 *  Created on: 29 paź 2022
 *      Author: Paulina
 */

#include "ToFSensor.hpp"

#ifndef USES_RTOS
#error("RTOS is needed for current implementation of Tof Sensor drivers")
#endif

TaskHandle_t ToF_Sensor::*__pTaskHandle = 0;
std::vector<ToF_Sensor*>  ToF_Sensor::__ToFSensorPointer;

ToF_Sensor::ToF_Sensor(e_ToF_Type type, e_ToF_Position position,
		CommManager *comm) :
		ToF_Type { type }, __CommunicationManager { comm }, __pos { position } {
	__ToFSensorPointer.push_back(this);
}

void ToF_Sensor::StartSensorTask(void) {

	//SetI2CAddress;
	for (auto SensorObj : __ToFSensorPointer)
	{
		SensorObj->SetI2CAddress();
	}
	//Start task for monitoring and initialization of ToF sensors
	if (xTaskCreate(__ToFSensorThread, "ToF Thread", 1024, NULL, 1,
			__pTaskHandle) != pdPASS) {
		Error_Handler();
	}
}

ToF_Sensor::~ToF_Sensor() {
	// TODO Auto-generated destructor stub
}

void ToF_Sensor::__ToFSensorThread(void *pvParameters) {
	for (auto SensorObj : __ToFSensorPointer) {
		switch (SensorObj->CheckSensorStatus()) {
			case TOF_STATE_OK: {

			}
				break;
			case TOF_STATE_INIT_WAIT: {

			}
				break;
			case TOF_INIT_NOT_DONE: {

			}
				break;
			case TOF_STATE_ERROR: {

			}
				break;
			case TOF_STATE_BUSY: {

			}
				break;
			case TOF_STATE_INIT_ONGOING: {

			}
				break;
			case TOF_STATE_NOT_CONNECTED: {

			}
				break;
			case TOF_STATE_COMM_ERROR: {

			}
				break;
			default: {

			}
				break;
		}
	}
	vTaskDelay(50);
}
