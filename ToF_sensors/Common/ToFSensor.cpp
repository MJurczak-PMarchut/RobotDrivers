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

TaskHandle_t* ToF_Sensor::__pTaskHandle;
ToF_Sensor*  ToF_Sensor::__ToFSensorPointers[10];
uint8_t ToF_Sensor::__no_of_sensors = 0;

ToF_Sensor::ToF_Sensor(e_ToF_Type type, e_ToF_Position position,
		CommManager *comm) :
		ToF_Type { type }, __CommunicationManager { comm }, __pos { position } {
	__ToFSensorPointers[__no_of_sensors] = this;
	__no_of_sensors++;
}

void ToF_Sensor::StartSensorTask(void) {

	ToF_Sensor* SensorObj;
	for (uint8_t i = 0; i < __no_of_sensors; i++)
	{
		SensorObj = __ToFSensorPointers[i];
		SensorObj->SetI2CAddress();
	}
//	Start task for monitoring and initialization of ToF sensors
//	if (xTaskCreate(__ToFSensorThread, "ToF Thread", 1024, NULL, 1, ToF_Sensor::__pTaskHandle) != pdPASS) {
//		Error_Handler();
//	}
}

ToF_Sensor::~ToF_Sensor() {
	// TODO Auto-generated destructor stub
}

void ToF_Sensor::__ToFSensorThread(void *pvParameters) {
	ToF_Sensor* SensorObj;
	while(1){
		for (uint8_t i = 0; i < __no_of_sensors; i++){
			SensorObj = __ToFSensorPointers[i];
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
}
