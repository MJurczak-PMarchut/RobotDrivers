/*
 * ToFSensor.cpp
 *
 *  Created on: 29 paź 2022
 *      Author: Paulina
 */

#include "ToFSensor.hpp"

#ifndef USES_RTOS
#error("RTOS is needed for current implementation of Tof Sensor drivers")
#else
#define NO_OF_WORKERS 2
#endif


TaskHandle_t* ToF_Sensor::__pTaskHandle;
ToF_Sensor*  ToF_Sensor::__ToFSensorPointers[10];
uint8_t ToF_Sensor::__no_of_sensors = 0;

static 	osapi::Mutex mutex;



ToF_Sensor::ToF_SensorMortalThread ToF_Sensor::Thread;

ToF_Sensor* ToF_Sensor::GetSensorPointer(uint8_t Sensor)
{
	if(Sensor >= GetNoOfSensors())
	{
		return NULL;
	}
	return __ToFSensorPointers[Sensor];
}

uint8_t ToF_Sensor::GetNoOfSensors(void)
{
	return __no_of_sensors;
}

ToF_Sensor* ToF_Sensor::GetSensorPointerFromPool()
{
	static uint8_t currentpointer = 0;
	if(currentpointer >= GetNoOfSensors()){
		return NULL;
	}
	else{
		currentpointer++;
		return GetSensorPointer(currentpointer - 1);
	}
}

ToF_Sensor::ToF_Sensor(e_ToF_Type type, e_ToF_Position position, CommManager *comm) :
		ToF_Type { type }, __CommunicationManager { comm }, __pos { position } {
	__ToFSensorPointers[__no_of_sensors] = this;
	__no_of_sensors++;
}


bool ToF_Sensor::CheckInitializationCplt(void)
{
	return Thread.isInitCompleted();
}


void ToF_Sensor::StartSensorTask(void) {
	Thread.run();
}

void ToF_Sensor::EXTI_Callback_func(uint16_t pin)
{
	ToF_Sensor* SensorObj;
		for (uint8_t i = 0; i < __no_of_sensors; i++)
		{
			SensorObj = __ToFSensorPointers[i];
			if(SensorObj->GetSensorITPin() == pin)
			{
				if(SensorObj->CheckSensorStatus() == TOF_STATE_OK){
					SensorObj->GetRangingData();
				}
			}
		}
}

ToF_Sensor::~ToF_Sensor() {
	// TODO Auto-generated destructor stub
}

void ToF_Sensor::__ToFSensorThread(void *pvParameters) {
	//nothing to do at this time
}


void ToF_Sensor::ToF_SensorMortalThread::begin()
{
	BaseType_t xReturned;
	ToF_Sensor* SensorObj;
	TaskHandle_t xHandle[NO_OF_WORKERS] = {NULL};
	SemaphoreHandle_t xSemaphore[NO_OF_WORKERS] = {NULL};
	uint8_t worker = 0;
	//Set mutex and disable sensors
	for (uint8_t i = 0; i < ToF_Sensor::GetNoOfSensors(); i++)
	{
		SensorObj = ToF_Sensor::GetSensorPointer(i);
		SensorObj->SetMutex(&mutex);
		SensorObj->DisableSensorComm();
	}
	//Set I2C addresses
	for (uint8_t i = 0; i < ToF_Sensor::GetNoOfSensors(); i++)
	{
		SensorObj = ToF_Sensor::GetSensorPointer(i);
		SensorObj->SetI2CAddress();
	}
	//Spawn NO_OF_WORKERS Workers
	for(worker = 0; worker < NO_OF_WORKERS; worker++){
		if(worker >= GetNoOfSensors())
		{
			break; //each sensor has its own task
		}
		xSemaphore[worker] = xSemaphoreCreateBinary();
		if(xSemaphore[worker] == NULL){
			if(worker == 0){
				Error_Handler(); //no semaphores spawned
			}
			break;
		}
		xReturned = xTaskCreate(ToF_SensorMortalThread::InitWorkerThread,
								"NAME",
								512,
								(void*) xSemaphore[worker],
								tskIDLE_PRIORITY,
								&xHandle[worker]);
		if(xReturned != pdPASS)
		{
			if(worker == 0){
				Error_Handler(); //no workers spawned
			}
			vSemaphoreDelete(xSemaphore[worker]); //delete unused semaphore after fail
			break;
		}
	}

	//wait till all workers are deleted, and if not just wait

	for(uint8_t worketstate = 0; worketstate < worker; worketstate++)
	{
		xSemaphoreTake(xSemaphore[worketstate], -1); 	// wait till task completed
		vSemaphoreDelete(xSemaphore[worketstate]); 		//delete its semaphore
	}

}

void ToF_Sensor::ToF_SensorMortalThread::loop()
	{
		ToF_Sensor* SensorObj;
		for (uint8_t i = 0; i < __no_of_sensors; i++)
		{
			SensorObj = ToF_Sensor::GetSensorPointer(i);
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
				case TOF_STATE_DATA_RDY: {
				}
					break;
				default: {

				}
					break;
			}
		}
		this->sleep(5);
	}

void ToF_Sensor::ToF_SensorMortalThread::InitWorkerThread(void *pvParametes)
{
	SemaphoreHandle_t xSemaphore = reinterpret_cast<SemaphoreHandle_t>(pvParametes);
	while(true){
		ToF_Sensor* SensorObj = GetSensorPointerFromPool();
		if(SensorObj == NULL)
		{
			break;
		}
		SensorObj->SensorInit();
	}
	xSemaphoreGive(xSemaphore);
	vTaskDelete(NULL);
}

void ToF_Sensor::ToF_SensorMortalThread::end()
	{
		Error_Handler();
	}
