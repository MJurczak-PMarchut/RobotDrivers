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
#include "osapi.h"

typedef enum  {vl53l5, vl53l1}e_ToF_Type;

typedef enum  {FRONT_LEFT=2, FRONT_RIGHT, FRONT}e_ToF_Position;
typedef enum {
	TOF_STATE_OK = 0,
	TOF_INIT_NOT_DONE,
	TOF_STATE_INIT_WAIT,
	TOF_STATE_DATA_RDY,
	TOF_STATE_ERROR,
	TOF_STATE_BUSY,
	TOF_STATE_INIT_ONGOING,
	TOF_STATE_NOT_CONNECTED,
	TOF_STATE_COMM_ERROR,
	TOF_READING_DATA
}ToF_Status_t;



class ToF_Sensor {
public:
	ToF_Sensor(e_ToF_Type type, e_ToF_Position position, CommManager *comm);
//	virtual HAL_StatusTypeDef SensorInit(MessageInfoTypeDef* MsgInfo) = 0;
	virtual HAL_StatusTypeDef SensorInit(void) = 0;
	virtual uint16_t GetSensorITPin(void) = 0;
	virtual HAL_StatusTypeDef GetRangingData(void) = 0;
	virtual ToF_Status_t CheckSensorStatus(void) = 0;
	e_ToF_Position getPosition(void) {return __pos;};
	virtual HAL_StatusTypeDef SetI2CAddress(void) = 0;
	virtual HAL_StatusTypeDef __GetData(void) = 0;
	virtual HAL_StatusTypeDef DisableSensorComm(void)=0;
	static uint8_t GetNoOfSensors(void);
	static bool CheckInitializationCplt(void);
	static ToF_Sensor* GetSensorPointer(uint8_t);
	static void EXTI_Callback_func(uint16_t pin);
	static void StartSensorTask(void);
	virtual ~ToF_Sensor();


protected:

	e_ToF_Type ToF_Type;
	CommManager *__CommunicationManager;
	e_ToF_Position __pos;
	static uint8_t __no_of_sensors;
	virtual void SetMutex(osapi::Mutex *pmutex) = 0;

private:

	class ToF_SensorMortalThread : public MortalThread
	{
	public:
		ToF_SensorMortalThread() : MortalThread(tskIDLE_PRIORITY, 1024) {
		}
	private:
		static void InitWorkerThread(void *pvParametes);
		virtual void begin();
		virtual void loop();
		virtual void end();
	};
	static ToF_Sensor* GetSensorPointerFromPool();
	static void __ToFSensorThread(void  *pvParameters);
	static ToF_Sensor*  __ToFSensorPointers[10];
	static TaskHandle_t *__pTaskHandle;
	static ToF_SensorMortalThread Thread;
};

#endif /* TOF_SENSORS_COMMON_TOFSENSOR_HPP_ */
