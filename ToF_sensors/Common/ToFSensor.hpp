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
typedef enum {
	TOF_STATE_OK = 0,
	TOF_INIT_NOT_DONE,
	TOF_STATE_INIT_WAIT,
	TOF_STATE_ERROR,
	TOF_STATE_BUSY,
	TOF_STATE_INIT_ONGOING,
	TOF_STATE_NOT_CONNECTED,
	TOF_STATE_COMM_ERROR
}ToF_Status_t;

class ToF_Sensor {
public:
	ToF_Sensor(e_ToF_Type type, e_ToF_Position position, CommManager *comm);
	virtual HAL_StatusTypeDef SensorInit(MessageInfoTypeDef* MsgInfo) = 0;
	virtual ToF_Status_t CheckSensorStatus(void) = 0;
	e_ToF_Position getPosition(void) {return __pos;};
	virtual HAL_StatusTypeDef SetI2CAddress(void) = 0;
	static void StartSensorTask(void);
	virtual ~ToF_Sensor();

protected:
	e_ToF_Type ToF_Type;
	CommManager *__CommunicationManager;
	e_ToF_Position __pos;

private:
	static std::vector<ToF_Sensor*>  __ToFSensorPointer;
	static TaskHandle_t *__pTaskHandle;
	static void __ToFSensorThread(void  *pvParameters);

};

#endif /* TOF_SENSORS_COMMON_TOFSENSOR_HPP_ */
