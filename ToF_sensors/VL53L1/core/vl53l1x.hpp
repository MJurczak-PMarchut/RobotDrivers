/*
 * vl53l1x.h
 *
 *  Created on: Nov 16, 2021
 *      Author: paulina
 */

#ifndef TOF_SENSORS_CORE_VL53L1X_H_
#define TOF_SENSORS_CORE_VL53L1X_H_

#include "VL53L1X_api.hpp"
#include "../../Configuration.h"
#include "CommManager.hpp"
#include "ToFSensor.hpp"

#define TOF_DEFAULT_ADDRESS 0x52
#define TIMING_BUDGET_MS         100U
#define MAX_TOF_NUMBER           6U

#define TOF0_Addr                0x54
#define TOF1_Addr                0x56
#define TOF2_Addr                0x58
#define TOF3_Addr                0x5A
#define TOF4_Addr                0x5C
#define TOF5_Addr                0x5E

#define TOF0				     0U
//#define TOF1				     1U
//#define TOF2				     2U
//#define TOF3				     3U
//#define TOF4				     4U
#define TOF5				     5U


//typedef struct
//{
//	bool Active;
//	uint8_t Address;
//	uint16_t GPIO_PIN;
//	GPIO_TypeDef *GPIO_GPIOx;
//	uint16_t XSHUT_PIN;
//	GPIO_TypeDef *XSHUT_GPIOx;
//}VL53L1X_Device;

class VL53L1X: public ToF_Sensor{
public:
	VL53L1X(e_ToF_Position position, CommManager *comm, I2C_HandleTypeDef *hi2c1);
	HAL_StatusTypeDef SetI2CAddress();
	ToF_Status_t CheckSensorStatus(void);
	~VL53L1X(void){};
protected:
	HAL_StatusTypeDef SensorInit(void);
private:
	I2C_HandleTypeDef *__hi2c1;
	static uint8_t __sensor_nb;
	uint8_t __sensor_index;
	TickType_t __wait_until_tick;
	ToF_Status_t __Status;
	uint32_t __data_count;
	uint16_t __address = TOF_DEFAULT_ADDRESS;
};



#endif /* TOF_SENSORS_CORE_VL53L1X_H_ */
