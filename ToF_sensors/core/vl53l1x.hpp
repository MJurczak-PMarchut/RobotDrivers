/*
 * vl53l1x.h
 *
 *  Created on: Nov 16, 2021
 *      Author: paulina
 */

#ifndef TOF_SENSORS_CORE_VL53L1X_H_
#define TOF_SENSORS_CORE_VL53L1X_H_

#include <core/VL53L1X_api.hpp>
#include "../../Configuration.h"
#include "../../CommManager/CommManager.hpp"

#define TOF_DEFAULT_ADDRESS 0x52

#define MAX_TOF_NUMBER           6U

#define TOF0_Addr                0x54
#define TOF1_Addr                0x56
#define TOF2_Addr                0x58
#define TOF3_Addr                0x5A
#define TOF4_Addr                0x5C
#define TOF5_Addr                0x5E

#define TOF0				     0U
#define TOF1				     1U
#define TOF2				     2U
#define TOF3				     3U
#define TOF4				     4U
#define TOF5				     5U


typedef struct
{
	bool Active;
	uint8_t Address;
	uint16_t GPIO_PIN;
	GPIO_TypeDef *GPIO_GPIOx;
	uint16_t XSHUT_PIN;
	GPIO_TypeDef *XSHUT_GPIOx;
}VL53L1X_Device;

class VL53L1X
{
	public:
		VL53L1X(I2C_HandleTypeDef *hi2c);
		~VL53L1X(void);
		VL53L1X_ERROR SetSensorPins(uint8_t sensor, uint16_t GPIO_PIN, GPIO_TypeDef *GPIO_GPIOx, uint16_t XSHUT_PIN, GPIO_TypeDef *XSHUT_GPIOx);
		VL53L1X_ERROR InitAllSensors(void);
		VL53L1X_ERROR InitSensor(uint8_t sensor);
		VL53L1X_ERROR StartAllSensors(void);
		VL53L1X_ERROR StartSensor(uint8_t sensor);
		VL53L1X_ERROR SetDistanceMode(void);
	private:
		VL53L1X_ERROR SetSensorAddress(uint8_t sensor);
		MessageInfoTypeDef _MessageInfo;
		VL53L1X_Device _Devices[MAX_TOF_NUMBER];
};



#endif /* TOF_SENSORS_CORE_VL53L1X_H_ */
