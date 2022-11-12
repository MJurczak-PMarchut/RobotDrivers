/*
 * vl53l5cx.cpp
 *
 *  Created on: 29 paź 2022
 *      Author: Paulina
 */

#include "vl53l5cx.hpp"

const static uint8_t __ToFAddr [] = {0x54, 0x56, 0x58, 0x60, 0x62, 0x64};
const static uint16_t __ToFX_SHUT_Pin [] = {XSHUT_3_Pin, XSHUT_3_Pin, XSHUT_3_Pin,
											XSHUT_4_Pin, XSHUT_5_Pin, XSHUT_6_Pin};
static GPIO_TypeDef *__ToFX_SHUT_Port [] = {XSHUT_3_GPIO_Port, XSHUT_3_GPIO_Port,
												 XSHUT_3_GPIO_Port, XSHUT_4_GPIO_Port,
												 XSHUT_5_GPIO_Port, XSHUT_6_GPIO_Port};

uint8_t VL53L5CX::__sensor_nb = 0;
uint8_t VL53L5CX::__sensor_init_tbd = 0;

VL53L5CX::VL53L5CX(e_ToF_Position position, CommManager *comm):
	ToF_Sensor(vl53l5, position, comm)
{
	this->__sensor_index = __sensor_nb;
	__sensor_nb++;
	this->__sensor_conf.platform.__CommunicationManager = comm;
	this->__sensor_conf.platform.address = DEFAULT_ADDR;

	HAL_GPIO_WritePin(__ToFX_SHUT_Port[this->__sensor_index], __ToFX_SHUT_Pin[this->__sensor_index], GPIO_PIN_RESET);
}

/*
 * Initializes Sensors
 * Could probably be made static to initialize all sensors present
 * Or be implemented in parent class
 */
HAL_StatusTypeDef VL53L5CX::SensorInit(void)
{
	HAL_StatusTypeDef ret = HAL_OK;
	while(__sensor_init_tbd != this->__sensor_index)
	{
#ifdef USES_RTOS
		//TODO We could use semaphores if RTOS is being used
		vTaskDelay(10);
#else
		HAL_Delay(10);
#endif
	}
	//Change sensor address and enable communication
	HAL_GPIO_WritePin(__ToFX_SHUT_Port[this->__sensor_index], __ToFX_SHUT_Pin[this->__sensor_index], GPIO_PIN_SET);

	//Set new address
	ret = SetI2CAddress();
	//Init sensor
	ret = (vl53l5cx_init(&(this->__sensor_conf)) == 0)? HAL_OK : HAL_ERROR ;
	//Increment __sensor_init_tbd
	VL53L5CX::__sensor_init_tbd++;
	return ret;
}

HAL_StatusTypeDef VL53L5CX::SetI2CAddress()
{
    uint8_t ret = vl53l5cx_set_i2c_address(&this->__sensor_conf, __ToFAddr[this->__sensor_index]);
	return (ret == 0)? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef VL53L5CX::IsAlive(uint8_t *is_alive)
{
	uint8_t ret = vl53l5cx_is_alive(&this->__sensor_conf, is_alive);
	return (ret == 0)? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef VL53L5CX::StartRanging(void)
{
	uint8_t ret = vl53l5cx_start_ranging(&this->__sensor_conf);
	return (ret == 0)? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef VL53L5CX::GetRangingData(void)
{
	uint8_t ret = vl53l5cx_get_ranging_data(&this->__sensor_conf, &this->result);
	return (ret == 0)? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef VL53L5CX::CheckDataReady(void)
{

}

