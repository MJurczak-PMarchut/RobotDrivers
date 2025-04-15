/*
 * lsm6dso.hpp
 *
 *  Created on: Apr 13, 2025
 *      Author: jurcz
 */

#ifndef LSM6DSO_LSM6DSO_HPP_
#define LSM6DSO_LSM6DSO_HPP_

#include "CommManager.hpp"
#include "lsm6dso_reg.h"

#ifndef USES_RTOS
#error "This lib cannot function without RTOS!"
#endif

class LSM6DSO{
public:
	LSM6DSO(CommManager *CommunicationManager, SPI_HandleTypeDef *hspi);
	HAL_StatusTypeDef Init(void);
	void InterruptCallback(uint16_t InterruptPin);
	void CalibrateOrientation(void);
protected:
private:
	SPI_HandleTypeDef *_hspi;
	CommManager *__CommManager;
	stmdev_ctx_t dev_ctx;
//	uint16_t _LSM6DSO_nCS_PIN;
//	GPIO_TypeDef *_LSM6DSO_nCS_Port;
};


#endif /* LSM6DSO_LSM6DSO_HPP_ */
