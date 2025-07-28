/*
 * L9960T.hpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Mateusz
 */

#ifndef SUMODRIVERS_L9960T_L9960T_HPP_
#define SUMODRIVERS_L9960T_L9960T_HPP_
#include "L9960_regs.hpp"
#include "MotorControl.hpp"
#include "RobotSpecificDefines.hpp"

#ifdef USES_RTOS
#include "osapi.h"
#endif

#define INIT_SEQUENCE_CONTEXT 3
#define STATUS_CHECK_CONTEXT 6

typedef enum {CURRENT_RANGE_0 = 0, CURRENT_RANGE_1 = 1, CURRENT_RANGE_2 = 2, CURRENT_RANGE_3 = 3} L9960T_CurrentRange;

typedef struct {
	uint16_t power;
	MotorDirectionTypeDef dir;
} L9660_SCS_t;

class L9960T : public MCInterface{
	public:
		L9960T(MotorSideTypeDef side, SPI_HandleTypeDef *hspi,
				CommManager *CommunicationManager, uint32_t Channel,
				TIM_HandleTypeDef *htim, bool inverted_pwm=false, bool use_sw_pwm=false, bool linerize_change=true);
		HAL_StatusTypeDef AttachPWMTimerAndChannel(TIM_HandleTypeDef *htim, uint32_t Channel);
		HAL_StatusTypeDef SetMotorPowerPWM(uint16_t PowerPWM);
		HAL_StatusTypeDef SetMotorPower(float Power);
		HAL_StatusTypeDef SetMotorDirection(MotorDirectionTypeDef Dir);
		HAL_StatusTypeDef SetMaxCurrent(L9960T_CurrentRange CurrentRange);
		HAL_StatusTypeDef Disable(void);
		HAL_StatusTypeDef Enable(void);
		HAL_StatusTypeDef EmergencyStop(void);
		HAL_StatusTypeDef CheckIfControllerInitializedOk(void);
		HAL_StatusTypeDef StartPWM(void);
		void SoftPWMCB_period();
		void SoftPWMCB_pulse();
		void Init(MessageInfoTypeDef<SPI>* MsgInfo);
	private:
		void __delay_ms(uint32_t TimeMs);
		void __delay_us(uint32_t TimeUs);
		SPI_HandleTypeDef *__hspi;
		CommManager *__CommunicationManager;
		uint16_t __CS_Pin;
		uint16_t __IN1_PWM_PIN;
		uint16_t __IN2_DIR_PIN;
		uint16_t __DIS_PIN;
		uint16_t _prev_context;
		GPIO_TypeDef *__CS_Port;
		GPIO_TypeDef *__IN1_PWM_PORT;
		GPIO_TypeDef *__IN2_DIR_PORT;
		GPIO_TypeDef *__DIS_PORT;
		GPIO_PinState __Direction;
		TIM_HandleTypeDef *__htim;
		uint32_t __Channel;
		uint8_t pRxData[2];
		uint8_t pTxData[2];
		uint16_t _status_regs[3] = {0};
		uint16_t __powerPWM;
		uint8_t __InitMessageID;
		L9660_SCS_t _L9660_SCS[20] = {0};
		bool __inverted_pwm;
		bool __use_sw_pwm;
		bool __linerize_change;
		int8_t SCS_index;

#ifdef USES_RTOS
public:
		HAL_StatusTypeDef CheckControllerState(void);

private:
		std::function<void(MessageInfoTypeDef<SPI> *MsgInfo)> _CallbackFunc;
		void _ControllerStateCB(MessageInfoTypeDef<SPI>* MsgInfo);
		StaticSemaphore_t _pxSemphrMemory;
		SemaphoreHandle_t _StatusSemaphore;
#endif

};


#endif /* SUMODRIVERS_L9960T_L9960T_HPP_ */
