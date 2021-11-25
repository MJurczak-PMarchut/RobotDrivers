/*
 * L9960T.hpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Mateusz
 */

#ifndef SUMODRIVERS_L9960T_L9960T_HPP_
#define SUMODRIVERS_L9960T_L9960T_HPP_
#include "MotorControl.hpp"
#include "CommManager.hpp"

struct L9960TWrite {
	//restart trigger
	bool SWreset_bit1;
	bool SWreset_bit0;
	bool HWSC_LBIST_Trigger;
	bool ConfigCC;
	//configuration 1
	bool CL_bit1;
	bool CL_bit0;
	bool NOSR;
	bool ISR;
	bool VSR;
	bool TDIAG1_bit2;
	bool TDIAG1_bit1;
	bool TDIAG1_bit0;
	bool TSW_low_current;
	bool DIAG_CLR_EN;
	//configuration 2
	bool in1_in2_if;
	bool OTsd_thr_var_bit2;
	bool OTsd_thr_var_bit1;
	bool OTsd_thr_var_bit0;
	bool OTwarn_thr_var_bit2;
	bool OTwarn_thr_var_bit1;
	bool OTwarn_thr_var_bit0;
	bool UV_PROT_EN;
	bool NSPREAD;
	bool UV_WIN;
	//configuration 3
	bool WL_MODE;
	bool TVVL_bit3;
	bool TVVL_bit2;
	bool TVVL_bit1;
	bool TVVL_bit0;
	bool OTWARN_TSEC_EN;
	//configuration 4
	bool TDSR;
	bool OL_ON;
};

struct L9960TRead {
	//Configuration request 1
	bool CL_echo_bit1;
	bool CL_echo_bit0;
	bool NOSR_echo;
	bool ISR_echo;
	bool VSR_echo;
	bool TDIAG1_echo_bit2;
	bool TDIAG1_echo_bit1;
	bool TDIAG1_echo_bit0;
	bool TSW_low_current_echo;
	bool DIAG_CLR_EN;
	//Configuration request 2
	bool in1_in2_if_echo;
	bool in1_in2_if_latch;
	bool OT_sd_thr_var_echo_bit2;
	bool OT_sd_thr_var_echo_bit1;
	bool OT_sd_thr_var_echo_bit0;
	bool OTwarn_thr_var_echo_bit2;
	bool OTwarn_thr_var_echo_bit1;
	bool OTwarn_thr_var_echo_bit0;
	bool UV_PROT_EN_echo;
	bool NSPREAD_echo;
	bool UV_WIN_echo;
	//Configuration request 3
	bool WLMODE_echo;
	bool TVVL_echo_bit3;
	bool TVVL_echo_bit2;
	bool TVVL_echo_bit1;
	bool TVVL_echo_bit0;
	bool OTWARN_TSEC_EN_echo;
	//configuration request 4
	bool TDSR_echo;
	//Configuration request 5
	bool POR_status;
	bool config_CC_status_echo;
	bool CC_latch_state;

	//states request 1
	bool NDIS_status;
	bool DIS_status;
	bool BRIDGE_EN;
	bool HWSC_LBIST_status_bit2;
	bool HWSC_LBIST_status_bit1;
	bool HWSC_LBIST_status_bit0;
	bool VPS_UV_REG;
	bool NGFAIL;
	bool ILIM_REG;
	bool VDO_OV_REG;
	bool VDO_UV_REG;
	bool VPS_UV;
	//states request 2
	bool OTSDcnt_bit5;
	bool OTSDcnt_bit4;
	bool OTSDcnt_bit3;
	bool OTSDcnt_bit2;
	bool OTSDcnt_bit1;
	bool OTSDcnt_bit0;
	bool OTWARN;
	bool OTWARN_REG;
	bool NOTSD;
	bool NOTSD_REG;
	bool OL_ON_STATUS_bit1;
	bool OL_ON_STATUS_bit0;
	//states request 3
	bool UV_CNT_REACHED;
	bool Error_count_bit3;
	bool Error_count_bit2;
	bool Error_count_bit1;
	bool Error_count_bit0;
	//OFF STATE diagnosis
	bool DIAG_OFF_bit2;
	bool DIAG_OFF_bit1;
	bool DIAG_OFF_bit0;
	//component traceability number request
	bool I[24];
	//elecrtonic id request
	bool ASIC_name[10];
	bool ASSP;
	//siicon version request
	bool Silicon_request[4];
	//Logic HW version request
	bool code_version[8];
};

extern uint16_t Compose16BitNumber(bool tab[]);
extern void Decompose16BitNumber(bool *tab[], uint16_t number);

typedef enum {CURRENT_RANGE_0 = 0, CURRENT_RANGE_1 = 1, CURRENT_RANGE_2 = 2, CURRENT_RANGE_3 = 3} L9960T_CurrentRange;

class L9960T : protected MCInterface{
	public:
		L9960T(MotorSideTypeDef side, SPI_HandleTypeDef *hspi,  uint16_t CS_Pin, GPIO_TypeDef *CS_Port, CommManager *CommunicationManager);
		L9960T(MotorSideTypeDef side);
		HAL_StatusTypeDef AttachPWMTimerAndChannel(TIM_HandleTypeDef *htim, uint32_t Channel);
		HAL_StatusTypeDef SetMotorPowerPWM(uint16_t PowerPWM);
		HAL_StatusTypeDef SetMotorDirection(MotorDirectionTypeDef Dir);
		HAL_StatusTypeDef SetMaxCurrent(L9960T_CurrentRange CurrentRange);
		HAL_StatusTypeDef Disable();
		HAL_StatusTypeDef EmergencyStop(void);
		HAL_StatusTypeDef CheckIfControllerInitializedOk(void);

		uint16_t SPI_RX;
		uint16_t SPI_TX;
		L9960TWrite RrgisterWrite;
		L9960TRead RegisterRead;

		HAL_StatusTypeDef ComposeSPIMess(uint16_t *mess, string name);
		HAL_StatusTypeDef AnalizeSPIMess(uint16_t *mess);
		HAL_StatusTypeDef SPISendReceive();
	private:
		CommManager *__CommunicationManager;
		MotorSideTypeDef __side;
		SPI_HandleTypeDef *__hspi;
		uint16_t __CS_Pin;
		uint16_t __IN1_PWM_PIN;
		uint16_t __IN2_DIR_PIN;
		GPIO_TypeDef *__CS_Port;
		GPIO_TypeDef *__IN1_PWM_PORT;
		GPIO_TypeDef *__IN2_DIR_PORT;
		GPIO_PinState __Direction;
		TIM_HandleTypeDef *__htim;
		uint32_t __Channel;

};


#endif /* SUMODRIVERS_L9960T_L9960T_HPP_ */
