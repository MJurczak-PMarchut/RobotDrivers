/*
 * L9960T.cpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Mateusz
 */

#include "L9960T.hpp"


L9960T::L9960T(MotorSideTypeDef side, SPI_HandleTypeDef *hspi, CommManager *CommunicationManager,
	void (*pRxCompletedCB)(struct MessageInfoTypeDef* MsgInfo), void (*pTxCompletedCB)(struct MessageInfoTypeDef* MsgInfo)):
	__side{side},
	__hspi{hspi},
	__CommunicationManager{CommunicationManager},
	__pRxCompletedCB{pRxCompletedCB},
	__pTxCompletedCB{pTxCompletedCB}
{
	if((side == MOTOR_LEFT) && ((__Instantiated_sides & (1 << MOTOR_LEFT)) == 0))
	{
		this->__IN1_PWM_PIN = MD_IN1_PWM_A_Pin;
		this->__IN2_DIR_PIN = MD_IN2_DIR_A_Pin;
		this->__IN1_PWM_PORT = MD_IN1_PWM_A_GPIO_Port;
		this->__IN2_DIR_PORT = MD_IN2_DIR_A_GPIO_Port;
		this->__Instantiated_sides |= (1 << MOTOR_LEFT);
		this->__Instantiated_sides |= MOTOR_LEFT_NDIS_ENABLED;
		this->__CS_Pin = MD_CS_1_Pin;
		this->__CS_Port = MD_CS_1_GPIO_Port;
#ifdef LEFT_MOTOR_INVERT_DIRECTION
		this->__Direction = GPIO_PIN_RESET;
#else
		this->__Direction = GPIO_PIN_SET;
#endif
	}
	else if ((side == MOTOR_RIGHT) && ((__Instantiated_sides & (1 << MOTOR_RIGHT)) == 0))
	{
		this->__IN1_PWM_PIN = MD_IN1_PWM_B_Pin;
		this->__IN2_DIR_PIN = MD_IN2_DIR_B_Pin;
		this->__IN1_PWM_PORT = MD_IN1_PWM_B_GPIO_Port;
		this->__IN2_DIR_PORT = MD_IN2_DIR_B_GPIO_Port;
		this->__Instantiated_sides |= (1 << MOTOR_RIGHT);
		this->__Instantiated_sides |= MOTOR_RIGHT_NDIS_ENABLED;
		this->__CS_Pin = MD_CS_2_Pin;
		this->__CS_Port = MD_CS_2_GPIO_Port;
#ifdef RIGHT_MOTOR_INVERT_DIRECTION
		this->__Direction = GPIO_PIN_RESET;
#else
		this->__Direction = GPIO_PIN_SET;
#endif
	}
	HAL_GPIO_WritePin(MD_NDIS_GPIO_Port, MD_NDIS_Pin, GPIO_PIN_SET);
	this->Disable();
	if(this->Init() != HAL_OK)
	{
		Error_Handler();
	}
}

HAL_StatusTypeDef L9960T::Init(void)
{
	uint16_t Message = 0;
	uint8_t address = 0;
	MessageInfoTypeDef MsgInfo = {0};
	static uint8_t MessageCounter = 0;
	MsgInfo.GPIO_PIN = this->__CS_Pin;
	MsgInfo.GPIOx = this->__CS_Port;
	MsgInfo.context = (1 << this->__side) |
					  (INIT_SEQUENCE_CONTEXT << CONTEXT_OFFSET) | //Tis a Init sequence!
			          (MessageCounter << 8);
	MsgInfo.eCommType = COMM_INT_SPI_TXRX;
	MsgInfo.len = 2;
//	MsgInfo.pRxCompletedCB = this->__pRxCompletedCB;
	MsgInfo.pTxCompletedCB = this->__pTxCompletedCB;
	MsgInfo.pRxData = this->pRxData;
	switch(MessageCounter)
	{
		case 0://Ask for ASIC number
			address = ELECTRONIC_ID_REQUEST_ADDR;
			Message = ELECTRONIC_ID_REQUEST_MSG;
			Message = (address << ADDRESS_OFFSET) | (Message << MESSAGE_OFFSET);
			Message |= (__builtin_parity(Message) & 1);
			this->__CommunicationManager->PushCommRequestIntoQueue(&MsgInfo);
			break;
		case 1://Answer for the ASIC number will come now
			//TODO: For now it will ask again and we will check the number
			address = ELECTRONIC_ID_REQUEST_ADDR;
			Message = ELECTRONIC_ID_REQUEST_MSG;
			Message = (address << ADDRESS_OFFSET) | (Message << MESSAGE_OFFSET);
			Message |= (__builtin_parity(Message) & 1);
			MsgInfo.context = (1 << this->__side) |
							  (0 << CONTEXT_OFFSET) | //Tis a Stop sequence!
					          (MessageCounter << 8);
			this->__CommunicationManager->PushCommRequestIntoQueue(&MsgInfo);
			break;
	}
	MessageCounter++;
	return HAL_OK;
}

HAL_StatusTypeDef L9960T::AttachPWMTimerAndChannel(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	assert_param(IS_TIM_CHANNELS(Channel));
	this->__htim = htim;
	this->__Channel = Channel;
	return HAL_OK;
}

HAL_StatusTypeDef L9960T::SetMotorPowerPWM(uint16_t PowerPWM)
{
	if(((1 << this->__side) << MOTOR_NDIS_OFFSET) & this->__Instantiated_sides)
	{
		return HAL_ERROR;
	}
	else if(PowerPWM < 1000)
	{
		__HAL_TIM_SET_COMPARE(this->__htim, this->__Channel, PowerPWM);
		return HAL_OK;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef L9960T::SetMotorDirection(MotorDirectionTypeDef Dir)
{

	GPIO_PinState Pin_STATE;
	if((Dir == MOTOR_DIR_FORWARD) || (Dir == MOTOR_DIR_BACKWARD))
	{
		Pin_STATE = (GPIO_PinState)((Dir ^ __Direction) & 0x01); //Do xor and take last bit
		HAL_GPIO_WritePin(__IN2_DIR_PORT, __IN2_DIR_PIN, Pin_STATE);
		return HAL_OK;
	}
	return HAL_ERROR;

}

HAL_StatusTypeDef L9960T::EmergencyStop(void)
{
	__HAL_TIM_SET_COMPARE(this->__htim, this->__Channel, 0);
	return this->Disable();
}

HAL_StatusTypeDef L9960T::Disable(void)
{
	SetMotorPowerPWM(0);
	this->__Instantiated_sides |=  ((1 << this->__side) << MOTOR_NDIS_OFFSET);
	HAL_GPIO_WritePin(this->__DIS_PORT, this->__DIS_PIN, GPIO_PIN_SET);
	return HAL_OK;
}

HAL_StatusTypeDef L9960T::Enable(void)
{
	if(((1 << this->__side) << MOTOR_NDIS_OFFSET) & this->__Instantiated_sides)
	{
		this->__Instantiated_sides &= ~ ((1 << this->__side) << MOTOR_NDIS_OFFSET);
		HAL_GPIO_WritePin(this->__DIS_PORT, this->__DIS_PIN, GPIO_PIN_RESET);
	}
	HAL_Delay(1); 	//Wait 1ms to satisfy wait on dis condition (1us is enough but too much work)
	return HAL_OK;
}

