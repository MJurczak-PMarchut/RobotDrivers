/*
 * CommManager.cpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Mateusz
 */
#include "CommManager.hpp"

#include "vector"

#ifndef NO_COMMUNICATION_INTERFACE_DEFINED
#ifndef MAX_MESSAGE_NO_IN_QUEUE
#define MAX_MESSAGE_NO_IN_QUEUE 5
#endif


#define VECTOR_NOT_FOUND 0xFF

CommManager::CommManager()
{

}

HAL_StatusTypeDef CommManager::PushCommRequestIntoQueue(MessageInfoTypeDef<SPI_HandleTypeDef> *MsgInfo)
{
	for(auto instance : _comm_SPI_vect)
	{
		if(instance->CheckIfSameInstance(&MsgInfo->IntHandle) == HAL_OK)
		{
			return instance->PushMessageIntoQueue(MsgInfo);
		}
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef CommManager::PushCommRequestIntoQueue(MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo)
{
	for(auto instance : _comm_I2C_vect)
	{
		if(instance->CheckIfSameInstance(&MsgInfo->IntHandle) == HAL_OK)
		{
			return instance->PushMessageIntoQueue(MsgInfo);
		}
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef CommManager::PushCommRequestIntoQueue(MessageInfoTypeDef<UART_HandleTypeDef> *MsgInfo)
{
	for(auto instance : _comm_UART_vect)
	{
		if(instance->CheckIfSameInstance(&MsgInfo->IntHandle) == HAL_OK)
		{
			return instance->PushMessageIntoQueue(MsgInfo);
		}
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef CommManager::MsgReceivedCB(UART_HandleTypeDef *huart, uint16_t len)
{
	return HAL_ERROR;
}

HAL_StatusTypeDef CommManager::MsgReceivedCB(UART_HandleTypeDef *huart)
{
//	return __MsgReceivedCB(huart, &this->__huartQueueVect);
	return HAL_ERROR;
}

HAL_StatusTypeDef CommManager::MsgReceivedCB(SPI_HandleTypeDef *hspi)
{
//	return __MsgReceivedCB(hspi, &this->__hspiQueueVect);
	return HAL_ERROR;
}


HAL_StatusTypeDef CommManager::MsgReceivedCB(I2C_HandleTypeDef *hi2c)
{
//	return __MsgReceivedCB(hi2c, &this->__hi2cQueueVect);
	return HAL_ERROR;
}

template<typename Handle, typename QueueVectTD>
HAL_StatusTypeDef CommManager::__MsgReceivedCB(Handle *IntHandle, QueueVectTD *Queue)
{
	//Find message
	uint8_t u8Iter;
	MessageInfoTypeDef<Handle> Msg;
	//Find peripheral
	for(u8Iter = 0; u8Iter < (*Queue).size(); u8Iter++)
	{
		if(IntHandle->Instance == (*Queue)[u8Iter].handle->Instance)
		{
			//Call CB functions if any
			Msg = (*Queue)[u8Iter].MsgInfo.front();
			//remove item from queue
			(*Queue)[u8Iter].MsgInfo.pop();
			//Clear CS Pin if any
			if(Msg.GPIOx != 0){
				HAL_GPIO_WritePin(Msg.GPIOx, Msg.GPIO_PIN, CSn_INACTIVE_PIN_STATE);
			}
			//Set transaction status
			if(Msg.TransactionStatus != 0){
				*Msg.TransactionStatus = HAL_OK;
			}
			switch(Msg.eCommType)
			{
				case COMM_INT_RX:
				{
					if(Msg.pRxCompletedCB != 0)
						{
							Msg.pRxCompletedCB(&Msg);
						}
				}
					break;
				case COMM_INT_TX:
				{
					if(Msg.pTxCompletedCB != 0)
						{
							Msg.pTxCompletedCB(&Msg);
						}
				}
					break;
				case COMM_INT_TXRX:
				{
					if(Msg.pRxCompletedCB != 0)
						{
							Msg.pRxCompletedCB(&Msg);
						}
					if(Msg.pTxCompletedCB != 0)
						{
							Msg.pTxCompletedCB(&Msg);
						}
				}
					break;
				default:
					break;
			}
			if(Msg.pCB != 0)
			{
				Msg.pCB(&Msg);
			}
			 return __CheckForNextCommRequestAndStart(IntHandle, Queue);
		}
	}
	return HAL_ERROR;
}

template<typename T>
HAL_StatusTypeDef CommManager::__CheckAndSetCSPins(MessageInfoTypeDef<T> *MsgInfo, uint8_t VectorIndex)
{
	return HAL_ERROR;
}

template<typename queue, typename T>
HAL_StatusTypeDef CommManager::__CheckAndSetCSPinsGeneric(queue *VectQueue, uint8_t VectorIndex, MessageInfoTypeDef<T> *MsgInfo)
{

		//Check if pin is already set
		if(((*VectQueue)[VectorIndex].GPIO_PIN == MsgInfo->GPIO_PIN) && ((*VectQueue)[VectorIndex].GPIOx == MsgInfo->GPIOx))
		{
			//if so return
			return HAL_OK;
		}
		else
		{
			//if not reset pin states and set new ones
			HAL_GPIO_WritePin((*VectQueue)[VectorIndex].GPIOx, (*VectQueue)[VectorIndex].GPIO_PIN, CSn_INACTIVE_PIN_STATE);
			if(MsgInfo->GPIOx == 0)
			{
				return HAL_OK;
			}
			HAL_GPIO_WritePin(MsgInfo->GPIOx, MsgInfo->GPIO_PIN, CSn_ACTIVE_PIN_STATE);
			return HAL_OK;
		}

}

#endif
