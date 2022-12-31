/*
 * CommI2C.cpp
 *
 *  Created on: 30 gru 2022
 *      Author: Paulina
 */

#include "CommI2C.hpp"


CommI2C::CommI2C(I2C_HandleTypeDef *hint, DMA_HandleTypeDef *hdma)
:CommBaseClass(hint, hdma)
{}

HAL_StatusTypeDef CommI2C::__CheckIfFreeAndSendRecv(MessageInfoTypeDef<I2C_HandleTypeDef> *MsgInfo)
{
	HAL_StatusTypeDef ret = HAL_BUSY;
	if(_Handle->State != HAL_I2C_STATE_READY)
	{
		return ret;
	}
	if(_commType != COMM_DUMMY)
	{
		__CheckAndSetCSPinsGeneric(MsgInfo);
	}
	switch(MsgInfo->eCommType)
	{
		case COMM_INT_RX:
		{
			if(_commType == COMM_DMA)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_I2C_Master_Receive_DMA(_Handle,MsgInfo->I2C_Addr, MsgInfo->pRxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(hdmaRx, DMA_IT_HT);
			}
			else if(_commType == COMM_INTERRUPT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_I2C_Master_Receive_IT(_Handle,MsgInfo->I2C_Addr, MsgInfo->pRxData, MsgInfo->len);
			}
			else if(_commType == COMM_WAIT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_I2C_Master_Receive(_Handle ,MsgInfo->I2C_Addr, MsgInfo->pRxData, MsgInfo->len, 1000);
			}
			else
			{
				ret = HAL_ERROR;
			}
		}
		break;
		case COMM_INT_TX:
		{
			if(_commType == COMM_DMA)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_I2C_Master_Transmit_DMA(_Handle, MsgInfo->I2C_Addr, MsgInfo->pTxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(hdmaRx, DMA_IT_HT);
			}
			else if(_commType == COMM_INTERRUPT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_I2C_Master_Transmit_IT(_Handle, MsgInfo->I2C_Addr, MsgInfo->pTxData, MsgInfo->len);
			}
			else if(_commType == COMM_WAIT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_I2C_Master_Transmit(_Handle, MsgInfo->I2C_Addr, MsgInfo->pTxData, MsgInfo->len, 1000);
			}
			else
			{
				ret = HAL_ERROR;
			}
		}
		break;
		case COMM_INT_MEM_RX:
		{
			if(_commType == COMM_DMA)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_I2C_Mem_Read_DMA(_Handle, MsgInfo->I2C_Addr, MsgInfo->I2C_MemAddr, I2C_MEMADD_SIZE_16BIT, MsgInfo->pRxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(hdmaRx, DMA_IT_HT);
			}
			else if(_commType == COMM_INTERRUPT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_I2C_Mem_Read_IT(_Handle, MsgInfo->I2C_Addr, MsgInfo->I2C_MemAddr, I2C_MEMADD_SIZE_16BIT, MsgInfo->pRxData, MsgInfo->len);
			}
			else if(_commType == COMM_WAIT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_I2C_Mem_Read(_Handle, MsgInfo->I2C_Addr, MsgInfo->I2C_MemAddr, I2C_MEMADD_SIZE_16BIT, MsgInfo->pRxData, MsgInfo->len, 1000);
			}
			else
			{
				ret = HAL_ERROR;
			}
		}
		break;
		case COMM_INT_MEM_TX:
		{
			if(_commType == COMM_DMA)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_I2C_Mem_Write_DMA(_Handle, MsgInfo->I2C_Addr, MsgInfo->I2C_MemAddr, I2C_MEMADD_SIZE_16BIT, MsgInfo->pTxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(hdmaRx, DMA_IT_HT);
			}
			else if(_commType == COMM_INTERRUPT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_I2C_Mem_Write_IT(_Handle, MsgInfo->I2C_Addr, MsgInfo->I2C_MemAddr, I2C_MEMADD_SIZE_16BIT, MsgInfo->pTxData, MsgInfo->len);
			}
			else if(_commType == COMM_WAIT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_I2C_Mem_Write(_Handle, MsgInfo->I2C_Addr, MsgInfo->I2C_MemAddr, I2C_MEMADD_SIZE_16BIT, MsgInfo->pTxData, MsgInfo->len, 1000);
			}
			else
			{
				ret = HAL_ERROR;
			}
		}
		break;
		default:
			ret = HAL_ERROR;
			break;
	}
	return ret;
}
