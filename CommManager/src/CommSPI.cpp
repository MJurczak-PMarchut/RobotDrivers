/*
 * CommSPI.cpp
 *
 *  Created on: 30 gru 2022
 *      Author: Paulina
 */

#include "CommSPI.hpp"

CommSPI::CommSPI(SPI_HandleTypeDef *hint, DMA_HandleTypeDef *hdma, CommModeTypeDef CommMode)
:CommBaseClass(hint, hdma, CommMode)
{}

HAL_StatusTypeDef CommSPI::__CheckIfInterfaceFree(MessageInfoTypeDef<SPI_HandleTypeDef> *MsgInfo)
{
	HAL_StatusTypeDef ret = HAL_BUSY;
	if(_Handle->State != HAL_SPI_STATE_READY)
	{
		return ret;
	}
	return HAL_OK;
}

HAL_StatusTypeDef CommSPI::__CheckIfFreeAndSendRecv(MessageInfoTypeDef<SPI_HandleTypeDef> *MsgInfo)
{
	HAL_StatusTypeDef ret =HAL_BUSY;
	if(__CheckIfInterfaceFree(MsgInfo) != HAL_OK)
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
				ret = HAL_SPI_Receive_DMA(_Handle, MsgInfo->pRxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(hdmaRx, DMA_IT_HT);
			}
			else if(_commType == COMM_INTERRUPT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_SPI_Receive_IT(_Handle, MsgInfo->pRxData, MsgInfo->len);
			}
			else if(_commType == COMM_WAIT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_SPI_Receive(_Handle, MsgInfo->pRxData, MsgInfo->len, 1000);
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
				ret = HAL_SPI_Transmit_DMA(_Handle, MsgInfo->pTxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(hdmaRx, DMA_IT_HT);
			}
			else if(_commType == COMM_INTERRUPT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_SPI_Transmit_IT(_Handle, MsgInfo->pTxData, MsgInfo->len);
			}
			else if(_commType == COMM_WAIT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_SPI_Transmit(_Handle, MsgInfo->pTxData, MsgInfo->len, 1000);
			}
			else
			{
				ret = HAL_ERROR;
			}
		}
		break;
		case COMM_INT_TXRX:
		{
			if(_commType == COMM_DMA)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_SPI_TransmitReceive_DMA(_Handle, MsgInfo->pTxData, MsgInfo->pRxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(hdmaRx, DMA_IT_HT);
			}
			else if(_commType == COMM_INTERRUPT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_SPI_TransmitReceive_IT(_Handle, MsgInfo->pTxData, MsgInfo->pRxData, MsgInfo->len);
			}
			else if(_commType == COMM_WAIT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_SPI_TransmitReceive(_Handle, MsgInfo->pTxData, MsgInfo->pRxData, MsgInfo->len, 1000);
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



