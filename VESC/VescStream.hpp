/*
 * VescStream.hpp
 *
 *  Created on: 26 mar 2023
 *      Author: Mateusz
 */

#ifndef VESC_VESCSTREAM_HPP_
#define VESC_VESCSTREAM_HPP_
#include <queue>
#include "CommManager.hpp"



class VescStream
{
public:
	VescStream(CommManager *comm, UART_HandleTypeDef *Uart)
	{
		_UART = Uart;
		_CommManager = comm;
		MessageInfoTypeDef<UART> MsgInfo;
		//Start receiving
		Vesc_memset(&MsgInfo, 0, sizeof(MsgInfo));
		MsgInfo.eCommType = COMM_INT_RX;
		MsgInfo.IntHandle = _UART;
		MsgInfo.len = 1;
		MsgInfo.pCB=std::bind(&VescStream::Callback, this, std::placeholders::_1);
		MsgInfo.pRxData = message;
		_CommManager->PushCommRequestIntoQueue(&MsgInfo);
	}
	uint32_t available(void)
	{
		return data.size();
	}
	uint8_t read()
	{
		uint8_t mes = data.front();
		data.pop();
		return mes;
	}
	void print(const char*)
	{

	}
	void println(const char*)
	{

	}
	void write(uint8_t* data, uint8_t count)
	{
		uint8_t* pTempCharArray = new uint8_t[count];
		MessageInfoTypeDef<UART> MsgInfo;
		Vesc_memset(&MsgInfo, 0, sizeof(MsgInfo));
		Vesc_memcpy(pTempCharArray, data, count);
		MsgInfo.eCommType = COMM_INT_TX;
		MsgInfo.IntHandle = _UART;
		MsgInfo.len = count;
		MsgInfo.pTxData = pTempCharArray;
		MsgInfo.pCB=std::bind(&VescStream::Callback, this, std::placeholders::_1);
		_CommManager->PushCommRequestIntoQueue(&MsgInfo);
	}
private:
	std::queue<uint8_t> data;
	UART_HandleTypeDef* _UART;
	CommManager* _CommManager;
	uint8_t message[2];
	void *Vesc_memset (void *dest, int val, size_t len)
	{
	  unsigned char *ptr = reinterpret_cast<unsigned char *>(dest);
	  while (len-- > 0)
	    *ptr++ = val;
	  return dest;
	}
	void* Vesc_memcpy (void *dest, const void *src, size_t len)
	{
	  char *d = reinterpret_cast<char *>(dest);
	  const char *s = reinterpret_cast<const char *>(src);
	  while (len--)
	    *d++ = *s++;
	  return dest;
	}
	void Callback(MessageInfoTypeDef<UART>* MsgInfo){
		if(MsgInfo->eCommType == COMM_INT_RX){
			_CommManager->PushCommRequestIntoQueue(MsgInfo);
			data.push(message[0]);
		}
		else{
			delete[] MsgInfo->pTxData;
		}
	}

};



#endif /* VESC_VESCSTREAM_HPP_ */
