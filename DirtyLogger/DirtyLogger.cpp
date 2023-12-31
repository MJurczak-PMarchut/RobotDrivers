/*
 * DirtyLogger.cpp
 *
 *  Created on: Dec 31, 2023
 *      Author: jurcz
 */

#include "DirtyLogger.hpp"
#include <string>
#include <string.h>


bool DirtyLogger::_instance_exists = 0;

DirtyLogger::DirtyLogger(uint8_t *retSD, char *SDPath, FATFS *SDFatFS, FIL *SDFile):
		_retSD{retSD}, _SDPath{SDPath}, _SDFatFS{SDFatFS}, _SDFile{SDFile}
{

}

void DirtyLogger::Init(void){
	TCHAR buff[25];
	DIR dp;
	FILINFO fno;
	TCHAR filename_pattern[] = "Log_*.txt";
	char filename[15] = {0};
	uint8_t iter = 0;
	if(f_getcwd(buff, 25) != FR_OK)
	{
		Error_Handler();
	}
	if(f_findfirst (&dp, &fno, buff, filename_pattern) != FR_OK)
	{
		Error_Handler();
	}
	if(fno.fname[0] == 0)
	{
		if(f_open(_SDFile, "Log_0.txt", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
		{
			Error_Handler();
		}
	}
	else
	{
		do
		{
			iter++;
			if(f_findnext(&dp, &fno) != FR_OK)
			{
				Error_Handler();
			}
		}
		while(fno.fname[0] != 0);
		iter++;
		iter = sprintf(filename, "Log_%d.txt", iter);
		if(f_open(_SDFile, filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
		{
			Error_Handler();
		}
	}
}
void DirtyLogger::Log(const char* message, bool include_timestamp)
{
	FRESULT res;
	uint32_t byteswritten;
	res = f_write(_SDFile, message, (UINT)strlen((char *)message), (UINT*)&byteswritten);
	if((byteswritten == 0) || (res != FR_OK))
	{
		Error_Handler();
	}
	f_sync(_SDFile);
}
void StartTimestamp(void)
{

}
