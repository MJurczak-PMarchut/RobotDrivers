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
const char* lvl_type[] = {"INFO", "DEBUG", "TRACE", "SENSOR_LOG"};

DirtyLogger::DirtyLogger(uint8_t *retSD, char *SDPath, FATFS *SDFatFS, FIL *SDFile):
		_available{false}, _retSD{retSD}, _SDPath{SDPath}, _SDFatFS{SDFatFS}, _SDFile{SDFile}
{

}

void DirtyLogger::Init(void){
	static TCHAR buff[35];
	static DIR dp;
	static FILINFO fno;
	static TCHAR filename_pattern[] = "Log_*.txt";
	char filename[25] = {0};
	uint8_t iter = 0;
	_available = false;
	// opt=1 forces the mount now so a missing/unreadable card is caught here instead of on first use.
	if(f_mount(_SDFatFS, (TCHAR const*)_SDPath, 1) != FR_OK)
	{
		return;
	}
	if(f_getcwd(buff, 25) != FR_OK)
	{
		return;
	}
	if(f_findfirst (&dp, &fno, buff, filename_pattern) != FR_OK)
	{
		return;
	}
	if(fno.fname[0] == 0)
	{
		if(f_open(_SDFile, "Log_0.txt", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
		{
			return;
		}
	}
	else
	{
		do
		{
			iter++;
			if(f_findnext(&dp, &fno) != FR_OK)
			{
				return;
			}
		}
		while(fno.fname[0] != 0);
		iter++;
		iter = sprintf(filename, "Log_%d.txt", iter);
		if(f_open(_SDFile, filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
		{
			return;
		}
	}
	_available = true;
}
void DirtyLogger::Log(const char* message, loglevel_t LogLevel)
{
	FRESULT res;
	uint32_t byteswritten;
	static char data[1024] = {0};
	int len;
	if(!_available)
	{
		return;
	}
	if(LogLevel > LOGLEVEL_TRACE)
	{
		Error_Handler();
	}
	len = sprintf(data, "+++LOG::%s::%u::%s::END_LOG---\n", lvl_type[LogLevel], (unsigned int)(HAL_GetTick()-_start_time), message);
	if(len < 0)
	{
		return;
	}
	res = f_write(_SDFile, data, (UINT)len, (UINT*)&byteswritten);
	if((byteswritten == 0) || (res != FR_OK))
	{
		// SD card may have been removed mid-run; stop trying rather than erroring out the robot.
		_available = false;
	}
}

void DirtyLogger::Sync(void)
{
	if(!_available)
	{
		return;
	}
	f_sync(_SDFile);
}

void DirtyLogger::StartTimestamp(void)
{
	_start_time = HAL_GetTick();
}
