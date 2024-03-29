/*
 * DirtyLogger.hpp
 *
 *  Created on: Dec 31, 2023
 *      Author: jurcz
 */

#ifndef DIRTYLOGGER_DIRTYLOGGER_HPP_
#define DIRTYLOGGER_DIRTYLOGGER_HPP_
#include "fatfs.h"

typedef enum {LOGLEVEL_INFO=0, LOGLEVEL_DEBUG, LOGLEVEL_TRACE, TOF_LOG} loglevel_t;

class DirtyLogger{
public:
	DirtyLogger(uint8_t *retSD, char *SDPath, FATFS *_SDFatFS, FIL *SDFile);
	void Init(void);
	void Log(const char* message, loglevel_t LogLevel);
	void Sync(void);
	void StartTimestamp(void);
private:
	static bool _instance_exists;
	uint32_t _start_time;
	uint8_t *_retSD;    /* Return value for SD */
	char *_SDPath;   /* SD logical drive path */
	FATFS *_SDFatFS;    /* File system object for SD logical drive */
	FIL *_SDFile;       /* File object for SD */

};


#endif /* DIRTYLOGGER_DIRTYLOGGER_HPP_ */
