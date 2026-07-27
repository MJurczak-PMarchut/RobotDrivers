/*
 * DirtyLogger.cpp
 *
 *  Created on: Dec 31, 2023
 *      Author: jurcz
 */

#include "DirtyLogger.hpp"
#include "ff.h"
#include <stdio.h>
#include <string>
#include <string.h>


bool DirtyLogger::_instance_exists = 0;
const char* lvl_type[] = {"INFO", "DEBUG", "TRACE", "SENSOR_LOG"};

/* Record delimiters. The format string below is built from them so the writer and the
 * boot-time scanner in TrimPreviousLog() can never drift apart. */
#define REC_PREFIX  "+++LOG::"
#define REC_SUFFIX  "::END_LOG---\n"
#define REC_FORMAT  REC_PREFIX "%s::%u::%s" REC_SUFFIX
#if DIRTYLOGGER_USE_PREALLOC
static const uint32_t REC_PREFIX_LEN = sizeof(REC_PREFIX) - 1u;
static const uint32_t REC_SUFFIX_LEN = sizeof(REC_SUFFIX) - 1u;

// Offset just past the record terminator within the first len bytes, or 0 if the
// record does not end inside them.
static uint32_t FindRecordEnd(const uint8_t *p, uint32_t len)
{
	if(len < REC_SUFFIX_LEN)
	{
		return 0;
	}
	for(uint32_t i = 0; i <= (len - REC_SUFFIX_LEN); i++)
	{
		if(memcmp(&p[i], REC_SUFFIX, REC_SUFFIX_LEN) == 0)
		{
			return i + REC_SUFFIX_LEN;
		}
	}
	return 0;
}
#endif /* DIRTYLOGGER_USE_PREALLOC */

DirtyLogger::DirtyLogger(uint8_t *retSD, char *SDPath, FATFS *SDFatFS, FIL *SDFile):
		_available{false}, _retSD{retSD}, _SDPath{SDPath}, _SDFatFS{SDFatFS}, _SDFile{SDFile},
		_head{0}, _tail{0}, _dropped{0}, _preallocated{false}, _prealloc_size{0},
		_drain_all{false}, _wake{NULL}, _drained{NULL}, _writer{NULL}
{

}


#if DIRTYLOGGER_USE_PREALLOC
void DirtyLogger::Preallocate(void)
{
	FSIZE_t want = DIRTYLOGGER_PREALLOC_BYTES;

	_preallocated = false;
	_prealloc_size = 0;
	while(want >= DIRTYLOGGER_PREALLOC_MIN_BYTES)
	{
		if(f_expand(_SDFile, want, 1) == FR_OK)
		{
			_prealloc_size = want;
			_preallocated = true;
			return;
		}
		want /= 2;
	}
}

// Reclaims the stale tail of a previous run's log. A power cut skips Flush(), so that
// file keeps its full preallocated size with whatever occupied those clusters before
// sitting past the last flush. Give the space back before reserving more for this run.
//
// Cost is bounded by how much was actually logged, not by the preallocated size: the
// scan walks the record stream and stops at the first byte that does not continue it,
// which is where the writes ended. It borrows _ring as its window - safe here because
// Init() has not started the writer task or set _available yet.
//
// It only ever moves the end of the file backwards to a record boundary, so it cannot
// discard anything the previous run actually wrote. It can however reclaim less than
// the whole tail: the writer stops on a sector boundary, usually mid-record, and if
// f_expand handed us clusters that still hold records from a deleted older log, the
// terminator search runs off the end of our partial record into those and keeps going.
// Nothing distinguishes them by content - same reason the log viewer cannot rely on a
// timestamp rule - so the conservative outcome is to keep them. Tails that are erased
// or hold non-log data, which is the usual case, are reclaimed exactly.
void DirtyLogger::TrimPreviousLog(const char *filename)
{
	FIL *fp = _SDFile;      /* this run's log is not open yet, so the object is free */
	FSIZE_t good = 0;       /* offset just past the last complete record */
	FSIZE_t base = 0;       /* file offset of _ring[0] */
	uint32_t fill = 0;
	bool scan_ok = true;

	if(f_open(fp, filename, FA_READ | FA_WRITE) != FR_OK)
	{
		return;             /* nothing there - first run, or the name does not exist */
	}
	for(;;)
	{
		UINT br = 0;
		uint32_t cursor = 0;
		bool garbage = false;

		if(f_read(fp, &_ring[fill], DIRTYLOGGER_RING_SIZE - fill, &br) != FR_OK)
		{
			// Never truncate on the strength of a failed read.
			scan_ok = false;
			break;
		}
		fill += br;
		while((fill - cursor) >= REC_PREFIX_LEN)
		{
			uint32_t span = fill - cursor;
			uint32_t end;
			if(memcmp(&_ring[cursor], REC_PREFIX, REC_PREFIX_LEN) != 0)
			{
				garbage = true;     /* this is exactly where our writes stopped */
				break;
			}
			// Log() never emits a record longer than this, so a "record" without a
			// terminator inside that span is not ours - stop rather than scan on.
			if(span > DIRTYLOGGER_MAX_RECORD)
			{
				span = DIRTYLOGGER_MAX_RECORD;
			}
			end = FindRecordEnd(&_ring[cursor], span);
			if(end == 0)
			{
				if(span >= DIRTYLOGGER_MAX_RECORD)
				{
					garbage = true;
				}
				break;              /* otherwise the record runs past the window; refill */
			}
			cursor += end;
			good = base + cursor;
		}
		if(garbage || (br == 0))
		{
			break;                  /* hit the tail, or reached end of file */
		}
		if((cursor == 0) && (fill == DIRTYLOGGER_RING_SIZE))
		{
			break;                  /* one "record" larger than the window: give up */
		}
		memmove(&_ring[0], &_ring[cursor], fill - cursor);
		base += cursor;
		fill -= cursor;
	}
	if(scan_ok && (good < f_size(fp)))
	{
		if(f_lseek(fp, good) == FR_OK)
		{
			f_truncate(fp);
		}
	}
	f_close(fp);                    /* commits the new size */
	_head = 0;                      /* hand the borrowed ring back empty */
	_tail = 0;
}

#else  /* !DIRTYLOGGER_USE_PREALLOC */

// Compiled out: leaving _preallocated false keeps the logger in append-and-sync mode
// for the whole run, which is the path every other function already handles.
void DirtyLogger::Preallocate(void)
{
	_preallocated = false;
	_prealloc_size = 0;
}

// Nothing to reclaim: an appending file's directory entry already matches what was
// synced, so a power cut leaves no oversized file behind.
void DirtyLogger::TrimPreviousLog(const char *)
{
}

#endif /* DIRTYLOGGER_USE_PREALLOC */

void DirtyLogger::Init(void){
	static TCHAR buff[35];
	static DIR dp;
	static FILINFO fno;
	static TCHAR filename_pattern[] = "Log_*.txt";
	char filename[25] = {0};
	uint16_t iter = 0;
	if(_available)
	{
		this->Log("---- Re-Init Attempted ----", LOGLEVEL_DEBUG);
		return;
	}
	_available = false;
	_head = 0;
	_tail = 0;
	_drain_all = false;
	if(_wake == NULL)
	{
		_wake = xSemaphoreCreateBinary();
		_drained = xSemaphoreCreateBinary();
		if((_wake == NULL) || (_drained == NULL))
		{
			return;
		}
	}
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
		// iter files exist, numbered Log_0..Log_(iter-1), so the newest is the run that
		// came before this one. Reclaim its stale tail before reserving space for ours.
		snprintf(filename, sizeof(filename), "Log_%u.txt", (unsigned)(iter - 1u));
		TrimPreviousLog(filename);
		iter = sprintf(filename, "Log_%d.txt", iter);
		if(f_open(_SDFile, filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
		{
			return;
		}
	}
	Preallocate();
	// f_expand() only updates the in-RAM file object, so until this sync the on-card
	// directory entry still reads "size 0" and a power cut would lose the entire log.
	// When preallocation succeeded this is also the last f_sync of the whole run.
	if(f_sync(_SDFile) != FR_OK)
	{
		return;
	}
	if(_writer == NULL)
	{
		// Everything from here on reaches the card through this task and nothing else.
		if(xTaskCreate(&DirtyLogger::WriterTaskEntry, "DirtyLogger", DIRTYLOGGER_WRITER_STACK,
				this, DIRTYLOGGER_WRITER_PRIO, &_writer) != pdPASS)
		{
			return;
		}
	}
	_available = true;
	// State this run's budget in the log itself: a file that just stops dead is far
	// easier to read when you can tell whether it simply ran out of reserved space.
	if(_preallocated)
	{
		char note[64];
		snprintf(note, sizeof(note), "Log preallocated: %lu KiB",
				(unsigned long)(_prealloc_size / 1024u));
		this->Log(note, LOGLEVEL_INFO);
	}
	else
	{
#if DIRTYLOGGER_USE_PREALLOC
		this->Log("Log NOT preallocated (no contiguous block) - appending, sync per flush", LOGLEVEL_INFO);
#else
		this->Log("Log preallocation disabled at build time - appending, sync per flush", LOGLEVEL_INFO);
#endif
	}
}

// Non-blocking by construction: formats on the caller's stack, copies into the ring
// under a critical section a few hundred nanoseconds long, and hands off. No FatFs
// call, no card access, no waiting on another task - so a card that stalls for
// hundreds of milliseconds costs log records, never algorithm-task latency.
void DirtyLogger::Log(const char* message, loglevel_t LogLevel)
{
	char record[DIRTYLOGGER_MAX_RECORD];
	int len;

	if(!_available)
	{
		return;
	}
	if(LogLevel > LOGLEVEL_TRACE)
	{
		Error_Handler();
	}
	// Deliberately outside the critical section: snprintf is by far the slowest part of
	// Log(), and nothing needs to be held off while it runs.
	len = snprintf(record, sizeof(record), REC_FORMAT,
			lvl_type[LogLevel], (unsigned int)(HAL_GetTick()-_start_time), message);
	if((len <= 0) || ((uint32_t)len >= sizeof(record)))
	{
		_dropped++;
		return;
	}
	if(!RingAppend((const uint8_t*)record, (uint32_t)len))
	{
		// Ring full - the writer has not kept up, most likely because the card is busy.
		// Dropping is the whole point: blocking here would put the card's stall straight
		// into the algorithm task. GetDroppedRecords() reports the gap afterwards.
		_dropped++;
		return;
	}
	if((uint32_t)(_head - _tail) >= DIRTYLOGGER_FLUSH_THRESHOLD)
	{
		WakeWriter();
	}
}

// The only place _head moves. The critical section is what makes this safe for more
// than one producer task; it spans a bounded memcpy of at most _MAX_RECORD bytes.
bool DirtyLogger::RingAppend(const uint8_t *data, uint32_t len)
{
	bool ok = false;

	taskENTER_CRITICAL();
	if((uint32_t)(DIRTYLOGGER_RING_SIZE - (_head - _tail)) >= len)
	{
		uint32_t index = _head & (DIRTYLOGGER_RING_SIZE - 1u);
		uint32_t run = DIRTYLOGGER_RING_SIZE - index;
		if(run > len)
		{
			run = len;
		}
		memcpy(&_ring[index], data, run);
		if(run < len)
		{
			memcpy(&_ring[0], data + run, len - run);   /* record straddles the wrap */
		}
		_head += len;
		ok = true;
	}
	taskEXIT_CRITICAL();
	return ok;
}

void DirtyLogger::WakeWriter(void)
{
	if(_wake != NULL)
	{
		// Binary semaphore: a give while one is already pending is a harmless no-op,
		// which is exactly the coalescing behaviour wanted here.
		xSemaphoreGive(_wake);
	}
}

void DirtyLogger::WriterTaskEntry(void *arg)
{
	static_cast<DirtyLogger*>(arg)->WriterLoop();
}

// The only task that touches FatFs. Everything expensive - the multi-block transfer,
// sd_diskio's non-yielding spin on BSP_SD_GetCardState(), and any f_sync - happens
// here, below defaultTask and above the algorithm task.
void DirtyLogger::WriterLoop(void)
{
	for(;;)
	{
		if(xSemaphoreTake(_wake, portMAX_DELAY) != pdTRUE)
		{
			continue;
		}
		const bool all = _drain_all;

		DrainToCard(all);

		if(all && _preallocated)
		{
			// Hand back the unused tail of the preallocated block so the file ends at
			// the real data and no reader can trip over whatever occupied those
			// clusters before. Clearing the flag also re-arms the sync path below.
			f_truncate(_SDFile);
			_preallocated = false;
			_prealloc_size = 0;
		}
		// A preallocated file needs no directory update at all: Init() already committed
		// its final size and start cluster, the chain is complete, and objsize never
		// grows because fptr stays inside the block. Anything else has to keep the
		// directory entry tracking the size, and f_sync short-circuits on FA_MODIFIED
		// so it is free when nothing was actually flushed.
		if(_available && !_preallocated)
		{
			f_sync(_SDFile);
		}
		if(all)
		{
			_drain_all = false;
			xSemaphoreGive(_drained);
		}
	}
}

// The only place _tail moves. While draining normally it stops on a sector boundary,
// which keeps both the FatFs file pointer and the ring read index 512-byte aligned -
// that alignment is what holds f_write on its direct multi-sector path. _RING_SIZE
// being a multiple of the sector size is what makes the wrap respect it too.
void DirtyLogger::DrainToCard(bool all)
{
	while(_available)
	{
		uint32_t avail = (uint32_t)(_head - _tail);
		uint32_t want = all ? avail : (avail & ~(DIRTYLOGGER_SECTOR_SIZE - 1u));
		if(want == 0)
		{
			break;
		}
		uint32_t index = _tail & (DIRTYLOGGER_RING_SIZE - 1u);
		uint32_t run = DIRTYLOGGER_RING_SIZE - index;   /* contiguous bytes before the wrap */
		if(run > want)
		{
			run = want;
		}
		if(!WriteToCard(&_ring[index], run))
		{
			break;
		}
		_tail += run;
	}
}

// Writer task only.
bool DirtyLogger::WriteToCard(const uint8_t *data, uint32_t len)
{
	UINT byteswritten = 0;
	// Out of reserved space. Keep logging, but past this point f_write stretches the
	// chain again and objsize outgrows the directory entry committed in Init(), so that
	// entry no longer covers what we write. Drop into exactly the append-and-sync mode
	// used when the card had no contiguous block to begin with: clearing the flag makes
	// the writer resume f_sync after every drain, which keeps the size and FAT current
	// and holds power-cut exposure to the last flush. Whole-sector writes and the
	// non-blocking Log() are unaffected; only the zero-metadata-write property is lost.
	if(_preallocated && (((FSIZE_t)_SDFile->fptr + len) > _prealloc_size))
	{
		_preallocated = false;
		_prealloc_size = 0;
		// Goes into the ring, so it lands in the file just after the data that filled
		// the block - which is exactly where a reader wants to find it.
		this->Log("Preallocated block exhausted - appending, periodic sync resumed", LOGLEVEL_INFO);
	}
	FRESULT res = f_write(_SDFile, data, (UINT)len, &byteswritten);
	if((res != FR_OK) || (byteswritten != len))
	{
		// SD card may have been removed mid-run; stop trying rather than erroring out the robot.
		_available = false;
		return false;
	}
	return true;
}

// Just a nudge. Called from the 1 ms defaultTask, so it must not touch the card: it
// only bounds how long a partly-filled ring can sit unwritten when the log is quiet.
void DirtyLogger::Sync(void)
{
	if(!_available)
	{
		return;
	}
	WakeWriter();
}

// Full drain, sub-sector tail included, then the preallocated tail is trimmed. That
// partial sector leaves the file pointer unaligned, so writes after it fall back to
// FatFs' single-sector cache - this is for the way out to a reset, not for periodic
// use. Unlike Sync() it does block, waiting for the writer to report the drain
// actually finished rather than guessing at a delay.
void DirtyLogger::Flush(void)
{
	if(!_available || (_writer == NULL))
	{
		return;
	}
	// Clear any token left behind by a previous drain that timed out, so the wait below
	// cannot be satisfied by stale news.
	xSemaphoreTake(_drained, 0);
	_drain_all = true;
	WakeWriter();
	xSemaphoreTake(_drained, DIRTYLOGGER_DRAIN_TIMEOUT);
}

void DirtyLogger::StartTimestamp(void)
{
	_start_time = HAL_GetTick();
}
