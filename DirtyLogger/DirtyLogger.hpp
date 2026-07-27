/*
 * DirtyLogger.hpp
 *
 *  Created on: Dec 31, 2023
 *      Author: jurcz
 */

#ifndef DIRTYLOGGER_DIRTYLOGGER_HPP_
#define DIRTYLOGGER_DIRTYLOGGER_HPP_
#include "fatfs.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

typedef enum {LOGLEVEL_INFO=0, LOGLEVEL_DEBUG, LOGLEVEL_TRACE, TOF_LOG} loglevel_t;

/*
 * Log() never touches FatFs and never waits on the card. A record is formatted on the
 * caller's stack, copied into a ring under a short critical section, and a dedicated
 * writer task performs every f_write/f_sync. That split matters because the card driver
 * busy-waits: sd_diskio.c spins on BSP_SD_GetCardState() without yielding until the card
 * finishes its internal program - milliseconds, and far worse if it hits garbage
 * collection. Only the writer task should ever pay that, never the algorithm task and
 * never the 1 ms defaultTask that calls Sync().
 *
 * The writer hands FatFs whole 512-byte sectors, which keeps the file pointer
 * sector-aligned and makes f_write take its direct multi-sector path (ff.c:
 * "cc = btw / SS(fs)") instead of dribbling through the single-sector cache in
 * FIL::buf - one multi-block transfer per flush rather than one single-block write per
 * sector filled.
 *
 * _RING_SIZE must be both a power of two (for the index masking) and a multiple of
 * _SECTOR_SIZE, so a wrap can never split a sector. _MAX_RECORD bounds one formatted
 * record and lives on the caller's stack, so keep it modest: the largest message any
 * caller passes today is 128 bytes and the framing adds 45.
 */
#define DIRTYLOGGER_SECTOR_SIZE      512u
#define DIRTYLOGGER_RING_SIZE        8192u
#define DIRTYLOGGER_FLUSH_THRESHOLD  3584u   /* 7 sectors staged -> wake the writer */
#define DIRTYLOGGER_MAX_RECORD       256u
/*
 * Writer priority sits below defaultTask (osPriorityNormal) so a card stall can never
 * delay the 1 ms tick, and above the algorithm task (tskIDLE_PRIORITY) so the ring keeps
 * draining while the FSM is busy. Raising it trades FSM latency for drain headroom.
 */
#define DIRTYLOGGER_WRITER_PRIO      (tskIDLE_PRIORITY + 1)
#define DIRTYLOGGER_WRITER_STACK     256u    /* words, as xTaskCreate() counts them */
#define DIRTYLOGGER_DRAIN_TIMEOUT    pdMS_TO_TICKS(2000)

/*
 * Master switch for the whole preallocation scheme. Set to 0 here, or build with
 * -DDIRTYLOGGER_USE_PREALLOC=0, to fall back to plain appending: the writer then
 * f_syncs after every drain so the directory entry keeps tracking the growing size.
 * That is the same path a card with no contiguous block already takes, so it is the
 * tested fallback rather than an untried mode. The boot-time trim goes with it, since
 * an appending file leaves no stale tail to reclaim. Staging, whole-sector writes and
 * the non-blocking Log() are unaffected either way.
 *
 * Turning it off costs the zero-metadata-write property: expect a directory rewrite
 * per flush, and a power cut then loses everything written since the last one.
 */
#ifndef DIRTYLOGGER_USE_PREALLOC
#define DIRTYLOGGER_USE_PREALLOC 0
#endif

/*
 * The log file is preallocated as one contiguous block (f_expand), which buys two
 * things:
 *   - no cluster chain is ever stretched mid-run, so f_write only reads the FAT to
 *     follow the existing chain instead of writing it;
 *   - the directory entry carries the final size and start cluster from the very
 *     first sync in Init(), so every sector that reaches the card is readable even
 *     after an unclean power cut. That is what lets the writer skip f_sync altogether
 *     and stop rewriting the same directory sector once per flush.
 * If the card has no contiguous block this big the size is halved until it fits;
 * below _MIN_BYTES the logger just falls back to ordinary appending (and to
 * syncing, because then the directory entry does have to track the growing size).
 * Filling the block mid-run drops into that same append-and-sync mode rather than
 * stopping - see WriteToCard() - so logging degrades gradually instead of ending.
 *
 * Sizing is a trade. A power cut skips Flush(), so the whole allocation stays on the
 * card until the next boot reclaims it (TrimPreviousLog). 1 MB is ~3.5 minutes at the
 * measured full-TRACE rate of 4.7 KB/s - comfortably longer than a match, and cheap to
 * strand on the card if the robot dies early. Overrunning it is no longer fatal, so
 * erring small costs little: the logger just degrades to append-and-sync.
 */
#define DIRTYLOGGER_PREALLOC_BYTES      (1u * 1024u * 1024u)
#define DIRTYLOGGER_PREALLOC_MIN_BYTES  (256u * 1024u)

class DirtyLogger{
public:
	DirtyLogger(uint8_t *retSD, char *SDPath, FATFS *_SDFatFS, FIL *SDFile);
	void Init(void);
	void Log(const char* message, loglevel_t LogLevel);
	void Sync(void);
	void Flush(void);
	void StartTimestamp(void);
	/* Records lost to a full ring or an oversized message; nonzero means the log has
	 * gaps, which is how a writer that could not keep up shows up after the fact. */
	uint32_t GetDroppedRecords(void) const { return _dropped; }
	/* False means f_expand() found no contiguous block and the logger fell back to
	 * ordinary appending, so the writer is issuing f_sync after every flush again. */
	bool IsPreallocated(void) const { return _preallocated; }
private:
	static void WriterTaskEntry(void *arg);
	void WriterLoop(void);
	void DrainToCard(bool all);
	bool RingAppend(const uint8_t *data, uint32_t len);
	void WakeWriter(void);
	void Preallocate(void);
	void TrimPreviousLog(const char *filename);
	bool WriteToCard(const uint8_t *data, uint32_t len);

	static bool _instance_exists;
	uint32_t _start_time;
	bool _available;    /* False if the SD card failed to mount / open a log file; Log/Sync become no-ops */
	uint8_t *_retSD;    /* Return value for SD */
	char *_SDPath;   /* SD logical drive path */
	FATFS *_SDFatFS;    /* File system object for SD logical drive */
	FIL *_SDFile;       /* File object for SD */
	/* Free-running byte counters; the ring index is the counter masked with
	 * _RING_SIZE-1, and unsigned wraparound keeps (_head - _tail) correct forever.
	 * _head is advanced only by producers (inside a critical section), _tail only by
	 * the writer. Both are word-aligned, so each side reads the other's counter
	 * without a lock: a stale read is always the conservative one. */
	volatile uint32_t _head;
	volatile uint32_t _tail;
	uint32_t _dropped;  /* Records that never made it into the ring */
	bool _preallocated; /* True while the file still holds its f_expand'd contiguous block */
	FSIZE_t _prealloc_size;    /* Size of that block; writes must not run past it */
	volatile bool _drain_all;  /* Set by Flush(): drain the sub-sector tail too, then trim */
	SemaphoreHandle_t _wake;    /* "there is work" - binary, extra gives are harmless */
	SemaphoreHandle_t _drained; /* "the requested full drain finished" */
	TaskHandle_t _writer;
	/* 32-byte aligned: f_write()'s direct path hands this pointer straight to the
	 * SDMMC IDMA, and sd_diskio.c's optional scratch and D-cache paths both need it. */
	uint8_t _ring[DIRTYLOGGER_RING_SIZE] __attribute__((aligned(32)));
};


#endif /* DIRTYLOGGER_DIRTYLOGGER_HPP_ */
