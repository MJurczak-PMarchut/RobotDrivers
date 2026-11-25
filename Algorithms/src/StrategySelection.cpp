/*
 * StrategySelection.cpp
 *
 * Flash and SD-card stores of the selected strategy; see StrategySelection.hpp. The design
 * and the power-loss analysis are in ~/Projekty/MultiAlgorithmV1WorkState.md
 * ("StrategySelection: projekt").
 */

#ifdef ROBOT_IS_MULTI_ALGORITHM

#include "StrategySelection.hpp"

#include <cctype>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"
#include "fatfs.h"   // SDFatFS, SDPath: the volume DirtyLogger logs to

namespace {

/*
 * Names
 */
struct StrategyInfo
{
	Strategy strategy;
	const char* name;
};

constexpr StrategyInfo kStrategies[] = {
	{Strategy::MiniTomiV4, "V4"},
	{Strategy::MiniTomiV5, "V5"},
	{Strategy::MiniNeuroV1, "NEURO1"},
	{Strategy::MiniNeuroV2, "NEURO2"},
};

bool IsKnownStrategy(uint32_t value)
{
	for(const StrategyInfo& info : kStrategies)
	{
		if(static_cast<uint32_t>(info.strategy) == value)
		{
			return true;
		}
	}
	return false;
}

/*
 * The two ways of keeping the strategy, behind one interface.
 */
class StrategyStore
{
public:
	virtual bool Load(Strategy& strategy) = 0;
	virtual bool Save(Strategy strategy) = 0;
protected:
	~StrategyStore() = default;   // never deleted through the base
};

/*
 * Flash store.
 *
 * The last sector of bank 2. Code runs from bank 1, so programming and erasing here do not
 * stall it; the sector is kept out of the FLASH region in STM32H743IIKX_FLASH.ld.
 *
 * The H7 programs flash in 256-bit words, each with its own ECC, so a word can be written
 * only once between two erases. One record fills one word, and the sector is a log: each
 * Save() appends a record to the next erased word, and the sector is erased only when it is
 * full (every 4096 saves). The newest record is the last one of the log.
 *
 * Power cuts. A cut while a word is being programmed (or the sector erased) can leave a word
 * with a double ECC error, and a plain read of such a word is a bus error: a HardFault on
 * every boot. So:
 *  - layer 1: every word of the sector is read only through SafeReadFlashWord(), which
 *    reports a damaged word instead of faulting;
 *  - layer 2: a damaged word counts as used and invalid: Load() skips it, Save() never
 *    programs over it and checks each record it writes by reading it back.
 * A cut while a record is written loses only that change; the previous record stays the
 * newest. A cut while the full sector is erased can leave a mix of erased and old words:
 * Load() then reports no strategy rather than one that may be stale, and the next Save()
 * erases the sector again.
 */
constexpr uint32_t kSectorAddress = 0x081E0000u;
constexpr uint32_t kSectorBank = FLASH_BANK_2;
constexpr uint32_t kSectorNumber = FLASH_SECTOR_7;
constexpr std::size_t kSectorBytes = 128u * 1024u;
constexpr std::size_t kWordUint32s = FLASH_NB_32BITWORD_IN_FLASHWORD;
constexpr std::size_t kWordBytes = kWordUint32s * sizeof(uint32_t);
constexpr std::size_t kWordCount = kSectorBytes / kWordBytes;

constexpr uint32_t kRecordMagic = 0x53545247u;   // "STRG"
constexpr uint32_t kRecordVersion = 1u;
constexpr uint32_t kErasedUint32 = 0xFFFFFFFFu;
// Records Save() tries to program before giving up; each word that fails stays used.
constexpr int kMaxProgramAttempts = 4;

struct StrategyRecord
{
	uint32_t magic;          // kRecordMagic
	uint32_t version;        // kRecordVersion
	uint32_t strategy;       // Strategy
	uint32_t strategy_inv;   // ~strategy: a damaged record does not pass for a valid one
	uint32_t reserved[4];    // kErasedUint32 (room for a later "sequence", see the design)
};
static_assert(sizeof(StrategyRecord) == kWordBytes, "one record per flash word");

// What one word of the sector holds.
enum class WordState : uint8_t
{
	Empty,    // erased (all ones) and no ECC error: free for the next record
	Record,   // a valid record
	Used,     // anything else: a double ECC error, a damaged record, garbage
};

uint32_t WordAddress(std::size_t index)
{
	return kSectorAddress + static_cast<uint32_t>(index * kWordBytes);
}

// Layer 1. Reads one flash word without faulting on a double ECC error. With FAULTMASK set
// (execution priority -1) and SCB->CCR.BFHFNMIGN, the core ignores a precise data bus fault
// of a load; the flash interface still flags the error in FLASH->SR2 (DBECCERR), which is
// how a damaged word is recognised. Interrupts are held off only while the 32 bytes are
// copied. Returns false for a damaged word (whose contents are then meaningless).
bool SafeReadFlashWord(uint32_t address, uint32_t (&words)[kWordUint32s])
{
	FLASH->CCR2 = FLASH_CCR_CLR_DBECCERR | FLASH_CCR_CLR_SNECCERR;
	const uint32_t faultmask = __get_FAULTMASK();
	const uint32_t ccr = SCB->CCR;
	__set_FAULTMASK(1u);
	SCB->CCR = ccr | SCB_CCR_BFHFNMIGN_Msk;
	__DSB();
	__ISB();
	const volatile uint32_t* source = reinterpret_cast<const volatile uint32_t*>(address);
	for(std::size_t i = 0; i < kWordUint32s; i++)
	{
		words[i] = source[i];
	}
	__DSB();
	SCB->CCR = ccr;
	__ISB();
	__set_FAULTMASK(faultmask);
	const bool intact = (FLASH->SR2 & FLASH_SR_DBECCERR) == 0u;
	// HAL_FLASH_Program() and HAL_FLASHEx_Erase() refuse to start while any bank-2 error
	// flag is set, ECC ones included, so never leave one behind. A single ECC error was
	// corrected in hardware: that data is good.
	FLASH->CCR2 = FLASH_CCR_CLR_DBECCERR | FLASH_CCR_CLR_SNECCERR;
	return intact;
}

WordState ReadWord(std::size_t index, Strategy& strategy)
{
	uint32_t words[kWordUint32s];
	if(!SafeReadFlashWord(WordAddress(index), words))
	{
		return WordState::Used;
	}
	bool erased = true;
	for(uint32_t word : words)
	{
		if(word != kErasedUint32)
		{
			erased = false;
			break;
		}
	}
	if(erased)
	{
		return WordState::Empty;
	}
	StrategyRecord record;
	std::memcpy(&record, words, sizeof(record));
	if((record.magic == kRecordMagic) && (record.version == kRecordVersion) &&
			(record.strategy == ~record.strategy_inv) && IsKnownStrategy(record.strategy))
	{
		strategy = static_cast<Strategy>(record.strategy);
		return WordState::Record;
	}
	return WordState::Used;
}

WordState ReadWord(std::size_t index)
{
	Strategy unused = kDefaultStrategy;
	return ReadWord(index, unused);
}

// True if word `index` reads back exactly as `record` (layer 2: verify after programming).
bool WordHolds(std::size_t index, const StrategyRecord& record)
{
	uint32_t words[kWordUint32s];
	return SafeReadFlashWord(WordAddress(index), words) &&
			(std::memcmp(words, &record, sizeof(record)) == 0);
}

// Layer 2. Walks the whole sector. A consistent log is "used words, then erased words up to
// the end"; damaged words inside the used part are skipped, they never end the log.
struct LogScan
{
	std::size_t first_empty = kWordCount;   // kWordCount: no erased word left
	bool consistent = true;                 // false: a used word after an erased one
	bool has_record = false;
	Strategy newest = kDefaultStrategy;     // the last record of the log
};

LogScan ScanLog(void)
{
	LogScan scan;
	for(std::size_t i = 0; i < kWordCount; i++)
	{
		Strategy strategy = kDefaultStrategy;
		const WordState state = ReadWord(i, strategy);
		if(state == WordState::Empty)
		{
			if(scan.first_empty == kWordCount)
			{
				scan.first_empty = i;
			}
			continue;
		}
		if(scan.first_empty != kWordCount)
		{
			// Left by an interrupted erase: old words between erased ones. Whatever records
			// survived may be older than the last one written, so none of them is trusted.
			scan.consistent = false;
			break;
		}
		if(state == WordState::Record)
		{
			scan.has_record = true;
			scan.newest = strategy;
		}
	}
	return scan;
}

bool ProgramWord(std::size_t index, const StrategyRecord& record)
{
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_ALL_ERRORS_BANK2);
	// HAL_FLASH_Program() copies the 32 bytes from this RAM address.
	const HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, WordAddress(index),
			reinterpret_cast<uint32_t>(&record));
	HAL_FLASH_Lock();
	return status == HAL_OK;
}

// Blocks the calling task for the erase (on the order of a second); code and interrupts keep
// running from bank 1.
bool EraseSector(void)
{
	FLASH_EraseInitTypeDef erase = {};
	erase.TypeErase = FLASH_TYPEERASE_SECTORS;
	erase.Banks = kSectorBank;
	erase.Sector = kSectorNumber;
	erase.NbSectors = 1;
	erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;   // 3.3 V supply, as in ST's H743 examples
	uint32_t sector_error = 0;
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_ALL_ERRORS_BANK2);
	const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &sector_error);
	HAL_FLASH_Lock();
	return status == HAL_OK;
}

StrategyRecord MakeRecord(Strategy strategy)
{
	StrategyRecord record;
	record.magic = kRecordMagic;
	record.version = kRecordVersion;
	record.strategy = static_cast<uint32_t>(strategy);
	record.strategy_inv = ~record.strategy;
	for(uint32_t& word : record.reserved)
	{
		word = kErasedUint32;
	}
	return record;
}

class FlashStrategyStore final : public StrategyStore
{
public:
	bool Load(Strategy& strategy) override
	{
		const LogScan scan = ScanLog();
		if(!scan.consistent || !scan.has_record)
		{
			return false;
		}
		strategy = scan.newest;
		return true;
	}

	bool Save(Strategy strategy) override
	{
		const LogScan scan = ScanLog();
		if(scan.consistent && scan.has_record && (scan.newest == strategy))
		{
			return true;   // already the newest record: no write, no wear
		}
		const StrategyRecord record = MakeRecord(strategy);
		std::size_t index = scan.first_empty;
		bool erased = false;
		if(!scan.consistent)
		{
			if(!EraseSector())
			{
				return false;
			}
			erased = true;
			index = 0;
		}
		int attempts = 0;
		while(attempts < kMaxProgramAttempts)
		{
			if(index >= kWordCount)
			{
				// Full: start the log over (once per Save()).
				if(erased || !EraseSector())
				{
					return false;
				}
				erased = true;
				index = 0;
			}
			if(ReadWord(index) != WordState::Empty)
			{
				index++;   // never program a word that is not erased
				continue;
			}
			attempts++;
			if(ProgramWord(index, record) && WordHolds(index, record))
			{
				return true;
			}
			// A word that took (part of) the write stays used: move on. One that still reads
			// erased (the write did not start) is tried again, so the log gets no gap.
			if(ReadWord(index) != WordState::Empty)
			{
				index++;
			}
		}
		return false;
	}
};

/*
 * SD-card store: strategy.txt in the root directory, one short name.
 *
 * Whether a card is there is decided only by mounting it and opening the file. SD_DETECT
 * (PB0) is mechanically damaged and cannot be trusted; if it reads "no card", BSP_SD_Init()
 * refuses the card and the mount fails, which here is just "SD not available".
 * A power cut while the file is rewritten can leave it empty or cut short; it then reads as
 * no valid strategy and the caller falls back to flash.
 */
constexpr const char* kStrategyFileName = "strategy.txt";
// Bytes of the file looked at: the longest name plus surrounding whitespace and line end.
constexpr std::size_t kMaxFileChars = 32;

// One file object, not on the caller's stack (FIL carries a 512-byte sector buffer).
FIL strategy_file;

bool SdAvailable(void)
{
	// The SD driver (sd_diskio.c) waits for its DMA on an RTOS queue and refuses to
	// initialise the card before the scheduler runs.
	if(osKernelGetState() != osKernelRunning)
	{
		return false;
	}
	// Mount only if nobody has: f_mount() on the volume DirtyLogger already mounted would
	// invalidate the log file it keeps open. A failed mount (no card, a card that does not
	// answer, SD_DETECT reading "no card") simply means no SD.
	if(SDFatFS.fs_type == 0)
	{
		return f_mount(&SDFatFS, static_cast<const TCHAR*>(SDPath), 1) == FR_OK;
	}
	return true;
}

bool IsBlank(char c)
{
	return (c == ' ') || (c == '\t') || (c == '\r') || (c == '\n');
}

// The file as written on a PC: optional UTF-8 BOM, whitespace, the name, whitespace / line end.
bool ParseStrategyText(char* text, Strategy& strategy)
{
	static const char kUtf8Bom[] = "\xEF\xBB\xBF";
	if(std::strncmp(text, kUtf8Bom, sizeof(kUtf8Bom) - 1u) == 0)
	{
		text += sizeof(kUtf8Bom) - 1u;
	}
	while(IsBlank(*text))
	{
		text++;
	}
	std::size_t length = std::strlen(text);
	while((length > 0u) && IsBlank(text[length - 1u]))
	{
		length--;
	}
	text[length] = '\0';
	return StrategyFromName(text, strategy);
}

class SdStrategyStore final : public StrategyStore
{
public:
	bool Load(Strategy& strategy) override
	{
		if(!SdAvailable())
		{
			return false;
		}
		if(f_open(&strategy_file, kStrategyFileName, FA_READ) != FR_OK)
		{
			return false;
		}
		char text[kMaxFileChars + 1u] = {0};
		UINT read = 0;
		const FRESULT result = f_read(&strategy_file, text, kMaxFileChars, &read);
		f_close(&strategy_file);
		if((result != FR_OK) || (read > kMaxFileChars))
		{
			return false;
		}
		text[read] = '\0';
		return ParseStrategyText(text, strategy);
	}

	bool Save(Strategy strategy) override
	{
		if(!SdAvailable())
		{
			return false;
		}
		char line[kMaxFileChars];
		const int length = std::snprintf(line, sizeof(line), "%s\n", StrategyName(strategy));
		if((length <= 0) || (static_cast<std::size_t>(length) >= sizeof(line)))
		{
			return false;
		}
		if(f_open(&strategy_file, kStrategyFileName, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
		{
			return false;
		}
		UINT written = 0;
		const FRESULT write_result = f_write(&strategy_file, line, static_cast<UINT>(length), &written);
		const FRESULT close_result = f_close(&strategy_file);   // also commits size and FAT
		return (write_result == FR_OK) && (close_result == FR_OK) &&
				(written == static_cast<UINT>(length));
	}
};

Strategy running_strategy = kDefaultStrategy;

FlashStrategyStore flash_store;
SdStrategyStore sd_store;

StrategyStore& StoreFor(StrategyStorage storage)
{
	if(storage == StrategyStorage::SdCard)
	{
		return sd_store;
	}
	return flash_store;
}

} // namespace

bool SetStrategy(Strategy strategy, StrategyStorage storage)
{
	if(!IsKnownStrategy(static_cast<uint32_t>(strategy)))
	{
		return false;
	}
	return StoreFor(storage).Save(strategy);
}

bool GetStrategy(Strategy& strategy, StrategyStorage storage)
{
	return StoreFor(storage).Load(strategy);
}

void SetRunningStrategy(Strategy strategy)
{
	running_strategy = strategy;
}

Strategy GetRunningStrategy(void)
{
	return running_strategy;
}

const char* StrategyName(Strategy strategy)
{
	for(const StrategyInfo& info : kStrategies)
	{
		if(info.strategy == strategy)
		{
			return info.name;
		}
	}
	return "?";
}

bool StrategyFromName(const char* name, Strategy& strategy)
{
	for(const StrategyInfo& info : kStrategies)
	{
		std::size_t i = 0;
		while((info.name[i] != '\0') &&
				(std::toupper(static_cast<unsigned char>(name[i])) == static_cast<unsigned char>(info.name[i])))
		{
			i++;
		}
		if((info.name[i] == '\0') && (name[i] == '\0'))
		{
			strategy = info.strategy;
			return true;
		}
	}
	return false;
}

#endif /* ROBOT_IS_MULTI_ALGORITHM */
