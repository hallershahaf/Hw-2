/* 046267 Computer Architecture - Spring 2016 - HW #2 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include <math.h>

// Defines and the likes...

#define MAX_HISTORY_SIZE 8
#define MAX_TABLE_SIZE (pow(2,MAX_HISTORY_SIZE))
#define DST_PC_BITS 30;
#define PC_SIZE 32;

typedef enum {
	SNT = 0,
	WNT,
	WT,
	ST
} TakenState;

// No reference to this on the pdf,
// so I am using what I found in "bp_main.c"
typedef enum {
	NOT_SHARED = 0,
	LSB_SHARE,
	MSB_SHARE
} ShareState;

typedef struct {
	// Tag has a max of 30 bits, so we can use int
	int tag;
	// Not always needed, but we allocate anyway to ease coding.
	char history;
	// for Dst pc we technically only need 30 bits, but to keep it uniform
	// we will use 32
	uint32_t dst_pc;
} BTBEntry;

// We will use max number of entries to avoid complicated allocations
// and initializations.
typedef	TakenState StateTable[MAX_TABLE_SIZE];

// Global Vars

// Need to be kept to calculate how many bits were required
// for the system.
// Possibly for other things as well.
int BTBSize;    // Possible values are 1,2,4,8,16,32
int HRSize;     // Possible values are 1...8
int TagSize;    // Possible values are 0...30

char global_HR; // max of 8 bits required
char PC_mask;   // So we can find the correct index from the pc
char HR_mask;   // In order to prevent overflow of history registers.
				// TODO: tag_mask might be needed as well...
ShareState shareState;

BTBEntry* btbTable = nullptr;
StateTable* stateArray = nullptr;

// So we know what struct to use
bool globalHist;
bool globalTable;

// For SIM_stats
int numOfFlushes;
int numOfBranches;

// Declarations

int log2(int num);

/** ACTUAL CODE **/

// math.h doesn't have a log2, so we fake one.
// Finally, my years of math studies pay out.
int log2(int num) {
	// while technically log2 can return a double,
	// btbsize is always a power of 2.
	return (int)(log(num) / log(2));
}

// NOTE - No deallocations occur within the confines of this exercise.
// Memory leaks will not be checked.

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize,
             bool isGlobalHist, bool isGlobalTable, int Shared){

	// Keep all these for future reference.
	BTBSize = btbSize;
	HRSize = historySize;
	TagSize = tagSize;
	globalHist = isGlobalHist;
	globalTable = isGlobalTable;

	int numOfBits = log2(BTBSize);
	PC_mask = 0xffff >> (PC_SIZE - numOfBits);

	HR_mask = 0xff >> (MAX_HISTORY_SIZE - historySize);
	global_HR = 0;
	shareState = Shared;

	// A BTB table needs to be allocated regardless of all other options
	btbTable = malloc(btbSize * sizeof(BTBEntry));
	if (!btbTable)
		goto bad_alloc;

	// TODO: Should dst pc be 0 on init?
	// That would mean that first branch predicts always give 0.
	memset(btbTable, 0, btbSize * sizeof(BTBEntry)); // init

	// Now to allocate either 1 or btbSize state tables.
	int tableSize = (isGlobalTable) ? 1 : btbSize;
	StateArray = malloc(tableSize * sizeof(StateTable));
	if (!StateArray)
		goto bad_alloc;

	// not sure how to use memset when not setting all to 0
	for (int i = 0; i < tableSize; i++) {
		for (int j = 0; j < (MAX_TABLE_SIZE); j++) {
			btbTable[i][j] = WNT;
		}
	}

	return 0;

bad_alloc:
	free(BTB_table);
	free(StateArray);
	return -1;
}

bool BP_predict(uint32_t pc, uint32_t *dst){
	numOfBranches++;

	int index = (pc >> 2) & PC_mask;
	BTBEntry btb_entry = btbTable[index];

	StateTable *table = (globalTable) ? stateArray : stateArray[index];

	int table_index = (globalHist) ? global_HR : btb_entry.history;

	// no need to check for GlobalTable because input is assumed correct.
	int xor_mask = 0xffff >> (PC_SIZE - HRSize);
	if (ShareState == LSB_SHARE)
		table_index ^= (pc >> 2) & xor_mask;
	if (ShareState == MSB_SHARE)
		table_index ^= (pc >> 16) & xor_mask;

	TakenState state = table[table_index];
	if (state == WT || state == ST) {
		*dst = btb_entry.dst_pc;
		return true;
	}

	// TODO: are more things needed here? anything to update here rather than in 
	// BP_update?

	*dst = pc + 4; 
	return false;
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
	return;
}

void BP_GetStats(SIM_stats *curStats) {
	int numOfBits = 0;
	// Tag bits
	numOfBits += BTBSize * TagSize; 
	// History bits
	numOfbits += (globalHist) ? HRSize : (HRSize * BTBSize);
	// Dst PC bits
	numOfBits += BTBSize * DST_PC_BITS;
	// State Array bits (state machine is 2 bits)
	numOfBits += (globalTable) ? (2 * pow(2,HRSize)) : (2 * pow(2,HRSize) * BTBSize);

	curStats->br_num = numOfBranches;
	curStats->flush_num = numOfFlushes;
	curStats->size = numOfBits;
	return;
}
