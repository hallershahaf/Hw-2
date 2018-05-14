/* 046267 Computer Architecture - Spring 2016 - HW #2 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// DEBUGGING TOOLS

// DEBUG( ... ); printf( __VA_ARGS__ );
// static const char  *StateToString[] = {"SNT", "WNT", "WT", "ST"};


// Defines and the likes...

#define MAX_HISTORY_SIZE 8
#define MAX_TABLE_SIZE 256
#define DST_PC_BITS 30
#define PC_SIZE 32
#define MAX_BTB_BITS 5

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
	int history;
	// We need use 32 bits for dst_pc,
	// though technically we can do with 30
	uint32_t dst_pc;
} BTBEntry;

// We will use max number of entries to avoid complicated allocations
// and initializations.
typedef	TakenState StateTable[MAX_TABLE_SIZE];

// Global Vars

// These are needed for overall predictor bit count, and for
// various mask calculations.
int BTBSize;    // Possible values are 1,2,4,8,16,32
int HRSize;     // Possible values are 1...8
int TagSize;    // Possible values are 0...30

int global_HR; // Max of 8 bits required
unsigned int PC_mask;   // So we can find the correct index from the pc
unsigned int HR_mask;   // In order to prevent overflow of history registers.
unsigned int tag_mask;  // So we can compare PC and tags.

bool decision;  // This is so we won't have to find the prediction both for 
				// predict and for update.

BTBEntry* btbTable = NULL;
StateTable* stateArray = NULL;

// So we know what struct to use
bool globalHist;
bool globalTable;
ShareState shareState;

// For SIM_stats
int numOfFlushes;
int numOfBranches;

// Declarations

void zeroEntry(int index);

/** ACTUAL CODE **/

// In case tags didn't match, we want to zero the relevant entry.
void zeroEntry(int index) {

	memset(&btbTable[index], 0, sizeof(BTBEntry));

	if (!globalTable) { // If the state table is not shared, we want to zero that too
		for (int i = 0; i < MAX_TABLE_SIZE; i++) {
			(stateArray[index])[i] = WNT;
		}
	}
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
	PC_mask = 0x1f >> (MAX_BTB_BITS - numOfBits);
	HR_mask = 0xff >> (MAX_HISTORY_SIZE - historySize);
	tag_mask = 0xffffffff >> (32 - tagSize);

	global_HR = 0;
	shareState = Shared;

	// A BTB table needs to be allocated regardless of all other options
	btbTable = malloc(btbSize * sizeof(BTBEntry));
	if (!btbTable)
		goto bad_alloc;

	memset(btbTable, 0, btbSize * sizeof(BTBEntry));

	// Now to allocate either 1 or btbSize state tables.
	int tableSize = (isGlobalTable) ? 1 : btbSize;
	stateArray = malloc(tableSize * sizeof(StateTable));
	if (!stateArray)
		goto bad_alloc;

	for (int i = 0; i < tableSize; i++) {
		for (int j = 0; j < (MAX_TABLE_SIZE); j++) {
			(stateArray[i])[j] = WNT;
		}
	}
	return 0;

bad_alloc:
	free(btbTable);
	free(stateArray);
	btbTable = NULL;
	stateArray = NULL;
	return -1;
}

bool BP_predict(uint32_t pc, uint32_t *dst){

	numOfBranches++;

	int index = (pc >> 2) & PC_mask;

	int tag = btbTable[index].tag;
	int pc_tag = (pc >> 2) & tag_mask;

	// In the case that (TagSize == 0), not point in even checking.
	if ((TagSize != 0) && (pc_tag != tag)) {
		zeroEntry(index);
		goto not_taken;
	}

	StateTable* table = (globalTable) ? stateArray : (stateArray + index);
	int table_index = (globalHist) ? global_HR : (btbTable[index].history);

	if (globalTable && shareState == LSB_SHARE) {
		table_index ^= (pc >> 2) & HR_mask;
	}
	if (globalTable && shareState == MSB_SHARE) {
		table_index ^= (pc >> 16) & HR_mask;
	}

	TakenState state = (*table)[table_index];

	if (state == WT || state == ST) {
		*dst = btbTable[index].dst_pc;
		decision = true;
		return true;
	}

not_taken:
	*dst = pc + 4; 
	decision = false;
	return false;
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){

	if ((taken ^ decision) || (taken && (pred_dst != targetPc)))
		numOfFlushes++;

	int index = (pc >> 2) & PC_mask;
	BTBEntry* btb_entry = &(btbTable[index]);

	// Update tag
	// At worst, this does nothing.
	int new_tag = (pc >> 2) & tag_mask;
	btb_entry->tag = new_tag;

	StateTable* table = (globalTable) ? &(stateArray[0]) : &(stateArray[index]);
	int *history = (globalHist) ? &global_HR : &(btb_entry->history);
	int table_index = *history;

	if (globalTable && shareState == LSB_SHARE) {
		table_index ^= (pc >> 2) & HR_mask;
	}
	if (globalTable && shareState == MSB_SHARE) {
		table_index ^= (pc >> 16) & HR_mask;
	}

	// Update relevant state
	switch ((*table)[table_index]) {
		case (SNT):
			if (taken) {
				(*table)[table_index] = WNT;
			}
			break;
		case (WNT):
			if (taken) {
				(*table)[table_index] = WT;
			}
			else {
				(*table)[table_index] = SNT;
			}
			break;
		case(WT):
			if (taken) {
				(*table)[table_index] = ST;
			}
			else {
				(*table)[table_index] = WNT;
			}
			break;
		case(ST):
			if (!taken) {
				(*table)[table_index] = WT;
			}
	}

	//Update history
	int added_bit = (taken) ? 1 : 0;
	*history = ((*history << 1) + added_bit) & HR_mask;

	// Update dst_pc
	btb_entry->dst_pc = targetPc;

	return;
}

void BP_GetStats(SIM_stats *curStats) {
	int numOfBits = 0;
	// Tag bits
	numOfBits += BTBSize * TagSize; 
	// History bits
	numOfBits += (globalHist) ? HRSize : (HRSize * BTBSize);
	// Dst PC bits
	numOfBits += BTBSize * DST_PC_BITS;
	// State Array bits (state machine is 2 bits)
	numOfBits += (globalTable) ? (2 * pow(2,HRSize)) : (2 * pow(2,HRSize) * BTBSize);

	curStats->br_num = numOfBranches;
	curStats->flush_num = numOfFlushes;
	curStats->size = numOfBits;
	return;
}
