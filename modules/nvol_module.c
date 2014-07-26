/*
===============================================================================
 Name        : nvol_module.c
 Author      : Konstantin Werner (adapted from Embedded Systems Academy, Inc.)
 Version     : 0.1
 Copyright   : Copyright (C) Konstantin Werner
 Description : Non-Volatile Storage of Variables in Flashmemory
===============================================================================
*/

#include "env.h"

#include "nvol_module.h"
#include "iap_module.h"

// Numbers for two Sectors used for storing Non-volatile Variables
// Sectors 6 and 7 are reserved for variables in the Linker Script
#define NVOL_SECTOR1_NUM	6
#define NVOL_SECTOR2_NUM	7

#if (NVOL_SECTOR1_NUM != 6)
#error Sector Number 1 must be 6 (or linker-script must be adapted)
#endif

#if (NVOL_SECTOR2_NUM != 7)
#error Sector Number 2 must be 7 (or linker-script must be adapted)
#endif

// size of sectors - they must be the same size
#define SECTOR_SIZE			0x1000		// 4096 bytes
// Sector Start Addresses
#define SECTOR1_STARTADDR	(SECTOR1_NUM * SECTOR_SIZE)
#define SECTOR2_STARTADDR	(SECTOR2_NUM * SECTOR_SIZE)
// CPU clock in kHz
#define CPU_CLK_KHZ			(CPU_CLK / 1000)

// maximum number of variables supported
#define MAX_VARIABLES		100
// max size of variable in bytes
#define MAX_VARIABLE_SIZE	12
// invalid variable offset into a sector
#define INVALID_VAR_OFFSET	0

#if (MAX_VARIABLES >  ((SECTOR_SIZE - 48) / (MAX_VARIABLE_SIZE + 4)))
#error Too many variables for sector size
#endif

// sector flags
#define SECTOR_EMPTY        0xFFFFFF
#define SECTOR_INITIALIZING 0xAAFFFF
#define SECTOR_VALID        0xAAAAFF
#define SECTOR_INVALID      0xAAAAAA

// defines a sector
typedef struct _Sector
{
	uint8_t *Addr;                                         // sector start address
	uint8_t Num;										   // sector number
} SECTOR;

// defines a sector record
// members must be byte aligned
// must be 48 bytes in size
typedef struct __attribute__((packed)) _Sector_Record
{
	uint8_t Flags1;				                           // flags indicate sector status
	uint8_t Reserved1[15];                                 // padding
	uint8_t Flags2;				                           // flags indicate sector status
	uint8_t Reserved2[15];                                 // padding
	uint8_t Flags3;				                           // flags indicate sector status
	uint8_t Reserved3[15];                                 // padding
} SECTOR_RECORD;

// defines a variable record
// members must be byte aligned
typedef struct __attribute__((packed)) _Variable_Record
{
	uint8_t Flags;							               // flags indicate variable status
	uint16_t Id;										   // unique variable id
	uint8_t Data[MAX_VARIABLE_SIZE];					   // variable data
	uint8_t Checksum;									   // 2's complement checksum of id and data
} VARIABLE_RECORD;

// defines an entry in the variable lookup table
typedef struct _Lookup_Table_Entry
{
	uint16_t Id;							               // unique id of variable
	uint32_t Offset;									   // offset of variable record in currently valid sector
} LOOKUP_TABLE_ENTRY;


/* Module variables */

// allocate memory for non-volatile memory so it isn't used by the linker
// for something else
//static uint8_t mSectorMemory1[SECTOR_SIZE] __attribute__((at(SECTOR1_STARTADDR)));
//static uint8_t mSectorMemory2[SECTOR_SIZE] __attribute__((at(SECTOR2_STARTADDR)));

//__attribute__ ((section(".nvol"))) static uint8_t mSectorMemory1[SECTOR_SIZE];
//__attribute__ ((section(".nvol"))) static uint8_t mSectorMemory2[SECTOR_SIZE];

//static const uint8_t *mSectorMemory1 = (uint8_t*) 0x00006000;
//static const uint8_t *mSectorMemory2 = (uint8_t*) 0x00007000;

#define mSectorMemory1	(uint8_t*) 0x00006000
#define mSectorMemory2	(uint8_t*) 0x00007000

// define sectors
static SECTOR mSector1 = {mSectorMemory1, NVOL_SECTOR1_NUM};
static SECTOR mSector2 = {mSectorMemory2, NVOL_SECTOR2_NUM};

static volatile LOOKUP_TABLE_ENTRY mLookupTable[MAX_VARIABLES];	// the variable lookup table
static volatile uint16_t mNumVariables;							// number of entries in the lookup table
static volatile uint32_t mNextFreeOffset;						// the next free offset in the valid sector
static volatile SECTOR *mValidSector;							// pointer to valid sector

/* Identifies which sector is the valid one and completes any
 * partially completed operations that may have been taking place
 * before the last reset. Returns zthe valid sector or zero for error
*/
static SECTOR *NVOL_InitSectors(void);
static int32_t NVOL_EraseSector(SECTOR *Sector);
static int32_t NVOL_SetSectorFlags(SECTOR *Sector, uint32_t Flags);

// Moves the data from one sector to another, removing old entries
static int32_t NVOL_SwapSectors( SECTOR *SrcSector,	SECTOR *DstSector);
static void NVOL_ConstructLookupTable(void);
static uint32_t NVOL_GetVariableOffset(uint16_t VariableId );
static int32_t NVOL_IsVariableRecordValid(VARIABLE_RECORD *VarRec);
static uint32_t NVOL_GetNextFreeOffset(SECTOR *Sector);
int32_t NVOL_SetVariableRecord(VARIABLE_RECORD *VarRec, SECTOR *Sector,	uint16_t Offset);


/**************************************************************************
DOES:    Initializes access to non-volatile memory
RETURNS: 1 for success, 0 for error
**************************************************************************/
int32_t NVOL_Init(void)
{
	// initialize sectors
	if ((mValidSector = NVOL_InitSectors()) == 0) return 0;

	// generate lookup table
	NVOL_ConstructLookupTable();

	return 1;
}

/**************************************************************************
DOES:    Identifies which sector is the valid one and completes any
         partially completed operations that may have been taking place
		 before the last reset
RETURNS: The valid sector or null for error
**************************************************************************/
SECTOR *NVOL_InitSectors(void)
{
	volatile SECTOR_RECORD *mSec1Rec = (SECTOR_RECORD *)(mSector1.Addr);
	volatile SECTOR_RECORD *mSec2Rec = (SECTOR_RECORD *)(mSector2.Addr);
	uint32_t Sector1Flags, Sector2Flags;

	// compile sector flags into single values
	Sector1Flags = ((uint32_t)(mSec1Rec->Flags1) << 16) | ((uint32_t)(mSec1Rec->Flags2) << 8) | mSec1Rec->Flags3;
	Sector2Flags = ((uint32_t)(mSec2Rec->Flags1) << 16) | ((uint32_t)(mSec2Rec->Flags2) << 8) | mSec2Rec->Flags3;

	// if sector 1 has invalid flags then erase it
	if ((Sector1Flags != SECTOR_EMPTY)        &&
		(Sector1Flags != SECTOR_INITIALIZING) &&
		(Sector1Flags != SECTOR_VALID)        &&
		(Sector1Flags != SECTOR_INVALID))
	{
		NVOL_EraseSector(&mSector1);
		Sector1Flags = SECTOR_EMPTY;
	}

	// if sector 2 has invalid flags then erase it
	if ((Sector2Flags != SECTOR_EMPTY)        &&
		(Sector2Flags != SECTOR_INITIALIZING) &&
		(Sector2Flags != SECTOR_VALID)        &&
		(Sector2Flags != SECTOR_INVALID))
	{
		NVOL_EraseSector(&mSector2);
		Sector2Flags = SECTOR_EMPTY;
	}

	// what happens next depends on status of both sectors
	switch (Sector1Flags)
	{
		case SECTOR_EMPTY:
			switch (Sector2Flags)
			{
				// sector 1 empty, sector 2 empty
				case SECTOR_EMPTY:
					// use sector 1
					if (!NVOL_SetSectorFlags(&mSector1, SECTOR_VALID)) return 0;
					mNextFreeOffset = sizeof(SECTOR_RECORD);
					return &mSector1;
				// sector 1 empty, sector 2 initializing
				case SECTOR_INITIALIZING:
					// use sector 2
					if (!NVOL_SetSectorFlags(&mSector2, SECTOR_VALID)) return 0;
					mNextFreeOffset = sizeof(SECTOR_RECORD);
					return &mSector2;
				// sector 1 empty, sector 2 valid
				case SECTOR_VALID:
					mNextFreeOffset = NVOL_GetNextFreeOffset(&mSector2);
					// sector 2 is already active
					return &mSector2;
				// sector 1 empty, sector 2 invalid
				case SECTOR_INVALID:
					// swap sectors 2 -> 1
					if (!NVOL_SwapSectors(&mSector2, &mSector1)) return 0;
					mNextFreeOffset = NVOL_GetNextFreeOffset(&mSector1);
					// use sector 1
					return &mSector1;
			}
			break;

		case SECTOR_INITIALIZING:
		switch (Sector2Flags)
		{
			// sector 1 initializing, sector 2 empty
		case SECTOR_EMPTY:
			// use sector 1
			if (!NVOL_SetSectorFlags(&mSector1, SECTOR_VALID)) return 0;
			mNextFreeOffset = sizeof(SECTOR_RECORD);
			return &mSector1;
			// sector 1 initializing, sector 2 initializing
		case SECTOR_INITIALIZING:
			// erase sector 2
			if (!NVOL_EraseSector(&mSector2)) return 0;
			// use sector 1
			if (!NVOL_SetSectorFlags(&mSector1, SECTOR_VALID)) return 0;
			mNextFreeOffset = sizeof(SECTOR_RECORD);
			return &mSector1;
			// sector 1 initializing, sector 2 valid
		case SECTOR_VALID:
			// erase sector 1
			if (!NVOL_EraseSector(&mSector1)) return 0;
			// swap sectors 2 -> 1
			if (!NVOL_SwapSectors(&mSector2, &mSector1)) return 0;
			mNextFreeOffset = NVOL_GetNextFreeOffset(&mSector1);
			// use sector 1
			return &mSector1;
			// sector 1 initializing, sector 2 invalid
		case SECTOR_INVALID:
			// erase sector 2
			if (!NVOL_EraseSector(&mSector2)) return 0;
			// use sector 1
			if (!NVOL_SetSectorFlags(&mSector1, SECTOR_VALID)) return 0;
			mNextFreeOffset = sizeof(SECTOR_RECORD);
			return &mSector1;
		}
		break;

		case SECTOR_VALID:
		switch (Sector2Flags)
		{
			// sector 1 valid, sector 2 empty
		case SECTOR_EMPTY:
			mNextFreeOffset = NVOL_GetNextFreeOffset(&mSector1);
			// sector 1 is active
			return &mSector1;
			// sector 1 valid, sector 2 initializing
		case SECTOR_INITIALIZING:
			// erase sector 2
			if (!NVOL_EraseSector(&mSector2)) return 0;
			// swap sectors 1 -> 2	  
			if (!NVOL_SwapSectors(&mSector1, &mSector2)) return 0;
			mNextFreeOffset = NVOL_GetNextFreeOffset(&mSector2);
			// use sector 2
			return &mSector2;
			// sector 1 valid, sector 2 valid
		case SECTOR_VALID:
			// erase sector 2 and use sector 1
			if (!NVOL_EraseSector(&mSector2)) return 0;
			mNextFreeOffset = NVOL_GetNextFreeOffset(&mSector1);
			return &mSector1;
			// sector 1 valid, sector 2 invalid
		case SECTOR_INVALID:
			// erase sector 2 and use sector 1
			if (!NVOL_EraseSector(&mSector2)) return 0;
			mNextFreeOffset = NVOL_GetNextFreeOffset(&mSector1);
			return &mSector1;
		}
		break;

	case SECTOR_INVALID:
		switch (Sector2Flags)
		{
			// sector 1 invalid, sector 2 empty
		case SECTOR_EMPTY:
			// swap sectors 1 -> 2
			if (!NVOL_SwapSectors(&mSector1, &mSector2)) return 0;
			// use sector 2
			mNextFreeOffset = NVOL_GetNextFreeOffset(&mSector2);
			return &mSector2;
			// sector 1 invalid, sector 2 initializing
		case SECTOR_INITIALIZING:
			// erase sector 1
			if (!NVOL_EraseSector(&mSector1)) return 0;
			// use sector 2
			if (!NVOL_SetSectorFlags(&mSector2, SECTOR_VALID)) return 0;
			mNextFreeOffset = sizeof(SECTOR_RECORD);
			return &mSector2;
			// sector 1 invalid, sector 2 valid
		case SECTOR_VALID:
			// erase sector 1
			if (!NVOL_EraseSector(&mSector1)) return 0;
			mNextFreeOffset = NVOL_GetNextFreeOffset(&mSector2);
			// use sector 2
			return &mSector2;
			// sector 1 invalid, sector 2 invalid
		case SECTOR_INVALID:
			// both sectors invalid so erase both and use sector 1
			if (!NVOL_EraseSector(&mSector1)) return 0;
			if (!NVOL_EraseSector(&mSector2)) return 0;
			if (!NVOL_SetSectorFlags(&mSector1, SECTOR_VALID)) return 0;
			mNextFreeOffset = sizeof(SECTOR_RECORD);
			return &mSector1;
		}
		break;
	}
	return 0;
}

/**************************************************************************
DOES:    Erases a sector
RETURNS: 1 for success, 0 for error
**************************************************************************/
int32_t NVOL_EraseSector(SECTOR *Sector)
{
	unsigned long Command[5], Result[5];

	// prepare sector
	Command[0] = IAP_CMD_PREPARE_SECTOR;
	Command[1] = Sector->Num;
	Command[2] = Sector->Num;
	__disable_irq();
	IAP_cmd(Command, Result);
	__enable_irq();
	if (Result[0] != IAP_STATUS_CMD_SUCCESS) return 0; 

	// erase sector
	Command[0] = IAP_CMD_ERASE_SECTOR;
	Command[1] = Sector->Num;
	Command[2] = Sector->Num;
	Command[3] = CPU_CLK_KHZ;
	__disable_irq();
	IAP_cmd(Command, Result);
	__enable_irq();
	if (Result[0] != IAP_STATUS_CMD_SUCCESS) return 0;

	return 1; 
}

/**************************************************************************
DOES:    Updates the flags for a sector
RETURNS: 1 for success, 0 for error
**************************************************************************/
int32_t NVOL_SetSectorFlags(SECTOR *Sector,	uint32_t Flags)
{
	uint8_t Buffer[256];
	uint32_t Byte;
	SECTOR_RECORD *SecRec;
	unsigned long Command[5], Result[5];

	// set up buffer
	for (Byte = 0; Byte < 256; Byte++) Buffer[Byte] = 0xFF;

	// configure sector record
	SecRec = (SECTOR_RECORD *)Buffer;
	SecRec->Flags1 = (Flags >> 16) & 0xFF;
	SecRec->Flags2 = (Flags >> 8) & 0xFF;
	SecRec->Flags3 = Flags & 0xFF;

	// prepare sector
	Command[0] = IAP_CMD_PREPARE_SECTOR;
	Command[1] = Sector->Num;
	Command[2] = Sector->Num;
	__disable_irq();
	IAP_cmd(Command, Result);
	__enable_irq();
	if (Result[0] != IAP_STATUS_CMD_SUCCESS) return 0; 

	// write to sector
	Command[0] = IAP_CMD_COPY_RAM_TO_FLASH;
	Command[1] = (uint32_t)(Sector->Addr);
	Command[2] = (uint32_t)Buffer;
	Command[3] = 256;
	Command[4] = CPU_CLK_KHZ;
	__disable_irq();
	IAP_cmd(Command, Result);
	__enable_irq();
	if (Result[0] != IAP_STATUS_CMD_SUCCESS) return 0;

	// verify
	Command[0] = IAP_CMD_COMPARE;
	Command[1] = (uint32_t)(Sector->Addr);
	Command[2] = (uint32_t)Buffer;
	Command[3] = sizeof(SecRec);
	__disable_irq();
	IAP_cmd(Command, Result);
	__enable_irq();
	if (Result[0] != IAP_STATUS_CMD_SUCCESS) return 0;

	return 1;
}

/**************************************************************************
DOES:    Moves the data from one sector to another, removing old entries
RETURNS: 1 for success, 0 for error
**************************************************************************/
int32_t NVOL_SwapSectors(SECTOR *SrcSector, SECTOR *DstSector)
{
	uint16_t Var;
	VARIABLE_RECORD *VarRec;
	uint16_t DstOffset;

	// make sure destination sector is erased
	if (DstSector->Addr[0] != 0xFF)
	{
		// erase it
		if (!NVOL_EraseSector(DstSector)) return 0;
	}

	// mark destination sector as being initialized
	if (!NVOL_SetSectorFlags(DstSector, SECTOR_INITIALIZING)) return 0; 
	DstOffset = sizeof(SECTOR_RECORD);

	// copy variables to destination sector
	for (Var = 0; Var < mNumVariables; Var++)
	{
		VarRec = (VARIABLE_RECORD *)(SrcSector->Addr + mLookupTable[Var].Offset);
		if (!NVOL_SetVariableRecord(VarRec, DstSector, DstOffset)) return 0;
		DstOffset += sizeof(VARIABLE_RECORD);
	}

	// mark source sector as being invalid
	if (!NVOL_SetSectorFlags(SrcSector, SECTOR_INVALID)) return 0; 
	// mark destination sector as being valid
	if (!NVOL_SetSectorFlags(DstSector, SECTOR_VALID)) return 0; 
	// erase source sector
	if (!NVOL_EraseSector(SrcSector)) return 0;

	// now using destination sector
	mValidSector = DstSector;
	// get next free location in destination sector
	mNextFreeOffset = NVOL_GetNextFreeOffset(DstSector);
	// regenerate lookup table
	NVOL_ConstructLookupTable();

	return 1;
}

/**************************************************************************
DOES:    Gets the offset of the next free location in a sector
RETURNS: Returns offset (offset = SECTOR_SIZE when sector is full)
**************************************************************************/
uint32_t NVOL_GetNextFreeOffset(SECTOR *Sector)
{
	VARIABLE_RECORD *VarRec = (VARIABLE_RECORD *)(Sector->Addr + sizeof(SECTOR_RECORD));

	// loop through variable records
	while (VarRec->Flags != 0xFF)
	{
		VarRec++;
		// if reached end of sector then we are finished
		if ((uint8_t *)VarRec >= (Sector->Addr + SECTOR_SIZE)) return SECTOR_SIZE;
	}

	return (uint32_t)((uint8_t *)VarRec - Sector->Addr);
}

/**************************************************************************
DOES:    Constructs the lookup table from the valid sector
**************************************************************************/
void NVOL_ConstructLookupTable(void)
{
	VARIABLE_RECORD *VarRec = (VARIABLE_RECORD *)(mValidSector->Addr + sizeof(SECTOR_RECORD));
	uint16_t Var;
	int32_t Found;

	// no variables yet
	mNumVariables = 0;

	// loop through variable records in sector
	while (VarRec->Flags != 0xFF)
	{
		Found = 0;

		// if variable record is valid then add to lookup table
		if (NVOL_IsVariableRecordValid(VarRec))
		{
			// search for variable in lookup table. if already found then update existing entry with new offset
			for (Var = 0; Var < mNumVariables; Var++)
			{
				if (mLookupTable[Var].Id == VarRec->Id)
				{
					mLookupTable[Var].Offset = (uint16_t)((uint8_t *)VarRec - mValidSector->Addr);
					Found = 1;
					break;
				}
			}

			// if variable not in lookup table then add it
			if (!Found)
			{
				mLookupTable[mNumVariables].Id = VarRec->Id;
				mLookupTable[mNumVariables++].Offset = (uint16_t)((uint8_t *)VarRec - mValidSector->Addr);
			}
		}
		
		// move to next record
		VarRec++;

		// if reached end of sector then we are finished
		if ((uint8_t *)VarRec >= (mValidSector->Addr + SECTOR_SIZE)) break;
		// if reached maximum number of variables then we are finished
		if (mNumVariables == MAX_VARIABLES) break;
	}
}

/**************************************************************************
DOES:    Checks if a variable record is valid or not
RETURNS: 1 if valid, 0 if not valid
**************************************************************************/
int32_t NVOL_IsVariableRecordValid(VARIABLE_RECORD *VarRec)
{
	uint8_t Checksum;
	uint16_t Byte;

	// check flags
	if (VarRec->Flags != 0xAA) return 0;

	// check checksum
	Checksum = VarRec->Id & 0xFF;
	Checksum += (VarRec->Id >> 8) & 0xFF;
	for (Byte = 0; Byte < MAX_VARIABLE_SIZE; Byte++)
	{
		Checksum += VarRec->Data[Byte];
	}
	Checksum = 0x100 - Checksum;
	if (VarRec->Checksum != Checksum) return 0;

	return 1;
}

/**************************************************************************
DOES:    Gets the offset of a variable into the valid sector
RETURNS: Offset or INVALID_VAR_OFFSET if not found
**************************************************************************/
uint32_t NVOL_GetVariableOffset(uint16_t VariableId)
{
	uint16_t Var;

	for (Var = 0; Var < mNumVariables; Var++)
	{
		if (mLookupTable[Var].Id == VariableId) return mLookupTable[Var].Offset;
	}

	return INVALID_VAR_OFFSET;
}

/**************************************************************************
DOES:    Gets the value of a variable
RETURNS: 1 for success, 0 for error/not found
**************************************************************************/
int32_t NVOL_GetVariable(uint16_t Id, uint8_t *Value, uint16_t Size)
{
	uint32_t Offset;
	VARIABLE_RECORD *VarRec;
	uint16_t Byte;

	// find offset for variable
	Offset = NVOL_GetVariableOffset(Id);
	if (Offset == INVALID_VAR_OFFSET) return 0;

	// get variable record
	VarRec = (VARIABLE_RECORD *)(mValidSector->Addr + Offset);

	// copy data
	for (Byte = 0; Byte < Size; Byte++)
	{
		Value[Byte] = VarRec->Data[Byte];
	}

	return 1;
}

/**************************************************************************
DOES:    Writes a variable record into a specific sector at a
         specific offset
RETURNS: 1 for success, 0 for error
**************************************************************************/
int32_t NVOL_SetVariableRecord(VARIABLE_RECORD *VarRec,	SECTOR *Sector, uint16_t Offset)
{
	uint16_t Byte;
	uint8_t Buffer[256];
	unsigned long Command[5], Result[5];
	VARIABLE_RECORD *TmpVarRec;

	// set up buffer
	for (Byte = 0; Byte < 256; Byte++) Buffer[Byte] = 0xFF;

	// get offset of temporary variable record in 256 byte segment
	TmpVarRec = (VARIABLE_RECORD *)(Buffer + (Offset % 256));
	// copy variable record
	*TmpVarRec = *VarRec;
	for (Byte = 0; Byte < MAX_VARIABLE_SIZE; Byte++) TmpVarRec->Data[Byte] = VarRec->Data[Byte];

	// prepare sector
	Command[0] = IAP_CMD_PREPARE_SECTOR;
	Command[1] = Sector->Num;
	Command[2] = Sector->Num;
	__disable_irq();
	IAP_cmd(Command, Result);
	__enable_irq();
	if (Result[0] != IAP_STATUS_CMD_SUCCESS) return 0; 

	// write to sector
	Command[0] = IAP_CMD_COPY_RAM_TO_FLASH;
	Command[1] = (uint32_t)(Sector->Addr + Offset - (Offset % 256));
	Command[2] = (uint32_t)Buffer;
	Command[3] = 256;
	Command[4] = CPU_CLK_KHZ;
	__disable_irq();
	IAP_cmd(Command, Result);
	__enable_irq();
	if (Result[0] != IAP_STATUS_CMD_SUCCESS) return 0;

	// verify
	Command[0] = IAP_CMD_COMPARE;
	Command[1] = (uint32_t)(Sector->Addr + Offset);
	Command[2] = (uint32_t)(Buffer + (Offset % 256));
	Command[3] = sizeof(VarRec);
	__disable_irq();
	IAP_cmd(Command, Result);
	__enable_irq();
	if (Result[0] != IAP_STATUS_CMD_SUCCESS) return 0;

	return 1;
}

/**************************************************************************
DOES:    Sets the value of a variable
RETURNS: 1 for success, 0 for error
**************************************************************************/
int32_t NVOL_SetVariable(uint16_t Id,  uint8_t *Value, uint16_t Size)
{
	uint16_t Byte;
	VARIABLE_RECORD VarRec;
	uint8_t CurrentValue[MAX_VARIABLE_SIZE];
	uint16_t NumSameBytes = 0;

	// get current value for this variable, if one exists
	// and compare with new value
	if (NVOL_GetVariable(Id, CurrentValue, Size))
	{
		for (Byte = 0; Byte < Size; Byte++)
		{
			if (Value[Byte] == CurrentValue[Byte]) NumSameBytes++;
		}
	}
	// if new value is the same as the current value then no need to store
	// the new value
	if (NumSameBytes == Size) return 1;

	// if sector is full then swap sectors
	if (mNextFreeOffset >= SECTOR_SIZE)
	{
		if (mValidSector == &mSector1)
		{
			if (!NVOL_SwapSectors(&mSector1, &mSector2)) return 0;
		}
		else if (mValidSector == &mSector2)
		{
			if (!NVOL_SwapSectors(&mSector2, &mSector1)) return 0;
		}
		// if no space in new sector then no room for more variables
		if (mNextFreeOffset >= SECTOR_SIZE) return 0;
	}

	// assemble variable record
	VarRec.Flags = 0xAA;
	VarRec.Id = Id;
	VarRec.Checksum = Id & 0xFF;
	VarRec.Checksum += (Id >> 8) & 0xFF;
	for (Byte = 0; Byte < MAX_VARIABLE_SIZE; Byte++)
	{
		if (Byte < Size)
		VarRec.Data[Byte] = Value[Byte];
		else
		VarRec.Data[Byte] = 0x00;
		VarRec.Checksum += VarRec.Data[Byte];
	}
	VarRec.Checksum = 0x100 - VarRec.Checksum;

	// store record in sector
	if (!NVOL_SetVariableRecord(&VarRec, (SECTOR *)mValidSector, mNextFreeOffset)) return 0;

	// get offset of next free location
	mNextFreeOffset += sizeof(VARIABLE_RECORD);

	// add new variable record to lookup table
	NVOL_ConstructLookupTable();

	return 1;
}
