/***********************************************************************************************
@Module Name: intel.h

@Purpose:	This module is the header file for intel.c

@author     Robert Guastella
***********************************************************************************************/
#ifndef __INTEL_H__
#define __INTEL_H__

typedef struct {
	  unsigned char	ByteCount;
	  unsigned long Address;
	  unsigned char Recordtype;
	  unsigned char Data[0x20];
	  unsigned char Checksum;
} INTELHEXREC;

bool secondaryBootloaderPresent(void);
bool CheckIntelHexFile(unsigned long FileAddress, unsigned long *StartAddress, unsigned long *EndAddress);
bool ReadIntelHexLine(int file, INTELHEXREC *rec);


#endif
