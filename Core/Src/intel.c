/***********************************************************************************************//**
 * @file       intel.c
 * @details    Read and program an Intel hex file into the internal flash memory.
 *
 * @author     Robert Guastella
 ***************************************************************************************************/
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
//#include "inc/hw_types.h"
//#include "inc/hw_flash.h"
//#include "driverlib/rom.h"
#include "flash.h"
#include "intel.h"

int linenum = 0;

static unsigned char calc_checksum(char *p, int len);
static char *EndOfLegalDigits(char *p);
static unsigned char AsciiHextobyte(char *str);

/*--------------------------------------------------------------------
Function Name:	CheckIntelHexFile()
Purpose:		Check for a valid hex file in external flash memory.

Input(s):	FileAddress, StartAddress, EndAddress pointers
Output(s):	TRUE - valid Intel HEX file found in Flash memory
			FALSE - no Intel HEX file found in flash memory
----------------------------------------------------------------------*/
bool CheckIntelHexFile(unsigned long FileAddress, unsigned long *StartAddress, unsigned long *EndAddress)
{
static INTELHEXREC	r;
int	file;
unsigned long AddressOffset = 0;
unsigned long MinAddress = ~0;
unsigned long MaxAddress = 0;
bool EndOfFileMarker = false;

	linenum = 0;

	file = FileOpen('r', FileAddress);
	if (file){
		while(ReadIntelHexLine(file, &r) && (EndOfFileMarker == false)){

			linenum++;

			switch(r.Recordtype){
			case 0:
				r.Address += AddressOffset;
				if (r.Address < MinAddress)	{
					MinAddress = r.Address;
				}
				if ((r.Address + r.ByteCount) > MaxAddress)	{
					MaxAddress = r.Address + r.ByteCount;
				}
				break;
			case 1:
				EndOfFileMarker = true;
				break;
			case 4:
				AddressOffset = 0x00010000 * ((r.Data[0] << 8) + r.Data[1]);
				break;
			default:
				//
				// Unknown record type.  Just close the file and return false.
				//
				FileClose(file);
				return(false);
			}
		}
		FileClose(file);
	}
	if (StartAddress) *StartAddress = MinAddress;
	if (EndAddress)	  *EndAddress = MaxAddress;
	return(EndOfFileMarker);
}

/*--------------------------------------------------------------------
Function Name:	ReadIntelHexLine()
Purpose:		Reads one line of Intel HEX from Flash memory.

Input(s):	file, pointer to Intel HEX data structure
Output(s):	TRUE - valid Intel HEX file found in Flash memory
			FALSE - invalid Intel HEX file found in flash memory
----------------------------------------------------------------------*/
bool ReadIntelHexLine(int file, INTELHEXREC *rec)
{
int i;
static char Ascii[100];

	if (file && rec){
		//
		// Initialize the entire data section to all FF.  This is based on the
		// assumption that the content of the hex record is going to be
		// programmed into flash memory.  And if there are unused portions of
		// the data then they will be left un-programmed.
		//
		for(i=0; i < sizeof(rec->Data); i++)
			rec->Data[i] = 0xff;

		memset(Ascii, 0, sizeof(Ascii));

		if (FileGets(file, Ascii, sizeof(Ascii))){
			if(Ascii[0] == ':'){
				/* Terminate the line after the last ASCII digit.	*/
				*EndOfLegalDigits(&Ascii[1]) = 0;
	
				/* The minimum record should contain a byte count (2),
				   address (4), record type (2), and a checksum (2) for a
				   total of 10 bytes. If the record is less then that assume
				   it is corrupt.
				*/
				if (strlen(&Ascii[1]) >= 10){
					rec->ByteCount = AsciiHextobyte(&Ascii[1]);
					if (rec->ByteCount > sizeof(rec->Data)) {
						return(0);
					}
					rec->Address = (AsciiHextobyte(&Ascii[3]) << 8) + AsciiHextobyte(&Ascii[5]);
					rec->Recordtype = AsciiHextobyte(&Ascii[7]);

					/* Read in the data. */
					for(i=0; i < rec->ByteCount; i++)
						rec->Data[i] = AsciiHextobyte(&Ascii[(2*i)+9]);
					/*	Then verify the calculated check sum matches what is
						at the end of the record. 
					*/
					rec->Checksum = AsciiHextobyte(&Ascii[(2*i)+9]);
					if(calc_checksum(&Ascii[1], strlen(Ascii) - 3) == rec->Checksum){
						return(true);
					}
				}
			}
		}
	}
	return(false);
}

/******************************************************************************
*	This function calculates the checksum on ASCII data contained in buffer
*	pointed to by "p" for lentgh of "len".
******************************************************************************/
static unsigned char calc_checksum(char *p, int len)
{
unsigned char i, sum = 0;

	for (i = 0; i < len; i += 2){
		sum += AsciiHextobyte(&p[i]);
	}
	sum = ((~sum)+1) & 0x00ff;
	return(sum);
}


/******************************************************************************
	Searches for the first instance of a character that is outside an ASCII
	representation of a hex digit. All '0'-'9', 'a'-'f', and 'A'-'F' are
	acceptable. Will return a pointer to the first instance of a charcter
	outside that range.
*******************************************************************************/
static char *EndOfLegalDigits(char *p)
{
	do {
		if (*p < '0' || (*p > '9' && *p < 'A') || (*p > 'F' && *p < 'a') || *p > 'f')
			break;
	}while(*++p);
	return(p);
}

/******************************************************************************
* This takes a pointer to a ywo byte hex string and converts it to an
* unsigned char
******************************************************************************/
static unsigned char AsciiHextobyte(char *str)
{
	unsigned char val;

	val = (*str <= '9') ? (*str++ & 0x0f) * 16 : (((*str++ -'A') + 10) * 16 );
	val |= (*str <= '9') ? (*str & 0x0f) : ((*str-'A') + 10);
	return(val);
}
