/***********************************************************************************************//**
 * @file       flash.c
 * @details    This module is responsible for providing all the support routines for interfacing
 *             to the Flash memory device.
 *
 * @author     Robert Guastella
 ***************************************************************************************************/
/***************************************************************************************************
 * INCLUDES
 ***************************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "DrvTypeDefs.h"
#include "DrvSpi.h"

/***************************************************************************************************
 * CONSTANTS AND DEFINITIONS
 **************************************************************************************************/

/***************************************************************************************************
 * MODULE VARIABLES
 **************************************************************************************************/
static int OpenFile = 0;
static unsigned long FilePosition = 0;
static char Mode = 0;

/***************************************************************************************************
 * PRIVATE FUNCTION PROTOTYPES
 **************************************************************************************************/
static void SectorErase(void);
static void WriteEnable(void);
static void WaitBusy(void);
static void Flash_Chip_Select(bool select);
static void SendCommandAndAddress(unsigned char Command);

/***************************************************************************************************
 * FUNCTION DEFINITIONS
 **************************************************************************************************/
///*****************************************************************************
//** Function name:		FlashOpen()
//**
//** Descriptions:		Function is responsible for opening the Flash memory
//**					device for a read or write operation.
//**
//** parameters:		Mode 'r', 'R', 'w', or 'W'
//**					startaddr is the starting address of the desired file.
//**
//** Returned value:		None zero file handle. If it is 0 open failed.
//**
//*****************************************************************************/
int FileOpen(char mode, unsigned long startaddr)
{
	/* If the file is already open return failed. */
	if (OpenFile) {
		return (0);
	}

	if (mode >= 'a' && mode <= 'z')
		mode -= ('a'-'A');
	switch(mode) {
		case 'W':
		case 'R':
			OpenFile = 1;
			FilePosition = startaddr;
			Mode = mode;
			break;
	}
	return(OpenFile);
}

/*****************************************************************************
** Function name:		FlashClose()
**
** Descriptions:		This function closes the Flash memory device.
**
** parameters:			f is the open file
** Returned value:		None
**
*****************************************************************************/
void FileClose(int f)
{
	OpenFile = 0;
}

/*****************************************************************************
** Function name:		FlashGets()
**
** Descriptions:		This function reads a line of text from the Flash
**						memory device.
**
** parameters:			f is the already open file.
**						p is a pointer to the data that is to be written.
**						len is the maximum length of data to read.
**
** Returned value:		The number of data bytes read.
**
*****************************************************************************/
int FileGets(int f, char *p, int len)
{
char r;
int LenRead = 0;

	if (OpenFile && (OpenFile == f) && len && p) {
		Flash_Chip_Select(true);
		HAL_Delay(1);
		SendCommandAndAddress(0x03);

		// Read the data from the SPI flash at FilePosition (incrementing as
		// it is read) use a NULL or 0xff as an end of file marker
		while(LenRead < len){
	        HAL_Delay(1);
			spiWriteByte(0);
	        HAL_Delay(1);
			r = (char)(spiReadByte() & 0xff);
			if (r == 0xff || r == 0x00) {
				*p = 0;
				break;
			}
			*p++ = r;
			FilePosition++;
			LenRead++;
			if (r == 0x0a)
				break;
		}
		Flash_Chip_Select(false);
	}
	return(LenRead);
}

/*****************************************************************************
** Function name:		FlashRead()
**
** Descriptions:		This function reads data from the Flash memory device.
**
** parameters:			f is the already open file.
**						p is a pointer to the data that is to be written.
**						len is the maximum length of data to read.
**
** Returned value:		The number of data bytes read.
**
*****************************************************************************/
int FileRead(int f, char *p, int len)
{
char r;
int LenRead = 0;

	if (OpenFile && (OpenFile == f) && len && p) {
		HAL_GPIO_WritePin(FLASH_NSS_GPIO_Port, FLASH_NSS_Pin, GPIO_PIN_SET);		// Toggle the CS line
		HAL_Delay(1);
		HAL_GPIO_WritePin(FLASH_NSS_GPIO_Port, FLASH_NSS_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		SendCommandAndAddress(0x03);

		// Read the data from the SPI flash at FilePosition (incrementing as
		// it is read) use a NULL or 0xff as an end of file marker
		while(LenRead < len){
			HAL_Delay(1);
			spiWriteByte(0);
			r = (char)(spiReadByte() & 0xff);
			*p++ = r;
			FilePosition++;
			LenRead++;
		}
	}
	return(LenRead);
}

///*****************************************************************************
//** Function name:		FlashDelete()
//**
//** Descriptions:		Function erase the file at the starting address.
//**
//** parameters:		startaddr is the starting address of the desired file.
//**
//** Returned value:	None
//**
//*****************************************************************************/
void FileDelete(unsigned long startaddr)
{
	FilePosition = startaddr;
	SectorErase();
}

/*****************************************************************************
** Function name:		FlashWrite()
**
** Descriptions:		This function writes data to the Flash memory device.
**
** parameters:			f is the already open file
**						p is a pointer to the NULL terminated data that is to
**						  be written is the flash device.
**
** Returned value:		Number of bytes written to the flash device.
**
*****************************************************************************/
int FileWrite(int f, char *p, unsigned long len)
{
int written = 0;
unsigned long cnt = 0;

	if (OpenFile && (OpenFile == f) && Mode == 'W') {
		if ((FilePosition & 0xffff) == 0) {
			SectorErase();
		}
		WriteEnable();
		Flash_Chip_Select(true);
		SendCommandAndAddress(0x02);
		while(cnt++ < len) {
			spiWriteByte(*p++);
			written++;
			FilePosition++;
			if ((FilePosition & 0xff) == 0) {
		        HAL_Delay(1);
				Flash_Chip_Select(false);
		        HAL_Delay(1);
				spiWriteByte(0);
		        HAL_Delay(1);
				WaitBusy();
		        HAL_Delay(1);
				if ((FilePosition & 0xffff) == 0) {
					SectorErase();
				}
				WriteEnable();
				Flash_Chip_Select(true);
				SendCommandAndAddress(0x02);
			}
		}
		Flash_Chip_Select(false);
		WaitBusy();
	}
	return(written);
}

///*****************************************************************************
//** Function name:		FileExist()
//**
//** Descriptions:		This function checks for the existence of a file.
//**
//** parameters:		startaddr is the starting address of the desired file.
//**
//** Returned value:	true if the file exist else false.
//**
//*****************************************************************************/
bool FileExist(unsigned long startaddr) {
	bool exist = false;
	char buf[10];
	if(FileOpen('r', startaddr)) {
		exist = (FileGets(OpenFile, buf, sizeof(buf)) > 0);
		FileClose(OpenFile);
	}
	return(exist);
}

/*****************************************************************************
** Function name:		SectorErase()
**
** Descriptions:		Function erase the current sector.
**
** parameters:			None
** Returned value:		None
**
*****************************************************************************/
static void SectorErase()
{
	HAL_Delay(1);
	WriteEnable();
	HAL_Delay(1);
	Flash_Chip_Select( 1 );
	HAL_Delay(1);
	spiWriteByte(0xc7);
	Flash_Chip_Select( 0 );
	HAL_Delay(1);
	WaitBusy();

}

/*****************************************************************************
** Function name:		WriteEnable()
**
** Descriptions:		Function puts the chip in write enable mode.
**
** parameters:			None
** Returned value:		None
**
*****************************************************************************/
static void WriteEnable(void)
{
	HAL_GPIO_WritePin(FLASH_NSS_GPIO_Port, FLASH_NSS_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
	spiWriteByte(0x06);
    HAL_Delay(1);
	HAL_GPIO_WritePin(FLASH_NSS_GPIO_Port, FLASH_NSS_Pin, GPIO_PIN_SET);
}

/*****************************************************************************
** Function name:		WaitBusy()
**
** Descriptions:		Function will wait for the write to complete.
**
** parameters:			None
** Returned value:		None
**
*****************************************************************************/
static void WaitBusy(void)
{
unsigned int r;

// Check the status register for completion
	HAL_GPIO_WritePin(FLASH_NSS_GPIO_Port, FLASH_NSS_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
	HAL_GPIO_WritePin(FLASH_NSS_GPIO_Port, FLASH_NSS_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
	spiWriteByte(0x05);
    HAL_Delay(1);
	do {
		spiWriteByte(0);
		r = spiReadByte();
	} while ((r & 0x00000001) == 0x00000001);
	HAL_GPIO_WritePin(FLASH_NSS_GPIO_Port, FLASH_NSS_Pin, GPIO_PIN_SET);
}

/*****************************************************************************
** Function name:		SendCommandAndAddress()
**
** Descriptions:		Function will write a command followed by the current
**						address to the flash chip.
**
** parameters:			The command
** Returned value:		None
**
*****************************************************************************/
static void SendCommandAndAddress(unsigned char Command)
{
	spiWriteByte(Command);
	spiWriteByte((FilePosition >> 16) & 0xff);
	spiWriteByte((FilePosition >>  8) & 0xff);
	spiWriteByte((FilePosition) & 0xff);
	spiReadByte();
}

/***************************************************************************************************
 * @details:    Function will controls the chip select line to the Flash chip.
 *
 * @param[in]   Select
 **************************************************************************************************/
static void Flash_Chip_Select(bool select)
{
	if(select == true) {
		HAL_GPIO_WritePin(FLASH_NSS_GPIO_Port, FLASH_NSS_Pin, GPIO_PIN_RESET);
	}
	else {
		HAL_GPIO_WritePin(FLASH_NSS_GPIO_Port, FLASH_NSS_Pin, GPIO_PIN_SET);
	}
}




