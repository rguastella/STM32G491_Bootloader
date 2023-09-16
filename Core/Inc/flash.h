/***********************************************************************************************//**
 * @file       flash.h
 * @details    This module is responsible for providing all the support routines for interfacing
 *             to the Flash memory device.
 *
 * @author     Robert Guastella
 ***************************************************************************************************/
#ifndef FLASH_H_
#define FLASH_H_
#include <stdbool.h>

/***************************************************************************************************
 * CONSTANTS AND DEFINITIONS
 **************************************************************************************************/
#define		EXT_FLASH_APPL_STARTING_ADDRESS		0x8000000
#define		APP_START_ADDRESS					0x8010000

/***************************************************************************************************
 * TYPEDEFS
 **************************************************************************************************/

/***************************************************************************************************
 * PUBLIC FUNCTION PROTOTYPES
 **************************************************************************************************/
int FileOpen(char mode, unsigned long startaddr);
void FileClose(int f);
bool FileExist(unsigned long startaddr);
int FileGets(int f, char *p, int len);
int FileRead(int f, char *p, int len);
void FileDelete(unsigned long startaddr);
int FileWrite(int f, char *p, unsigned long len);

#endif
