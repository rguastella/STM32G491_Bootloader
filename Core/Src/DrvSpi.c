/*********************************************************
 * @file   DrvSpi.c
 * @author Robert Guastella - Design Integrity
 * @date   06/21/2023
 * @brief  Low level SPI routines.
 *
 * Contains the low level Read, Write and Chip Select
 * routine for the SPI bus.
 ********************************************************/
#include "main.h"
#include <stdint.h>
#include <DrvTypeDefs.h>
#include <DrvSpi.h>

extern SPI_HandleTypeDef hspi3;

/**************************************************************************
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
**************************************************************************/
void DrvSpiInit(void)
{
	DrvSpiSelectDevice(SPI_DEVICE_EEPROM, SPI_CS_DEACTIVE);
	DrvSpiSelectDevice(SPI_DEVICE_FLASH, SPI_CS_DEACTIVE);
}

/*--------------------------------------------------------------------
Function Name:	spiWrite()

Purpose:	This function writes data to the SSI port.

Input(s):	Pointer to data
			Size of the data to be written

Output(s):	Write Status
----------------------------------------------------------------------*/
HAL_StatusTypeDef spiWrite(uint8_t *pData, uint16_t Size)
{
	return(HAL_SPI_Transmit(&hspi3, pData, Size, 100));
}

/*--------------------------------------------------------------------
Function Name:	spiRead()

Purpose:	This function reads from the SSI port.

Input(s):	None
Output(s):	None
----------------------------------------------------------------------*/
HAL_StatusTypeDef spiRead(uint8_t *pData, uint16_t Size)
{
	return(HAL_SPI_Receive(&hspi3, pData, Size, 100));
}

/*--------------------------------------------------------------------
Function Name:	spiWrite()

Purpose:	This function writes data to the SSI port.

Input(s):	Pointer to data
			Size of the data to be written

Output(s):	Write Status
----------------------------------------------------------------------*/
HAL_StatusTypeDef spiWriteByte(uint8_t Data)
{
uint8_t pData;

	pData = Data;
	return(HAL_SPI_Transmit(&hspi3, &pData, 1, 100));
}

/*--------------------------------------------------------------------
Function Name:	spiRead()

Purpose:	This function reads from the SSI port.

Input(s):	None
Output(s):	None
----------------------------------------------------------------------*/
uint8_t spiReadByte(void)
{
uint8_t pData;

	HAL_SPI_Receive(&hspi3, &pData, 1, 100);
	return(pData);
}

/*--------------------------------------------------------------------
Function Name:	spiReadReg()

Purpose:	This function reads from the SSI port's DR register.

Input(s):	None
Output(s):
----------------------------------------------------------------------*/
uint32_t spiReadReg(void)
{
uint32_t data;

	data = hspi3.Instance->DR;
	return(data);
}

/***************************************************************
 * @name    DrvSpiSelectDevice
 * @brief   Selects SPI device
 * @ingroup Drivers
 *
 * This function selects the passed SPI device.
 * @param _SpiDevice
 * * SPI_DEVICE_NONE
 * * SPI_DEVICE_EEPROM
 * * SPI_DEVICE_STEPPER
 *
 * @retval true or false
 *
 * Example Usage:
 * @code
 *    DrvSpiSelectDevice(SPI_DEVICE_EEPROM, SPI_CS_ACTIVE);
 * @endcode
*****************************************************************/
BOOL DrvSpiSelectDevice(uint8_t device, uint8_t state)
{
	switch (device) {
	case SPI_DEVICE_EEPROM:
		if(state == SPI_CS_ACTIVE) {
			HAL_GPIO_WritePin(EEPROM_NSS_GPIO_Port, EEPROM_NSS_Pin, GPIO_PIN_RESET);
			return(true);
		}
		else {
			HAL_GPIO_WritePin(EEPROM_NSS_GPIO_Port, EEPROM_NSS_Pin, GPIO_PIN_SET);
			return(true);
		}
		break;

	case SPI_DEVICE_FLASH:
		if(state == SPI_CS_ACTIVE) {
			HAL_GPIO_WritePin(FLASH_NSS_GPIO_Port, FLASH_NSS_Pin, GPIO_PIN_RESET);
			return(true);
		}
		else {
			HAL_GPIO_WritePin(FLASH_NSS_GPIO_Port, FLASH_NSS_Pin, GPIO_PIN_SET);
			return(true);
		}
		break;

	default:
		return(false);
		break;
	}
	return(false);
}
