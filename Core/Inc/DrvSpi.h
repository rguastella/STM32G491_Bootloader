#ifndef DRV_SPI_H
#define DRV_SPI_H

#include "main.h"
#include "DrvTypeDefs.h"

enum {
   SPI_DEVICE_EEPROM = 0,
   SPI_DEVICE_FLASH,
   SPI_CS_ACTIVE,
   SPI_CS_DEACTIVE
};

//#define SPI_DEVICE_FLASH SPI_DEVICE_FLASH0

void DrvSpiInit(void);

void DrvSpiExtCsEnabled(void);
void DrvSpiExtCsDisabled(void);

HAL_StatusTypeDef spiRead(uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef spiWrite(uint8_t *pData, uint16_t Size);

uint8_t spiReadByte(void);
uint32_t spiReadReg(void);
HAL_StatusTypeDef spiWriteByte(uint8_t Data);
BOOL DrvSpiSelectDevice(uint8_t device, uint8_t state);

#endif
