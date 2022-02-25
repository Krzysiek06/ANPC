/*
 * SPI_PROJ.h
 *
 *  Created on: 28 lip 2021
 *      Author: Rafal Miskiewicz
 */

#ifndef SPI_PROJ_H_
#define SPI_PROJ_H_


#include "f28x_project.h"
#include "driverlib.h"
#include "device.h"

//Definicje ADS7028
//OPCODE

#define NOP 0x00
#define SRR 0x10
#define SRW 0x08
#define SB  0x18
#define CB  0x20


//Adresy
#define SYSTEM_STATUS









#define CS_HIGH 0
#define CS_LOW 1



#define CS1LOW GpioDataRegs.GPBCLEAR.bit.GPIO57=1;
#define CS1HI GpioDataRegs.GPBSET.bit.GPIO57=1;

#define CS2LOW GpioDataRegs.GPBCLEAR.bit.GPIO58=1;
#define CS2HI GpioDataRegs.GPBSET.bit.GPIO58=1;

#define CS3LOW GpioDataRegs.GPBCLEAR.bit.GPIO59=1;
#define CS3HI GpioDataRegs.GPBSET.bit.GPIO59=1;
//
// Function Prototypes
//
void initSPI(void);
uint16_t readStatusRegister(void);
void writeDataSPI(uint8_t rw, uint16_t address, uint16_t  data);
void readDataSPI(uint16_t address, uint16_t * data, uint16_t length);
void enableWrite(void);
void readDataSPIADC70(uint16_t address, uint16_t *dataout, uint16_t *oldadress);

#endif /* SPI_PROJ_H_ */
