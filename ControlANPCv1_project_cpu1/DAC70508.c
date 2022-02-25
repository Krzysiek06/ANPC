/*
 * DAC70508.c
 *
 *  Created on: 25 sie 2021
 *      Author: Rafal Miskiewicz
 */


#include "DAC70508.h"

#include "driverlib.h"
#include "device.h"


void writeSPIDAC70508(uint16_t address, uint16_t * data, uint16_t length){
    CS_LOW_DAC70508;


    // Dummy read to clear INT_FLAG.
    //
    SPI_readDataBlockingNonFIFO(SPIA_BASE);



    CS_HIGH_DAC70508;

}
void readSPIDAC70508(uint16_t address, uint16_t * data, uint16_t length){

    CS_LOW_DAC70508;




    CS_HIGH_DAC70508;
}
