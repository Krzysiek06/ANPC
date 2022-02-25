/*
 * SPI_PROJ.c
 *
 *  Created on: 28 lip 2021
 *      Author: Rafal Miskiewicz
 */


#include <SPI_PROJ.h>
#include "GPIO_PROJ.h"

#include "driverlib.h"
#include "device.h"



void initSPI()
{
    //
    // Must put SPI into reset before configuring it.
    //
    SPI_disableModule(SPIA_BASE);
    //
    // SPI configuration. Use a 2MHz SPICLK and 8-bit word size.
    //
    SPI_setConfig(SPIA_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL1PHA0,
                  SPI_MODE_MASTER, 1000000, 8);

     //SPI_setConfig(SPIA_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA1,
     //              SPI_MODE_MASTER, 1000000, 8);
    //
    // Configuration complete. Enable the module.
    //
    SPI_enableModule(SPIA_BASE);
}






//
// Function to send RDSR opcode and return the status of the EEPROM
//
uint16_t readStatusRegister(void)
{
    uint16_t temp;
    //
    // Pull chip select low.
    //
    CS_LOW;
    //
    // Send RDSR opcode
    //
  //  SPI_writeDataBlockingNonFIFO(SPIA_BASE, RDSR);
    //
    // Dummy read to clear INT_FLAG.
    //
    temp = SPI_readDataBlockingNonFIFO(SPIA_BASE);
    //
    // Send dummy data to receive the status.
    //
    SPI_writeDataBlockingNonFIFO(SPIA_BASE, 0x0000);
    //
    // Read status register.
    //
    temp = SPI_readDataBlockingNonFIFO(SPIA_BASE);
    //
    // Pull chip select high.
    //
    CS_HIGH;
    //
    // Read the status from the receive buffer
    //
    return(temp);
}

//
// Function to send the WREN opcode
//
void enableWrite(void)
{
    //
    // Pull chip select low.
    //
    CS_LOW;
    //
    // Send the WREN opcode.
    //
  //  SPI_writeDataBlockingNonFIFO(SPIA_BASE, WREN);
    //
    // Dummy read to clear INT_FLAG.
    //
    SPI_readDataBlockingNonFIFO(SPIA_BASE);
    //
    // Pull chip select high.
    //
    CS_HIGH;
}





//
// Function to write data to the EEPROM
// - address is the byte address of the EEPROM
// - data is a pointer to an array of data being sent
// - length is the number of characters in the array to send
//
void writeDataSPI(uint8_t rw, uint16_t address, uint16_t  data)

{
    uint16_t i;
    uint8_t data1;



    //
    // Pull chip select low.
    //
    CS1LOW;
    //
    // Send the WRITE/READ opcode.
    //
    data1=(rw<<15) + (000 <<12) + (address<<8);

   // SPI_writeDataBlockingNonFIFO(SPIA_BASE, ((data1<<7) && 0xFF00) );

    SPI_writeDataBlockingNonFIFO(SPIA_BASE, data1);
    //
    // Dummy read to clear INT_FLAG.
    //
    SPI_readDataBlockingNonFIFO(SPIA_BASE);
    //
    // Send the Uper data opcode.
    //
    SPI_writeDataBlockingNonFIFO(SPIA_BASE, (data<<4));
    //
    // Dummy read to clear INT_FLAG.
    //
    SPI_readDataBlockingNonFIFO(SPIA_BASE);
    //
    // Send the Down data opcode.
    //
    SPI_writeDataBlockingNonFIFO(SPIA_BASE, (data<<12));
    //
    // Dummy read to clear INT_FLAG.
    //
    SPI_readDataBlockingNonFIFO(SPIA_BASE);

    CS1HI;
}



void readDataSPIADC70(uint16_t address, uint16_t *dataout, uint16_t *oldadress)
{
    uint16_t dataM;
    uint16_t dataL;






    CS2LOW;

    //
    // Send the READ opcode
    //
    //SPI_writeDataBlockingNonFIFO(SPIA_BASE, READ);

    //
    // Dummy read to clear INT_FLAG.
    //
   // SPI_readDataBlockingNonFIFO(SPIA_BASE);


   // SPI_writeDataBlockingNonFIFO(SPIA_BASE, 0x0000);




    //
    // Send the MSB of the address of the EEPROM
    //
    SPI_writeDataBlockingNonFIFO(SPIA_BASE, 0x0800);


    dataM = SPI_readDataBlockingNonFIFO(SPIA_BASE);

    //
    // Dummy read to clear INT_FLAG.
    //
    //SPI_readDataBlockingNonFIFO(SPIA_BASE);

   //

    //
    // TO REGister CHANEL_SEL
    //
    SPI_writeDataBlockingNonFIFO(SPIA_BASE, 0x1100);

    dataL = SPI_readDataBlockingNonFIFO(SPIA_BASE);

    //
    // Dummy read to clear INT_FLAG.
    //
    //SPI_readDataBlockingNonFIFO(SPIA_BASE);


    //
    // TO REGister 0-3 nomber of chanel
    //
    SPI_writeDataBlockingNonFIFO(SPIA_BASE, (address<<8));

    //
    // Dummy read to clear INT_FLAG.
    //
    SPI_readDataBlockingNonFIFO(SPIA_BASE);


   // CS2HI;

  //  CS2LOW;
    //
    // Read the data from the EEPROM
    //
    //for(i = 0; i < 16; i++)
    //{
        //
        // Send dummy data to receive the EEPROM data
        //


   // }

   CS2HI;

   *dataout=(dataM<<4)+((dataL & 0xF0)>>4);
   *oldadress=address;

}



/*
void writeDataSPI(uin8_t rw, uint16_t address, uint8_t  data)

{
    uint16_t i;
    uint8_t data1;



    //
    // Pull chip select low.
    //
    CS1LOW;
    //
    // Send the WRITE/READ opcode.
    //

    data1=(rw) + (000 <<3) + (address<<7);

    SPI_writeDataBlockingNonFIFO(SPIA_BASE, (data1<<7) && 0xFF00);
    //
    // Dummy read to clear INT_FLAG.
    //
    SPI_readDataBlockingNonFIFO(SPIA_BASE);
    //
    // Send the WRITE opcode.
    //
    SPI_writeDataBlockingNonFIFO(SPIA_BASE, 0x3000);
    //
    // Dummy read to clear INT_FLAG.
    //
    SPI_readDataBlockingNonFIFO(SPIA_BASE);

    SPI_writeDataBlockingNonFIFO(SPIA_BASE, 0x0000);
    //
    // Dummy read to clear INT_FLAG.
    //
    SPI_readDataBlockingNonFIFO(SPIA_BASE);

;
    //
    // Send the data.
    //
   // for(i = 0; i < length; i++)
    //{
        //
        // Send the data.
        //
    //    SPI_writeDataBlockingNonFIFO(SPIA_BASE, data[i] << 8);

        //
        // Dummy read to clear INT_FLAG.
        //
    //   SPI_readDataBlockingNonFIFO(SPIA_BASE);
    //}

    //
    // Pull chip select high.
    //
    CS1HI;
}

*/

//
// Function to read data from the EEPROM
// - address is the byte address of the EEPROM
// - data is a pointer to an array of data being received
// - length is the number of characters in the array to receive
//
void readDataSPI(uint16_t address, uint16_t * data, uint16_t length)
{
    uint16_t i;





    CS_LOW;

    //
    // Send the READ opcode
    //
    //SPI_writeDataBlockingNonFIFO(SPIA_BASE, READ);

    //
    // Dummy read to clear INT_FLAG.
    //
    SPI_readDataBlockingNonFIFO(SPIA_BASE);

    //
    // Send the MSB of the address of the EEPROM
    //
    SPI_writeDataBlockingNonFIFO(SPIA_BASE, (address & 0xFF00));

    //
    // Dummy read to clear INT_FLAG.
    //
    SPI_readDataBlockingNonFIFO(SPIA_BASE);

    //
    // Send the LSB of the address of the EEPROM
    //
    SPI_writeDataBlockingNonFIFO(SPIA_BASE, (address << 8));

    //
    // Dummy read to clear INT_FLAG.
    //
    SPI_readDataBlockingNonFIFO(SPIA_BASE);

    //
    // Read the data from the EEPROM
    //
    for(i = 0; i < length; i++)
    {
        //
        // Send dummy data to receive the EEPROM data
        //
        SPI_writeDataBlockingNonFIFO(SPIA_BASE, 0x0000);
        data[i] = SPI_readDataBlockingNonFIFO(SPIA_BASE);
    }

    CS_HIGH;
}
