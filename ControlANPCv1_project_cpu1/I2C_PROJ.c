/*
 * I2C_PROJ.c
 *
 *  Created on: 28 lip 2021
 *      Author: Rafal Miskiewicz
 */

#include "I2C_PROJ.h"
#include "driverlib.h"
#include "device.h"



struct I2CMsg i2cMsgOut = {MSG_STATUS_SEND_WITHSTOP,
                           SLAVE_ADDRESS,
                           NUM_BYTES,
                           EEPROM_HIGH_ADDR,
                           EEPROM_LOW_ADDR,
                           0x01,                // Message bytes
                           0x23,
                           0x45,
                           0x67,
                           0x89,
                           0xAB,
                           0xCD,
                           0xEF};
 struct I2CMsg i2cMsgIn  = {MSG_STATUS_SEND_NOSTOP,
                           SLAVE_ADDRESS,
                           NUM_BYTES,
                           EEPROM_HIGH_ADDR,
                           EEPROM_LOW_ADDR};

 struct I2CMsg *currentMsgPtr;                   // Used in interrupt


void initI2C()
{
    //
    // Must put I2C into reset before configuring it
    //
    I2C_disableModule(I2CA_BASE);

    //
    // I2C configuration. Use a 400kHz I2CCLK with a 33% duty cycle.
    //
    I2C_initMaster(I2CA_BASE, DEVICE_SYSCLK_FREQ, 400000, I2C_DUTYCYCLE_33);
    I2C_setBitCount(I2CA_BASE, I2C_BITCOUNT_8);
    I2C_setSlaveAddress(I2CA_BASE, SLAVE_ADDRESS);
    I2C_setEmulationMode(I2CA_BASE, I2C_EMULATION_FREE_RUN);

    //
    // Enable stop condition and register-access-ready interrupts
    //
    I2C_enableInterrupt(I2CA_BASE, I2C_INT_STOP_CONDITION |
                        I2C_INT_REG_ACCESS_RDY);

    //
    // FIFO configuration
    //
    I2C_enableFIFO(I2CA_BASE);
    I2C_clearInterruptStatus(I2CA_BASE, I2C_INT_RXFF | I2C_INT_TXFF);

    //
    // Configuration complete. Enable the module.
    //
    I2C_enableModule(I2CA_BASE);
}

//
// writeData - Function to send the data that is to be written to the EEPROM
//
uint16_t writeDataI2C(struct I2CMsg *msg)
{
    uint16_t i;

    //
    // Wait until the STP bit is cleared from any previous master
    // communication. Clearing of this bit by the module is delayed until after
    // the SCD bit is set. If this bit is not checked prior to initiating a new
    // message, the I2C could get confused.
    //
    if(I2C_getStopConditionStatus(I2CA_BASE))
    {
        return(ERROR_STOP_NOT_READY);
    }

    //
    // Check if bus busy
    //
    if(I2C_isBusBusy(I2CA_BASE))
    {
        return(ERROR_BUS_BUSY);
    }

    //
    // Setup number of bytes to send msgBuffer and address
    //
    I2C_setDataCount(I2CA_BASE, (msg->numBytes + 2));

    //
    // Setup data to send
    //
    I2C_putData(I2CA_BASE, msg->memoryHighAddr);
    I2C_putData(I2CA_BASE, msg->memoryLowAddr);

    for (i = 0; i < msg->numBytes; i++)
    {
        I2C_putData(I2CA_BASE, msg->msgBuffer[i]);
    }

    //
    // Send start as master transmitter
    //
    I2C_setConfig(I2CA_BASE, I2C_MASTER_SEND_MODE);
    I2C_sendStartCondition(I2CA_BASE);
    I2C_sendStopCondition(I2CA_BASE);

    return(SUCCESS);
}

//
// readData - Function to prepare for the data that is to be read from the EEPROM
//
uint16_t readDataI2C(struct I2CMsg *msg)
{
    //
    // Wait until the STP bit is cleared from any previous master
    // communication. Clearing of this bit by the module is delayed until after
    // the SCD bit is set. If this bit is not checked prior to initiating a new
    // message, the I2C could get confused.
    //
    if(I2C_getStopConditionStatus(I2CA_BASE))
    {
        return(ERROR_STOP_NOT_READY);
    }

    //
    // If we are in the the address setup phase, send the address without a
    // stop condition.
    //
    if(msg->msgStatus == MSG_STATUS_SEND_NOSTOP)
    {
        //
        // Check if bus busy
        //
        if(I2C_isBusBusy(I2CA_BASE))
        {
            return(ERROR_BUS_BUSY);
        }

        //
        // Send data to setup EEPROM address
        //
        I2C_setDataCount(I2CA_BASE, 2);
        I2C_putData(I2CA_BASE, msg->memoryHighAddr);
        I2C_putData(I2CA_BASE, msg->memoryLowAddr);
        I2C_setConfig(I2CA_BASE, I2C_MASTER_SEND_MODE);
        I2C_sendStartCondition(I2CA_BASE);
    }
    else if(msg->msgStatus == MSG_STATUS_RESTART)
    {
        //
        // Address setup phase has completed. Now setup how many bytes expected
        // and send restart as master-receiver.
        //
        I2C_setDataCount(I2CA_BASE, (msg->numBytes));
        I2C_setConfig(I2CA_BASE, I2C_MASTER_RECEIVE_MODE);
        I2C_sendStartCondition(I2CA_BASE);
        I2C_sendStopCondition(I2CA_BASE);
    }

    return(SUCCESS);
}
