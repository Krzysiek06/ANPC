/*
 * CAN_PROJ.c
 *
 *  Created on: 27 lip 2021
 *      Author: Rafal Miskiewicz
 */

#include <CAN_PROJ.h>

#include "f28x_project.h"
#include "driverlib.h"
#include "device.h"





//extern uint16_t datarx1[8];

void CANAobject(void){
    CAN_setupMessageObject(CANA_BASE, 0x01, 0x01, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, 8);
    txdata1[8];
    CAN_setupMessageObject(CANA_BASE, 0x02, 0x02, CAN_MSG_FRAME_STD,  CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, 8);
    txdata2[8];
    CAN_setupMessageObject(CANA_BASE, 0x03, 0x03, CAN_MSG_FRAME_STD,  CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, 8);
    txdata2[8];
    CAN_setupMessageObject(CANA_BASE, 0x04, 0x04, CAN_MSG_FRAME_STD,  CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, 8);
    txdata2[8];
    CAN_setupMessageObject(CANA_BASE, 0xB, 0xB, CAN_MSG_FRAME_STD,  CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, 8);
    txdata6[8];




    CAN_setupMessageObject(CANA_BASE, 5, 0x05, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE , 0);
    rxdata1[8];
    CAN_setupMessageObject(CANA_BASE, 6, 0x06, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE , 0);
    rxdata2[8];
    CAN_setupMessageObject(CANA_BASE, 7, 0x07, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE , 0);
    rxdata3[8];
    CAN_setupMessageObject(CANA_BASE, 8, 0x08, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE , 0);
    rxdata4[8];
    CAN_setupMessageObject(CANA_BASE, 9, 0x09, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE , 0);
    rxdata5[8];
    CAN_setupMessageObject(CANA_BASE, 10, 0xA, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE , 0);
    rxdata6[8];

}

void CANAMsgWrite(void){



   // CAN_setupMessageObject(CANA_BASE, 0x03, 0x01, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, 8);

    txdata1[0]=( (int) (-Id*10.0+30.0) & 0xFF) ;
    txdata1[1]=(((int) (-Id*10.0+30.0) >>8) & 0xFF);
    txdata1[2]=( (int) (Iq*10.0+30.0) & 0xFF) ;
    txdata1[3]=(((int) (Iq*10.0+30.0) >>8) & 0xFF);
    txdata1[4]=( (int) (Ud+450) & 0xFF) ;
    txdata1[5]=(((int) (Ud+450) >>8) & 0xFF);
    txdata1[6]=( (int) (Uq+450) & 0xFF) ;
    txdata1[7]=(((int) (Uq+450) >>8) & 0xFF);


   // txdata1[0]=( (int) (temp1*100.0) & 0xFF) ;
    //txdata1[1]=(((int) (temp1*100.0) >>8) & 0xFF);
    //txdata1[2]=( (int) (temp2*100.0) & 0xFF) ;
    //txdata1[3]=(((int) (temp2*100.0) >>8) & 0xFF);
   // txdata1[4]=( (int) (temp3*100.0) & 0xFF) ;
  // txdata1[5]=(((int) (temp3*100.0) >>8) & 0xFF);
   // txdata1[6]=( (int) (temp4*100.0) & 0xFF) ;
   // txdata1[7]=(((int) (temp4*100.0) >>8) & 0xFF);



    CAN_sendMessage(CANA_BASE, 1, 8, txdata1);
   // DEVICE_DELAY_US(100000);
   // while(((HWREGH(CANA_BASE + CAN_O_ES) & CAN_ES_TXOK)) !=  CAN_ES_TXOK)
   // {
   // }
    while(HWREGH(CANA_BASE + CAN_O_IF1CMD) & CAN_IF1CMD_BUSY)
    {
    }
   // CAN_setupMessageObject(CANA_BASE, 0x03, 0x02, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, 8);


    //txdata2[0]=( (int) (temp1*100.0) & 0xFF) ;
    //txdata2[1]=(((int) (temp1*100.0) >>8) & 0xFF);
    //txdata2[2]=( (int) (temp2*100.0) & 0xFF) ;
    //txdata2[3]=(((int) (temp2*100.0) >>8) & 0xFF);
    //txdata2[4]=( (int) (temp3*100.0) & 0xFF) ;
    //txdata2[5]=(((int) (temp3*100.0) >>8) & 0xFF);
    //txdata2[6]=( (int) (temp4*100.0) & 0xFF) ;
    //txdata2[7]=(((int) (temp4*100.0) >>8) & 0xFF);
    txdata2[0]=( (int) (IAC1maxw*100.0) & 0xFF) ;
    txdata2[1]=(((int) (IAC1maxw*100.0) >>8) & 0xFF);
    txdata2[2]=( (int) (IAC2maxw*100.0) & 0xFF) ;
    txdata2[3]=(((int) (IAC2maxw*100.0) >>8) & 0xFF);
    txdata2[4]=( (int) (IAC3maxw*100.0) & 0xFF) ;
    txdata2[5]=(((int) (IAC3maxw*100.0) >>8) & 0xFF);
    txdata2[6]=( (int) (IAC1maxw*100.0) & 0xFF) ;
    txdata2[7]=(((int) (IAC1maxw*100.0) >>8) & 0xFF);




    CAN_sendMessage(CANA_BASE, 2, 8, txdata2);
   // DEVICE_DELAY_US(100000);
  //  while(((HWREGH(CANA_BASE + CAN_O_ES) & CAN_ES_TXOK)) !=  CAN_ES_TXOK)
  //  {
  //  }
    while(HWREGH(CANA_BASE + CAN_O_IF1CMD) & CAN_IF1CMD_BUSY)
    {
    }

   // CAN_setupMessageObject(CANA_BASE, 0x03, 0x03, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, 8);


    txdata3[0]=( (int) (FaultCode) & 0xFF) ;
    txdata3[1]=(((int) (FaultCode) >>8) & 0xFF);
    //txdata3[2]=( (int) (temp5*100.0) & 0xFF) ;
    //txdata3[3]=(((int) (temp5*100.0) >>8) & 0xFF);
    //txdata3[4]=( (int) (temp6*100.0) & 0xFF) ;
    //txdata3[5]=(((int) (temp6*100.0) >>8) & 0xFF);
    //txdata3[6]=( (int) (temp6*100.0) & 0xFF) ;
    //txdata3[7]=(((int) (temp6*100.0) >>8) & 0xFF);
    txdata3[2]=( (int) (IAC2maxw*100.0) & 0xFF) ;
    txdata3[3]=(((int) (IAC2maxw*100.0) >>8) & 0xFF);
    txdata3[4]=( (int) (IAC3maxw*100.0) & 0xFF) ;
    txdata3[5]=(((int) (IAC3maxw*100.0) >>8) & 0xFF);
    txdata3[6]=( (int) (IAC2maxw*100.0) & 0xFF) ;
    txdata3[7]=(((int) (IAC2maxw*100.0) >>8) & 0xFF);





    CAN_sendMessage(CANA_BASE, 3, 8, txdata3);
   // DEVICE_DELAY_US(100000);
  //  while(((HWREGH(CANA_BASE + CAN_O_ES) & CAN_ES_TXOK)) !=  CAN_ES_TXOK)
    //{
   // }

    while(HWREGH(CANA_BASE + CAN_O_IF1CMD) & CAN_IF1CMD_BUSY)
    {
    }
   // CAN_setupMessageObject(CANA_BASE, 0x03, 0x04, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, 8);


    txdata4[0]=( (int) (UDC1mean) & 0xFF) ;
    txdata4[1]=(((int) (UDC1mean) >>8) & 0xFF);
    txdata4[2]=( (int) (UDC2mean) & 0xFF) ;
    txdata4[3]=(((int) (UDC2mean) >>8) & 0xFF);
    txdata4[4]=( ((int) (IDC1*10.0+300.0) & 0xFF));
    txdata4[5]=((((int) (IDC1*10.0+300.0) >> 8)  & 0xFF));
    txdata4[6]=( ((int) (IDC2*10.0+300.0)  & 0xFF)) ;
    txdata4[7]=((((int) (IDC2*10.0+300.0) >> 8)  & 0xFF));




    CAN_sendMessage(CANA_BASE, 4, 8, txdata4);
   // DEVICE_DELAY_US(100000);
  //  while(((HWREGH(CANA_BASE + CAN_O_ES) & CAN_ES_TXOK)) !=  CAN_ES_TXOK)
   // {
   // }

    while(HWREGH(CANA_BASE + CAN_O_IF1CMD) & CAN_IF1CMD_BUSY)
    {
    }

    txdata6[0]= CONNECTION_TI_PI;
    CAN_sendMessage(CANA_BASE, 11, 8, txdata6);

    while(HWREGH(CANA_BASE + CAN_O_IF1CMD) & CAN_IF1CMD_BUSY)
    {
    }


};

void CANAMsgRead(void){
    uint32_t status;

    //
    // Read the CAN-B interrupt status to find the cause of the interrupt
    //
    status = CAN_getInterruptCause(CANA_BASE);
   // stat=status;
    //status
    // If the cause is a controller status interrupt, then get the status
    //
    if(status == CAN_INT_INT0ID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        //
        status = CAN_getStatus(CANA_BASE);

        //
        // Check to see if an error occurred.
        //
        if(((status  & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_MSK) &&
           ((status  & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_NONE))
        {
            //
            // Set a flag to indicate some errors may have occurred.
            //
            //errorFlag = 1;
        }
    }
    //
    // Check if the cause is the CAN-B receive message object 1
    //

    else if(status == 5)
    {
        //
        // Get the received message
        //
        //CAN_setupMessageObject(CANA_BASE, 1, 2, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE , 0);
        CAN_readMessage(CANA_BASE, 5, rxdata1);
        START_PROC= (rxdata1[0] & 0x01);
        STOP_PROC= ((rxdata1[0]>>1) & 0x01);
        START_PRECH= ((rxdata1[0]>>2) & 0x01);
        STOP_PRECH= ((rxdata1[0]>>3) & 0x01);
        RESET_FLT= ((rxdata1[0]>>4) & 0x01);

        Urefd=(((rxdata1[2] & 0x00ff) + (rxdata1[3]<<8 & 0xff00))/100.0);
        Irefd=(((rxdata1[4] & 0x00ff) + (rxdata1[5]<<8 & 0xff00))/100.0);

        CAN_clearInterruptStatus(CANA_BASE, 5);

        //
        // Increment a counter to keep track of how many messages have been
        // received. In a real application this could be used to set flags to
        // indicate when a message is received.
        //
      //  rxMsgCount++;

        //
        // Since the message was received, clear any error flags.
        //
     //   errorFlag = 0;
    }else if(status == 6){
        //CAN_setupMessageObject(CANA_BASE, 1, 3, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE , 0);
        CAN_readMessage(CANA_BASE, 6, rxdata2);

        kp1=(((rxdata2[0] & 0x00ff) + (rxdata2[1]<<8 & 0xff00))/1000.0);
        ti1=(((rxdata2[2] & 0x00ff) + (rxdata2[3]<<8 & 0xff00))/1000.0);
        kp2=(((rxdata2[4] & 0x00ff) + (rxdata2[5]<<8 & 0xff00))/1000.0);
        ti2=(((rxdata2[6] & 0x00ff) + (rxdata2[7]<<8 & 0xff00))/1000.0);
        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CAN_clearInterruptStatus(CANA_BASE, 6);

    }else if(status == 7){
        //CAN_setupMessageObject(CANA_BASE, 1, 3, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE , 0);
        CAN_readMessage(CANA_BASE, 7, rxdata3);

        kp3=(((rxdata3[0] & 0x00ff) + (rxdata3[1]<<8 & 0xff00))/1000.0);
        ti3=(((rxdata3[2] & 0x00ff) + (rxdata3[3]<<8 & 0xff00))/1000.0);
        kp4=(((rxdata3[4] & 0x00ff) + (rxdata3[5]<<8 & 0xff00))/1000.0);
        ti4=(((rxdata3[6] & 0x00ff) + (rxdata3[7]<<8 & 0xff00))/1000.0);
        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CAN_clearInterruptStatus(CANA_BASE, 7);

    }else if(status == 8){
        //CAN_setupMessageObject(CANA_BASE, 1, 3, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE , 0);
        CAN_readMessage(CANA_BASE, 8, rxdata4);

        Iacmax = ((rxdata4[0] & 0x00ff) + (rxdata4[1]<<8 & 0xff00));
        Iacmin = (-1)*(int)((rxdata4[2] & 0x00ff) + (rxdata4[3]<<8 & 0xff00));
        Idcmax = ((rxdata4[4] & 0x00ff) + (rxdata4[5]<<8 & 0xff00));
        Idcmin = (-1)*(int)((rxdata4[6] & 0x00ff) + (rxdata4[7]<<8 & 0xff00));
        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CAN_clearInterruptStatus(CANA_BASE, 8);

    }else if(status == 9){
        //CAN_setupMessageObject(CANA_BASE, 1, 3, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE , 0);
        CAN_readMessage(CANA_BASE, 9, rxdata5);

        Uacmax =((rxdata5[0] & 0x00ff) + (rxdata5[1]<<8 & 0xff00));
        Uacmin =(-1)*(int)((rxdata5[2] & 0x00ff) + (rxdata5[3]<<8 & 0xff00));
        Udcmax =((rxdata5[4] & 0x00ff) + (rxdata5[5]<<8 & 0xff00));
        Udcmin =(-1)*(int)((rxdata5[6] & 0x00ff) + (rxdata5[7]<<8 & 0xff00));
        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CAN_clearInterruptStatus(CANA_BASE, 9);

    }else if(status == 10){
        //CAN_setupMessageObject(CANA_BASE, 1, 3, CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE , 0);
        CAN_readMessage(CANA_BASE, 10, rxdata6);
        CONNECTION_TI_PI = (int)(rxdata6[0]);
        CAN_clearInterruptStatus(CANA_BASE, 10);
    }




    //
    // If something unexpected caused the interrupt, this would handle it.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }

    //
    // Clear the global interrupt flag for the CAN interrupt line
    //
    CAN_clearGlobalInterruptStatus(CANA_BASE, CAN_GLOBAL_INT_CANINT0);

    //
    // Acknowledge this interrupt located in group 9
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);

}

