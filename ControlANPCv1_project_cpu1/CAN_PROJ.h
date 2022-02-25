/*
 * CAN_PROJ.h
 *
 *  Created on: 27 lip 2021
 *      Author: Rafal Miskiewicz
 */


#ifndef CAN_PROJ_H_
#define CAN_PROJ_H_

#include <stdint.h>



extern int MSG_DATA_LENGTH ;


extern int TX_MSG_OBJ_ID1;
extern int TX_MSG_OBJ_ID2;
extern int TX_MSG_OBJ_ID3;
extern int TX_MSG_OBJ_ID4;

extern int RX_MSG_OBJ_ID1;
extern int RX_MSG_OBJ_ID2;
extern int RX_MSG_OBJ_ID3;
extern int RX_MSG_OBJ_ID4;



extern int txdata1[8];
extern int txdata2[8];
extern int txdata3[8];
extern int txdata4[8];
extern int txdata6[8];

extern int rxdata1[8];
extern int rxdata2[8];
extern int rxdata3[8];
extern int rxdata4[8];
extern int rxdata5[8];

//int stat=0;
extern float temp1;
extern float temp2;
extern float temp3;
extern float temp4;
extern float temp5;
extern float temp6;


extern float Uacmax;
extern float Uacmin;
extern float Udcmax;
extern float Udcmin;
extern float Iacmax;
extern float Iacmin;
extern float Idcmax;
extern float Idcmin;


extern float Id;
extern float Iq;
extern float Ud;
extern float Ud1;
extern float Uq;
extern int FaultCode;
extern float UDC1, UDC2, IDC1, IDC2,UDC1mean,UDC2mean;
extern int START_PROC;
extern int STOP_PROC;
extern float Irefd;
extern float Urefd;
extern float IAC1maxw,IAC2maxw,IAC3maxw;
extern float UAC1maxw,UAC2maxw,UAC3maxw;


extern int rxdata6[8];
extern int CONNECTION_RPI, CONNECTION_TI, CONNECTION_TI_PI;

extern float kp1, ti1, kp2, ti2;
extern float kp3, ti3, kp4, ti4;
extern int START_PRECH, STOP_PRECH, RESET_FLT;





//extern struct mes measure;



void CANAMsgRead(void);
void CANAMsgWrite(void);
void CANAobject(void);


#endif /* CAN_PROJ_H_ */
