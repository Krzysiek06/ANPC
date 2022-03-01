/*
 * VARIABLE_PROJ.h
 *
 *  Created on: 28 lip 2021
 *      Author: Rafal Miskiewicz
 */

#ifndef VARIABLE_PROJ_H_
#define VARIABLE_PROJ_H_


//VAR CAN
//#include "f28x_project.h"
//#include "driverlib.h"
//#include "device.h"

//
// Defines
//

#define  CS_HIGH GPIO_writePin(80, 0)
#define  CS_LOW GPIO_writePin(80, 1)





int MSG_DATA_LENGTH =   8;
int TX_MSG_OBJ_ID1  =  1;
int TX_MSG_OBJ_ID2  =  2;
int TX_MSG_OBJ_ID3  =  3;
int TX_MSG_OBJ_ID4  =  4;

#define RX_MSG_OBJ_ID    1




int RX_MSG_OBJ_ID1 =1;
int RX_MSG_OBJ_ID2 =2;
int RX_MSG_OBJ_ID3 =3;
int RX_MSG_OBJ_ID4 =4;

int x1=0, x2=0, x3=0, x5=0;

//extern int stat;



int txdata1[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int txdata2[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int txdata3[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int txdata4[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int txdata6[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

int rxdata1[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int rxdata2[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int rxdata3[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int rxdata4[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int rxdata5[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int rxdata6[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


// DECLARTION I2C


//
// Defines
//
#define SLAVE_ADDRESS               0x50
#define EEPROM_HIGH_ADDR            0x00
#define EEPROM_LOW_ADDR             0x30
#define NUM_BYTES                   8
#define MAX_BUFFER_SIZE             14      // Max is currently 14 because of
                                            // 2 address bytes and the 16-byte
                                            // FIFO

//
// I2C message states for I2CMsg struct
//
#define MSG_STATUS_INACTIVE         0x0000 // Message not in use, do not send
#define MSG_STATUS_SEND_WITHSTOP    0x0010 // Send message with stop bit
#define MSG_STATUS_WRITE_BUSY       0x0011 // Message sent, wait for stop
#define MSG_STATUS_SEND_NOSTOP      0x0020 // Send message without stop bit
#define MSG_STATUS_SEND_NOSTOP_BUSY 0x0021 // Message sent, wait for ARDY
#define MSG_STATUS_RESTART          0x0022 // Ready to become master-receiver
#define MSG_STATUS_READ_BUSY        0x0023 // Wait for stop before reading data

//
// Error messages for read and write functions
//
#define ERROR_BUS_BUSY              0x1000
#define ERROR_STOP_NOT_READY        0x5555
#define SUCCESS                     0x0000



extern struct I2CMsg i2cMsgOut;
extern struct I2CMsg i2cMsgIn ;
extern struct I2CMsg *currentMsgPtr;                   // Used in interrupt

#define PI 3.14159265358979323846
#define omega 314.159265
#define omegaL 0.103672558 //2*PI*f*L L=330uH

#define ReadyToGo           0
#define ReadyToPreCharge    1
#define Precharging         2
#define AcRelayOn           3
#define CurrentMode         4
#define DcRelayOn           5
#define TurnOnVoltageMode   6
#define VoltageMode         7



// Zmienne do programu

int Licznik=0,state;
float timer=0;
int stan_pracy=ReadyToGo;
int warunek;
float Duty1=0.0, Duty2=0.0, Duty3=0.0, Duty4=0.0,Duty5=0.0, Duty6=0.0,Duty7=0.0, Duty8=0.0,Duty9=0.0,Duty10A=0.0, Duty10B=0.0, Duty11A=0.0, Duty11B=0.0, Duty12A=0.0, Duty12B=0.0, Duty13A=0.0, Duty13B=0.0;
float Dutylevel=0.01,dV=0;
float wspdt=0;
int res_on=1;
int TurnOffInverter=0;
int Duty1f=0,Duty2f=0,Duty3f=0;
uint32_t pom1=0, pom2=0, pom3=0, pom4=0, pom5=0,pom6=0, pom7=0, pom8=0, pom9=0, pom10=0, zm=0, cnt=0;
uint16_t value=0, value2=0, cnt1=0;
uint16_t x=0, y=0, z=0, p=0, rm1=0, rm2=0, rm3=0, x7=0, x8=0;
float rad=0.0, theta=0.0, theta1=0.0, ma=0.0, theta2=0.0;
uint32_t GPIO7;


int RelayPRCH=0,RelayAC=0,RelayDC=0,RelayDC_p=0,delayrelay=0;

//Zmienne z Transformat ab/dq
float Ud=0.0, Uq=0.0, Ua=0.0, Ub=0.0, Id=0.0, Iq=0.0, Ia=0.0, Ib=0.0;
float Idfiltr = 0.0, Iqfiltr = 0.0, Idrampref = 0, Idramp=0;
float sogi_alfa,sogi_beta;
float Ialfaref=0.0,Ibetaref=0.0;
float Ialfa_out=0.0,Ibeta_out=0.0,Kp_RC;
float Iamax=0.0, Ibmax =0.0, Icmax=0.0;
float rc_wcut,rc_ki;
float UDCref=0.0,UDCramp=0.0,UDCon=500.0;
float kpU=1.0,tiU=1.0;
float harm3 = 0.0;
float harm3_v2 = 0.0;
float Bal_DCout = 0.0,kDC=1.0,tDC=1.0;
float Vout=0.0,Vref=0.0;
float Iinit;
float Fir_UI_AC=0.5;
float Fir_UI_DC=0.1;
float rc_ki3_init,rc_ki5_init,rc_ki7_init;

float res_mx3=1;
float res_mx5=1;
float res_mx7=1;
float res_mx11=1;

float rc_ki3,rc_ki5,rc_ki7,rc_ki11;


int zmiana_wyswietl = 0.0;
float Ubal_out=0.0, Ubal_int=0.0, Ubal_in=0.0;

float tablicapll[999];
int ct=0,ct2=0,ert=1;
int i=0,j=0;
float k=0.0;

float y_pll=0.0,p_pll=0.0,i_pll=0.0,omegapll=0.0;

int StartPWM=0, Fault=0, FaultCode=0, ResetFault=0;

int CONNECTION_RPI = 0;
int CONNECTION_TI = 0, CONNECTION_TI_PI = 1;
int CONNECTION_WAIT = 0, CONNECTION_FAULT = 0;


//float Kp_pll=1.0, Ki_pll=300, kfilter=0.9;
float Kp_pll=2.22, Ki_pll=61.69, kfilter=0.9;
//float Ts=0.00001602;
float Ts=0.00003204;
float kp1=0.0, ti1=0.0, kp2=0.0, ti2=0.0;
float kp3=0.0, ti3=1.0, kp4=0.0, ti4=0.0;
float kpreg = 5.0, tireg = 0.01;

float integral1=0.0, integral2=0.0, integral3=0.0;
//
//Zmienne do Pomiaru temperatury
//
float temp1=0.0, temp2=0.0, temp3=0.0, temp4=0.0, temp5=0.0, temp6=0.0,skalatemp=0.04585;
int adress_ch=0, old_chanel=0, adc_value=0, adc_chanel=0;

int START_PROC=0, STOP_PROC=1;
int START_PRECH=0,START_PROC_p=0, STOP_PRECH=1;
int RESET_FLT=0;
int procedura = 0,flagaprocedura=0, proceduraON = 0, proceduraOFF = 0;
float Irefd=0.0, Urefd=0.0;
float tempmin=30.0;
float tempmax=45.0;
float Duty3r=0.0, Duty2r=0.0, Duty1r=0.0;
float UDC=0.0;
int startreg=0,startreg_fl=1;
float delayPRCH_x=0.0;
float delayPWM = 0.0;
float delayRegUx = 0.0;
int flagaPRCH=0, flagaPRCH2=0,flagaON=0,cosx=0;
int flagastopu_m=0;
float Rinit=220; //doda³em jako pocz¹tkowe obci¹¿enie (MH)
float Ipkinit=2; // doda³em jako maksymalny pr¹d przy rozuruch
float rcinit;
float wcut;

//
//Zmienne i struktyry  od pomiarów
//
float fUAC1=0.0, fUAC1OLD=0.0, fUAC2=0.0, fUAC2OLD=0.0, fUAC3=0.0, fUAC3OLD=0.0;
float UDC1=0.0, UDC2=0.0, IDC1=0.0, IDC2=0.0;
float UDC1mean=0.0, UDC1mean_old=0.0, UDC2mean=0.0, UDC2mean_old=0.0;

float IAC1=0.0,IAC2=0.0,IAC3=0.0,IAC1_old=0.0, IAC2_old=0.0, IAC3_old=0.0, pomfilter1=0.5;
float UDC1_old=0.0, UDC2_old=0.0, IDC1_old=0.0, IDC2_old=0.0, pomfilter=0.1, pomfilter2=0.01;
float UAC1_old=0.0,UAC1=0.0,UAC2_old=0.0,UAC2=0.0,UAC3_old=0.0,UAC3=0.0;
float Ud1=0.0, Uq1=0.0, Udout=0.0, Uqout=0.0, Ualfaout=0.0, Ubetaout=0.0;
float Id_old=0.0, Iq_old=0.0;
float IAC1max=0, IAC2max=0, IAC3max=0;
float UAC1max=0, UAC2max=0, UAC3max=0;
float IAC1maxw=0, IAC2maxw=0, IAC3maxw=0;
float IAC1_old1=0, IAC2_old1=0, IAC3_old1=0;
float UAC1maxw=0, UAC2maxw=0, UAC3maxw=0;
float timerscal=0;




float Uacmax = 500.0;
float Uacmin = -500.0;
float Udcmax = 600.0;
float Udcmin = -500.0;
float Iacmax = 15.0;
float Iacmin = -15.0;
float Idcmax = 28.0;
float Idcmin = -28.0;



typedef struct measure{
    float IAC1;
    float IAC2;
    float IAC3;
    float IDC1;
    float IDC2;
    float UAC1;
    float UAC2;
    float UAC3;
    float UDC1;
    float UDC2;
    float UDC2p;

    uint16_t Offset_IAC1;
    uint16_t Offset_IAC2;
    uint16_t Offset_IAC3;
    uint16_t Offset_IDC1;
    uint16_t Offset_IDC2;
    uint16_t Offset_UDC1;
    uint16_t Offset_UDC2;
    uint16_t Offset_UAC1;
    uint16_t Offset_UAC2;
    uint16_t Offset_UAC3;

    float skalaIAC1;
    float skalaIAC2;
    float skalaIAC3;
    float skalaIDC1;
    float skalaIDC2;
    float skalaUDC1;
    float skalaUDC2;
    float skalaUAC1;
    float skalaUAC2;
    float skalaUAC3;
} measure;

typedef struct{
    float Iacmin;
    float Iacmax;
    float Idcmin;
    float Idcmax;
    float Uacmax;
    float Uacmin;
    float Udcmax;
    float Udcmin;

} protect;

typedef struct{
    float   a0;
    float   a1;
    float   a2;
    float   b0;
    float   b1;

    float   b0a0;
    float   a1a0;
    float   a2a0;
    float   b1a0;

    float   k_sogi;
}sogi_init_struct;

typedef struct{
    float v_out;
    float v_out1;
    float v_out2;
    float v_in;
    float v_in1;
    float v_in2;
    float q_out;
    float q_out1;
    float q_out2;
}sogi_stuct;

void sogi_exec(sogi_stuct *st);

typedef struct{
    float n0;
    float n2;
    float d1;
    float d2;
    float u_res01;
    float y_res01;
    float u_res21;
    float u_res11;
    float y_res11;
    float y_res21;
    float K;
}RC_struct;

void RC_init(RC_struct *RC_st,float x,float wcut,float Ki,float Ts1);
void RC_exec(RC_struct *RC_st,float in, float res_k);

////////////             Struktura regulatora         ///////////
typedef struct Regulator {
   float kp;
   float ki;
   float ti;
   float td;
   float T;
   float upsat;
   float lowsat;
   float regulator_in[2];
   float regulator_out[2];
}RegulatorPI;
/*
////////////             Struktura regulatora         ///////////
typedef struct {
   float kp;
   float ki;
   float ti;
   float td;
   float T;
   float upsat;
   float lowsat;
   float regulator_in[2];
   float regulator_out[2];
}RegulatorPI;
*/
void Reg_Pi(RegulatorPI *st,float PIin);

#endif /* VARIABLE_PROJ_H_ */


