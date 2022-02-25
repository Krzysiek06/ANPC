/*
 * control.h
 *
 *  Created on: 23 sie 2021
 *      Author: Krzysztof Kalinowski
 */

#ifndef CONTROL_H_
#define CONTROL_H_



struct Regulator;
struct sogi_stuct;
typedef struct x stany;
struct RC_struct;

//typedef struct Regulator RegulatorPI;


void abctodq(float *d, float *q, float a,float b, float c,float theta);
void dqtoabc(float *d, float *q, float *a,float *b, float *c,float *theta);
void abctoalfabeta(float *alfa, float *beta, float a,float b, float c);
void alfabetatoabc(float alfa, float beta, float *a,float *b, float *c);
void alfabetatodq(float alfa, float beta, float *d,float *q, float theta);
void alfabetatodq1(float alfa, float beta, float *d,float *q, float theta);
void dqtoalfabeta(float *alfa, float *beta, float d,float q, float theta);

void res_int2(float *n0, float *n2, float *d1, float *d2, float wo, float wcut, float Ts1, float Ki);
void regres1(float *dutyres, float *icos, float the, float izad, float izm, float kharm, float kpr, float Cxx1, float Cxx2,
                 int zero, float *y_res21, float *y_res11, float *y_res01, float *u_res21, float *u_res11, float *u_res01);
void regres3(float *dutyres, float *icos, float the, float izad,  float izm, float Kp, float n0,float n2, float d1,
             float d2, float *y_res21, float *y_res11, float *y_res01, float *u_res21, float *u_res11, float *u_res01, float limup, float limdown);
void pll(float *theta_pll, float f, float wpll2, float kp_pll, float ki_pll,float x_pll);



void filter(float *out, float *old, float coff, float in);
void speedcontrol(float *dutyout, float tempin, float tempmin, float tempmax);

float SPWMSVM(float va, float vb, float vc);
float Bal_DC(float ia, float ib, float ic,float da, float db, float dc);

//void relays(float UDCf,float UDC_lvl1,float UDC_lvl2,float delayPRCH,float *delayPRCH_xf,int *flagaPRCHf,int *flagaPRCH2f,int *RelayPRCHf,int *RelayACf,int *RelayDCf,float Tsf);
void relays(float UDCf,float UDC_lvl1,float delayPRCH,float *delayPRCH_xf,float delayRegU,float *delayRegUx,int *flagaPRCHf,int *RelayPRCHf,int *RelayACf,int *RelayDCf,float Tsf);

float rampaPRCH(float Vdc,float Vout, float Vref, int soft);

int zboczeUP(int x, int x_p);
int zboczeDOWN(int x, int x_p);

//void regulatorPI(float32 *out, float32 *integral, float32 in, float32 in_zad, float32 limp, float32 limn, float32 kp, float32 ti, float32 Ts1);




extern float y_pll,p_pll,i_pll,omegapll;




//void pomiary(mes *tmp, off *tmp1);
/*
void pll(void);
void abctodq(void);
void dqtoabc(void);
void abctoalfabeta(void);
void alfabetatoabc(void);
void alfabetatodq(void);
void dqtoalfabeta(void);
void res_int2(void);
void regres1(void);
void regres3(void);
void modulator(void);
void Reg_Pi(void);
*/



#endif /* CONTROL_H_ */
