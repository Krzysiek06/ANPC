/*
 * control.c
 *
 *  Created on: 23 sie 2021
 *      Author: Krzysztof Kalinowski
 */
#include "math.h"
#include "CONTROL.h"

#include "f28x_project.h"
#include "driverlib.h"
#include "device.h"





float s1qrt3 = 0.5773502691896515;
float s2qrt3 = 1.154700538379252;
float pi=3.14159265359;
float sqrt3 = 1.7320508075688772;



///////////             TRANSFORMACJE               ///////////////
void abctodq(float *d, float *q, float a,float b, float c,float theta){
    float dwapi3 = 0.66666666666 * pi;
    *d = (2.0/3.0) * ((a) * cosf(theta) + (b) * cosf(theta - dwapi3) + (c) * cosf(theta + dwapi3));
    *q = (2.0/3.0) * (a * -sinf(theta) + b * -sinf(theta - dwapi3) + c * -sinf(theta + dwapi3));
}


void dqtoabc(float *d, float *q, float *a,float *b, float *c,float *theta){
    float dwapi3 = 0.66666666666 * pi;
    *a = *d * cosf(*theta) - *q * sinf(*theta);
    *b = *d * cosf(*theta - dwapi3) - *q * sinf(*theta - dwapi3);
    *c = *d * cosf(*theta + dwapi3) - *q * sinf(*theta + dwapi3);
}

void abctoalfabeta(float *alfa, float *beta, float a,float b, float c){
    *alfa = (a) * 0.666666667 - (b+c) * 0.3333333333;
    *beta = s1qrt3 * (b-c);
}


void alfabetatoabc(float alfa, float beta, float *a,float *b, float *c){
    *a = alfa;
    *b = 0.5 * (-alfa+sqrt3*beta);
    *c = 0.5 * (-alfa-sqrt3*beta);
}

void alfabetatodq(float alfa, float beta, float *d,float *q, float theta){
    *d = alfa * cosf(theta) + beta * sinf(theta);
    *q =-alfa * sinf(theta) + beta * cosf(theta);
}

void alfabetatodq1(float alfa, float beta, float *d,float *q, float theta){
    *d = alfa * sinf(theta) - beta * cosf(theta);
    *q =-alfa * cosf(theta) + beta * sinf(theta);
}


void dqtoalfabeta(float *alfa, float *beta, float d,float q, float theta){
    *alfa = d * cosf(theta) - q * sinf(theta);
    *beta = q * cosf(theta) + d * sinf(theta);
}

float SPWMSVM(float va, float vb, float vc){
    float max,min,SPWM_SVM_output;

    if (vb > vc) {
        max = vb;
        if (va > max) max = va;
        min = vc;
        if (va < min) min = va;}
    else {
        max = vc;
        min = vb;
        if (va > max) max = va;
        if (va < min) min = va;}

    SPWM_SVM_output = -0.5 * (min+max);
    return SPWM_SVM_output;
}

float Bal_DC(float ia, float ib, float ic,float da, float db, float dc){

    float dmin,dmax,dmid,imax,imin,imid,Inp,dcomp;
    if ((ib > ic) && (ic > ia))
    {
        imax = ib;
        imid = ic;
        imin = ia;
    }
    else if ((ib > ia) && (ia > ic))
    {
        imax = ib;
        imid = ia;
        imin = ic;
    }
    else if ((ia > ib) && (ib > ic))
    {
        imax = ia;
        imid = ib;
        imin = ic;
    }
    else if ((ia > ic) && (ic > ib))
    {
        imax = ia;
        imid = ic;
        imin = ib;
    }
    else if ((ic > ia) && (ia > ib))
    {
        imax = ic;
        imid = ia;
        imin = ib;
    }
    else if ((ic > ib) && (ib > ia))
    {
        imax = ic;
        imid = ib;
        imin = ia;
    }


    if ((db > dc) && (dc > da))
    {
        dmax = db;
        dmid = dc;
        dmin = da;
    }
    else if ((db > da) && (da > dc))
    {
        dmax = db;
        dmid = da;
        dmin = dc;
    }
    else if ((da > db) && (db > dc))
    {
        dmax = da;
        dmid = db;
        dmin = dc;
    }
    else if ((da > dc) && (dc > db))
    {
        dmax = da;
        dmid = dc;
        dmin = db;
    }
    else if ((dc > da) && (da > db))
    {
        dmax = dc;
        dmid = da;
        dmin = db;
    }
    else if ((dc > db) && (db > da))
    {
        dmax = dc;
        dmid = db;
        dmin = da;
    }

    Inp=-(fabs(dmax)*imax+fabs(dmid)*imid+fabs(dmin)*imin);


        if (Inp>0)
        {
            if(dmid>0)
            {
                dcomp=Inp/(imax-imin+imid);

            }
            else
            {
                dcomp=(Inp+2*fabs(dmid)*imid)/(imax-imin+imid);

                if(fabs(dcomp)<fabs(dmid))
                {
                    dcomp=Inp/(imax-imin-imid);

                    //dcomp=1;
                }
            }
        }
        else
        {
            if(dmid>0)
            {
                dcomp=(Inp+2*fabs(dmid)*imid)/(imax-imin-imid);

                if(fabs(dcomp)<fabs(dmid))
                {
                    dcomp=Inp/(imax-imin+imid);

                }
            }
            else
            {
                dcomp=Inp/(imax-imin-imid);

            }
        }





    return dcomp;
}




/*
void sogi(float v_in, float v_out, float q_out, float k, float Ts){
    float omega = 314.159265;

    a0 = 4 + 2 * k * omega * Ts + omega * omega * Ts * Ts;
    a1 = 2 * omega * omega * Ts * Ts - 8;
    a2 = 4 - 2 * k * omega * Ts + omega * omega * Ts * Ts;
    b0 = 2 * k * omega * Ts;
    b1 = k * omega * omega * Ts * Ts;

    b0a0 = b0/a0;
    a1a0 = a1/a0;
    a2a0 = a2/a0;
    b1a0 = b1/a0;

    v_out = (b0a0)*v_in -(b0a0) * v_in2 -(a1a0)*v_out1-(a2a0)*v_out2;
    q_out = (b1a0)*v_in +2.0*(b1a0)*v_in1 + (b1a0)*v_in2-(a1a0)*q_out1-(a2a0)*q_out2;

    v_out2 = v_out1;
    v_out1 = v_out;

    v_in2 = v_in1;
    v_in1 = v_in;

    q_out2 = q_out1;
    q_out1 = q_out;
}
*/

/*
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


//potrzebna deklaracja np. RegulatorPI reg1;

////////////         REGULATOR PI             //////////////////

void Reg_Pi(RegulatorPI *st,float PIin){
    st->regulator_in[1] = PIin;
    st->regulator_out[1] = st->regulator_out[0] + ((st->ki*st->T*0.5)-st->kp)*st->regulator_in[0] + (st->kp+(st->ki*st->T*0.5))*st->regulator_in[1];
    st->ki = __divf32(st->kp,st->ti); // kp/ti;
    st->td = st->regulator_out[1];

    if (st->regulator_out[1]>st->upsat)  //limit
    {   st->td = st->upsat;
    st->ki = 0;
    }
    else if (st->regulator_out[1]<st->lowsat)
    {  st->td = st->lowsat;
    st->ki = 0;
    }
    st->regulator_out[0] = st->regulator_out[1];    //uaktualnienie
    st->regulator_in[0] = st->regulator_in[1];
}
*/



////////////       REGULATOR REZONANSOWY           /////////////////
/*funkcja wyliczj¹ca sk³adniki regulatora rezonansowego
 * wcut pulsacja odciêcia ustaian na oko³o mkilka rad
 * w pulsacja danje harmonicznej
 */


void res_int2(float *n0, float *n2, float *d1, float *d2, float wo, float wcut, float Ts1, float Ki)
{


    *n0=(4.0*Ki*Ts1*wcut)/(4.0+4.0*Ts1*wcut+wo*wo*Ts1*Ts1);
    *n2=(-4.0*Ki*Ts1*wcut)/(4.0+4.0*Ts1*wcut+wo*wo*Ts1*Ts1);
    *d1=(2.0*wo*wo*Ts1*Ts1-8.0)/(4.0+4.0*Ts1*wcut+wo*wo*Ts1*Ts1);
    *d2=(4.0-4.0*Ts1*wcut+wo*wo*Ts1*Ts1)/(4.0+4.0*Ts1*wcut+wo*wo*Ts1*Ts1);



}


// funkcja regulatora rezonasowego

// przez float *dutyres lub inne mo¿emy dostac sie do wielu zmiennych funkcji
// odow³anie do nich pooprze wskanik & przyk³ad pokazany na dole

void regres1(float *dutyres, float *icos, float the, float izad, float izm, float kharm, float kpr, float Cxx1, float Cxx2,
                 int zero, float *y_res21, float *y_res11, float *y_res01, float *u_res21, float *u_res11, float *u_res01)
{

    float deltares1;

    *icos = izad * cos(the);

        /* Regulator rezonansowy pradu */
        deltares1 = *icos - izm;

        *y_res21=*y_res11;
        *y_res11=*y_res01;

        *u_res21=*u_res11;
        *u_res11=*u_res01;

        *u_res01= deltares1;
        *y_res01 = (kharm*Cxx1)*(*u_res01-*u_res21) - *y_res21 - (*y_res11*Cxx2);

        *dutyres = deltares1*kpr + *y_res01;

        if(*dutyres > 1.0)
        {
            *dutyres =1.0;
        }
        if(*dutyres < -1.0){
            *dutyres = -1.0;
        }



}



void regres3(float *dutyres, float *icos, float the, float izad,  float izm, float Kp, float n0,float n2, float d1,
             float d2, float *y_res21, float *y_res11, float *y_res01, float *u_res21, float *u_res11, float *u_res01, float limup, float limdown)
{

    float deltares1;

    *icos = izad * cos(the);

        /* Regulator rezonansowy pradu */
        deltares1 = *icos - izm;

        *y_res21=*y_res11;
        *y_res11=*y_res01;

        *u_res21=*u_res11;
        *u_res11=*u_res01;

        *u_res01= deltares1;
        *y_res01 =n0*(*u_res01)+n2*(*u_res21)-d1*(*y_res11)-d2*(*y_res21);


        *dutyres = *y_res01+ Kp*deltares1;

        if(*dutyres > limup)
        {
            *dutyres =limup;
        }
        if(*dutyres < limdown){
            *dutyres = limdown;
        }



}


/////////////////     Phase Locked Loop     /////////////////
void pll(float *theta_pll,float x_pll, float wpll2, float kp_pll, float ki_pll, float dt){
    //x_pll = iq; i_pll = 0.0;

    p_pll = kp_pll * x_pll;
    i_pll = i_pll + ki_pll * x_pll * dt;

    if(i_pll >= wpll2){
        i_pll=wpll2;}
    if(i_pll <=(-1*wpll2)){
        i_pll=(-1*wpll2);}

    y_pll = p_pll +  i_pll;

    if (y_pll >= wpll2){
        y_pll = wpll2;
    }
    if (y_pll <= -wpll2){
        y_pll = -wpll2;
    }

    omegapll = wpll2 + y_pll;
    *theta_pll = *theta_pll + omegapll * dt;

    if(*theta_pll >= 2.0*pi){
        *theta_pll = *theta_pll - 2.0*pi;
        }
    if(theta_pll <= 0){
        *theta_pll = *theta_pll + 2.0*pi;
        }
}



void regulatorPI(float32 *out, float32 *integral, float32 in, float32 in_zad, float32 limp, float32 limn, float32 kp, float32 ti, float32 Ts1)
{
    float32 delta1;

    delta1 = in_zad - in;
    *integral=*integral + delta1 * (kp / ti) * Ts1;
    if(*integral >=limp){
        *integral = limp;
    }
    if(*integral <=limn){
        *integral = limn;
    }
    *out = (delta1 * kp + *integral);
    if(*out >= limp){
        *out = limp;
    }
    if(*out <= limn){
        *out = limn;
    }
}



void filter(float *out, float *old, float coff, float in)
{
    *out= coff*in +(1-coff)*(*old);
    *old=*out;
}

void speedcontrol(float *dutyout, float tempin, float tempmin, float tempmax)
{

    if(tempin<=tempmin){
        *dutyout=0.0;
    }
    if(tempin>=tempmin){
        *dutyout=((1.0/(tempmax-tempmin))*tempin - (tempmin/(tempmax-tempmin)));
    }
    if(*dutyout<=0.0){
        *dutyout=0.0;
    }
    if(*dutyout>=1.0){
        *dutyout=1.0;
    }

}


void relays(float UDCf,float UDC_lvl1,float delayPRCH,float *delayPRCH_xf,float delayRegU,float *delayRegUx,int *flagaPRCHf,int *RelayPRCHf,int *RelayACf,int *RelayDCf,float Tsf){
    if (*flagaPRCHf == 0){  //Tryb pr¹dowy
        *RelayPRCHf = 1;
        *RelayACf = 1;
        if (UDCf > UDC_lvl1){

            *RelayDCf = 1;
            *delayPRCH_xf = *delayPRCH_xf + Tsf;  //opoznienie wy³¹czenia prechargingu
            if (*delayPRCH_xf > delayPRCH){
                *RelayPRCHf = 0;
            }
            *delayRegUx = *delayRegUx + Tsf;  //opoznienie wy³¹czenia trybu napieciowego
            if (*delayRegUx > delayRegU){
                *flagaPRCHf = 1;  //wejscie w tryb napieciowy
            }

        }
    }else{              //Tryb napiêciowy
        /**delayPRCH_xf = *delayPRCH_xf + Tsf;
        if (*delayPRCH_xf > delayPRCH){
            *RelayPRCHf = 0;
        }*/
        *RelayPRCHf = 0;
        *RelayACf = 1;
        *RelayDCf = 1;
    }
}

float rampaPRCH(float Vdc,float Vout, float Vref, int soft){
    if (soft==0)
    {
            if((Vref-Vout)>1.0)
            {
                Vout=Vout+0.0003125;
            }
            else if ((Vref-Vout)<(-1.0))
            {
                Vout=Vout-0.0003125;
            }
            else
            {
                Vout=Vout;
            }
    }
    else{
        Vout=Vdc;
    }
    return Vout;
}

int zboczeUP(int x, int x_p){
    int out;
    if (x>x_p) out = 1;
    else out = 0;
    return out;
}

int zboczeDOWN(int x, int x_p){
    int out;
    if (x<x_p) out = 1;
    else out = 0;
    return out;
}
