//#############################################################################
//
// FILE:   ControlANPCv1_project_cpu1
//
// TITLE:  Control for ANPC converter in MoReSiC Project
//
// AUTHORS: Rafa³ Miœkiewicz
//          Krzysztof Kalinowski
//
//
// $TI Release: F2838x Support Library v3.04.00.00 $
// $Release Date: Fri Feb 12 19:08:49 IST 2021 $
//#############################################################################
//
// Included Files
//
#include "f28x_project.h"
#include "driverlib.h"
#include "device.h"

#include "EPWM_PROJ.h"
#include "ADC_PROJ.h"
#include "GPIO_PROJ.h"

#include "VARIABLE_PROJ.h"
#include "I2C_PROJ.h"
#include "SPI_PROJ.h"
#include "CONTROL.h"
#include "math.h"
#include "CAN_PROJ.h"

//
// Globals
//

//Dekalaracja struktur
measure mes;
protect zab;
RegulatorPI reg1, reg2;
sogi_init_struct si;
sogi_stuct ss_alfa,ss_beta;
RC_struct rc_init_a3,rc_init_a5,rc_init_a7,rc_init_a11,rc_init_b3,rc_init_b5,rc_init_b7,rc_init_b11;

void regulatorPI(float32 *out, float32 *integral, float32 in, float32 in_zad, float32 limp, float32 limn, float32 kp, float32 ti, float32 Ts1);


//RegulatorPI reg1;

void pomiary(void);
void pomiaryinit(void);
void zabezpieczeniainit(void);
void zabezpieczenia(void);
void sogi_init(void);

//
// Function Prototypes
//

__interrupt void epwm1ISR(void);
__interrupt void canaISR(void);
__interrupt void i2cAISR(void);
__interrupt void faulterror1(void);
__interrupt void faulterror3(void);
__interrupt void faulterror2(void);
__interrupt void epwm14ISR(void);
//
// Main
//
void main(void){

        Device_init(); // Initializes system control, device clock, and peripherals
        Device_initGPIO();// Disable pin locks and enable internal pull ups.
        Interrupt_initModule();        // Initializes PIE and clear PIE registers. Disables CPU interrupts.
        // and clear all CPU interrupt flags.
        Interrupt_initVectorTable();        // Initialize the PIE vector table with pointers to the shell interrupt
        // Service Routines (ISR).
        SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);        // Disable sync(Freeze clock to PWM as well)
        GPIOSetup();        // Set up the ADC and the ePWM and initialize the SOC

        configureADC(ADCA_BASE);
        configureADC(ADCB_BASE);
        configureADC(ADCD_BASE);

       // initADC();
        initEPWM();
        initI2C();
        initADCSOC();
        initSPI();
        InitXBar();

        SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);// Enable sync and clock to PWM
        // Assign the interrupt service routines to ePWM interrupts
        // Interrupts that are used in this example are re-mapped to
        // ISR functions found within this file.
        // This registers the interrupt handler in PIE vector table.
        Interrupt_register(INT_EPWM1, &epwm1ISR);
        Interrupt_register(INT_CANA0,&canaISR);
        Interrupt_register(INT_I2CA, &i2cAISR);
        Interrupt_register(INT_XINT1, &faulterror1);
        Interrupt_register(INT_XINT2, &faulterror2);
        Interrupt_register(INT_XINT3, &faulterror3);
        Interrupt_register(INT_EPWM14, &epwm14ISR);

        // Enable ePWM interrupts
        Interrupt_enable(INT_EPWM1);
        Interrupt_enable(INT_XINT1);
        Interrupt_enable(INT_XINT2);
        Interrupt_enable(INT_XINT3);
        Interrupt_enable(INT_EPWM14);

        // Initialize the CAN controllers
        CAN_initModule(CANA_BASE);
        CAN_setBitRate(CANA_BASE, DEVICE_SYSCLK_FREQ, 500000, 20);

        // Enable interrupts on the CAN A peripheral.
        CAN_enableInterrupt(CANA_BASE, CAN_INT_IE0 | CAN_INT_ERROR |
                            CAN_INT_STATUS);
        // Enable global Interrupts and higher priority real-time debug events:
        EINT;  // Enable Global interrupt INTM
        ERTM;  // Enable Global realtime interrupt DBGM


        // Enable the CAN-A interrupt signal
        Interrupt_enable(INT_CANA0);
        CAN_enableGlobalInterrupt(CANA_BASE, CAN_GLOBAL_INT_CANINT0);

        // Start ePWM1, enabling SOCA and putting the counter in up-count mode
        EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
        EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);

        CANAobject();
        // Start CAN module A operations
        CAN_startModule(CANA_BASE);

        XintRegs.XINT1CR.bit.POLARITY=0;
        XintRegs.XINT1CR.bit.ENABLE=1;
        XintRegs.XINT2CR.bit.POLARITY=0;
        XintRegs.XINT2CR.bit.ENABLE=1;
        XintRegs.XINT3CR.bit.POLARITY=0;
        XintRegs.XINT3CR.bit.ENABLE=1;

        pomiaryinit();
        zabezpieczeniainit();
        si.k_sogi = 1.41;
        sogi_init();
        //sogi_init(sogi_init_stuct_a);
        Kp_RC = 2.0;
        //Inicjalizacja czynników w regulatorze rezonansowym

        rc_init_a3.K = 1.0;
        rc_init_a5.K = 1.0;
        rc_init_a7.K = 1.0;
        rc_init_a11.K = 1.0;

        rc_init_b3.K = 1.0;
        rc_init_b5.K = 1.0;
        rc_init_b7.K = 1.0;
        rc_init_b11.K = 1.0;

        rc_wcut=0.02;
        rc_ki3=5000;
        rc_ki5=20000;
        rc_ki7=20000;
        rc_ki11=15000;

        RC_init(&rc_init_a3,3,rc_wcut,rc_ki3,Ts);
        RC_init(&rc_init_a5,5,rc_wcut,rc_ki5,Ts);
        RC_init(&rc_init_a7,7,rc_wcut,rc_ki7,Ts);
        RC_init(&rc_init_a11,11,rc_wcut,rc_ki11,Ts);

        RC_init(&rc_init_b3,3,rc_wcut,rc_ki3,Ts);
        RC_init(&rc_init_b5,5,rc_wcut,rc_ki5,Ts);
        RC_init(&rc_init_b7,7,rc_wcut,rc_ki7,Ts);
        RC_init(&rc_init_b11,11,rc_wcut,rc_ki11,Ts);
        kDC=0.1;


        while(1){
            // Za³czenie Buffora PWM Wentylatorów
            EN_PWM_GROUP4_ON;

            EPwm10Regs.CMPA.bit.CMPA=Duty10A*EPWM10_TIMER_TBPRD;
            EPwm10Regs.CMPB.bit.CMPB=Duty10B*EPWM10_TIMER_TBPRD;
            EPwm11Regs.CMPA.bit.CMPA=Duty11A*EPWM11_TIMER_TBPRD;

            // Wentylatory pp

            speedcontrol(&Duty12A, temp2, tempmin, tempmax);
            speedcontrol(&Duty12B, temp3, tempmin, tempmax);
            speedcontrol(&Duty11B, temp1, tempmin, tempmax);
            EPwm11Regs.CMPB.bit.CMPB=Duty11B*EPWM11_TIMER_TBPRD;
            EPwm12Regs.CMPA.bit.CMPA=Duty12A*EPWM12_TIMER_TBPRD;
            EPwm12Regs.CMPB.bit.CMPB=Duty12B*EPWM12_TIMER_TBPRD;

            //
            EPwm13Regs.CMPA.bit.CMPA=Duty13A*EPWM13_TIMER_TBPRD;
            EPwm13Regs.CMPB.bit.CMPB=Duty13B*EPWM13_TIMER_TBPRD;


            //Obs³uga Przetwornika CA
            cnt++;
            if(cnt>1){
                cnt1=cnt1+1;
                if(cnt1>=4095){
                    cnt1=0;
                }
            }

            switch(zmiana_wyswietl){
            case 1: // Ud Uq
                writeDataSPI(0, 8, Udout*10);
                writeDataSPI(0, 9, Uqout*100+1500);
                break;
            case 2: // uchyb balans napiec DC
                writeDataSPI(0, 8, (UDC1-UDC2)*10+1500);
                writeDataSPI(0, 9, Ubal_out*10+1500);

                break;
            case 3: // Udc1,Udc2
                writeDataSPI(0, 8, UDC2*10);
                writeDataSPI(0, 9, UDC1*10);
                break;
            case 4: // rezonansowe3
                writeDataSPI(0, 8, rc_init_a3.y_res01*10+1500);
                writeDataSPI(0, 9, rc_init_b3.y_res01*10+1500);
                break;
            case 5: // rezonansowe5
                writeDataSPI(0, 8, rc_init_a5.y_res01*10+1500);
                writeDataSPI(0, 9, rc_init_b5.y_res01*10+1500);
                break;
            case 6: // rezonansowe7
                writeDataSPI(0, 8, rc_init_a7.y_res01*10+1500);
                writeDataSPI(0, 9, rc_init_b7.y_res01*10+1500);
                break;
            case 7: // rezonansowe11
                writeDataSPI(0, 8, rc_init_a11.y_res01*10+1500);
                writeDataSPI(0, 9, rc_init_b11.y_res01*10+1500);
                break;
            case 8:
                writeDataSPI(0, 8, IAC1*10+1500);
                writeDataSPI(0, 9, UAC1*10+1500);
                break;
            case 9:
                writeDataSPI(0, 8, UDC1mean*10);
                writeDataSPI(0, 9, UDC2mean*10);
                break;

            case 10:
                writeDataSPI(0, 8, Duty1*1000+1500);
                writeDataSPI(0, 9, Duty2*1000+1500);
                break;

            case 13:
                writeDataSPI(0, 8, Duty1*1000+1500);
                writeDataSPI(0, 9, Duty2*1000+1500);
                break;

            case 14:
                writeDataSPI(0, 8, Duty1*1000+1500);
                writeDataSPI(0, 9, Duty2*1000+1500);
                break;

            case 20:
                writeDataSPI(0, 8, Idramp*10+1500);
                writeDataSPI(0, 9, (UDC-UDCramp)*10+1500);
                break;
            default:
                writeDataSPI(0, 8, stan_pracy*200);
                writeDataSPI(0, 9, stan_pracy*200);
            }
            // Wpisywanie zmiennych do wystawienia na przetwornik CA
            //writeDataSPI(0, 8, Id*100+1500);
            //writeDataSPI(0, 9, Duty1*1000+1500);
            //writeDataSPI(0, 8, harm3*1000+1500);
            //writeDataSPI(0, 9, -1.0*Id*100);
            //writeDataSPI(0, 9, Udout*100);
            //writeDataSPI(0, 8, mes.UDC2*10);
            //writeDataSPI(0, 8, Ubal_out*1000+1500);

            //writeDataSPI(0, 8, sogi_alfa*10+1500);
            //writeDataSPI(0, 9, sogi_beta*10+1500);
            //writeDataSPI(0, 9, mes.UDC1*10);
            //writeDataSPI(0, 9, Bal_DCout*1000+1500);

/*
            // OBSLUGA Przetwornika do poniaru temperatury++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            readDataSPIADC70(adc_chanel,&adc_value,&old_chanel);
            switch(old_chanel){
                case 0:
                    temp5=skalatemp*adc_value;
                break;

                case 1:
                    temp6=skalatemp*adc_value;
                break;
                case 2:
                    temp1=skalatemp*adc_value;
                break;
                case 3:
                    temp2=skalatemp*adc_value;
                break;
                case 4:
                    temp3=skalatemp*adc_value;
                break;
                case 5:
                    temp4=skalatemp*adc_value;
                break;

            }
            adc_chanel++;
            if(adc_chanel>5){
                adc_chanel=0;
            }
*/
        }
}

//
// epwm1ISR - ePWM 1 ISR


__interrupt void epwm1ISR(void)
{
    LED4_ON;

    pomiary();      //Zebranie wartoœci pomiarów z przetworników A/D 16bit single
    zabezpieczenia(); //Sprawdzenie zabezpieczeñ (Nadpr¹dowe, Wysokie napiêcie) obs³uga desatu

    Fir_UI_AC=0.4;
    filter(&IAC1,&IAC1_old,Fir_UI_AC,mes.IAC1);
    filter(&IAC2,&IAC2_old,Fir_UI_AC,mes.IAC2);
    filter(&IAC3,&IAC3_old,Fir_UI_AC,mes.IAC3);
    filter(&UAC1,&UAC1_old,Fir_UI_AC,mes.UAC1);
    filter(&UAC2,&UAC2_old,Fir_UI_AC,mes.UAC2);
    filter(&UAC3,&UAC3_old,Fir_UI_AC,mes.UAC3);


    Fir_UI_DC=0.2;
    filter(&UDC1,&UDC1_old,Fir_UI_DC,mes.UDC1);
    filter(&UDC2,&UDC2_old,Fir_UI_DC,mes.UDC2p);

    filter(&UDC1mean,&UDC1mean_old,0.001,UDC1);
    filter(&UDC2mean,&UDC2mean_old,0.001,UDC2);

    UDC=(UDC1+UDC2);
    Rinit=220;
    Iinit=5.4*(-Ud)/Rinit;

    if(STOP_PROC==1 || UDC<30 ){
        Urefd = UDCon*0.1;
        /*if (abs(UDC-Urefd*10)<5.0){
            StartPWM = 0;
            RelayAC = 0;
        }*/

        REL7_OFF
        RelayAC = 0;


        StartPWM = 0;
        integral2 = 0;
        integral3 = 0;
        Ubal_int = 0;
        Ud1 = 2*(-Ud);
        integral1 = Ud1;
        Uq1 = 0;
        Idramp = 0.0;
        startreg = 0;
        startreg_fl = 1;
        flagaprocedura = 0;


       if (stan_pracy>AcRelayOn)
        {
            stan_pracy=ReadyToGo;
            REL4_OFF
            RelayPRCH = 0;

        }
       // stan_pracy=ReadyToGo;
        //flagastopu_m=1;
        if(UDC<20)
            {
                REL6_OFF
                RelayDC = 0;
            }
    }
    warunek=((STOP_PRECH==1 && STOP_PROC==1) || Fault==1 || ((stan_pracy<TurnOnVoltageMode) && (UDC>3*UDCon)) || stan_pracy==ReadyToGo || ((RelayAC==1) && (UDC<UDCon)) || TurnOffInverter==1);

    warunek=0;
    if(warunek==1)
        {
        TurnOffInverter=0;
        warunek=0;
        REL4_OFF
        RelayPRCH = 0;
        REL7_OFF
        RelayAC = 0;
        StartPWM=0;
        flagaON = 0;
        startreg = 0;
        delayPWM = 0.0;
        delayRegUx = 0.0;
        stan_pracy=ReadyToGo;
        if(UDC<20)
            {
                REL6_OFF
                RelayDC = 0;
            }

    }

    UDCon = 1.6 * (-Ud);


    switch (stan_pracy)
    {
    case ReadyToGo:
            if(RelayAC==0 && RelayDC==0 && RelayPRCH == 0 && START_PROC==0 && START_PRECH==0)//tutaj trzeba odczyta wartoœæ wyjœcia steruj¹cego przekaŸnikiem zamiast relay==0
            {
                stan_pracy=ReadyToPreCharge;
            }


            break;
    case ReadyToPreCharge:
            if(-Ud>20 && RelayAC==0 && RelayDC==0 && RelayPRCH == 0 && START_PROC==0 && START_PRECH==1)
            {
                REL4_ON
                RelayPRCH=1;
                stan_pracy=AcRelayOn;
            }
            else
            {
                REL4_OFF
                RelayPRCH=0;
            }
            break;
    case AcRelayOn:
            if(UDC>UDCon && START_PROC==1 && RelayDC==0 && RelayAC==0 && RelayPRCH==1)
            {
                REL7_ON
                RelayAC=1;
                REL4_OFF
                RelayPRCH = 0;
                timer=0;
            }
            else if (UDC>UDCon && START_PROC==1 && RelayDC==0 && RelayAC==1 && RelayPRCH==0)
            {
                timer=timer+Ts;
            }
            else
            {
                REL7_OFF
                RelayAC=0;
                timer=0;
            }
            if(timer>1 && RelayAC==1)
            {
                timer=0;
                stan_pracy=CurrentMode;
            }
            break;
    case CurrentMode:
            Ud1 = 2*(-Ud);
            integral1 = Ud1;
            StartPWM=1;
            startreg=0;//tryb pr¹dowy??
            stan_pracy=DcRelayOn;
            break;
    case DcRelayOn:
            if (RelayDC==0)
            {
                REL6_ON
                RelayDC=1;
                timer=0;
            }
            else if (UDC>UDCon && RelayDC==1)
            {
                timer=timer+Ts;
            }
             else
            {
                REL6_OFF
                RelayDC=0;
                timer=0;
            }
            if(timer>0.02)
            {
                timer=0;
                stan_pracy=TurnOnVoltageMode;
                UDCramp = UDC;
                //integral3 = Idramp;
            }
            break;
    case TurnOnVoltageMode:
            startreg=1;
            break;
    default:
        stan_pracy=ReadyToGo;
        break;


    }


    if (IAC1 > IAC1max) IAC1max = IAC1;
    if (IAC2 > IAC2max) IAC2max = IAC2;
    if (IAC3 > IAC3max) IAC3max = IAC3;

    if (UAC1 > UAC1max) UAC1max = UAC1;
    if (UAC2 > UAC2max) UAC2max = UAC2;
    if (UAC3 > UAC3max) UAC3max = UAC3;


    if (timerscal >= 0.02){

        IAC1maxw = IAC1max;
        IAC2maxw = IAC2max;
        IAC3maxw = IAC3max;
        UAC1maxw = UAC1max;
        UAC2maxw = UAC2max;
        UAC3maxw = UAC3max;

        IAC1max = 0;
        IAC2max = 0;
        IAC3max = 0;
        UAC1max = 0;
        UAC2max = 0;
        UAC3max = 0;

        timerscal = 0;
    }
    timerscal = timerscal + Ts;


    //Transformcje ABC do alfa beta
    abctoalfabeta(&Ua, &Ub, UAC1, UAC2 , UAC3);

    //SOGI -PLL
    ss_alfa.v_in = Ub;
    sogi_exec(&ss_alfa);
    ss_beta.v_in = Ua;
    sogi_exec(&ss_beta);
    sogi_alfa = 0.5 * (ss_alfa.v_out-ss_beta.q_out);
    sogi_beta = 0.5 * (ss_alfa.q_out+ss_beta.v_out);

    abctoalfabeta(&Ia, &Ib, IAC1, IAC2 , IAC3);

    //Transformcje alfa beta do dq
    alfabetatodq(sogi_beta, sogi_alfa, &Ud, &Uq, theta2);
    alfabetatodq(Ia, Ib, &Id, &Iq, theta2);

    //PLL synchronizacja z sieci¹
    pll(&theta, Uq, omega, Kp_pll, Ki_pll, Ts);
    // Odwrocenie k¹ta
    theta2=2*PI-theta;

    // Wyswietlanie adc
    zmiana_wyswietl = ti4;
    switch(zmiana_wyswietl){
    case 2:
        kDC = kp4;
        tDC = ti3;
        break;
    case 3:
        pomfilter = kp4;
        break;
    case 4:
        res_mx3 = kp4;
         break;
    case 5:
        res_mx5 = kp4;
        break;
    case 6:
        res_mx7 = kp4;
        break;
    case 7:
        res_mx11 = kp4;
        break;

    case 12:
        Fir_UI_DC=kp4;
        Fir_UI_AC=ti3;
        break;
    case 13:
        if(stan_pracy<AcRelayOn)
        {
            res_on=kp4;
        }

        break;
    case 14:
        if(stan_pracy<AcRelayOn)
        {
            //wspdt = kp4*0.01;

        }
        wspdt = kp4*0.01;
        break;
    case 20:
        kpU = kp4;
        tiU = ti3;
        break;
    }
    //
    /////////////////////   START PWM   ////////////////////////
    //
    if(StartPWM==1 && Fault==0){
         EN_PWM_GROUP1_ON;
         EN_PWM_GROUP2_ON;
         EN_PWM_GROUP3_ON;
         ENABLE1_ON;
         ENABLE2_ON;
         ENABLE3_ON;

         if (startreg == 1.0){//Tryb pr¹dowy - chyba napiêciowy? (MH)
             UDCramp=rampaPRCH(UDC,UDCramp,Urefd*10,RelayPRCH);  // Rampa napiêciowa
             regulatorPI(&Idramp,&integral3,-UDCramp,-UDC,10,0,0.02*kpU,0.005*tiU,Ts);  // Regulator napiêcia
         }
         else{ //Tryb napiêciowy - chyba pr¹dowy? (MH)
             //  Rampa do zadawania pr¹du
             if ( Idramp <= Iinit){
                 Idramp = Idramp + 0.001;
             }
             if ( Idramp >= Iinit){
                 Idramp = Idramp - 0.001;
             }
            integral3 = Irefd;  //wartosc poczatkowa calki do regulatora napieciiowego
         }

         kpreg = kp1;
         tireg = ti1;

         regulatorPI(&Ud1, &integral1, Id, -Idramp, 660.0, -660.0, kpreg, tireg*0.01, Ts);   //Regulator pradu w osi d
         regulatorPI(&Uq1, &integral2, Iq, 0, 300.0, -300.0, kpreg, tireg*0.01, Ts);        //Regulator pradu w osi q

         Udout=Ud1+Ud-omegaL*Iq;
         Uqout=Uq1+Uq+omegaL*Id;

         // Transformacje odwrotnie dq->alfabeta->abc
         dqtoalfabeta(&Ualfaout, &Ubetaout, Udout, Uqout,theta2);
         dqtoalfabeta(&Ialfaref,&Ibetaref, -Idramp, 0, theta2);

         // Proportional Resonant Controller
         Kp_RC = kp3;
         RC_exec(&rc_init_a3,Ialfaref-Ia,res_mx3);
         RC_exec(&rc_init_a5,Ialfaref-Ia,res_mx5);
         RC_exec(&rc_init_a7,Ialfaref-Ia,res_mx7);
         RC_exec(&rc_init_a11,Ialfaref-Ia,res_mx11);
         Ialfa_out = Kp_RC * (Ialfaref-Ia) + rc_init_a3.y_res01 + rc_init_a5.y_res01 + rc_init_a7.y_res01 + rc_init_a11.y_res01;

         RC_exec(&rc_init_b3,Ibetaref-Ib,res_mx3);
         RC_exec(&rc_init_b5,Ibetaref-Ib,res_mx5);
         RC_exec(&rc_init_b7,Ibetaref-Ib,res_mx7);
         RC_exec(&rc_init_b11,Ibetaref-Ib,res_mx11);
         Ibeta_out = Kp_RC * (Ibetaref-Ib) + rc_init_b3.y_res01 + rc_init_b5.y_res01 + rc_init_b7.y_res01 + rc_init_b11.y_res01;

         if(res_on==1)
         {
             alfabetatoabc(Ualfaout+Ialfa_out, Ubetaout+Ibeta_out, &Duty1r,&Duty2r,&Duty3r);
         }
         else if (res_on==0)
         {
             alfabetatoabc(Ualfaout, Ubetaout, &Duty1r,&Duty2r,&Duty3r);  //wy³¹czony rezonansowy
         }
         else TurnOffInverter=1;

         //alfabetatoabc(Ualfaout, Ubetaout, &Duty1r,&Duty2r,&Duty3r);  //wy³¹czony rezonansowy

         if(UDC1>0.0 & UDC2>0.0){

             //harm3 =  sinf(3.0 *(theta2 + PI*0.5)) * 0.15 * (Udout/UDC); // Basic ZSVI
             harm3_v2 = SPWMSVM(Duty1r,Duty2r,Duty3r);  // Zero Sequence Voltage Injector - SPWM SVM
             //harm3 = 0.0;

             Duty1r = Duty1r + harm3_v2;
             Duty1 = 2.0*Duty1r/UDC;
             //Duty1 = Duty1  + harm3;

             Duty2r = Duty2r + harm3_v2;
             Duty2 = 2.0*Duty2r/UDC;
             //Duty2 = Duty2 + harm3;

             Duty3r = Duty3r + harm3_v2;
             Duty3 = 2.0*Duty3r/UDC;
             //Duty3 = Duty3 + harm3;

             // Voltage Balancer
             Ubal_in = (UDC1-UDC2)/(UDC1+UDC2);
             regulatorPI(&Ubal_out, &Ubal_int, 0, Ubal_in, 1, -1, kDC, 0.01 * tDC, Ts);
             //Bal_DCout = Bal_DC(mes.IAC1,mes.IAC2,mes.IAC3,Duty1,Duty2,Duty3);

             Bal_DCout=1.0;
             //Ubal_out = 0;

             Duty1 = Duty1 - (Ubal_out*Bal_DCout);//Bal_DCout);
             Duty2 = Duty2 - (Ubal_out*Bal_DCout);//Bal_DCout);
             Duty3 = Duty3 - (Ubal_out*Bal_DCout);//Bal_DCout);


             // Dead time compensator
             if((Duty1>0) && (Duty2<0) && (Duty3<0)){
                 state=-1;
             }
             if((Duty2>0) && (Duty1<0) && (Duty3<0)){
                 state=-1;
             }
             if((Duty3>0) && (Duty2<0) && (Duty1<0)){
                 state=-1;
             }
             if((Duty1<0) && (Duty2>0) && (Duty3>0)){
                 state=1;
             }
             if((Duty2<0) && (Duty1>0) && (Duty3>0)){
                 state=1;
             }
             if((Duty3<0) && (Duty2>0) && (Duty1>0)){
                 state=1;
             }

             Duty1 = Duty1 + state*wspdt;
             Duty2 = Duty2 + state*wspdt;
             Duty3 = Duty3 + state*wspdt;

             if(Duty1>=0){
                 Duty1 = Duty1 + wspdt;
             }
             if(Duty1<0){
                 Duty1 = Duty1 - wspdt;
             }
             if(Duty2>=0){
                 Duty2 = Duty2 + wspdt;
             }
             if(Duty2<0){
                 Duty2 = Duty2 - wspdt;
             }
             if(Duty3>=0){
                 Duty3 = Duty3 + wspdt;
             }
             if(Duty3<0){
                 Duty3 = Duty3 - wspdt;
             }


             // modulation signal saturation
             if (Duty1 >= 1.0){
                 Duty1 = 1.0;
             }
             if (Duty1 <= -1.0){
                 Duty1 = -1.0;
             }
             if (Duty2 >= 1.0){
                 Duty2 = 1.0;
             }
             if (Duty2 <= -1.0){
                 Duty2 = -1.0;
             }
             if (Duty3 >= 1.0){
                 Duty3 = 1.0;
             }
             if (Duty3 <= -1.0){
                 Duty3 = -1.0;
             }


         }else{
             Duty1=0.0;
             Duty2=0.0;
             Duty3=0.0;
         }

         //Modulator SPWM
         if(Duty1>=0.0){
              EPwm1Regs.CMPA.bit.CMPA=Duty1*EPWM1_TIMER_TBPRD;
              EPwm2Regs.CMPA.bit.CMPA=1.0*EPWM2_TIMER_TBPRD;
              EPwm3Regs.CMPA.bit.CMPA=0.0*EPWM3_TIMER_TBPRD;
              //Duty1f = 1;
         }
         if(Duty1<0.0){
             EPwm1Regs.CMPA.bit.CMPA=0.0*EPWM1_TIMER_TBPRD;
             EPwm2Regs.CMPA.bit.CMPA=0.0*EPWM2_TIMER_TBPRD;
             EPwm3Regs.CMPA.bit.CMPA=-1.0*Duty1*EPWM3_TIMER_TBPRD;
             //Duty1f = 0;
         }
         if(Duty2>=0.0){
             EPwm4Regs.CMPA.bit.CMPA=Duty2*EPWM4_TIMER_TBPRD;
             EPwm5Regs.CMPA.bit.CMPA=1.0*EPWM5_TIMER_TBPRD;
             EPwm6Regs.CMPA.bit.CMPA=0.0*EPWM6_TIMER_TBPRD;
         }
         if(Duty2<0.0){
             EPwm4Regs.CMPA.bit.CMPA=0.0*EPWM4_TIMER_TBPRD;
             EPwm5Regs.CMPA.bit.CMPA=0.0*EPWM5_TIMER_TBPRD;
             EPwm6Regs.CMPA.bit.CMPA=-1.0*Duty2*EPWM6_TIMER_TBPRD;
         }
         if(Duty3>=0.0){
             EPwm7Regs.CMPA.bit.CMPA=Duty3*EPWM7_TIMER_TBPRD;
             EPwm8Regs.CMPA.bit.CMPA=1.0*EPWM8_TIMER_TBPRD;
             EPwm9Regs.CMPA.bit.CMPA=0.0*EPWM9_TIMER_TBPRD;
         }
         if(Duty3<0.0){
             EPwm7Regs.CMPA.bit.CMPA=0.0*EPWM7_TIMER_TBPRD;
             EPwm8Regs.CMPA.bit.CMPA=0.0*EPWM8_TIMER_TBPRD;
             EPwm9Regs.CMPA.bit.CMPA=-1.0*Duty3*EPWM9_TIMER_TBPRD;
         }


    }else{
        EN_PWM_GROUP3_OFF;
        EN_PWM_GROUP2_OFF;
        EN_PWM_GROUP1_OFF;

        ENABLE1_OFF;
        ENABLE2_OFF;
        ENABLE3_OFF;

        EPwm1Regs.CMPA.bit.CMPA=0.0*EPWM1_TIMER_TBPRD;
        EPwm2Regs.CMPA.bit.CMPA=0.0*EPWM2_TIMER_TBPRD;
        EPwm3Regs.CMPA.bit.CMPA=0.0*EPWM3_TIMER_TBPRD;
        EPwm4Regs.CMPA.bit.CMPA=0.0*EPWM4_TIMER_TBPRD;
        EPwm5Regs.CMPA.bit.CMPA=0.0*EPWM5_TIMER_TBPRD;
        EPwm6Regs.CMPA.bit.CMPA=0.0*EPWM6_TIMER_TBPRD;
        EPwm7Regs.CMPA.bit.CMPA=0.0*EPWM7_TIMER_TBPRD;
        EPwm8Regs.CMPA.bit.CMPA=0.0*EPWM8_TIMER_TBPRD;
        EPwm9Regs.CMPA.bit.CMPA=0.0*EPWM9_TIMER_TBPRD;
    }


    LED4_OFF;

    //
    // Clear INT flag for this timer
    //
    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);

    //
    // Acknowledge interrupt group
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

__interrupt void epwm14ISR(void)
{


    zab.Iacmax=Iacmax;
    zab.Iacmin=Iacmin;
    zab.Idcmax=Idcmax;
    zab.Idcmin=Idcmin;
    zab.Uacmax=Uacmax;
    zab.Uacmin=Uacmin;
    zab.Udcmax=Udcmax;
    zab.Udcmin=Udcmin;

       // STOP_PROC= ((rxdata1[0]>>1) & 0x01);)

    if (CONNECTION_TI_PI == 1){
        CONNECTION_TI_PI = 0;
        CONNECTION_WAIT = 0;
    }else{
        if (CONNECTION_FAULT==0) CONNECTION_WAIT = CONNECTION_WAIT + 1;
        if (CONNECTION_WAIT >= 1000){
            CONNECTION_FAULT = 1;
        }
    }


    x5++;
    if(x5>=100){

        CANAMsgWrite();
        x5=0;
    }

// Clear INT flag for this timer
//
    EPWM_clearEventTriggerInterruptFlag(EPWM14_BASE);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);

}

__interrupt void canaISR(void)
{
    x3++;
    CANAMsgRead();
    //
    // Clear the global interrupt flag for the CAN interrupt line
    //
    CAN_clearGlobalInterruptStatus(CANA_BASE, CAN_GLOBAL_INT_CANINT0);
    //
    // Acknowledge this interrupt located in group 9
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

__interrupt void faulterror1(void)
{

    Fault=1;
    FaultCode=14;

    EALLOW;
    EPwm1Regs.TZFRC.bit.OST = 1;
    EPwm2Regs.TZFRC.bit.OST = 1;
    EPwm3Regs.TZFRC.bit.OST = 1;
    EPwm4Regs.TZFRC.bit.OST = 1;
    EPwm5Regs.TZFRC.bit.OST = 1;
    EPwm6Regs.TZFRC.bit.OST = 1;
    EPwm7Regs.TZFRC.bit.OST = 1;
    EPwm8Regs.TZFRC.bit.OST = 1;
    EPwm9Regs.TZFRC.bit.OST = 1;
    EDIS;

    //
    // Acknowledge this interrupt located in group 1
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}


__interrupt void faulterror2(void)
{

    Fault=1;
    FaultCode=15;
    EALLOW;
    EPwm1Regs.TZFRC.bit.OST = 1;
    EPwm2Regs.TZFRC.bit.OST = 1;
    EPwm3Regs.TZFRC.bit.OST = 1;
    EPwm4Regs.TZFRC.bit.OST = 1;
    EPwm5Regs.TZFRC.bit.OST = 1;
    EPwm6Regs.TZFRC.bit.OST = 1;
    EPwm7Regs.TZFRC.bit.OST = 1;
    EPwm8Regs.TZFRC.bit.OST = 1;
    EPwm9Regs.TZFRC.bit.OST = 1;
    EDIS;


    //
    // Acknowledge this interrupt located in group 1
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt void faulterror3(void)
{

///rtyu
    Fault=1;
    FaultCode=16;

    EALLOW;
    EPwm1Regs.TZFRC.bit.OST = 1;
    EPwm2Regs.TZFRC.bit.OST = 1;
    EPwm3Regs.TZFRC.bit.OST = 1;
    EPwm4Regs.TZFRC.bit.OST = 1;
    EPwm5Regs.TZFRC.bit.OST = 1;
    EPwm6Regs.TZFRC.bit.OST = 1;
    EPwm7Regs.TZFRC.bit.OST = 1;
    EPwm8Regs.TZFRC.bit.OST = 1;
    EPwm9Regs.TZFRC.bit.OST = 1;
    EDIS;


    //
    // Acknowledge this interrupt located in group 12
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP12);
}



//
// i2cAISR - I2C A ISR (non-FIFO)
//
__interrupt void i2cAISR(void)
{
    I2C_InterruptSource intSource;
    uint16_t i;

    //
    // Read interrupt source
    //
    intSource = I2C_getInterruptSource(I2CA_BASE);

    //
    // Interrupt source = stop condition detected
    //
    if(intSource == I2C_INTSRC_STOP_CONDITION)
    {
        //
        // If completed message was writing data, reset msg to inactive state
        //
        if(currentMsgPtr->msgStatus == MSG_STATUS_WRITE_BUSY)
        {
            currentMsgPtr->msgStatus = MSG_STATUS_INACTIVE;
        }
        else
        {
            //
            // If a message receives a NACK during the address setup portion of
            // the EEPROM read, the code further below included in the register
            // access ready interrupt source code will generate a stop
            // condition. After the stop condition is received (here), set the
            // message status to try again. User may want to limit the number
            // of retries before generating an error.
            //
            if(currentMsgPtr->msgStatus == MSG_STATUS_SEND_NOSTOP_BUSY)
            {
                currentMsgPtr->msgStatus = MSG_STATUS_SEND_NOSTOP;
            }
            //
            // If completed message was reading EEPROM data, reset message to
            // inactive state and read data from FIFO.
            //
            else if(currentMsgPtr->msgStatus == MSG_STATUS_READ_BUSY)
            {
                currentMsgPtr->msgStatus = MSG_STATUS_INACTIVE;
                for(i=0; i < NUM_BYTES; i++)
                {
                    currentMsgPtr->msgBuffer[i] = I2C_getData(I2CA_BASE);
                }

                //
                // Check received data
                //
                for(i=0; i < NUM_BYTES; i++)
                {
                    if(i2cMsgIn.msgBuffer[i] == i2cMsgOut.msgBuffer[i])
                    {
                     //   passCount++;
                    }
                    else
                    {//
                        //failCount++;
                    }
                }
            }
        }
    }
    //
    // Interrupt source = Register Access Ready
    //
    // This interrupt is used to determine when the EEPROM address setup
    // portion of the read data communication is complete. Since no stop bit
    // is commanded, this flag tells us when the message has been sent
    // instead of the SCD flag.
    //
    else if(intSource == I2C_INTSRC_REG_ACCESS_RDY)
    {
        //
        // If a NACK is received, clear the NACK bit and command a stop.
        // Otherwise, move on to the read data portion of the communication.
        //
        if((I2C_getStatus(I2CA_BASE) & I2C_STS_NO_ACK) != 0)
        {
            I2C_sendStopCondition(I2CA_BASE);
            I2C_clearStatus(I2CA_BASE, I2C_STS_NO_ACK);
        }
        else if(currentMsgPtr->msgStatus == MSG_STATUS_SEND_NOSTOP_BUSY)
        {
            currentMsgPtr->msgStatus = MSG_STATUS_RESTART;
        }
    }
    else
    {
        //
        // Generate some error from invalid interrupt source
        //
        asm("   ESTOP0");
    }

    //
    // Issue ACK to enable future group 8 interrupts
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);
}



void pomiary(void){

    mes.IAC1=mes.skalaIAC1*(int)(ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0) - mes.Offset_IAC1);
    mes.IAC2=mes.skalaIAC2*(int)(ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1) - mes.Offset_IAC2);
    mes.IAC3=mes.skalaIAC3*(int)(ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER2) - mes.Offset_IAC3);

    mes.UAC1=mes.skalaUAC1*(int)(ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER3) - mes.Offset_UAC1);
    mes.UAC2=mes.skalaUAC2*(int)(ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER2) - mes.Offset_UAC2);
    mes.UAC3=mes.skalaUAC3*(int)(ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1) - mes.Offset_UAC3);

    mes.IDC2=mes.skalaIDC2*(int)(ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER3) - mes.Offset_IDC2);
    mes.IDC1=mes.skalaIDC1*(int)(ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER4) - mes.Offset_IDC1);

   // mes.UDC2=mes.skalaUDC2*(int)(ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER5) - mes.Offset_UDC2);
    mes.UDC1=mes.skalaUDC1*(int)(ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0) - mes.Offset_UDC1);

    mes.UDC2p=mes.skalaUDC2*(int)(ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER0) - mes.Offset_UDC2);

};

void zabezpieczenia(void){
    GPIO7 = GI7; //Wy³¹cznik awaryjny
    if(RESET_FLT==1 ){
        ResetFault=1;
        RESET_FLT=0;

    }
    if(Fault==0){

        if(mes.IAC1>zab.Iacmax || mes.IAC1<zab.Iacmin){
            Fault=1;
            FaultCode=1;
        }
        if(mes.IAC2>zab.Iacmax || mes.IAC2<zab.Iacmin){
            Fault=1;
            FaultCode=2;
        }
        if(mes.IAC3>zab.Iacmax || mes.IAC3<zab.Iacmin){
            Fault=1;
            FaultCode=2;
        }
        if(mes.IDC1>zab.Idcmax || mes.IDC1<zab.Idcmin){
            Fault=1;
            FaultCode=3;
        }
        if(mes.IDC2>zab.Idcmax || mes.IDC2<zab.Idcmin){
            Fault=1;
            FaultCode=4;
        }

        if(mes.UAC1>zab.Uacmax || mes.UAC1<zab.Uacmin){
            Fault=1;
            FaultCode=5;
        }
        if(mes.UAC2>zab.Uacmax || mes.UAC2<zab.Uacmin){
            Fault=1;
            FaultCode=6;
        }
        if(mes.UAC3>zab.Uacmax || mes.UAC3<zab.Uacmin){
            Fault=1;
            FaultCode=7;
        }
        if(mes.UDC1>zab.Udcmax || mes.UDC1<zab.Udcmin){
            Fault=1;
            FaultCode=8;
        }
        if(mes.UDC2>zab.Udcmax || mes.UDC2<zab.Udcmin){
            Fault=1;
            FaultCode=9;
        }
        if(FAULT1==0){
            Fault=1;
            FaultCode=10;

        }
        if(FAULT2==0){
            Fault=1;
            FaultCode=11;

        }
        if(FAULT3==0){
            Fault=1;
            FaultCode=12;

        }
        if(GPIO7==0){ //Awaryjne wy³¹czenie - przycisk
            Fault=1;
            FaultCode=13;
        }
        if(CONNECTION_FAULT==1){
            Fault=1;
            FaultCode=17;
        }
    }
    if(Fault==1 && StartPWM==0 && ResetFault==1){
        ENABLE1_OFF;
        Fault=0;
        FaultCode=0;
        ResetFault=0;
        CONNECTION_FAULT = 0;
        EALLOW;
        EPwm1Regs.TZCLR.bit.OST = 1;
        EPwm2Regs.TZCLR.bit.OST = 1;
        EPwm3Regs.TZCLR.bit.OST = 1;
        EPwm4Regs.TZCLR.bit.OST = 1;
        EPwm5Regs.TZCLR.bit.OST = 1;
        EPwm6Regs.TZCLR.bit.OST = 1;
        EPwm7Regs.TZCLR.bit.OST = 1;
        EPwm8Regs.TZCLR.bit.OST = 1;
        EPwm9Regs.TZCLR.bit.OST = 1;
        EDIS;

    }
};

void pomiaryinit(void){

    mes.Offset_IAC1=35123;
    mes.Offset_IAC2=35310;
    mes.Offset_IAC3=35160;

    mes.Offset_IDC1=32551;
    mes.Offset_IDC2=32235;

    mes.Offset_UAC1=31583;
    mes.Offset_UAC2=31610;
    mes.Offset_UAC3=31475;

    mes.Offset_UDC1=31608;
    mes.Offset_UDC2=31655;

    mes.skalaIAC1=0.0021656943;//0.0020834;
    mes.skalaIAC2=0.0021412;
    mes.skalaIAC3=0.002179411416;//0.00210084;

    mes.skalaIDC1=0.00095;
    mes.skalaIDC2=0.00095;

    mes.skalaUAC1=0.01998209;//0.0194;
    mes.skalaUAC2=0.020040384;//0.0195;
    mes.skalaUAC3=0.020117988;//0.0198;

    mes.skalaUDC1=0.024017691;//0.024187;
    mes.skalaUDC2=0.0240759192;//0.022936;

};

void zabezpieczeniainit(){
    zab.Iacmax=38000.0;
    zab.Iacmin=-38000.0;
    zab.Idcmax=38000.0;
    zab.Idcmin=-38000.0;
    zab.Uacmax=45000.0;
    zab.Uacmin=-45000.0;
    zab.Udcmax=90000;
    zab.Udcmin=-55550.0;

};



//Funkcja inicjalizuj¹ca wspolczynniki do SOGI -PLL
void sogi_init(void){
    si.a0 = 4 + 2 * si.k_sogi * omega * Ts + omega * omega * Ts * Ts;
    si.a1 = 2 * omega * omega * Ts * Ts - 8;
    si.a2 = 4 - 2 * si.k_sogi * omega * Ts + omega * omega * Ts * Ts;
    si.b0 = 2 * si.k_sogi * omega * Ts;
    si.b1 = si.k_sogi * omega * omega * Ts * Ts;

    si.b0a0 = si.b0/si.a0;
    si.a1a0 = si.a1/si.a0;
    si.a2a0 = si.a2/si.a0;
    si.b1a0 = si.b1/si.a0;
}

// Funkcja wyzwalaj¹ca SOGI -PLL
void sogi_exec(sogi_stuct *ss_st){
    ss_st->v_out = (si.b0a0)*ss_st->v_in -(si.b0a0) * ss_st->v_in2 -(si.a1a0)*ss_st->v_out1-(si.a2a0)*ss_st->v_out2;
    ss_st->q_out = (si.b1a0)*ss_st->v_in +2.0*(si.b1a0)*ss_st->v_in1 + (si.b1a0)*ss_st->v_in2-(si.a1a0)*ss_st->q_out1-(si.a2a0)*ss_st->q_out2;

    ss_st->v_out2 = ss_st->v_out1;
    ss_st->v_out1 = ss_st->v_out;

    ss_st->v_in2 = ss_st->v_in1;
    ss_st->v_in1 = ss_st->v_in;

    ss_st->q_out2 = ss_st->q_out1;
    ss_st->q_out1 = ss_st->q_out;
}


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

void RC_init(RC_struct *RC_st,float x,float wcut,float Ki,float Ts1){ //Inicjalizacja regulatora propoecjonalno rezonansowego - struktura, harmoniczna, filtr, okres probkowania
    float wo;
    wo = 2 * PI * 50 * x;

    RC_st->n0 = (4.0*Ki*Ts1*wcut)/(4.0+4.0*Ts1*wcut+wo*wo*Ts1*Ts1);
    RC_st->n2 = (-4.0*Ki*Ts1*wcut)/(4.0+4.0*Ts1*wcut+wo*wo*Ts1*Ts1);
    RC_st->d1 = (2.0*wo*wo*Ts1*Ts1-8.0)/(4.0+4.0*Ts1*wcut+wo*wo*Ts1*Ts1);
    RC_st->d2 = (4.0-4.0*Ts1*wcut+wo*wo*Ts1*Ts1)/(4.0+4.0*Ts1*wcut+wo*wo*Ts1*Ts1);
}

void RC_exec(RC_struct *RC_st,float in,float res_k){
    RC_st->u_res01 = in;
    RC_st->n0=RC_st->n0*res_k;
    RC_st->n2=RC_st->n2*res_k;
    RC_st->y_res01 = RC_st->n0 * (RC_st->u_res01) + RC_st->n2 * (RC_st->u_res21) - RC_st->d1 * (RC_st->y_res11) - RC_st->d2 * (RC_st->y_res21);

    RC_st->y_res21 = RC_st->y_res11;
    RC_st->y_res11 = RC_st->y_res01;

    RC_st->u_res21 = RC_st->u_res11;
    RC_st->u_res11 = RC_st->u_res01;
}

//
// End of File
//
