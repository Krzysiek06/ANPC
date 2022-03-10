/*
 * EPWM.c
 *
 *  Created on: 26 lip 2021
 *      Author: Rafal Miskiewicz
 */
#include <EPWM_PROJ.h>
#include "f28x_project.h"
#include "driverlib.h"
#include "device.h"

void initEPWM(void){
    initEPWM1();
    initEPWM2();
    initEPWM3();
    initEPWM4();
    initEPWM5();
    initEPWM6();
    initEPWM7();
    initEPWM8();
    initEPWM9();
    initEPWM10();
    initEPWM11();
    initEPWM12();
    initEPWM13();
    initEPWM14();

}


void initEPWM1(void){

    //
    // Disable SOCA
    //
    EPWM_disableADCTrigger(EPWM1_BASE, EPWM_SOC_A);

    //
    // Configure the SOC to occur on the first up-count event
    //
    //EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_U_CMPA);
    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);
   // EPWM_SOC_TBCTR_ZERO
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1);




    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM1_BASE, EPWM1_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM1_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM1_BASE);

   // EPwm2Regs.TBCTL.bit.SWFSYNC= TB_CTR_ZERO;
    //
    // Set ePWM clock pre-scaler
    //
    EPWM_setClockPrescaler(EPWM1_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);
    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM1_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);


   EPWM_setCounterCompareShadowLoadMode(EPWM1_BASE,
                                         EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    //
    // Set Compare values
    //
   // EPWM_setCounterCompareValue(EPWM1_BASE,
      //                          EPWM_COUNTER_COMPARE_A,
      //                          6000U);


    //
    // Set-up compare
    //
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, EPWM1_TIMER_TBPRD);
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_B, EPWM1_TIMER_TBPRD);



    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);


    EPWM_setActionQualifierAction(EPWM1_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

/*
    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);



    //

    //
      // Use EPWMA as the input for both RED and FED
      //
      EPWM_setRisingEdgeDeadBandDelayInput(EPWM1_BASE, EPWM_DB_INPUT_EPWMA);
      EPWM_setFallingEdgeDeadBandDelayInput(EPWM1_BASE, EPWM_DB_INPUT_EPWMA);

    //
    // Disable RED
    //
    EPWM_setDeadBandDelayMode(EPWM1_BASE, EPWM_DB_RED, true);

    //
    // Disable FED
    //
    EPWM_setDeadBandDelayMode(EPWM1_BASE, EPWM_DB_FED, true);

    // Do not invert the delayed outputs (AH)
    //
    EPWM_setDeadBandDelayPolarity(EPWM1_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM1_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);

    //
    // Set the RED and FED values
    //
    EPWM_setFallingEdgeDelayCount(EPWM1_BASE, DEAD_TIME);
    EPWM_setRisingEdgeDelayCount(EPWM1_BASE, DEAD_TIME);
    //
    // Switch Output A with Output B
    //
    EPWM_setDeadBandOutputSwapMode(EPWM1_BASE, EPWM_DB_OUTPUT_A, true);
    EPWM_setDeadBandOutputSwapMode(EPWM1_BASE, EPWM_DB_OUTPUT_B, true);

*/
    //
    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 3rd event
    //
    EPWM_setInterruptSource(EPWM1_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM1_BASE);
    EPWM_setInterruptEventCount(EPWM1_BASE, 2U);

    EALLOW;
    EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // Force high-side low on trip
    EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // Force high-side low on trip
    EDIS;


}


void initEPWM2(void){



    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM2_BASE, EPWM2_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM2_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM2_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM2_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM2_BASE);

   // EPwm2Regs.TBCTL.bit.SWFSYNC= TB_SYNC_IN;
    //
    // Set ePWM clock pre-scaler
    //
    EPWM_setClockPrescaler(EPWM2_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);
    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM2_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareShadowLoadMode(EPWM2_BASE,
                                         EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    //
    // Set Compare values
    //
   // EPWM_setCounterCompareValue(EPWM1_BASE,
      //                          EPWM_COUNTER_COMPARE_A,
      //                          6000U);


    //
    // Set-up compare
    //
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, EPWM2_TIMER_TBPRD);
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_B, EPWM2_TIMER_TBPRD);



    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setActionQualifierAction(EPWM2_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
     //
/*
     //
       // Use EPWMA as the input for both RED and FED
       //
       EPWM_setRisingEdgeDeadBandDelayInput(EPWM2_BASE, EPWM_DB_INPUT_EPWMA);
       EPWM_setFallingEdgeDeadBandDelayInput(EPWM2_BASE, EPWM_DB_INPUT_EPWMA);

     //
     // Disable RED
     //
     EPWM_setDeadBandDelayMode(EPWM2_BASE, EPWM_DB_RED, true);

     //
     // Disable FED
     //
     EPWM_setDeadBandDelayMode(EPWM2_BASE, EPWM_DB_FED, true);

     // Do not invert the delayed outputs (AH)
     //
     EPWM_setDeadBandDelayPolarity(EPWM2_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
     EPWM_setDeadBandDelayPolarity(EPWM2_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);

     //
     // Set the RED and FED values
     //
     EPWM_setFallingEdgeDelayCount(EPWM2_BASE, DEAD_TIME);
     EPWM_setRisingEdgeDelayCount(EPWM2_BASE, DEAD_TIME);
     //
     // Switch Output A with Output B
     //
     EPWM_setDeadBandOutputSwapMode(EPWM2_BASE, EPWM_DB_OUTPUT_A, true);
     EPWM_setDeadBandOutputSwapMode(EPWM2_BASE, EPWM_DB_OUTPUT_B, true);
*/
     EALLOW;
     EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // Force high-side low on trip
     EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // Force high-side low on trip
     EDIS;


}

void initEPWM3(void){



    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM3_BASE, EPWM3_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM3_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM3_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM3_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM3_BASE);

    //
    // Set ePWM clock pre-scaler
    //
    EPWM_setClockPrescaler(EPWM3_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);
    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM3_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareShadowLoadMode(EPWM3_BASE,
                                         EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    //
    // Set Compare values
    //
   // EPWM_setCounterCompareValue(EPWM1_BASE,
      //                          EPWM_COUNTER_COMPARE_A,
      //                          6000U);


    //
    // Set-up compare
    //
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, EPWM3_TIMER_TBPRD);
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_B, EPWM3_TIMER_TBPRD);



    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM3_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM3_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM3_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);


    EPWM_setActionQualifierAction(EPWM3_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM3_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM3_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM3_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

     //
/*
     //
       // Use EPWMA as the input for both RED and FED
       //
       EPWM_setRisingEdgeDeadBandDelayInput(EPWM3_BASE, EPWM_DB_INPUT_EPWMA);
       EPWM_setFallingEdgeDeadBandDelayInput(EPWM3_BASE, EPWM_DB_INPUT_EPWMA);

     //
     // Disable RED
     //
     EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_RED, true);

     //
     // Disable FED
     //
     EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_FED, true);

     // Do not invert the delayed outputs (AH)
     //
     EPWM_setDeadBandDelayPolarity(EPWM3_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
     EPWM_setDeadBandDelayPolarity(EPWM3_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);

     //
     // Set the RED and FED values
     //
     EPWM_setFallingEdgeDelayCount(EPWM3_BASE, DEAD_TIME);
     EPWM_setRisingEdgeDelayCount(EPWM3_BASE, DEAD_TIME);
     //
     // Switch Output A with Output B
     //
     EPWM_setDeadBandOutputSwapMode(EPWM3_BASE, EPWM_DB_OUTPUT_A, true);
     EPWM_setDeadBandOutputSwapMode(EPWM3_BASE, EPWM_DB_OUTPUT_B, true);
*/

     EALLOW;
     EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // Force high-side low on trip
     EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // Force high-side low on trip
     EDIS;

}

void initEPWM4(void){



    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM4_BASE, EPWM4_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM4_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM4_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM4_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM4_BASE);

    //
    // Set ePWM clock pre-scaler
    //
    EPWM_setClockPrescaler(EPWM4_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);
    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM4_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    EPWM_setCounterCompareShadowLoadMode(EPWM4_BASE,
                                         EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    //
    // Set Compare values
    //
   // EPWM_setCounterCompareValue(EPWM1_BASE,
      //                          EPWM_COUNTER_COMPARE_A,
      //                          6000U);


    //
    // Set-up compare
    //
    EPWM_setCounterCompareValue(EPWM4_BASE, EPWM_COUNTER_COMPARE_A, EPWM4_TIMER_TBPRD);
    EPWM_setCounterCompareValue(EPWM4_BASE, EPWM_COUNTER_COMPARE_B, EPWM4_TIMER_TBPRD);



    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setActionQualifierAction(EPWM4_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM4_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

     //
/*
     //
       // Use EPWMA as the input for both RED and FED
       //
       EPWM_setRisingEdgeDeadBandDelayInput(EPWM4_BASE, EPWM_DB_INPUT_EPWMA);
       EPWM_setFallingEdgeDeadBandDelayInput(EPWM4_BASE, EPWM_DB_INPUT_EPWMA);

     //
     // Disable RED
     //
     EPWM_setDeadBandDelayMode(EPWM4_BASE, EPWM_DB_RED, true);

     //
     // Disable FED
     //
     EPWM_setDeadBandDelayMode(EPWM4_BASE, EPWM_DB_FED, true);

     // Do not invert the delayed outputs (AH)
     //
     EPWM_setDeadBandDelayPolarity(EPWM4_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
     EPWM_setDeadBandDelayPolarity(EPWM4_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);

     //
     // Set the RED and FED values
     //
     EPWM_setFallingEdgeDelayCount(EPWM4_BASE, DEAD_TIME);
     EPWM_setRisingEdgeDelayCount(EPWM4_BASE, DEAD_TIME);
     //
     // Switch Output A with Output B
     //
     EPWM_setDeadBandOutputSwapMode(EPWM4_BASE, EPWM_DB_OUTPUT_A, true);
     EPWM_setDeadBandOutputSwapMode(EPWM4_BASE, EPWM_DB_OUTPUT_B, true);

*/
     EALLOW;
     EPwm4Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // Force high-side low on trip
     EPwm4Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // Force high-side low on trip
     EDIS;


     //
     // Action on TZ1
     //
     //EPWM_setTripZoneAction(EPWM1_BASE,
     //                       EPWM_TZ_ACTION_EVENT_TZA,
     //                       EPWM_TZ_ACTION_HIGH);
}

void initEPWM5(void){



    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM5_BASE, EPWM5_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM5_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM5_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM5_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM5_BASE);

    //
    // Set ePWM clock pre-scaler
    //
    EPWM_setClockPrescaler(EPWM5_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);
    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM5_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    EPWM_setCounterCompareShadowLoadMode(EPWM5_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    //
    // Set Compare values
    //
   // EPWM_setCounterCompareValue(EPWM1_BASE,
      //                          EPWM_COUNTER_COMPARE_A,
      //                          6000U);


    //
    // Set-up compare
    //
    EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_A, EPWM5_TIMER_TBPRD);
    EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_B, EPWM5_TIMER_TBPRD);


    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM5_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM5_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM5_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM5_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setActionQualifierAction(EPWM5_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM5_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM5_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM5_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

/*
     //

     //
       // Use EPWMA as the input for both RED and FED
       //
       EPWM_setRisingEdgeDeadBandDelayInput(EPWM5_BASE, EPWM_DB_INPUT_EPWMA);
       EPWM_setFallingEdgeDeadBandDelayInput(EPWM5_BASE, EPWM_DB_INPUT_EPWMA);

     //
     // Disable RED
     //
     EPWM_setDeadBandDelayMode(EPWM5_BASE, EPWM_DB_RED, true);

     //
     // Disable FED
     //
     EPWM_setDeadBandDelayMode(EPWM5_BASE, EPWM_DB_FED, true);

     // Do not invert the delayed outputs (AH)
     //
     EPWM_setDeadBandDelayPolarity(EPWM5_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
     EPWM_setDeadBandDelayPolarity(EPWM5_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);

     //
     // Set the RED and FED values
     //
     EPWM_setFallingEdgeDelayCount(EPWM5_BASE, DEAD_TIME);
     EPWM_setRisingEdgeDelayCount(EPWM5_BASE, DEAD_TIME);
     //
     // Switch Output A with Output B
     //
     EPWM_setDeadBandOutputSwapMode(EPWM5_BASE, EPWM_DB_OUTPUT_A, true);
     EPWM_setDeadBandOutputSwapMode(EPWM5_BASE, EPWM_DB_OUTPUT_B, true);

*/
     EALLOW;
     EPwm5Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // Force high-side low on trip
     EPwm5Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // Force high-side low on trip
     EDIS;


}


void initEPWM6(void){



    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM6_BASE, EPWM5_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM6_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM6_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM6_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM6_BASE);

    //
    // Set ePWM clock pre-scaler
    //
    EPWM_setClockPrescaler(EPWM6_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);
    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM6_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareShadowLoadMode(EPWM6_BASE,
                                         EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    //
    // Set Compare values
    //
   // EPWM_setCounterCompareValue(EPWM1_BASE,
      //                          EPWM_COUNTER_COMPARE_A,
      //                          6000U);


    //
    // Set-up compare
    //
    EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_A, EPWM6_TIMER_TBPRD);
    EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_B, EPWM6_TIMER_TBPRD);



    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM6_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM6_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM6_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM6_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setActionQualifierAction(EPWM6_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM6_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM6_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM6_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

/*
     //

     //
       // Use EPWMA as the input for both RED and FED
       //
       EPWM_setRisingEdgeDeadBandDelayInput(EPWM6_BASE, EPWM_DB_INPUT_EPWMA);
       EPWM_setFallingEdgeDeadBandDelayInput(EPWM6_BASE, EPWM_DB_INPUT_EPWMA);

     //
     // Disable RED
     //
     EPWM_setDeadBandDelayMode(EPWM6_BASE, EPWM_DB_RED, true);

     //
     // Disable FED
     //
     EPWM_setDeadBandDelayMode(EPWM6_BASE, EPWM_DB_FED, true);

     // Do not invert the delayed outputs (AH)
     //
     EPWM_setDeadBandDelayPolarity(EPWM6_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
     EPWM_setDeadBandDelayPolarity(EPWM6_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);

     //
     // Set the RED and FED values
     //
     EPWM_setFallingEdgeDelayCount(EPWM6_BASE, DEAD_TIME);
     EPWM_setRisingEdgeDelayCount(EPWM6_BASE, DEAD_TIME);
     //
     // Switch Output A with Output B
     //
     EPWM_setDeadBandOutputSwapMode(EPWM6_BASE, EPWM_DB_OUTPUT_A, true);
     EPWM_setDeadBandOutputSwapMode(EPWM6_BASE, EPWM_DB_OUTPUT_B, true);
*/
     EALLOW;
     EPwm6Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // Force high-side low on trip
     EPwm6Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // Force high-side low on trip
     EDIS;

}

void initEPWM7(void){



    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM7_BASE, EPWM5_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM7_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM7_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM7_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM7_BASE);

    //
    // Set ePWM clock pre-scaler
    //
    EPWM_setClockPrescaler(EPWM7_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);
    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM7_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    EPWM_setCounterCompareShadowLoadMode(EPWM7_BASE,
                                         EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    //
    // Set Compare values
    //
   // EPWM_setCounterCompareValue(EPWM1_BASE,
      //                          EPWM_COUNTER_COMPARE_A,
      //                          6000U);


    //
    // Set-up compare
    //
    EPWM_setCounterCompareValue(EPWM7_BASE, EPWM_COUNTER_COMPARE_A, EPWM7_TIMER_TBPRD);
    EPWM_setCounterCompareValue(EPWM7_BASE, EPWM_COUNTER_COMPARE_B, EPWM7_TIMER_TBPRD);



    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM7_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM7_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM7_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM7_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setActionQualifierAction(EPWM7_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM7_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM7_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM7_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

/*
     //

     //
       // Use EPWMA as the input for both RED and FED
       //
       EPWM_setRisingEdgeDeadBandDelayInput(EPWM7_BASE, EPWM_DB_INPUT_EPWMA);
       EPWM_setFallingEdgeDeadBandDelayInput(EPWM7_BASE, EPWM_DB_INPUT_EPWMA);

     //
     // Disable RED
     //
     EPWM_setDeadBandDelayMode(EPWM7_BASE, EPWM_DB_RED, true);

     //
     // Disable FED
     //
     EPWM_setDeadBandDelayMode(EPWM7_BASE, EPWM_DB_FED, true);

     // Do not invert the delayed outputs (AH)
     //
     EPWM_setDeadBandDelayPolarity(EPWM7_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
     EPWM_setDeadBandDelayPolarity(EPWM7_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);

     //
     // Set the RED and FED values
     //
     EPWM_setFallingEdgeDelayCount(EPWM7_BASE, DEAD_TIME);
     EPWM_setRisingEdgeDelayCount(EPWM7_BASE, DEAD_TIME);
     //
     // Switch Output A with Output B
     //
     EPWM_setDeadBandOutputSwapMode(EPWM7_BASE, EPWM_DB_OUTPUT_A, true);
     EPWM_setDeadBandOutputSwapMode(EPWM7_BASE, EPWM_DB_OUTPUT_B, true);
*/
     EALLOW;
     EPwm7Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // Force high-side low on trip
     EPwm7Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // Force high-side low on trip
     EDIS;

}

void initEPWM8(void){



    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM8_BASE, EPWM8_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM8_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM8_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM8_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM8_BASE);

    //
    // Set ePWM clock pre-scaler
    //
    EPWM_setClockPrescaler(EPWM8_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);
    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM8_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareShadowLoadMode(EPWM8_BASE,
                                         EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    //
    // Set Compare values
    //
   // EPWM_setCounterCompareValue(EPWM1_BASE,
      //                          EPWM_COUNTER_COMPARE_A,
      //                          6000U);


    //
    // Set-up compare
    //
    EPWM_setCounterCompareValue(EPWM8_BASE, EPWM_COUNTER_COMPARE_A, EPWM8_TIMER_TBPRD);
    EPWM_setCounterCompareValue(EPWM8_BASE, EPWM_COUNTER_COMPARE_B, EPWM8_TIMER_TBPRD);


    //
     // Set actions
     //
     EPWM_setActionQualifierAction(EPWM8_BASE,
                                       EPWM_AQ_OUTPUT_A,
                                       EPWM_AQ_OUTPUT_HIGH,
                                       EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
     EPWM_setActionQualifierAction(EPWM8_BASE,
                                       EPWM_AQ_OUTPUT_A,
                                       EPWM_AQ_OUTPUT_LOW,
                                       EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
     EPWM_setActionQualifierAction(EPWM8_BASE,
                                       EPWM_AQ_OUTPUT_A,
                                       EPWM_AQ_OUTPUT_NO_CHANGE,
                                       EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
     EPWM_setActionQualifierAction(EPWM8_BASE,
                                       EPWM_AQ_OUTPUT_A,
                                       EPWM_AQ_OUTPUT_HIGH,
                                       EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

     EPWM_setActionQualifierAction(EPWM8_BASE,
                                       EPWM_AQ_OUTPUT_B,
                                       EPWM_AQ_OUTPUT_LOW,
                                       EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
     EPWM_setActionQualifierAction(EPWM8_BASE,
                                       EPWM_AQ_OUTPUT_B,
                                       EPWM_AQ_OUTPUT_HIGH,
                                       EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
     EPWM_setActionQualifierAction(EPWM8_BASE,
                                       EPWM_AQ_OUTPUT_B,
                                       EPWM_AQ_OUTPUT_NO_CHANGE,
                                       EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
     EPWM_setActionQualifierAction(EPWM8_BASE,
                                       EPWM_AQ_OUTPUT_B,
                                       EPWM_AQ_OUTPUT_LOW,
                                       EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);


     //
/*
     //
       // Use EPWMA as the input for both RED and FED
       //
       EPWM_setRisingEdgeDeadBandDelayInput(EPWM8_BASE, EPWM_DB_INPUT_EPWMA);
       EPWM_setFallingEdgeDeadBandDelayInput(EPWM8_BASE, EPWM_DB_INPUT_EPWMA);

     //
     // Disable RED
     //
     EPWM_setDeadBandDelayMode(EPWM8_BASE, EPWM_DB_RED, true);

     //
     // Disable FED
     //
     EPWM_setDeadBandDelayMode(EPWM8_BASE, EPWM_DB_FED, true);

     // Do not invert the delayed outputs (AH)
     //
     EPWM_setDeadBandDelayPolarity(EPWM8_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
     EPWM_setDeadBandDelayPolarity(EPWM8_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);

     //
     // Set the RED and FED values
     //
     EPWM_setFallingEdgeDelayCount(EPWM8_BASE, DEAD_TIME);
     EPWM_setRisingEdgeDelayCount(EPWM8_BASE, DEAD_TIME);
     //
     // Switch Output A with Output B
     //
     EPWM_setDeadBandOutputSwapMode(EPWM8_BASE, EPWM_DB_OUTPUT_A, true);
     EPWM_setDeadBandOutputSwapMode(EPWM8_BASE, EPWM_DB_OUTPUT_B, true);
*/
     EALLOW;
     EPwm8Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // Force high-side low on trip
     EPwm8Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // Force high-side low on trip
     EDIS;


}

void initEPWM9(void){



    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM9_BASE, EPWM9_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM9_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM9_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM9_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM8_BASE);

    //
    // Set ePWM clock pre-scaler
    //
    EPWM_setClockPrescaler(EPWM9_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);
    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM9_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareShadowLoadMode(EPWM9_BASE,
                                         EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    //
    // Set Compare values
    //
   // EPWM_setCounterCompareValue(EPWM1_BASE,
      //                          EPWM_COUNTER_COMPARE_A,
      //                          6000U);


    //
    // Set-up compare
    //
    EPWM_setCounterCompareValue(EPWM9_BASE, EPWM_COUNTER_COMPARE_A, EPWM9_TIMER_TBPRD);
    EPWM_setCounterCompareValue(EPWM9_BASE, EPWM_COUNTER_COMPARE_B, EPWM9_TIMER_TBPRD);



    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM9_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM9_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM9_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM9_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setActionQualifierAction(EPWM9_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM9_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM9_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM9_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);


     //

     //
       // Use EPWMA as the input for both RED and FED
       //
       EPWM_setRisingEdgeDeadBandDelayInput(EPWM9_BASE, EPWM_DB_INPUT_EPWMA);
       EPWM_setFallingEdgeDeadBandDelayInput(EPWM9_BASE, EPWM_DB_INPUT_EPWMA);

     //
     // Disable RED
     //
     EPWM_setDeadBandDelayMode(EPWM9_BASE, EPWM_DB_RED, true);

     //
     // Disable FED
     //
     EPWM_setDeadBandDelayMode(EPWM9_BASE, EPWM_DB_FED, true);

     // Do not invert the delayed outputs (AH)
     //
     EPWM_setDeadBandDelayPolarity(EPWM9_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
     EPWM_setDeadBandDelayPolarity(EPWM9_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);

     //
     // Set the RED and FED values
     //
     EPWM_setFallingEdgeDelayCount(EPWM9_BASE, DEAD_TIME);
     EPWM_setRisingEdgeDelayCount(EPWM9_BASE, DEAD_TIME);
     //
     // Switch Output A with Output B
     //
     EPWM_setDeadBandOutputSwapMode(EPWM9_BASE, EPWM_DB_OUTPUT_A, true);
     EPWM_setDeadBandOutputSwapMode(EPWM9_BASE, EPWM_DB_OUTPUT_B, true);

     EALLOW;
     EPwm9Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // Force high-side low on trip
     EPwm9Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // Force high-side low on trip
     EDIS;

}
void initEPWM10(void){

    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM10_BASE, EPWM10_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM10_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM10_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM10_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM10_BASE);

    //
    // Set ePWM clock pre-scaler
    //
    EPWM_setClockPrescaler(EPWM10_BASE,
                           EPWM_CLOCK_DIVIDER_4,
                           EPWM_HSCLOCK_DIVIDER_4);
    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM10_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set-up compare
    //
    EPWM_setCounterCompareValue(EPWM10_BASE, EPWM_COUNTER_COMPARE_A, EPWM10_TIMER_TBPRD/4);
    EPWM_setCounterCompareValue(EPWM10_BASE, EPWM_COUNTER_COMPARE_B, EPWM10_TIMER_TBPRD/4);


    //
    // Set actions
    //
    //CHANEL A
    EPWM_setActionQualifierAction(EPWM10_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM10_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM10_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM10_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    //CHANEL B
    EPWM_setActionQualifierAction(EPWM10_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM10_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM10_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM10_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);


}

void initEPWM11(void){

    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM11_BASE, EPWM10_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM11_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM11_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM11_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM11_BASE);

    //
    // Set ePWM clock pre-scaler
    //
    EPWM_setClockPrescaler(EPWM11_BASE,
                           EPWM_CLOCK_DIVIDER_4,
                           EPWM_HSCLOCK_DIVIDER_4);
    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM11_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set-up compare
    //
    EPWM_setCounterCompareValue(EPWM11_BASE, EPWM_COUNTER_COMPARE_A, EPWM11_TIMER_TBPRD/4);
    EPWM_setCounterCompareValue(EPWM11_BASE, EPWM_COUNTER_COMPARE_B, EPWM11_TIMER_TBPRD/4);


    //
    // Set actions
    //
    //CHANEL A
    EPWM_setActionQualifierAction(EPWM11_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM11_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM11_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM11_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    //CHANEL B
    EPWM_setActionQualifierAction(EPWM11_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM11_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM11_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM11_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);


}

void initEPWM12(void){

    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM12_BASE, EPWM12_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM12_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM12_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM12_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM12_BASE);

    //
    // Set ePWM clock pre-scaler
    //
    EPWM_setClockPrescaler(EPWM12_BASE,
                           EPWM_CLOCK_DIVIDER_4,
                           EPWM_HSCLOCK_DIVIDER_4);
    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM12_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set-up compare
    //
    EPWM_setCounterCompareValue(EPWM12_BASE, EPWM_COUNTER_COMPARE_A, EPWM12_TIMER_TBPRD/4);
    EPWM_setCounterCompareValue(EPWM12_BASE, EPWM_COUNTER_COMPARE_B, EPWM12_TIMER_TBPRD/4);


    //
    // Set actions
    //
    //CHANEL A
    EPWM_setActionQualifierAction(EPWM12_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM12_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM12_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM12_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    //CHANEL B
    EPWM_setActionQualifierAction(EPWM12_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM12_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM12_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM12_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);


}

void initEPWM13(void){

    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM13_BASE, EPWM13_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM13_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM13_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM13_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM13_BASE);

    //
    // Set ePWM clock pre-scaler
    //
    EPWM_setClockPrescaler(EPWM13_BASE,
                           EPWM_CLOCK_DIVIDER_4,
                           EPWM_HSCLOCK_DIVIDER_4);
    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM13_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set-up compare
    //
    EPWM_setCounterCompareValue(EPWM13_BASE, EPWM_COUNTER_COMPARE_A, EPWM13_TIMER_TBPRD/4);
    EPWM_setCounterCompareValue(EPWM13_BASE, EPWM_COUNTER_COMPARE_B, EPWM13_TIMER_TBPRD/4);


    //
    // Set actions
    //
    //CHANEL A
    EPWM_setActionQualifierAction(EPWM13_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM13_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM13_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM13_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    //CHANEL B
    EPWM_setActionQualifierAction(EPWM13_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM13_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM13_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM13_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);


}

void initEPWM14(void){

    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM14_BASE, EPWM13_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM14_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM14_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM14_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM14_BASE);

    //
    // Set ePWM clock pre-scaler
    //
    EPWM_setClockPrescaler(EPWM14_BASE,
                           EPWM_CLOCK_DIVIDER_4,
                           EPWM_HSCLOCK_DIVIDER_4);
 /*   //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM13_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set-up compare
    //
    EPWM_setCounterCompareValue(EPWM13_BASE, EPWM_COUNTER_COMPARE_A, EPWM13_TIMER_TBPRD/4);
    EPWM_setCounterCompareValue(EPWM13_BASE, EPWM_COUNTER_COMPARE_B, EPWM13_TIMER_TBPRD/4);


    //
    // Set actions
    //
    //CHANEL A
    EPWM_setActionQualifierAction(EPWM13_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM13_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM13_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM13_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    //CHANEL B
    EPWM_setActionQualifierAction(EPWM13_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM13_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM13_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM13_BASE,
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
*/
    //
    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 3rd event
    //
    EPWM_setInterruptSource(EPWM14_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM14_BASE);
    EPWM_setInterruptEventCount(EPWM14_BASE, 1U);
}
