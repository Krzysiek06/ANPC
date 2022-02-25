/*
 * ADC.h
 *
 *  Created on: 26 lip 2021
 *      Author: Rafal Miskiewicz
 */

#ifndef ADC_PROJ_H_
#define ADC_PROJ_H_

#include "driverlib.h"
#include "device.h"

//
// Defines
//
#define EX_ADC_RESOLUTION       16
// 12 for 12-bit conversion resolution, which supports single-ended signaling
// Or 16 for 16-bit conversion resolution, which supports single-ended or
// differential signaling
#define EX_ADC_SIGNALMODE       "SINGLE-ENDED"
//"SINGLE-ENDED" for ADC_MODE_SINGLE_ENDED:
// Sample on single pin (VREFLO is the low reference)
// Or "Differential" for ADC_MODE_DIFFERENTIAL:
// Sample on pair of pins (difference between pins is converted, subject to
// common mode voltage requirements; see the device data manual)
void initADCSOC(void);
void configureADC(uint32_t adcBase);
void initADC(void);

#endif /* ADC_PROJ_H_ */
