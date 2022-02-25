/*
 * DAC70508.h
 *
 *  Created on: 25 sie 2021
 *      Author: Rafal Miskiewicz
 */

#ifndef DAC70508_H_
#define DAC70508_H_

#include "driverlib.h"
#include "device.h"
// Definicje
#define CS_LOW_DAC70508   GPIO_writePin(57, 0);
#define CS_HIGH_DAC70508  GPIO_writePin(57, 1);

void writeSPIDAC70508(uint16_t address, uint16_t * data, uint16_t length);
void readSPIDAC70508(uint16_t address, uint16_t * data, uint16_t length);
#endif /* DAC70508_H_ */
