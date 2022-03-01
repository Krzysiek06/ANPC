/*
 * GPIO_PROJ.h
 *
 *  Created on: 27 lip 2021
 *      Author: Rafal Miskiewicz
 */

#ifndef GPIO_PROJ_H_
#define GPIO_PROJ_H_
/*
#define LED1_ON GPIO_writePin(40, 1);
#define LED1_OFF GPIO_writePin(40, 0);

#define EN_PWM_GROUP1_ON  GPIO_writePin(81, 0);
#define EN_PWM_GROUP1_OFF GPIO_writePin(81, 1);

#define EN_PWM_GROUP2_ON  GPIO_writePin(82, 0);
#define EN_PWM_GROUP2_OFF GPIO_writePin(82, 1);

#define EN_PWM_GROUP3_ON  GPIO_writePin(83, 0);
#define EN_PWM_GROUP3_OFF GPIO_writePin(83, 1);

#define EN_PWM_GROUP4_ON  GPIO_writePin(80, 0);
#define EN_PWM_GROUP4_OFF GPIO_writePin(80, 1);
*/
#define LED1_ON GpioDataRegs.GPBSET.bit.GPIO38=1;
#define LED1_OFF GpioDataRegs.GPBCLEAR.bit.GPIO38=1;

#define LED2_ON GpioDataRegs.GPBSET.bit.GPIO39=1;
#define LED2_OFF GpioDataRegs.GPBCLEAR.bit.GPIO39=1;


#define LED3_ON GpioDataRegs.GPBSET.bit.GPIO40=1;
#define LED3_OFF GpioDataRegs.GPBCLEAR.bit.GPIO40=1;

#define LED4_ON GpioDataRegs.GPBSET.bit.GPIO41=1;
#define LED4_OFF GpioDataRegs.GPBCLEAR.bit.GPIO41=1;

#define EN_PWM_GROUP1_ON  GpioDataRegs.GPCCLEAR.bit.GPIO81=1;
#define EN_PWM_GROUP1_OFF GpioDataRegs.GPCSET.bit.GPIO81=1;

#define EN_PWM_GROUP2_ON  GpioDataRegs.GPCCLEAR.bit.GPIO82=1;
#define EN_PWM_GROUP2_OFF GpioDataRegs.GPCSET.bit.GPIO82=1;

#define EN_PWM_GROUP3_ON  GpioDataRegs.GPCCLEAR.bit.GPIO83=1;
#define EN_PWM_GROUP3_OFF GpioDataRegs.GPCSET.bit.GPIO83=1;

#define EN_PWM_GROUP4_ON  GpioDataRegs.GPCCLEAR.bit.GPIO80=1;
#define EN_PWM_GROUP4_OFF GpioDataRegs.GPCSET.bit.GPIO80=1;

#define ENABLE1_OFF GpioDataRegs.GPCCLEAR.bit.GPIO87=1;
#define ENABLE1_ON GpioDataRegs.GPCSET.bit.GPIO87=1;

#define ENABLE2_OFF GpioDataRegs.GPCCLEAR.bit.GPIO86=1;
#define ENABLE2_ON GpioDataRegs.GPCSET.bit.GPIO86=1;

#define ENABLE3_OFF GpioDataRegs.GPCCLEAR.bit.GPIO85=1;
#define ENABLE3_ON GpioDataRegs.GPCSET.bit.GPIO85=1;


#define REL1_ON  GpioDataRegs.GPCSET.bit.GPIO75=1;
#define REL1_OFF  GpioDataRegs.GPCCLEAR.bit.GPIO75=1;

#define REL2_ON  GpioDataRegs.GPCSET.bit.GPIO74=1;
#define REL2_OFF  GpioDataRegs.GPCCLEAR.bit.GPIO74=1;

#define REL3_ON  GpioDataRegs.GPCSET.bit.GPIO72=1;
#define REL3_OFF  GpioDataRegs.GPCCLEAR.bit.GPIO72=1;

#define REL4_ON  GpioDataRegs.GPCSET.bit.GPIO71=1;
#define REL4_OFF  GpioDataRegs.GPCCLEAR.bit.GPIO71=1;

#define REL5_ON  GpioDataRegs.GPCSET.bit.GPIO70;
#define REL5_OFF  GpioDataRegs.GPCCLEAR.bit.GPIO70;

#define REL6_ON  GpioDataRegs.GPCSET.bit.GPIO69=1;
#define REL6_OFF  GpioDataRegs.GPCCLEAR.bit.GPIO69=1;

#define REL7_ON  GpioDataRegs.GPCSET.bit.GPIO68;
#define REL7_OFF  GpioDataRegs.GPCCLEAR.bit.GPIO68;

#define GI0  GpioDataRegs.GPCDAT.bit.GPIO48;
#define GI2  GpioDataRegs.GPCDAT.bit.GPIO49;
#define GI3  GpioDataRegs.GPCDAT.bit.GPIO50;
#define GI4  GpioDataRegs.GPCDAT.bit.GPIO51;
#define GI5  GpioDataRegs.GPCDAT.bit.GPIO52;
#define GI6  GpioDataRegs.GPCDAT.bit.GPIO53;
#define GI7  GpioDataRegs.GPBDAT.bit.GPIO60;

#define READY1 GpioDataRegs.GPCDAT.bit.GPIO93;
#define READY2 GpioDataRegs.GPCDAT.bit.GPIO91;
#define READY3 GpioDataRegs.GPCDAT.bit.GPIO89;

#define FAULT1 GpioDataRegs.GPCDAT.bit.GPIO92
#define FAULT2 GpioDataRegs.GPCDAT.bit.GPIO90
#define FAULT3 GpioDataRegs.GPCDAT.bit.GPIO88




void GPIOSetup(void);
void InitXBar(void);



#endif /* GPIO_PROJ_H_ */
