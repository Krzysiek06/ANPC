/*
 * GPIO_PROJ.c
 *
 *  Created on: 27 lip 2021
 *      Author: Rafal Miskiewicz
 */

#include <GPIO_PROJ.h>
#include "f28x_project.h"
#include "driverlib.h"
#include "device.h"

void GPIOSetup(void){


    EALLOW;
    //
    // Enable PWM1-12 on GPIO0-GPIO5
    //
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO0
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO1
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO2
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO3
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO4
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO5
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO6
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO7
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO8
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO9
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO10
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO11
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO12
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO13
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO14
    GPIO_setPadConfig(15, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO15
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO16
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO17
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO18
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO19
    GPIO_setPadConfig(20, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO20
    GPIO_setPadConfig(21, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO21
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO22
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO23

    GPIO_setPinConfig(GPIO_0_EPWM1A);                // GPIO0 = PWM1A
    GPIO_setPinConfig(GPIO_1_EPWM1B);                // GPIO1 = PWM1B
    GPIO_setPinConfig(GPIO_2_EPWM2A);                // GPIO2 = PWM2A
    GPIO_setPinConfig(GPIO_3_EPWM2B);                // GPIO3 = PWM2B
    GPIO_setPinConfig(GPIO_4_EPWM3A);                // GPIO4 = PWM3A
    GPIO_setPinConfig(GPIO_5_EPWM3B);                // GPIO5 = PWM3B
    GPIO_setPinConfig(GPIO_6_EPWM4A);                // GPIO6 = PWM4A
    GPIO_setPinConfig(GPIO_7_EPWM4B);                // GPIO7 = PWM4B
    GPIO_setPinConfig(GPIO_8_EPWM5A);                // GPIO8 = PWM5A
    GPIO_setPinConfig(GPIO_9_EPWM5B);                // GPIO9 = PWM5B
    GPIO_setPinConfig(GPIO_10_EPWM6A);               // GPIO10 = PWM6A
    GPIO_setPinConfig(GPIO_11_EPWM6B);               // GPIO11 = PWM6B
    GPIO_setPinConfig(GPIO_12_EPWM7A);               // GPIO12 = PWM7A
    GPIO_setPinConfig(GPIO_13_EPWM7B);               // GPIO13 = PWM7B
    GPIO_setPinConfig(GPIO_14_EPWM8A);               // GPIO14 = PWM8A
    GPIO_setPinConfig(GPIO_15_EPWM8B);               // GPIO15 = PWM8B
    GPIO_setPinConfig(GPIO_16_EPWM9A);               // GPIO16 = PWM9A
    GPIO_setPinConfig(GPIO_17_EPWM9B);               // GPIO17 = PWM9B
    GPIO_setPinConfig(GPIO_18_EPWM10A);              // GPIO18 = PWM10A
    GPIO_setPinConfig(GPIO_19_EPWM10B);              // GPIO19 = PWM10B
    GPIO_setPinConfig(GPIO_20_EPWM11A);              // GPIO20 = PWM11A
    GPIO_setPinConfig(GPIO_21_EPWM11B);              // GPIO21 = PWM11B
    GPIO_setPinConfig(GPIO_22_EPWM12A);              // GPIO22 = PWM12A
    GPIO_setPinConfig(GPIO_23_EPWM12B);              // GPIO23 = PWM12B

    // ENABLE_PWM OUTPUT
    // Enable GPIO outputs on GPIO80 - GPIO83, set it high
    //
    GPIO_setPadConfig(80, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO80
    GPIO_writePin(80, 0);                             // Load output latch
    GPIO_setPinConfig(GPIO_80_GPIO80);                // GPIO80 = GPIO80 ENABLE_PWM_GROUP1
    GPIO_setDirectionMode(80, GPIO_DIR_MODE_OUT);     // GPIO80 = output

    GPIO_setPadConfig(81, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO81
    GPIO_writePin(81, 0);                             // Load output latch
    GPIO_setPinConfig(GPIO_81_GPIO81);                // GPIO81 = GPIO81 ENABLE_PWM_GROUP2
    GPIO_setDirectionMode(81, GPIO_DIR_MODE_OUT);     // GPIO81 = output

    GPIO_setPadConfig(82, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO82
    GPIO_writePin(82, 0);                             // Load output latch
    GPIO_setPinConfig(GPIO_82_GPIO82);                // GPIO82 = GPIO82 ENABLE_PWM_GROUP3
    GPIO_setDirectionMode(82, GPIO_DIR_MODE_OUT);     // GPIO82 = output

    GPIO_setPadConfig(83, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO82
    GPIO_writePin(83, 0);                             // Load output latch
    GPIO_setPinConfig(GPIO_83_GPIO83);                // GPIO83 = GPIO83 ENABLE_PWM_GROUP4
    GPIO_setDirectionMode(83, GPIO_DIR_MODE_OUT);     // GPIO83 = output

    // ENABLE_DRIVER OUTPUT
    // Enable GPIO outputs on GPIO85 - GPIO87, set it high
    //
    GPIO_setPadConfig(85, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO85
    GPIO_writePin(85, 0);                             // Load output latch
    GPIO_setPinConfig(GPIO_85_GPIO85);                // GPIO80 = GPIO80 ENABLE_PWM_GROUP1
    GPIO_setDirectionMode(85, GPIO_DIR_MODE_OUT);     // GPIO80 = output

    GPIO_setPadConfig(86, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO86
    GPIO_writePin(86, 0);                             // Load output latch
    GPIO_setPinConfig(GPIO_86_GPIO86);                // GPIO81 = GPIO81 ENABLE_PWM_GROUP2
    GPIO_setDirectionMode(86, GPIO_DIR_MODE_OUT);     // GPIO81 = output

    GPIO_setPadConfig(87, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO87
    GPIO_writePin(87, 0);                             // Load output latch
    GPIO_setPinConfig(GPIO_87_GPIO87);                // GPIO82 = GPIO82 ENABLE_PWM_GROUP3
    GPIO_setDirectionMode(87, GPIO_DIR_MODE_OUT);     // GPIO82 = output

    // FAULTS and READY INPUTS
    // Enable GPIO outputs on GPIO88 - GPIO93, set it high
    //
    GPIO_setPadConfig(88, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPI89
    GPIO_setPinConfig(GPIO_88_GPIO88);                // GPIO89 = GPIO89 FAULT3
    GPIO_setDirectionMode(88, GPIO_DIR_MODE_IN);      // GPIO89 = input

    GPIO_setPadConfig(89, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPI89
    GPIO_setPinConfig(GPIO_89_GPIO89);                // GPIO89 = GPIO89 READY3
    GPIO_setDirectionMode(89, GPIO_DIR_MODE_IN);      // GPIO89 = input

    GPIO_setPadConfig(90, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPI89
    GPIO_setPinConfig(GPIO_90_GPIO90);                // GPIO90 = GPIO90 FAULT2
    GPIO_setDirectionMode(90, GPIO_DIR_MODE_IN);      // GPIO90 = input

    GPIO_setPadConfig(91, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPI89
    GPIO_setPinConfig(GPIO_91_GPIO91);                // GPIO91 = GPIO91 READY2
    GPIO_setDirectionMode(91, GPIO_DIR_MODE_IN);      // GPIO91 = input


    GPIO_setPadConfig(92, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPI89
    GPIO_setPinConfig(GPIO_92_GPIO92);                // GPIO92 = GPIO92 FAULT1
    GPIO_setDirectionMode(92, GPIO_DIR_MODE_IN);      // GPIO92 = input

    GPIO_setPadConfig(93, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPI89
    GPIO_setPinConfig(GPIO_93_GPIO93);                // GPIO93 = GPIO93 READY1
    GPIO_setDirectionMode(93, GPIO_DIR_MODE_IN);      // GPIO93 = input

    //
    // Enable CAN-A on GPIO62 - GPIO63
    //
    GPIO_setPadConfig(62, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO62
    GPIO_setPinConfig(GPIO_62_CANA_RX);               // GPIO62 = CANRXA

    GPIO_setPadConfig(63, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO63
    GPIO_setQualificationMode(63, GPIO_QUAL_ASYNC);  // asynch input
    GPIO_setPinConfig(GPIO_63_CANA_TX);               // GPIO63 = CANTXA

    //
    // Enable I2C-A on GPIO67 - GPIO43
    //
    GPIO_setPadConfig(42, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO42
    GPIO_setPinConfig(GPIO_42_I2CA_SDA);                 // GPIO3]42 = SDAA
    GPIO_setQualificationMode(42, GPIO_QUAL_ASYNC);  // asynch input

    GPIO_setPadConfig(43, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO43
    GPIO_setQualificationMode(43, GPIO_QUAL_ASYNC);  // asynch input
    GPIO_setPinConfig(GPIO_43_I2CA_SCL);                 // GPIO43 = SCLA


    //
    // Enable SPI-A on GPIO54 - GPIO33
    //
    GPIO_setMasterCore(54, GPIO_CORE_CPU1);
    GPIO_setPadConfig(54, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO54
    GPIO_setPinConfig(GPIO_54_SPIA_SIMO);            // GPIO54 = SIMO
    GPIO_setQualificationMode(54, GPIO_QUAL_ASYNC);  // asynch output

    GPIO_setPadConfig(55, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO55
    GPIO_setQualificationMode(55, GPIO_QUAL_ASYNC);  // asynch input
    GPIO_setPinConfig(GPIO_55_SPIA_SOMI);            // GPIO55 = SOMI

    GPIO_setMasterCore(56, GPIO_CORE_CPU1);
    GPIO_setPadConfig(56, GPIO_PIN_TYPE_PULLUP);     // Enable pullup on GPIO55
    GPIO_setPinConfig(GPIO_56_SPIA_CLK);             // GPIO54 = SCLK
    GPIO_setQualificationMode(56, GPIO_QUAL_ASYNC);  // asynch output

    GPIO_setPadConfig(57, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO57
    GPIO_writePin(57,1);                             // Load output latch
    GPIO_setPinConfig(GPIO_57_GPIO57);                // GPIO57 = GPIO57 CS1
    GPIO_setDirectionMode(57, GPIO_DIR_MODE_OUT);     // GPIO57 = output

    GPIO_setPadConfig(58, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO58
    GPIO_writePin(58, 1);                             // Load output latch
    GPIO_setPinConfig(GPIO_58_GPIO58);                // GPIO58 = GPIO58 CS2
    GPIO_setDirectionMode(58, GPIO_DIR_MODE_OUT);     // GPIO58 = output

    GPIO_setPadConfig(59, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO59
    GPIO_writePin(59, 1);                             // Load output latch
    GPIO_setPinConfig(GPIO_59_GPIO59);                // GPIO59 = GPIO59 CS3
    GPIO_setDirectionMode(59, GPIO_DIR_MODE_OUT);     // GPIO59 = output


    // RELAY OUTPUT
    // Enable GPIO outputs on GPIO68 - GPIO75, set it high
    //
    GPIO_setPadConfig(68, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO68
    GPIO_writePin(68, 0);                             // Load output latch
    GPIO_setPinConfig(GPIO_68_GPIO68);                // GPIO68 = GPIO68
    GPIO_setDirectionMode(68, GPIO_DIR_MODE_OUT);     // GPIO68 = output

    GPIO_setPadConfig(69, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO69
    GPIO_writePin(69, 0);                             // Load output latch
    GPIO_setPinConfig(GPIO_69_GPIO69);                // GPIO69 = GPIO69
    GPIO_setDirectionMode(69, GPIO_DIR_MODE_OUT);     // GPIO69 = output

    GPIO_setPadConfig(70, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO70
    GPIO_writePin(70, 0);                             // Load output latch
    GPIO_setPinConfig(GPIO_70_GPIO70);                // GPIO70 = GPIO70
    GPIO_setDirectionMode(70, GPIO_DIR_MODE_OUT);     // GPIO70 = output

    GPIO_setPadConfig(71, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO71
    GPIO_writePin(71, 0);                             // Load output latch
    GPIO_setPinConfig(GPIO_71_GPIO71);                // GPIO71 = GPIO71
    GPIO_setDirectionMode(71, GPIO_DIR_MODE_OUT);     // GPIO71 = output

    GPIO_setPadConfig(72, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO72
    GPIO_writePin(72, 0);                             // Load output latch
    GPIO_setPinConfig(GPIO_72_GPIO72);                // GPIO72 = GPIO72
    GPIO_setDirectionMode(72, GPIO_DIR_MODE_OUT);     // GPIO72 = output

    GPIO_setPadConfig(73, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO73
    GPIO_writePin(73, 0);                             // Load output latch
    GPIO_setPinConfig(GPIO_73_GPIO73);                // GPIO73 = GPIO73
    GPIO_setDirectionMode(73, GPIO_DIR_MODE_OUT);     // GPIO73 = output

    GPIO_setPadConfig(74, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO74
    GPIO_writePin(74, 0);                             // Load output latch
    GPIO_setPinConfig(GPIO_74_GPIO74);                // GPIO74 = GPIO74
    GPIO_setDirectionMode(74, GPIO_DIR_MODE_OUT);     // GPIO74 = output

    GPIO_setPadConfig(75, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO75
    GPIO_writePin(75, 0);                             // Load output latch
    GPIO_setPinConfig(GPIO_75_GPIO75);                // GPIO75 = GPIO75
    GPIO_setDirectionMode(75, GPIO_DIR_MODE_OUT);     // GPIO75 = output

    // LED OUTPUT
    // Enable GPIO outputs on GPIO38 - GPIO41, set it high
    //
    GPIO_setPadConfig(38, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO38
    GPIO_writePin(38, 0);                             // Load output latch
    GPIO_setPinConfig(GPIO_38_GPIO38);                // GPIO38 = GPIO38
    GPIO_setDirectionMode(38, GPIO_DIR_MODE_OUT);     // GPIO38 = output

    GPIO_setPadConfig(39, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO39
    GPIO_writePin(39, 0);                             // Load output latch
    GPIO_setPinConfig(GPIO_39_GPIO39);                // GPIO39 = GPIO39
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_OUT);     // GPIO39 = output

    GPIO_setPadConfig(40, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO40
    GPIO_writePin(40, 0);                             // Load output latch
    GPIO_setPinConfig(GPIO_40_GPIO40);                // GPIO40 = GPIO40
    GPIO_setDirectionMode(40, GPIO_DIR_MODE_OUT);     // GPIO40 = output

    GPIO_setPadConfig(41, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPIO41
    GPIO_writePin(41, 0);                             // Load output latch
    GPIO_setPinConfig(GPIO_41_GPIO41);                // GPIO41 = GPIO41
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_OUT);     // GPIO41 = output

    //
    // Enable GPIO GI inputs on GPIO48 - GPIO53, set it high
    //
    GPIO_setPadConfig(48, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPI48
    GPIO_setPinConfig(GPIO_48_GPIO48);                // GPIO48 = GPIO48 GI0
    GPIO_setDirectionMode(48, GPIO_DIR_MODE_IN);      // GPIO48 = input

    GPIO_setPadConfig(49, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPI49
    GPIO_setPinConfig(GPIO_49_GPIO49);                // GPIO49 = GPIO49 GI2
    GPIO_setDirectionMode(49, GPIO_DIR_MODE_IN);      // GPIO49 = input

    GPIO_setPadConfig(50, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPI50
    GPIO_setPinConfig(GPIO_50_GPIO50);                // GPIO50 = GPIO50 GI3
    GPIO_setDirectionMode(50, GPIO_DIR_MODE_IN);      // GPIO50 = input

    GPIO_setPadConfig(51, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPI51
    GPIO_setPinConfig(GPIO_51_GPIO51);                // GPIO51 = GPIO51 GI4
    GPIO_setDirectionMode(51, GPIO_DIR_MODE_IN);      // GPIO51 = input

    GPIO_setPadConfig(52, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPI52
    GPIO_setPinConfig(GPIO_52_GPIO52);                // GPIO52 = GPIO52 GI5
    GPIO_setDirectionMode(52, GPIO_DIR_MODE_IN);      // GPIO52 = input

    GPIO_setPadConfig(53, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPI53
    GPIO_setPinConfig(GPIO_53_GPIO53);                // GPIO53 = GPIO53 GI6
    GPIO_setDirectionMode(53, GPIO_DIR_MODE_IN);      // GPIO53 = input

    GPIO_setPadConfig(60, GPIO_PIN_TYPE_PULLUP);      // Enable pullup on GPI60
    GPIO_setPinConfig(GPIO_60_GPIO60);                // GPIO60 = GPIO60 GI7
    GPIO_setDirectionMode(60, GPIO_DIR_MODE_IN);      // GPIO60 = input
    EDIS;
}

void InitXBar(void){
    EALLOW;
    InputXbarRegs.INPUT4SELECT = 88;
    InputXbarRegs.INPUT5SELECT = 90;
    InputXbarRegs.INPUT6SELECT = 92;

    EDIS;
}
