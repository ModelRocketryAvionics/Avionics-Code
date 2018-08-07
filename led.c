/*
 * led.c
 *
 *  Created on: Aug 6, 2018
 *      Author: Michael Graves
 */
#include <stdint.h>
#include <stdbool.h>

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

#include "led.h"

//#############################################//
//                    Defines
//#############################################//

#define LED_PIN_RED             GPIO_PIN_1
#define LED_PIN_GRN             GPIO_PIN_3


//#############################################//
//                   Variables
//#############################################//


//#############################################//
//                   Functions
//#############################################//

/*
 * LedToggle()
 *  Function to Toggle the led
 */
void LedToggle() {
    // Read the current state of the GPIO pin and write the opposite
    if(GPIOPinRead(GPIO_PORTF_BASE, LED_PIN_RED)) {
        GPIOPinWrite(GPIO_PORTF_BASE, LED_PIN_RED | LED_PIN_GRN, LED_PIN_GRN);
    } else {
        GPIOPinWrite(GPIO_PORTF_BASE, LED_PIN_RED | LED_PIN_GRN, LED_PIN_RED);
    }
}

/*
 * InitLed()
 *  Function to initialize the GPIO to operate an LED
 */
void InitLed() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_PIN_RED | LED_PIN_GRN);
}


