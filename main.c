#include "stdint.h"
#include "stdbool.h"

#include "inc/tm4c123gh6pm.h"

#include "driverlib/sysctl.h"

#include "sch.h"
#include "ser.h"
#include "rf_cc2500.h"
#include "led.h"

//#############################################//
//                    Defines
//#############################################//

#define SCHEDULER_SYSTICK_PERIOD        80000
#define SCHEDULER_MILLISECOND_PERIOD    1

#define SENS_ACCEL_UPDATE_PERIOD        2
#define SENS_GYRO_UPDATE_PERIOD         2
#define SENS_ALT_UPDATE_PERIOD          1000

#define RF_PACKET_LENGTH                21

//#############################################//
//                   Variables
//#############################################//

extern uint32_t currentTime;
extern _Bool RF_packetReceived;

//#############################################//
//                 User Functions
//#############################################//

uint8_t Task_Millis_Init(void) {
    //Nothing to initialize
    return 0;
}

_Bool Task_Millis_Exec(uint8_t taskID){
    // Task for things that need to execute/check every ms

    // ..Put stuff here..

    // Returns true to set the task back to IDLE state
    return true;
}

uint8_t Task_Idle_Init(void) {
    InitLed();
    return 0;
}

_Bool Task_Idle_Exec(uint8_t taskID){
    LedToggle();
    return true;
}

_Bool Task_OnPacketReceived(uint8_t taskID) {
    uint8_t receivedPacket[RF_PACKET_LENGTH];

    // Get the received packet, this function returns false if it failed
    if(RFReceivePacket(&(receivedPacket[0]))) {
        // Print out the received packet as integers (each byte is a line)
        SerialPrintln("[RF] PACKET RECEIVED!");
        SerialPrint(" > ");
        uint8_t i;
        char decodedMessage[RF_PACKET_LENGTH + 1];
        for (i = 0; i < RF_PACKET_LENGTH; i++) {
            SerialPrintInt(receivedPacket[i]); SerialPrint(" ");

            decodedMessage[i] = (char)(receivedPacket[i]);
        }
        // Add end of string character
        decodedMessage[RF_PACKET_LENGTH] = '\0';
        SerialPrintln("");

        SerialPrint(" > "); SerialPrint(decodedMessage); SerialPrintln("");
    }
    return true;
}

/*
 * SerialOnCharReceived(character)
 *  Function to handle received character events over serial
 * This function is called in an interrupt, keep its exec time really short or use..
 *  other non-interrupt functions.
 */
void SerialOnCharReceived(char character) {
    //Echo the character
    //SerialWrite(character);
}

/*
 * SerialOnCharReceived(character)
 *  Function to handle received line events over serial configure the line endings..
 *   in the #define section found in ser.c.
 *  This function is called in an interrupt, keep its exec time really short or use..
 *   other non-interrupt functions.
 */
void SerialOnLineReceived(volatile char* stringStart, volatile char* stringEnd) {
    //Do nothing.. Below snippet is provided as an example

    uint8_t length = stringEnd - stringStart;

    // If its the right length for a packet (minus address byte), send it!
    if(length == RF_PACKET_LENGTH - 1) {
        RFEnterTxModeCS();

        uint8_t bytes[RF_PACKET_LENGTH - 1];
        uint8_t i = 0;
        volatile char *c;
        for(c = stringStart; c < stringEnd; c++) {
            bytes[i] = (uint8_t)(*c);
            i++;
        }
        RFTransmitPacket(bytes);
        SerialPrintln("[RF] Sent Packet");
        return;
    }

    if(*stringStart == 'r') {
        RFEnterRxModeCS();
        SerialPrintln("[RF] Entered RX Mode");
    } else if (*stringStart == 's') {
        RFPrintStatus();
    }
}

//#############################################//
//                     Setup()
//#############################################//
void Setup(void) {
    // Setting Clock to 80MHz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    // Add peripheral initializations here
    InitScheduler(SCHEDULER_SYSTICK_PERIOD);
    InitSerial(&SerialOnCharReceived, &SerialOnLineReceived);
    InitRF();

    // Add Tasks Here
    AddTaskTime(&Task_Millis_Init, &Task_Millis_Exec, SCHEDULER_MILLISECOND_PERIOD);
    AddTaskCond(&VoidInit, &Task_OnPacketReceived, &RF_packetReceived);

    AddTaskIdle(&Task_Idle_Init, &Task_Idle_Exec);

    //
}


//#############################################//
//                     Loop()
//#############################################//
void Loop(void) {
    UpdateScheduler();
}

//#############################################//
//                  Main Function
//#############################################//
/*  You shouldn't need to edit anything here as..
 *   its brought out to Setup() and Loop().
 */
int main(void) {
    Setup();
    while(1){
        Loop();
    }
}
