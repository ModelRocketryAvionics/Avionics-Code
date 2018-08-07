/*
 * rf_cc2500.c
 *
 *  Created on: Jul 26, 2018
 *      Author: Michael Graves
 *
 *  With reference to: https://github.com/farrellf/Balancing_Robot_Firmware
 */
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/tm4c123gh6pm.h"

#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"

#include "ser.h"
#include "rf_cc2500.h"

//#############################################//
//        CC2500 Configuration Defines
//#############################################//

// Access modes for the configuration and multi-byte registers. Add/or them to a register name to set the appropriate bits.
#define WRITE_BYTE      0x00
#define WRITE_BURST     0x40
#define READ_BYTE       0x80
#define READ_BURST      0xC0

// Configuration registers.
// see: http://www.ti.com/lit/ds/symlink/cc2500.pdf
#define IOCFG2          0x00    /* pg61,53  GDO2 output pin config */
#define IOCFG1          0x01    /* pg61,53  GDO1 output pin config */
#define IOCFG0          0x02    /* pg61,53  GDO0 output pin config */
#define FIFOTHR         0x03    /* pg62     RX FIFO and TX FIFO thresholds */
#define SYNC1           0x04    /* pg62     sync word, high byte */
#define SYNC0           0x05    /* pg62     sync word, low byte */
#define PKTLEN          0x06    /* pg62     packet length */
#define PKTCTRL1        0x07    /* pg63     packet automation control */
#define PKTCTRL0        0x08    /* pg64     packet automation control */
#define ADDR            0x09    /* pg64     device address */
#define CHANNR          0x0A    /* pg64     channel number */
#define FSCTRL1         0x0B    /* pg65     frequency synthesizer control */
#define FSCTRL0         0x0C    /* pg65     frequency synthesizer control */
#define FREQ2           0x0D    /* pg65     frequency control word, high byte */
#define FREQ1           0x0E    /* pg65     frequency control word, middle byte */
#define FREQ0           0x0F    /* pg65     frequency control word, low byte */
#define MDMCFG4         0x10    /* pg66     modem configuration */
#define MDMCFG3         0x11    /* pg66     modem configuration */
#define MDMCFG2         0x12    /* pg67     modem configuration */
#define MDMCFG1         0x13    /* pg68     modem configuration */
#define MDMCFG0         0x14    /* pg68     modem configuration */
#define DEVIATN         0x15    /* pg69     modem deviation setting */
#define MCSM2           0x16    /* pg70     main radio control state machine config */
#define MCSM1           0x17    /* pg71     main radio control state machine config */
#define MCSM0           0x18    /* pg72     main radio control state machine config */
#define FOCCFG          0x19    /* pg73     frequency offset compensation config */
#define BSCFG           0x1A    /* pg74     bit synchronization config */
#define AGCCTRL2        0x1B    /* pg75     agc control */
#define AGCCTRL1        0x1C    /* pg76     agc control */
#define AGCCTRL0        0x1D    /* pg77     agc control */
#define WOREVT1         0x1E    /* pg77     event0 timeout, high byte */
#define WOREVT0         0x1F    /* pg78     event0 timeout, low byte */
#define WORCTRL         0x20    /* pg78     wake on radio control */
#define FREND1          0x21    /* pg78     front end rx config */
#define FREND0          0x22    /* pg79     front end tx config */
#define FSCAL3          0x23    /* pg79     frequency synthesizer calibration */
#define FSCAL2          0x24    /* pg79     frequency synthesizer calibration */
#define FSCAL1          0x25    /* pg80     frequency synthesizer calibration */
#define FSCAL0          0x26    /* pg80     frequency synthesizer calibration */
#define RCCTRL1         0x27    /* pg80     rc oscillator config */
#define RCCTRL0         0x28    /* pg80     rc oscillator config */
#define FSTEST          0x29    /* pg80     frequency synthesizer calibration control */
#define PTEST           0x2A    /* pg80     production test */
#define AGCTEST         0x2B    /* pg81     agc test */
#define TEST2           0x2C    /* pg82     various test settings */
#define TEST1           0x2D    /* pg82     various test settings */
#define TEST0           0x2E    /* pg82     various test settings */

// Multi-byte registers.
#define PATABLE         0x3E    /* pg46     output power setting */
#define FIFO            0x3F    /* pg43     read the RX FIFO or write the TX FIFO one byte at a time */

// Command strobes. Will be written as individual bytes (not bursts)
#define SRES            (0x30 + WRITE_BYTE)     /* pg57     reset chip */
#define SFSTXON         (0x31 + WRITE_BYTE)     /* pg57     if MCSM0.FS_AUTOCAL=1: enable and calibrate frequency synthesizer
                                                            if RX with CCA: go to wait state (only synth. running) for quick TX/RX turnaround */
#define SXOFF           (0x32 + WRITE_BYTE)     /* pg57     turn off crystal oscillator */
#define SCAL            (0x33 + WRITE_BYTE)     /* pg57     calibrate frequency synthesizer and turn it off */
#define SRX             (0x34 + WRITE_BYTE)     /* pg57     enable RX. will also perform calibration if coming from idle and MCSM0.FS_AUTOCAL=1 */
#define STX             (0x35 + WRITE_BYTE)     /* pg57     enable TX. will also perform calibration if coming from idle and MCSM0.FS_AUTOCAL=1
                                                            if RX with CCA: only go to TX mode if channel clear */
#define SIDLE           (0x36 + WRITE_BYTE)     /* pg57     exit TX/RX mode. turns off frequency synthesizer and exits WOR mode if applicable. */
#define SWOR            (0x38 + WRITE_BYTE)     /* pg57     start automatic RX polling if WORCTRL.RC_PD=1 */
#define SPWD            (0x39 + WRITE_BYTE)     /* pg57     enter power down mode when CS goes high */
#define SFRX            (0x3A + WRITE_BYTE)     /* pg57     flush the RX FIFO. only use when in IDLE or RXFIFO_OVERFLOW states */
#define SFTX            (0x3B + WRITE_BYTE)     /* pg57     flush the TX FIFO. only use when in IDLE or TXFIFO_UNDERFLOW states */
#define SWORRST         (0x3C + WRITE_BYTE)     /* pg57     reset RTC to Event1 value */
#define SNOP            (0x3D + WRITE_BYTE)     /* pg57     no operation */

// Status registers. Will be read as individual bytes (NOT actually a burst as listed below.)
#define PARTNUM         (0x30 + READ_BURST)     /* pg81     chip part number, = 0x80 */
#define VERSION         (0x31 + READ_BURST)     /* pg81     chip version number, = 0x03 */
#define FREQEST         (0x32 + READ_BURST)     /* pg81     frequency offset estimate from demodulator */
#define LQI             (0x33 + READ_BURST)     /* pg82     link quality estimate from demodulator */
#define RSSI            (0x34 + READ_BURST)     /* pg82     received signal strength indication */
#define MARCSTATE       (0x35 + READ_BURST)     /* pg82     main radio control state machine state */
#define WORTIME1        (0x36 + READ_BURST)     /* pg83     WOR time, high byte */
#define WORTIME0        (0x37 + READ_BURST)     /* pg83     WOR time, low byte */
#define PKTSTATUS       (0x38 + READ_BURST)     /* pg83     current GDOx status and packet status */
#define VCO_VC_DAC      (0x39 + READ_BURST)     /* pg83     current setting from PLL calibration module */
#define TXBYTES         (0x3A + READ_BURST)     /* pg83     underflow and number of bytes in TX FIFO */
#define RXBYTES         (0x3B + READ_BURST)     /* pg84     overflow and number of bytes in RX FIFO */
#define RCCTRL1_STATUS  (0x3C + READ_BURST)     /* pg85     last rc oscillator calibration result */
#define RCCTRL0_STATUS  (0x3D + READ_BURST)     /* pg85     last rc oscillator calibration result */

//#############################################//
//                 CC2500 Defines
//#############################################//

// This was given in Tiva C Peripheral Driver Library Example
#define RF_SSI_DATA_RATE    1000000

#define RF_SSI_GPIO_CLK     GPIO_PIN_0
#define RF_SSI_GPIO_FSS     GPIO_PIN_1
#define RF_SSI_GPIO_RX      GPIO_PIN_2
#define RF_SSI_GPIO_TX      GPIO_PIN_3

// PE5 connected to GDO0 which acts as an interrupt pin for a received packet
#define RF_GPIO_INTERRUPT   GPIO_PIN_5

#define RF_ADDRESS          69
#define RF_PACKET_LENGTH    21


//#############################################//
//                CC2500 Variables
//#############################################//

_Bool RF_packetReceived = false;

uint32_t count = 0;

//#############################################//
//                CC2500 Functions
//#############################################//

void RFChipSelectOn() {
    GPIOPinWrite(GPIO_PORTD_BASE, RF_SSI_GPIO_FSS, 0);
}

void RFChipSelectOff() {
    GPIOPinWrite(GPIO_PORTD_BASE, RF_SSI_GPIO_FSS, RF_SSI_GPIO_FSS);
}

uint32_t RFWriteByte(uint8_t reg) {
    SSIDataPut(SSI1_BASE, reg);

    uint32_t ui32Data;
    SSIDataGet(SSI1_BASE, &ui32Data);
    return ui32Data;
}

/*
 * RFWriteRegisteR(reg, value)
 *  Writes the register with address reg with value.
 *   reg should be ORd or Added with the correct access mode
 *  This function DOES NOT handle the chip select line!
 */
uint32_t RFWriteRegister(uint8_t reg, uint8_t value) {
    RFWriteByte(reg);
    return RFWriteByte(value);
}

/*
 * RFWriteRegisteR(reg, value)
 *  Writes the register with address reg with value.
 *   reg should be ORd or Added with the correct access mode
 *  This function HANDLES the chip select line!
 */
uint32_t RFWriteRegisterCS(uint8_t reg, uint8_t value) {
    RFChipSelectOn();

    uint32_t result = RFWriteRegister(reg, value);

    RFChipSelectOff();
    return result;
}

/*
 * RFReadRegister(reg)
 *  Returns the register with address register.
 *   reg should be ORd or Added with the correct access mode
 *  This function DOES NOT handle the chip select line!
 */
uint8_t RFReadRegister(uint8_t reg) {
    return RFWriteRegister(reg, 0x00);
}

/*
 * RFReadRegister(reg)
 *  Returns the register with address register.
 *   reg should be ORd or Added with the correct access mode
 *  This function HANDLES the CS line.
 */
uint8_t RFReadRegisterCS(uint8_t reg) {
    return RFWriteRegisterCS(reg, 0x00);
}

/*
 * RFSendStrobe(reg)
 *  Sends the single byte strobe command and returns the status byte
 */
uint8_t RFSendStrobe(uint8_t reg) {
    return RFWriteByte(reg);;
}

/*
 * RFSendStrobeCS(reg)
 *  Sends the single byte strobe command and returns the status byte
 */
uint8_t RFSendStrobeCS(uint8_t reg) {
    RFChipSelectOn();

    uint8_t result = RFSendStrobe(reg);

    RFChipSelectOff();

    return result;
}

// Enters Tx mode
void RFEnterTxMode(void) {
    RFSendStrobe(STX);
}

// Enters Tx mode, handles CS
void RFEnterTxModeCS(void) {
    RFSendStrobeCS(STX);
}

// Flushes the Tx FIFO
void RFFlushTx(void) {
    RFSendStrobe(SFTX);
}

// Enters Rx mode
void RFEnterRxMode(void) {
    RFSendStrobe(SRX);
}

// Enters Rx mode, handles CS
void RFEnterRxModeCS(void) {
    RFSendStrobeCS(SRX);
}

// Flushes the Rx FIFO
void RFFlushRx(void) {
    RFSendStrobe(SFRX);
}

/*
 * RFTransmitPacket(bytes[])
 *  Must give 20 bytes!
 *      4B: TIME[32b]
 *      6B: ACCEL[16b + 16b + 16b]
 *      6B: GRYO[16b + 16b + 16b]
 *      4B: ALT[20b] + TEMP[12b]
 *
 *  The Address Byte is automatically added
 */
void RFTransmitPacket(uint8_t bytes[]) {
    //Wait for an empty FIFO
    while(RFReadRegisterCS(TXBYTES) != 0) {
        SerialPrintlnInt(RFReadRegisterCS(TXBYTES));
    }

    //Send bytes
    RFChipSelectOn();

    // Send write command as burst
    //RFWriteByte( FIFO | WRITE_BURST);

    //Send RF Address
    RFWriteRegister(FIFO, RF_ADDRESS);

    //Send payload
    uint8_t i;
    for(i = 0; i < RF_PACKET_LENGTH-1; i++) {
        RFWriteRegister(FIFO, bytes[i]);
    }

    //Get status and check for TX FIFO underflow
    uint8_t status = RFSendStrobeCS(SNOP);
    if((status >> 4) == 7) {

        //TX underflow, flush buffer and enter tx mode
        RFFlushTx();
        RFEnterTxMode();
    }

    RFChipSelectOff();
}

/*
 * RFReceivePacket(*receivedPacket[21])
 *  Must give array of size 21 bytes!
 *      1B: ADDRESS[8b]
 *      4B: TIME[32b]
 *      6B: ACCEL[16b + 16b + 16b]
 *      6B: GRYO[16b + 16b + 16b]
 *      4B: ALT[20b] + TEMP[12b]
 */
_Bool RFReceivePacket(uint8_t *receivedPacket) {
    if(RFReadRegisterCS(RXBYTES) < RF_PACKET_LENGTH) return false;

    RFChipSelectOn();

    // Send read command as burst
    RFWriteByte( FIFO | READ_BURST);

    uint8_t i;
    for(i = 0; i < RF_PACKET_LENGTH;i++) {
        *(receivedPacket + i) = RFWriteByte(0x00);
    }

    RFChipSelectOff();
    return true;
}

void RFReceivePacketHandler(void) {
    GPIOIntClear(GPIO_PORTE_BASE, RF_GPIO_INTERRUPT);

    // GDO0 as interrupt: Associated to the RX FIFO: Asserts when RX FIFO is..
    //  filled at or above the RX FIFO threshold or the end of packet is..
    //  reached. De-asserts when the RX FIFO is empty.
    if((GPIOPinRead(GPIO_PORTE_BASE, RF_GPIO_INTERRUPT) && RF_GPIO_INTERRUPT) == 0) {
        //De-assert: RX FIFO is empty
        RF_packetReceived = false;
    } else {
        //Assert: END OF PACKET
        RF_packetReceived = true;

        count++;
    }
}

uint8_t InitRF(void) {
    // SPI SETUP
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) {}

    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI1)) {}

    GPIOPinConfigure(GPIO_PD0_SSI1CLK);
    //GPIOPinConfigure(GPIO_PD1_SSI1FSS);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, RF_SSI_GPIO_FSS);
    GPIOPinConfigure(GPIO_PD2_SSI1RX);
    GPIOPinConfigure(GPIO_PD3_SSI1TX);

    //GPIOPinTypeSSI(GPIO_PORTD_BASE, RF_SSI_GPIO_CLK | RF_SSI_GPIO_FSS | RF_SSI_GPIO_RX | RF_SSI_GPIO_TX);
    GPIOPinTypeSSI(GPIO_PORTD_BASE, RF_SSI_GPIO_CLK | RF_SSI_GPIO_RX | RF_SSI_GPIO_TX);

    SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, RF_SSI_DATA_RATE, 8);

    //Raise the Chip Select line (disable the chip)
    RFChipSelectOff();

    // RF_Interrupt Setup (PE5)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) {}

    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, RF_GPIO_INTERRUPT);

    // Register, configure and enable the GDO0 interrupt handler
    GPIOIntRegister(GPIO_PORTE_BASE, RFReceivePacketHandler);
    GPIOIntTypeSet(GPIO_PORTE_BASE, RF_GPIO_INTERRUPT, GPIO_BOTH_EDGES);

    SSIEnable(SSI1_BASE);
    SysCtlDelay(1000);

    //Clear the received FIFO to make sure it starts empty.
    uint32_t garbageBin;
    while(SSIDataGetNonBlocking(SSI1_BASE, &garbageBin)) {}

    //CC2500 SETUP
    //Reset the CC2500
    RFSendStrobeCS(SRES);
    SysCtlDelay(1000);

    //Check status of the CC2500 (bottom 4 bits is the available slots in the FIFO should be all 1s)
    uint8_t status = RFSendStrobeCS(SNOP);
    if(status>>4){
        SerialPrint("[RF] Status: ERR (");
        SerialPrintInt(status);
        SerialPrintln(")");
    } else {
        SerialPrint("[RF] Status: OK (");
        SerialPrintInt(status);
        SerialPrintln(")");
    }

    SerialPrint("[RF] TXBYTES: ");
    SerialPrintlnInt(RFReadRegisterCS(TXBYTES | READ_BYTE));

    //Initialize Configuration Variables
    // These were obtained using the TI tool: SmartRF Studio and configuring the CC2500 for..
    //  Data rate: 2.4kBaud, Dev.: 38kHz, Mod.: 2-FSK, RX BW: 203kHz, Optimized  for sensitivity
    //  Fixed packet length mode. Length configured in PKTLEN register, Length: 21 Bytes
    //   1B: ADDRESS[8b]
    //   4B: TIME[32b]
    //   6B: ACCEL[16b + 16b + 16b]
    //   6B: GRYO[16b + 16b + 16b]
    //   4B: ALT[20b] + TEMP[12b]

    RFChipSelectOn();

    RFWriteRegister(IOCFG0,   0x01);      // GDO0 as interrupt: Associated to the RX FIFO: Asserts when RX FIFO is..
                                          //  filled at or above the RX FIFO threshold or the end of packet is..
                                          //  reached. De-asserts when the RX FIFO is empty.

    RFWriteRegister(PKTLEN,   RF_PACKET_LENGTH);     // Packet length +1 for the address at byte 0
    RFWriteRegister(PKTCTRL0, 0x44);      // Packet control: data whitening, CRC enabled, fixed packet length mode
    RFWriteRegister(CHANNR,   0x01);      // Channel: 1

    //RFWriteRegisterCS(ADDR,     RF_ADDRESS);    // Address: 0x69

    RFWriteRegister(FSCTRL1,  0x08);
    RFWriteRegister(FREQ2,    0x5D);
    RFWriteRegister(FREQ1,    0x93);
    RFWriteRegister(FREQ0,    0xB1);
    RFWriteRegister(MDMCFG4,  0x86);
    RFWriteRegister(MDMCFG3,  0x83);
    RFWriteRegister(MDMCFG2,  0x03);
    RFWriteRegister(MDMCFG1,  0x00);
    RFWriteRegister(DEVIATN,  0x44);
    RFWriteRegister(MCSM1,    0x0E);      // Radio state machine: stay in TX mode after sending a packet, stay in RX mode after receiving
    RFWriteRegister(MCSM0,    0x18);
    RFWriteRegister(FOCCFG,   0x16);
    RFWriteRegister(FSCAL1,   0x00);
    RFWriteRegister(FSCAL0,   0x11);

    //RFWriteRegister(PATABLE,  0xFE);      // Output power: 0dBm
    RFWriteRegister(PATABLE,  0xFF);    // Output power: +1dBm (the maximum possible, UNLIMITED POOOOWWAAAAAA!)
    RFChipSelectOff();

    // Enable the RF_Received Interrupt
    GPIOIntClear(GPIO_PORTE_BASE, RF_GPIO_INTERRUPT);
    GPIOIntEnable(GPIO_PORTE_BASE, RF_GPIO_INTERRUPT);

    SerialPrintln("[RF] Entered RX Mode");
    RFEnterRxModeCS();

    uint8_t rssi = RFSendStrobeCS(RSSI);
    SerialPrint("[RF] RSSI: ");
    SerialPrintlnInt(rssi);

    return 0;
}
