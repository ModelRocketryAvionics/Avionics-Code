/*
 * rf_cc2500.h
 *
 *  Created on: Jul 27, 2018
 *      Author: Newhb
 */

#ifndef RF_CC2500_H_
#define RF_CC2500_H_

//#############################################//
//                CC2500 Functions
//#############################################//

uint32_t RFWriteByte(uint8_t);

uint32_t RFWriteRegister(uint8_t, uint8_t);
uint8_t RFReadRegister(uint8_t reg);

uint32_t RFWriteRegisterCS(uint8_t, uint8_t);
uint8_t RFReadRegisterCS(uint8_t reg);

uint8_t RFSendStrobeCS(uint8_t reg);

void RFEnterTxMode(void);
void RFEnterTxModeCS(void);
void RFEnterRxMode(void);
void RFEnterRxModeCS(void);

void RFTransmitPacket(uint8_t bytes[]);
_Bool RFReceivePacket(uint8_t *receivedPacket);

void RFPrintStatus(void);

uint8_t InitRF(void);



#endif /* RF_CC2500_H_ */
