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

uint32_t RFCC2500WriteByte(uint8_t);

uint32_t RFCC2500WriteRegister(uint8_t, uint8_t);
uint8_t RFCC2500ReadRegister(uint8_t reg);

uint8_t RFCC2500ReadStatusByte();

uint8_t InitRFCC2500(void);



#endif /* RF_CC2500_H_ */
