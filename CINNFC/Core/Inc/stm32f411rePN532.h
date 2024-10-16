/*
 * stm32f411rePN532.h
 *
 *  Created on: Oct 10, 2024
 *      Author: MARGHOUB Ayoub
 */

#ifndef INC_STM32F411REPN532_H_
#define INC_STM32F411REPN532_H_

#include "PN532.h"

void PN532_Init(PN532* dev);
int PN532_Reset(void);
void PN532_Log(const char* log);

int PN532_I2C_ReadData(uint8_t* data, uint16_t count);
int PN532_I2C_WriteData(uint8_t *data, uint16_t count);
bool PN532_I2C_WaitReady(uint32_t timeout);
int PN532_I2C_Wakeup(void);
void PN532_I2C_Init(PN532* dev);

#endif /* INC_STM32F411REPN532_H_ */
