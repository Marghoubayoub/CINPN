/*
 * stm32f411rePN532.c
 *
 *  Created on: Oct 10, 2024
 *      Author: MARGHOUB Ayoub
 */

#include "stm32f4xx_hal.h"
#include "main.h"
#include "stm32f411rePN532.h"

#define _I2C_ADDRESS                    0x48
#define _I2C_TIMEOUT                    10

extern I2C_HandleTypeDef hi2c1;


/*************Reset and Log implements*****************/
int PN532_Reset(void) {
    /*HAL_GPIO_WritePin(PN532_RST_GPIO_Port, PN532_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(PN532_RST_GPIO_Port, PN532_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(PN532_RST_GPIO_Port, PN532_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(100);*/
    return PN532_STATUS_OK;
}

void PN532_Log(const char* log) {
    printf("%s\r\n", log);
}

void PN532_Init(PN532* pn532) {
    PN532_SPI_Init(pn532);
}


/********************I2C************************/
void i2c_read(uint8_t* data, uint16_t count) {
    HAL_I2C_Master_Receive(&hi2c1, _I2C_ADDRESS, data, count, _I2C_TIMEOUT);
}

void i2c_write(uint8_t* data, uint16_t count) {
    HAL_I2C_Master_Transmit(&hi2c1, _I2C_ADDRESS, data, count, _I2C_TIMEOUT);
}

int PN532_I2C_ReadData(uint8_t* data, uint16_t count) {
    uint8_t status[] = {0x00};
    uint8_t frame[count + 1];
    i2c_read(status, sizeof(status));
    if (status[0] != PN532_I2C_READY) {
        return PN532_STATUS_ERROR;
    }
    i2c_read(frame, count + 1);
    for (uint8_t i = 0; i < count; i++) {
        data[i] = frame[i + 1];
    }
    return PN532_STATUS_OK;
}

int PN532_I2C_WriteData(uint8_t *data, uint16_t count) {
    i2c_write(data, count);
    return PN532_STATUS_OK;
}

bool PN532_I2C_WaitReady(uint32_t timeout) {
    uint8_t status[] = {0x00};
    uint32_t tickstart = HAL_GetTick();
    while (HAL_GetTick() - tickstart < timeout) {
        i2c_read(status, sizeof(status));
        if (status[0] == PN532_I2C_READY) {
            return true;
        } else {
            HAL_Delay(5);
        }
    }
    return false;
}

int PN532_I2C_Wakeup(void) {
    // TODO
    /*HAL_GPIO_WritePin(PN532_REQ_GPIO_Port, PN532_REQ_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(PN532_REQ_GPIO_Port, PN532_REQ_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(PN532_REQ_GPIO_Port, PN532_REQ_Pin, GPIO_PIN_SET);
    HAL_Delay(500);*/
    return PN532_STATUS_OK;
}

void PN532_I2C_Init(PN532* pn532) {
    // init the pn532 functions
    pn532->reset =  PN532_Reset;
    pn532->read_data = PN532_I2C_ReadData;
    pn532->write_data = PN532_I2C_WriteData;
    pn532->wait_ready = PN532_I2C_WaitReady;
    pn532->wakeup = PN532_I2C_Wakeup;
    pn532->log = PN532_Log;

    // hardware wakeup
    pn532->wakeup();
}

