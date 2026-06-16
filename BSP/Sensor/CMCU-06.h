//
// Created by blin on 2026/4/30.
//

#ifndef CONTROLSYSTEM_CMCU_06_H
#define CONTROLSYSTEM_CMCU_06_H
#include <stdbool.h>

// 初始化CMCU-06
void CMCU_06_Init(void);

void CMCU_06_Write_Protect(uint8_t addr, bool state);

void CMCU_06_Reset(uint8_t addr);

void CMCU_06_ResetPins(uint8_t addr);

void CMCU_06_Cal(uint8_t addr);

void CMCU_06_single_read(uint8_t addr);

static uint16_t CRC16_Modbus(uint8_t *buf, uint8_t len);
#endif //CONTROLSYSTEM_CMCU_06_H
