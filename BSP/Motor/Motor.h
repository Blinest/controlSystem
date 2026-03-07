//
// Created by blin on 2026/3/7.
//

#ifndef CONTROLSYSTEM_MOTOR_H
#define CONTROLSYSTEM_MOTOR_H
#include "stdint.h"
void motor_init();
void motor_enable(uint8_t addr);
void motor_stop();
void motor_single_control(uint8_t addr, uint8_t direction, uint16_t distance);
void motor_sync_control(uint8_t count, uint8_t start_addr, uint16_t *distances);
void motor_kinematic_control();
void motor_custom_control(uint8_t count, uint8_t *params);
#endif //CONTROLSYSTEM_MOTOR_H