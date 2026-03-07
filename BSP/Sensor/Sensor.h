//
// Created by blin on 2026/3/7.
//

#ifndef CONTROLSYSTEM_SENSOR_H
#define CONTROLSYSTEM_SENSOR_H
#include <stdint.h>
void sensor_init(void);
void sensor_single_read(uint8_t sensor_id);
void sensor_multi_read(void);
void sensor_self_test();
#endif //CONTROLSYSTEM_SENSOR_H