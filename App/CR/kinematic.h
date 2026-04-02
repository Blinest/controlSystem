#ifndef __KINEMATIC_H
#define __KINEMATIC_H
#include <stdint.h>
#include "Motor/Motor.h"
/**********************************************************
***	编写作者：blinest

***	qq：1071378062
**********************************************************/

Kinematic calculate_L(uint8_t R, float theta, float phi, float deltaL[]);

#endif
