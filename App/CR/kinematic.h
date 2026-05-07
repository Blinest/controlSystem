#ifndef __KINEMATIC_H
#define __KINEMATIC_H
#include <stdint.h>
/**********************************************************
***	编写作者：blinest

***	qq：1071378062
**********************************************************/

typedef void (*Kinematic)(float R, float theta[], float phi[], float deltaL[]);

void calculate_L(float R, float theta[], float phi[], float deltaL[]);
void inverse_kinematics(float R, const float deltaL[], float theta[], float phi[]);

#endif
