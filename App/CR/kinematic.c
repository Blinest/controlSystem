#include <stdint.h>
#include <math.h>

#include "kinematic.h"

#define pi 3.1415926535

/**********************************************************
***	编写作者：blinest

***	qq：1071378062
**********************************************************/
Kinematic calculate_L(uint8_t R, float theta, float phi, float deltaL[]) {
	deltaL[1] = R * theta * cos(phi);
	deltaL[2] = R * theta * cos(phi + 2.0 / 3.0 * pi);
	deltaL[3] = R * theta * cos(phi + 4.0 / 3.0 * pi);
}
