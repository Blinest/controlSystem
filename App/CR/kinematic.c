#include <stdint.h>
#include <math.h>
#include "Emm_V5.h"
#include "kinematic.h"

#define pi 3.1415926535

/**********************************************************
***	编写作者：blinest

***	qq：1071378062
**********************************************************/
// 基础版本的常曲率模型
void calculate_L(uint8_t R, float theta[], float phi[], float deltaL[]) {
	deltaL[1] = R * theta[0] * cos(phi[0]);
	deltaL[2] = R * theta[0] * cos(phi[0] + 2.0 / 3.0 * pi);
	deltaL[3] = R * theta[0] * cos(phi[0] + 4.0 / 3.0 * pi);
}
