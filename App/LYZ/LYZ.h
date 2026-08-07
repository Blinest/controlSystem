/**********************************************************
***	编写作者：blinest

***	qq：1071378062
**********************************************************/
#ifndef __LYZ_H
#define __LYZ_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

/* LYZ几何常量 */


typedef struct LYZParams
{
    double L[9]; // 连杆长度
} LYZParams;

typedef struct LYZNozzle
{
	float current_theta;
	float target_theta;
	float current_phi;
	float target_phi;
	volatile float current_S;
	float target_S;
    bool state;
} LYZNozzle;

void LYZ_init(void);

/** LYZ运动学模型*/
// 反推控制
void LYZ_thrust_reverser_kinematic_control(const uint8_t dir, const float theta);
// 截面控制(上位机命令入口：记录目标，速度/位置模式由 update 持续驱动)
void LYZ_cross_section_kinematic_control(const uint8_t dir, const float S);
// 偏转控制
void LYZ_deflect_kinematic_control(const uint8_t dir, const float phi);
// 初态复位
void LYZ_homing();
extern LYZNozzle LYZ;
#endif
