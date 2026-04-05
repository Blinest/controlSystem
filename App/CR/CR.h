#ifndef __CR_H
#define __CR_H

#include <stdint.h>
#include <stdbool.h>
/**********************************************************
***	编写作者：blinest

***	qq：1071378062
**********************************************************/

typedef struct CR_Parameter
{
	double r;
} CR_Parameter;

typedef struct JointSpace
{
	float phi;
	float theta;
	float deltaL[4]; //这里不使用 deltal[0]
} JointSpace;

typedef struct OperationSpace
{
	float scale;
}OperationSpace;

typedef struct ArmParams
{
	double L;//每段长度
	double tendon_preload; // 预紧力
	double friction_coeff; // 摩擦系数

	double backbone_stiffness; //臂体弯曲刚度
	double material_damping; //材料阻尼系数

	double calibrate_offset[3]; // 肌腱零点偏移量
	double direction_gain[4]; //方向增益，对应(u,r,d,l)
} ArmParams;

typedef struct LQTS
{
	JointSpace joint_space;
	OperationSpace operation_space;
	CR_Parameter parameter;
	ArmParams arm_params[2];
	bool state;
} LQTS;

void LQTS_init(void);
uint8_t armBend(int seg, char direction, double val);
uint8_t armBend_edit(int seg, char direction, double val, double g_u, double g_r, double g_d, double g_l, double seg1_limit, double seg2_limit);
void deltaL_update(void);
void autostraight(void);
int direction_to_index(char direction);
void scale_squared(uint8_t direction, float val);
double tendonCompensation(int seg, char direction, double angle_deg);
extern LQTS lqts;
#endif
