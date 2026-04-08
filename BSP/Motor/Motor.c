//
// Created by blin on 2026/3/7.
//
/**
* @file motor.c
 * @brief 电机指令处理模块
 *
 * 本模块提供电机指令处理功能：
 * - motor_init()：电机初始化，初始化流程包括电机参数设置、
 * - motor_run()：启动电机，并设置绝对目标位置
 * - motor_position_control_snf()：
 * - motor_emergency_stop_all()：紧急停止所有电机
 *
 */
#include "Motor.h"
#include "ZDT_X42_V2.h"
#include "math.h"
#include "motor_limits.h"
#include <stdio.h>
#include "usart.h"
#include "CR/kinematic.h"

// 创建电机与电机反馈数据结构体
MotorFeedback motor_feedback[MOTOR_NUM];
GlobalMotor global_motor[MOTOR_NUM];
//电机初始化函数
void motor_init()
{
	// 初始化电机相关参数
	for (int i = 0; i < MOTOR_NUM; i++)
	{
		global_motor[i].id = MOTOR_ID + i;
		global_motor[i].stepper_motor.daocheng = 9; // 根据丝杠导程设置，9mm
		global_motor[i].stepper_motor.xifen = 128; // 平滑控制
		global_motor[i].stepper_motor.step_angle = 1.8; // 步距角
		global_motor[i].stepper_motor.current_vel = 10; // 如果要完成指标，至少是 10mm/s
		global_motor[i].vel_max = 50; // 最大速度建议50rpm以下，速度过高，步进电机(FUYU35, 2025年)会出现抖动
		global_motor[i].current_acc = 0; // 由于位移量较小，为提高响应速度，直接启动，不做加减速处理 (0-255)

		// 初始化历史记录 (防止信号干扰检测)
		motor_history_init(i);
	}
	
	printf("[MOTOR] 电机初始化完成，限制条件已启用\n");
}
// 单电机使能函数
void motor_enable(uint8_t addr,bool enable)
{
	// 更新电机状态，(电机使能状态 0x02)
	ZDT_X42_V2_En_Control(addr, enable, 0);

}

// 错误处理函数（增强版，支持限制条件错误）
void motor_error_handler(uint8_t addr, uint8_t error_code)
{
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        if (global_motor[i].id == addr)
        {
            // 记录错误信息
            global_motor[i].state = 0xEE; // 错误状态
            
            // 根据错误代码采取不同措施
            switch(error_code)
            {
                case 0x01: // 通信超时
                    printf("[MOTOR] 电机%d通信超时\n", addr);
                    // 尝试重新使能电机建立连接
                    ZDT_X42_V2_En_Control(addr,1,0);
                    break;
                    
                case 0x02: // 过流保护
                    printf("[MOTOR] 电机%d过流保护\n", addr);
                    // 紧急停止所有电机
                    motor_stop_all();
                    break;
                    
                case 0x03: // 位置超限
                    printf("[MOTOR] 电机%d位置超限\n", addr);
                    // 重置位置参数到安全位置
                    global_motor[i].current_pos = 0;
                    global_motor[i].target_pos = 0;
                    global_motor[i].stepper_motor.current_pos = 0;
                    global_motor[i].stepper_motor.target_pos = 0;
                    break;
                    
                case 0x04: // 限制条件错误
                    printf("[MOTOR] 电机%d限制条件错误\n", addr);
                    // 不执行动作，只记录错误
                    // 可以增加错误计数，达到阈值后采取行动
                    break;
                    
                case 0x05: // 参数突变错误
                    printf("[MOTOR] 电机%d参数突变错误\n", addr);
                    // 重置历史记录，重新开始
                    motor_reset_error_count(i);
                    break;
                    
                case 0x06: // 信号干扰检测
                    printf("[MOTOR] 电机%d检测到信号干扰\n", addr);

                    break;
                    
                default:
                    printf("[MOTOR] 电机%d未知错误: 0x%02X\n", addr, error_code);
                    // 未知错误，记录日志
                    break;
            }
            
            // 更新最后响应时间（标记为错误时间）
            global_motor[i].last_response_time = HAL_GetTick();
            
            // 增加错误计数
            uint8_t error_count = motor_get_error_count(i);
            if (error_count >= MOTOR_ERROR_COUNT_MAX) {
                printf("[MOTOR] 电机%d错误次数过多(%d)，进入保护模式\n", addr, error_count);
                // 可以进入安全模式，如降低速度限制等
                global_motor[i].vel_max *= 0.5f; // 降低最大速度
            }
            
            break;
        }
    }
    // send_error_notification(addr, error_code);
}

// 电机参数自动校准函数
void motor_auto_calibrate(uint8_t addr)
{
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        if (global_motor[i].id == addr)
        {
            // 1. 寻找机械零点
            // Emm_V5_Find_Home(addr);
            
            // 2. 校准步进参数
            global_motor[i].stepper_motor.daocheng = 2; // 默认9mm，可根据实际测量调整
            global_motor[i].stepper_motor.xifen = 128;  // 默认128细分
            
            // 3. 校准最大速度
            global_motor[i].vel_max = 50; // 默认50rpm
            
            // 4. 重置位置计数器
            global_motor[i].current_pos = 0;
            global_motor[i].target_pos = 0;
            
            // 5. 标记校准完成
            global_motor[i].state = 0x02; // 正常状态
            
            // 可选：保存校准参数到EEPROM
            // save_calibration_params(addr, &motor[i]);
            
            break;
        }
    }
}
/**
  * @brief 启动步进电机，并达到指定位置（带限制条件）
  * @param addr: 电机地址
  * @param vel 速度值, mm/s
  * @param target: 目标位置(绝对位置), mm
  * @param snf: 同步标志位，true同步
  */
void motor_run(int idx, float vel, float target, bool snf) {

	// ==================== 限制条件检查 ====================
	float current_pos = global_motor[idx].stepper_motor.current_pos;
	float displacement = target - current_pos;

	// 3. 应用限制条件 (钳位)
	float min_speed = 10;
	float limited_displacement = motor_apply_displacement_limit(displacement, current_pos);
	float limited_target = current_pos + limited_displacement;

	// ==================== 执行控制 ====================
	const int xifen = global_motor[idx].stepper_motor.xifen;
	const int daocheng = global_motor[idx].stepper_motor.daocheng;
	const double step_angle = global_motor[idx].stepper_motor.step_angle;
	const float current_vel = min_speed * 60.0f / (float)daocheng;
	global_motor[idx].current_pos = fabs(limited_target / daocheng * 360.0f);
	global_motor[idx].current_vel =  current_vel;
	// 位置计算 (使用限制后的参数)
	const float distance = xifen * 360.0f / step_angle * limited_target / daocheng;
	const int dir = distance > 0 ? 1:0;
	//const uint32_t clk = (uint32_t)fabs(distance);
	
	// 更新电机状态
	global_motor[idx].stepper_motor.target_pos = limited_target;
	global_motor[idx].stepper_motor.target_vel = (uint16_t)min_speed;

	// 模拟电机运行
	global_motor[0].stepper_motor.current_acc = 0;
	global_motor[idx].stepper_motor.current_pos = limited_target;
	ZDT_X42_V2_Bypass_Position_LV_Control(global_motor[idx].id, dir, current_vel, global_motor[idx].current_pos, 1, snf);

}

// 电机紧急停止函数
void motor_stop_all()
{
	for(int i = 0; i < MOTOR_NUM; i++) {
		ZDT_X42_V2_Stop_Now(global_motor[i].id, false);
		global_motor[i].state = 0;
	}
}
// 单电机控制函数（带限制条件）
void motor_single_control(uint8_t addr, uint8_t direction, float distance, float vel)
{
	int idx = (addr >= MOTOR_ID && addr < MOTOR_ID + MOTOR_NUM) ? (addr - MOTOR_ID) : 0;
	
	// ==================== 限制条件检查 ====================
	float current_pos = global_motor[idx].stepper_motor.current_pos;
	float displacement = (direction == 0) ? -distance : distance;
	
	// 1. 检查所有限制条件 (使用默认速度)
	float default_speed = global_motor[idx].vel_max * 0.5f; // 使用50%最大速度
	if (vel > default_speed) vel = default_speed;

	// 2. 应用限制条件
	float limited_displacement = motor_apply_displacement_limit(displacement, current_pos);
	float limited_target = current_pos + limited_displacement;

	// ==================== 执行控制 ====================
	// 更新电机状态
	global_motor[idx].stepper_motor.target_pos = limited_target;
	global_motor[idx].stepper_motor.target_vel = vel;
	// 调用底层控制函数
	motor_run(addr, (uint16_t)vel, limited_target, false);
}
// 多电机同步控制函数（带限制条件）
void motor_sync_control(uint8_t count, uint8_t start_addr, float distance[])
{
	int size = count;
	
	// ==================== 限制条件检查 ====================
	float limited_distances[MOTOR_NUM];
	float max_distance = 0;
	uint16_t speed[MOTOR_NUM];
	uint8_t error_count = 0;
	
	// 检查每个电机的位移限制
	for (int i = 0; i < size; i++)
	{
		uint8_t motor_id = start_addr + i;
		int idx = (motor_id >= MOTOR_ID && motor_id < MOTOR_ID + MOTOR_NUM) ? (motor_id - MOTOR_ID) : 0;
		
		float current_pos = global_motor[idx].stepper_motor.current_pos;
		float displacement = distance[i];
		


		// 应用位移限制
		float limited_displacement = motor_apply_displacement_limit(displacement, current_pos);
		limited_distances[i] = limited_displacement;

		// 计算绝对位移用于速度分配
		float abs_distance = fabsf(limited_displacement);
		max_distance = fmax(max_distance, abs_distance);
		
		if (fabsf(limited_displacement - displacement) > 0.1f) {
			printf("[MOTOR] 电机%d同步位移从%.2f限制到%.2f mm\n", 
			       motor_id, displacement, limited_displacement);
		}
	}

	
	// ==================== 速度分配 ====================
	// 动态速度调整：线性比例控制，考虑每个电机的最大速度限制
	for (int i = 0; i < size; i++)
	{
		uint8_t motor_id = start_addr + i;
		int idx = (motor_id >= MOTOR_ID && motor_id < MOTOR_ID + MOTOR_NUM) ? (motor_id - MOTOR_ID) : 0;
		
		float abs_distance = fabsf(limited_distances[i]);
		float ratio = (max_distance > 0) ? (abs_distance / max_distance) : 0;
		
		// 计算速度，考虑电机自身速度限制
		float calculated_speed = ratio * global_motor[idx].vel_max;
		
		// 应用速度限制
		float limited_speed = motor_apply_velocity_limit(calculated_speed);
		speed[i] = (uint16_t)limited_speed;
		
		// 更新目标位置
		float target_pos = global_motor[idx].stepper_motor.current_pos + limited_distances[i];
		global_motor[idx].stepper_motor.target_pos = target_pos;
		global_motor[idx].stepper_motor.target_vel = speed[i];
	}
	
	// ==================== 执行同步控制 ====================
	printf("[MOTOR] 同步控制 %d个电机: 最大位移=%.2fmm\n", size, max_distance);
	ZDT_X42_V2_Synchronous_motion(0);

	
	for (int i = 0; i < size; i++)
	{
		uint8_t motor_id = start_addr + i;
		int idx = (motor_id >= MOTOR_ID && motor_id < MOTOR_ID + MOTOR_NUM) ? (motor_id - MOTOR_ID) : 0;
		
		float target_pos = global_motor[idx].stepper_motor.current_pos + limited_distances[i];
		motor_run(motor_id, speed[i], target_pos, true);
	}
	
	ZDT_X42_V2_Synchronous_motion(0);
}
// 基于运动学的多电机控制函数（带限制条件）

void motor_kinematic_control (Kinematic kinematic, uint8_t R, float theta, float phi, float deltaL[])
{

	// ==================== 计算肌腱长度变化 ====================
	kinematic(R, theta, phi, deltaL);
	
	// ==================== 检查计算结果 ====================
	for (int i = 0; i < MOTOR_NUM; i++) {
		if (isnan(deltaL[i]) || isinf(deltaL[i])) {
			printf("[MOTOR] 运动学控制: 电机%d计算结果无效: %.2f\n", i, deltaL[i]);
			deltaL[i] = 0.0f; // 设为0防止错误传播
		}
		
		// 检查变化量是否过大
		if (fabsf(deltaL[i]) > MOTOR_MAX_DISPLACEMENT_MM * 2) {
			printf("[MOTOR] 运动学控制: 电机%d变化量过大: %.2fmm\n", i, deltaL[i]);
			deltaL[i] = (deltaL[i] > 0) ? MOTOR_MAX_DISPLACEMENT_MM : -MOTOR_MAX_DISPLACEMENT_MM;
		}
	}
	
	// ==================== 执行同步控制 ====================
	printf("[MOTOR] 运动学控制: R=%d, theta=%.2f°, phi=%.2f°\n", R, theta, phi);
	for (int i = 0; i < MOTOR_NUM; i++) {
		printf("电机%d: ΔL=%.2fmm\n", i, deltaL[i]);
	}

	motor_sync_control(MOTOR_NUM, MOTOR_ID, deltaL);
}

// 自定义多电机控制函数（带限制条件）
void motor_custom_control(uint8_t count, uint8_t *params)
{
	// ==================== 参数解析和检查 ====================
	// 假设参数格式: [电机数量][电机1地址][速度高][速度低][位移高][位移低]...
	// 实际格式应根据具体协议定义

	uint8_t motor_count = params[0];

	// ==================== 解析并检查每个电机参数 ====================
	float distances[MOTOR_NUM] = {0};
	uint16_t speeds[MOTOR_NUM] = {0};
	uint8_t error_count = 0;
	
	uint8_t param_idx = 1;
	for (int i = 0; i < motor_count; i++) {
		uint8_t motor_addr = params[param_idx++];

		
		// 解析速度 (2字节)
		uint16_t speed = (params[param_idx] << 8) | params[param_idx + 1];
		param_idx += 2;
		
		// 解析位移 (2字节，有符号)
		int16_t displacement_raw = (params[param_idx] << 8) | params[param_idx + 1];
		float displacement = (float)displacement_raw / 100.0f; // 假设单位0.01mm
		param_idx += 2;
		
		// 检查参数有效性
		int idx = motor_addr - MOTOR_ID;
		float current_pos = global_motor[idx].stepper_motor.current_pos;


		
		// 应用限制条件
		float limited_speed = motor_apply_velocity_limit((float)speed);
		float limited_displacement = motor_apply_displacement_limit(displacement, current_pos);
		
		speeds[i] = (uint16_t)limited_speed;
		distances[i] = limited_displacement;

	}


	// ==================== 执行控制 ====================
	
	// 可以使用同步控制或单独控制
	// 这里使用单独控制，因为每个电机可能有不同速度
	param_idx = 1;
	for (int i = 0; i < motor_count; i++) {
		uint8_t motor_addr = params[param_idx];
		param_idx += 5; // 移动到下一个电机参数
		
		int idx = motor_addr - MOTOR_ID;
		if (idx >= 0 && idx < MOTOR_NUM) {
			float target_pos = global_motor[idx].stepper_motor.current_pos + distances[i];
			motor_run(motor_addr, speeds[i], target_pos, false);
		}
	}
}

/**
 * @brief 将步进电机的角度信息转换为位移信息
 * @param motor_index: 电机索引
 * @param angle: 角度信息（单位：度）
 * @return 位移信息（单位：mm）
 */
float motor_angle_to_displacement(uint8_t motor_index, float angle)
{
    if (motor_index >= MOTOR_NUM) {
        return 0.0f;
    }
    
    StepperMotor *stepper = &global_motor[motor_index].stepper_motor;
    
    // 计算每转的步数
    float steps_per_rev = 360.0f / stepper->step_angle * stepper->xifen;
    
    // 计算角度对应的步数
    float steps = angle / 360.0f * steps_per_rev;
    
    // 计算位移：步数 * 导程 / 每转步数
    float displacement = steps * stepper -> daocheng / (360.0f / stepper->step_angle * stepper->xifen);
    
    // 更新电机结构体中的位置信息
    stepper -> current_pos = displacement;
    global_motor[motor_index].current_pos = angle * 180.0f / 3.1415926f; // 转换为弧度并存储
    
    return displacement;
}

/**
 * @brief 将位移信息转换为步进电机的角度信息
 * @param motor_index: 电机索引
 * @param displacement: 位移信息（单位：mm）
 * @return 角度信息（单位：度）
 */
float motor_displacement_to_angle(uint8_t motor_index, float displacement)
{
    if (motor_index >= MOTOR_NUM) {
        return 0.0f;
    }
    
    StepperMotor *stepper = &global_motor[motor_index].stepper_motor;
    
    // 计算每转的步数
    float steps_per_rev = 360.0f / stepper->step_angle * stepper->xifen;
    
    // 计算位移对应的步数
    float steps = displacement * steps_per_rev / stepper->daocheng;
    
    // 计算角度：步数 / 每转步数 * 360度
    float angle = steps / steps_per_rev * 360.0f;
    
    // 更新电机结构体中的位置信息
    stepper->current_pos = displacement;
    global_motor[motor_index].current_pos = angle * 180.0f / 3.1415926f; // 转换为弧度并存储
    
    return angle;
}

// ==================== 新增函数：电机状态定期检查 ====================

/**
 * @brief 电机状态定期检查函数
 * 
 * 应定期调用（如每100ms），检查电机状态和限制条件
 */
void motor_status_check(void)
{
    static uint32_t last_check_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 每100ms检查一次
    if (current_time - last_check_time < 100) {
        return;
    }
    
    last_check_time = current_time;
    
    for (int i = 0; i < MOTOR_NUM; i++) {
        uint8_t motor_id = MOTOR_ID + i;
        
        // 1. 检查位置是否在限位内
        float current_pos = global_motor[i].stepper_motor.current_pos;
        if (!motor_check_position_limit(current_pos)) {
            printf("[MOTOR] 定期检查: 电机%d位置超出限位: %.2fmm\n", motor_id, current_pos);
            motor_error_handler(motor_id, 0x03); // 位置超限错误
        }
        
        // 2. 检查错误计数
        uint8_t error_count = motor_get_error_count(i);
        if (error_count >= MOTOR_ERROR_COUNT_MAX) {
            printf("[MOTOR] 定期检查: 电机%d错误计数过高: %d\n", motor_id, error_count);
            // 可以采取进一步措施，如降低速度限制等
        }
        
        // 3. 检查温度（如果有温度传感器）
        // if (motor_temperature > MOTOR_MAX_TEMPERATURE_C) {
        //     printf("[MOTOR] 电机%d温度过高: %.1f°C\n", motor_id, motor_temperature);
        //     motor_error_handler(motor_id, 0x07); // 温度过高错误
        // }
        
        // 4. 检查运行时间（防止过热）
        // 可以添加运行时间统计和过热保护
        
        // 5. 检查信号质量（通过错误率）
        // 可以统计一段时间内的错误率，判断信号质量
    }
}

/**
 * @brief 电机信号干扰检测函数
 * 
 * 检测可能的信号干扰，如参数突变、无效值等
 */
uint8_t motor_signal_interference_check(uint8_t motor_id, float velocity, float acceleration, float position)
{
    int idx = (motor_id >= MOTOR_ID && motor_id < MOTOR_ID + MOTOR_NUM) ? (motor_id - MOTOR_ID) : 0;
    
    // 1. 检查数值有效性
    if (isnan(velocity) || isinf(velocity) ||
        isnan(acceleration) || isinf(acceleration) ||
        isnan(position) || isinf(position)) {
        return 0x06; // 信号干扰检测
    }
    
    // 2. 检查突变
    uint32_t current_time = HAL_GetTick();
    uint8_t change_error = motor_check_change_with_history(idx, velocity, acceleration, position, current_time);
    if (change_error != MOTOR_LIMIT_OK) {
        return 0x05; // 参数突变错误
    }
    
    // 3. 检查范围（比正常限制更严格，用于干扰检测）
    if (fabsf(velocity) > MOTOR_MAX_VELOCITY_RPM * 1.2f) { // 允许20%余量
        return 0x06; // 信号干扰检测
    }
    
    if (fabsf(acceleration) > MOTOR_MAX_ACCELERATION_MM_S2 * 1.2f) {
        return 0x06; // 信号干扰检测
    }
    
    return 0x00; // 正常
}

/**
 * @brief 喷管弯曲控制函数
 * @param direction 弯曲方向 (0:负方向, 1:正方向)
 * @param angle 弯曲角度 (单位: 度，已乘以100)
 * 
 * 根据弯曲角度和方向，控制多个电机实现喷管弯曲
 */
void motor_bend_control(uint8_t direction, float angle)
{
    printf("[MOTOR] 喷管弯曲控制: 方向=%s, 角度=%.2f度\n", 
           direction ? "正" : "负", angle / 100.0f);
    // 1. 将角度转换为电机位移
    // 这里需要根据实际机械结构计算每个电机需要的位移
    // 假设有4个电机控制喷管弯曲，使用简单的线性模型
    
    float motor_displacements[MOTOR_NUM] = {0};
    
    // 简化模型：角度转换为位移
    // 实际应该根据运动学模型计算
    float base_displacement = angle / 100.0f * 10.0f; // 每度对应10mm位移
    
    // 根据方向分配位移

    
    // 2. 限制位移范围
    for (int i = 0; i < MOTOR_NUM; i++) {
        if (motor_displacements[i] > 100.0f) motor_displacements[i] = 100.0f;
        if (motor_displacements[i] < -100.0f) motor_displacements[i] = -100.0f;
    }
    // 3. 调用多电机同步控制
    motor_sync_control(MOTOR_NUM, MOTOR_ID, motor_displacements);
    

    for (int i = 0; i < MOTOR_NUM; i++) {
        printf("M%d:%.2fmm ", i+1, motor_displacements[i]);
    }
    printf("\n");
}

/**
 * @brief 截面收缩控制函数
 * @param dir 方向
 * @param scale 收缩比例 (单位: 百分比，已乘以100)
 * 
 * 根据收缩比例，控制单个电机实现
 */
void motor_scale_control(uint8_t dir, float scale)
{
    printf("[MOTOR] 截面收缩控制: 比例=%.2f%%\n", scale / 100.0f);
    
    // 1. 将比例转换为电机位移
    // 这里需要根据实际机械结构计算每个电机需要的位移
    // 假设所有电机同步向内移动实现截面收缩
    
    float motor_displacements[MOTOR_NUM] = {0};
    
    // 简化模型：比例转换为位移
    // 50%比例对应0位移，0%和100%对应最大位移
    float base_displacement = (scale / 100.0f - 0.5f) * 20.0f; // 比例转换为位移
	global_motor[3].stepper_motor.target_pos = base_displacement;
	global_motor[3].stepper_motor.current_acc = 3;
    
    // 3. 调用电机运行函数
    motor_run(global_motor[3].id, global_motor[3].stepper_motor.current_vel, global_motor[3].stepper_motor.target_pos, 0);
    

}

/**
 * @brief 完整电机控制函数
 * @param addr 电机地址
 * @param dir 方向 (0:负, 1:正)
 * @param dist 位移
 * @param velocity 速度
 * @param acceleration 加速度
 * 
 * 控制单个电机的完整参数
 */
void motor_full_control(uint8_t idx, uint8_t dir, float dist,  float velocity, float acceleration)
{
    // 更新全局结构体中的目标值
    float displacement = (dir == 0) ? -dist  : dist ;
    
    // 更新目标位置和速度
    global_motor[idx].stepper_motor.target_pos = global_motor[idx].stepper_motor.current_pos + displacement;
    global_motor[idx].stepper_motor.target_vel = velocity;
    global_motor[idx].stepper_motor.current_acc = acceleration;
    
    // 调用电机运行函数
    motor_run(global_motor[idx].id, velocity / 100.0f,
              global_motor[idx].stepper_motor.current_pos + displacement, false);

}
