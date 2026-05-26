#ifndef __SERVO_H
#define __SERVO_H

#include "./BSP/TIMER/gtim.h"

/* 当前工程使用 6 路舵机 PWM */
#define SERVO_NUM           GTIM_SERVO_CH_NUM

/* 标准舵机脉宽范围，单位: us */
#define SERVO_MIN_PULSE_US  500U
#define SERVO_MID_PULSE_US  1500U
#define SERVO_MAX_PULSE_US  2500U

/* 工程里统一按 0~180 度描述舵机角度 */
#define SERVO_MAX_ANGLE     180U

/* 四轴逆解的目标点:
 * x/y/z      : 目标位置，单位 mm
 * pitch_deg  : 末端俯仰角 alpha，单位 度
 */
typedef struct
{
    float x;
    float y;
    float z;
    float pitch_deg;
    uint8_t prefer_wrist_perpendicular;
} arm4dof_target_t;

/* 机械臂参数配置:
 * base_height         : 底座中心到肩关节的高度
 * upper_arm           : 大臂长度
 * forearm             : 小臂长度
 * wrist               : 腕部到末端参考点的长度
 * fixed_joint_deg[2]  : J5/J6 固定角度，当前不参与主逆解
 * angle_offset_deg[]  : 舵机零偏
 * angle_min_deg[]     : 舵机最小允许角度
 * angle_max_deg[]     : 舵机最大允许角度
 * angle_dir[]         : 舵机安装方向，正装填 1，反装填 -1
 */
typedef struct
{
    float base_height;
    float upper_arm;
    float forearm;
    float wrist;
    float fixed_joint_deg[2];
    float angle_offset_deg[SERVO_NUM];
    float angle_min_deg[SERVO_NUM];
    float angle_max_deg[SERVO_NUM];
    int8_t angle_dir[SERVO_NUM];
} arm4dof_config_t;

/* 逆解输出结果:
 * joint_deg[] 保存最终要输出给 6 个舵机的目标角度
 */
typedef struct
{
    float joint_deg[SERVO_NUM];
    float posture_deg;
} arm4dof_solution_t;

/* 联调测试用的摆动范围 */
typedef struct
{
    uint8_t angle_min;   /* 测试到的较小角度 */
    uint8_t angle_mid;   /* 中位角度 */
    uint8_t angle_max;   /* 测试到的较大角度 */
} servo_test_range_t;

/* 初始化舵机模块并默认回中 */
void servo_init(void);

/* 直接设置单个舵机角度 */
void servo_set_angle(uint8_t index, uint8_t angle);

/* 让所有舵机设置到同一个角度 */
void servo_set_all_angle(uint8_t angle);

/* 设置单个舵机浮点角度 */
void servo_set_angle_float(uint8_t index, float angle);

/* 单个舵机缓动到目标角度 */
void servo_move_angle_slow(uint8_t index, float angle, uint16_t step_delay_ms);

/* 一次下发 6 路关节角 */
void servo_set_joint_angles(const float joint_deg[SERVO_NUM]);

/* 6 路舵机一起缓动到目标角度 */
void servo_move_joint_angles_slow(const float joint_deg[SERVO_NUM], uint16_t step_delay_ms);

/* 全部舵机回到 90 度中位 */
void servo_center_all(void);

/* 加载联调时默认的测试角度范围 */
void servo_load_default_test_ranges(servo_test_range_t ranges[SERVO_NUM]);

/* 按给定角度范围测试单个舵机 */
void servo_test_one_with_range(uint8_t index, const servo_test_range_t *range);

/* 依次测试全部舵机 */
void servo_test_all(const servo_test_range_t ranges[SERVO_NUM]);

/* 加载默认机械臂参数 */
void arm4dof_load_default_config(arm4dof_config_t *config);

/* 四轴逆解:
 * 成功返回 1，失败返回 0
 */
uint8_t arm4dof_solve(const arm4dof_config_t *config, const arm4dof_target_t *target, arm4dof_solution_t *solution);

/* 逆解成功后直接驱动舵机到目标点 */
uint8_t arm4dof_move_target(const arm4dof_config_t *config, const arm4dof_target_t *target, arm4dof_solution_t *solution);

#endif
