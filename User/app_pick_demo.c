/**
 ****************************************************************************************************
 * @file        app_pick_demo.c
 * @brief       OpenMV 像素坐标 -> 桌面标定 -> 机械臂逆解抓取演示
 ****************************************************************************************************
 */

#include "app_pick_demo.h"
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/LED/led.h"
#include "./BSP/SERVO/servo.h"
#include <stdio.h>
#include <string.h>

/* 机械臂运动参数 */
#define ARM_MOVE_STEP_DELAY_MS           12U
#define GRIPPER_STEP_DELAY_MS            12U
#define ARM_SETTLE_DELAY_MS              100U
#define TARGET_RETRY_DELAY_MS            30U
#define ARM_LINEAR_STEP_MM                8.0f
#define ARM_LINEAR_PITCH_STEP_DEG         6.0f
#define ARM_LINEAR_EASE_MIN_SEGMENTS      4U
#define CALIB_DESK_Y_BIAS                10.0f
#define GRIPPER_OPEN_DEG                125.0f
#define GRIPPER_CLOSE_DEG                10.0f

/* 相机到桌面的二维平面单应性标定参数。
 * 使用 2026-05-25 新标定点重新拟合（QVGA 320x240）：
 *   pixel(137,182) -> desk(175, 20)
 *   pixel(178,163) -> desk(225, 40)
 *   pixel(177,130) -> desk(225, 80)
 *   pixel(211,141) -> desk(265, 65)
 *   pixel(244,160) -> desk(305, 40)
 *   pixel(243,127) -> desk(305, 80)
 */
#define CALIB_H11                        1.09459165f
#define CALIB_H12                       (-0.06556004f)
#define CALIB_H13                       28.97632970f
#define CALIB_H21                       (-0.06443871f)
#define CALIB_H22                       (-1.14794621f)
#define CALIB_H23                      236.75642503f
#define CALIB_H31                       (-0.00018720f)
#define CALIB_H32                       (-0.00011256f)
#define CALIB_H33                        1.00000000f

/* 机械臂工作位姿参数，单位：mm / deg */
#define HOME_X                           220.0f
#define HOME_Y                          (-120.0f)
#define HOME_Z                           140.0f
#define HOME_PITCH_DEG                  (-20.0f)
#define TARGET_Z_ABOVE                   135.0f
#define TARGET_Z_VERTICAL_ABOVE           100.0f
#define TARGET_Z_GRAB                     60.0f
#define PICK_Z_SEARCH_STEP_MM              5.0f
#define PICK_APPROACH_MARGIN_MM           20.0f
#define GRAB_PREWRIST_SERVO_DEG           90.0f
#define GRAB_WRIST_SERVO_DEG              60.0f
#define GRAB_WRIST_LOW_X_DELTA_DEG        30.0f
#define TARGET_PITCH_ABOVE_DEG          (-30.0f)
#define GRIPPER_VERTICAL_DOWN_PITCH_DEG (-90.0f)
#define TARGET_PITCH_GRAB_DEG           GRIPPER_VERTICAL_DOWN_PITCH_DEG

#define DROP_X                           200.0f
#define DROP_Y                          (-120.0f)
#define DROP_Z_ABOVE                     145.0f
#define DROP_Z_RELEASE                   130.0f
#define DROP_PITCH_DEG                  (-25.0f)

#define OPENMV_LINE_BUF_SIZE             64U
#define OPENMV_UART_DEBUG_ONLY           0U
#define AXIS_IDENTIFY_STARTUP_DEMO       0U
#define TARGET_STABLE_COUNT_REQUIRED     5U
#define TARGET_PIXEL_JITTER_TOLERANCE    2
#define TARGET_PIXEL_LOW_X_FULL_COMP    120
#define TARGET_PIXEL_LOW_X_THRESHOLD   185

typedef struct
{
    int16_t x;
    int16_t y;
} pixel_point_t;

typedef struct
{
    float x;
    float y;
} desk_point_t;

typedef struct
{
    float h11;
    float h12;
    float h13;
    float h21;
    float h22;
    float h23;
    float h31;
    float h32;
    float h33;
} handeye_calib_2d_t;

typedef struct
{
    float x;
    float y;
    float z;
    float pitch_deg;
} arm_pose_t;

static const handeye_calib_2d_t g_handeye_calib =
{
    CALIB_H11,
    CALIB_H12,
    CALIB_H13,
    CALIB_H21,
    CALIB_H22,
    CALIB_H23,
    CALIB_H31,
    CALIB_H32,
    CALIB_H33
};

static float g_gripper_angle_deg = 90.0f;
static float g_grab_wrist_servo_deg = GRAB_WRIST_SERVO_DEG;
static arm_pose_t g_arm_last_pose = {0.0f, 0.0f, 0.0f, 0.0f};
static uint8_t g_arm_last_pose_valid = 0U;

/* 运动相关辅助函数 */
static float arm_absf(float value)
{
    return (value >= 0.0f) ? value : (-value);
}

static float arm_maxf(float a, float b)
{
    return (a >= b) ? a : b;
}

static float arm_smoothstep01(float t)
{
    if (t <= 0.0f)
    {
        return 0.0f;
    }

    if (t >= 1.0f)
    {
        return 1.0f;
    }

    return (t * t) * (3.0f - (2.0f * t));
}

static uint16_t arm_div_ceil_u16(float value, float step)
{
    uint16_t count;

    if (step <= 0.0f)
    {
        return 0U;
    }

    if (value <= 0.0f)
    {
        return 0U;
    }

    count = (uint16_t)(value / step);
    if (((float)count * step) + 1e-6f < value)
    {
        count++;
    }

    return count;
}

static void arm_store_last_pose(float x, float y, float z, float pitch_deg)
{
    g_arm_last_pose.x = x;
    g_arm_last_pose.y = y;
    g_arm_last_pose.z = z;
    g_arm_last_pose.pitch_deg = pitch_deg;
    g_arm_last_pose_valid = 1U;
}

static void arm_invalidate_last_pose(void)
{
    g_arm_last_pose_valid = 0U;
}

static float arm_get_grab_wrist_servo_deg_from_pixel(const pixel_point_t *pixel)
{
    float ratio;

    if (pixel == 0)
    {
        return GRAB_WRIST_SERVO_DEG;
    }

    if (pixel->x >= TARGET_PIXEL_LOW_X_THRESHOLD)
    {
        return GRAB_WRIST_SERVO_DEG;
    }

    if (pixel->x <= TARGET_PIXEL_LOW_X_FULL_COMP)
    {
        return GRAB_WRIST_SERVO_DEG - GRAB_WRIST_LOW_X_DELTA_DEG;
    }

    ratio = (float)(TARGET_PIXEL_LOW_X_THRESHOLD - pixel->x) /
            (float)(TARGET_PIXEL_LOW_X_THRESHOLD - TARGET_PIXEL_LOW_X_FULL_COMP);
    return GRAB_WRIST_SERVO_DEG - (GRAB_WRIST_LOW_X_DELTA_DEG * ratio);
}

static uint8_t arm_prepare_pose_solution(const arm4dof_config_t *config,
                                         float x,
                                         float y,
                                         float z,
                                         float pitch_deg,
                                         uint8_t prefer_wrist_perpendicular,
                                         arm4dof_solution_t *solution)
{
    arm4dof_target_t target;

    if ((config == 0) || (solution == 0))
    {
        return 0;
    }

    target.x = x;
    target.y = y;
    target.z = z;
    target.pitch_deg = pitch_deg;
    target.prefer_wrist_perpendicular = prefer_wrist_perpendicular;

    if (arm4dof_solve(config, &target, solution) == 0)
    {
        printf("IK failed: x=%.1f y=%.1f z=%.1f pitch=%.1f\r\n", x, y, z, pitch_deg);
        return 0;
    }

    if (prefer_wrist_perpendicular != 0U)
    {
        solution->joint_deg[3] = GRAB_PREWRIST_SERVO_DEG;
        solution->joint_deg[4] = g_grab_wrist_servo_deg;
        printf("grab axes fixed: joint_deg[3]=%.1f joint_deg[4]=%.1f\r\n",
               GRAB_PREWRIST_SERVO_DEG,
               g_grab_wrist_servo_deg);
    }

    solution->joint_deg[5] = g_gripper_angle_deg;
    return 1;
}

static void arm_execute_solution(const arm4dof_solution_t *solution, uint16_t settle_delay_ms)
{
    if (solution == 0)
    {
        return;
    }

    servo_move_joint_angles_slow(solution->joint_deg, ARM_MOVE_STEP_DELAY_MS);
    if (settle_delay_ms > 0U)
    {
        delay_ms(settle_delay_ms);
    }
}

static void arm_move_to_vertical_up_pose(void)
{
    float joint_deg[SERVO_NUM] =
    {
        90.0f,
        100.0f,
        60.0f,
        90.0f,
        90.0f,
        GRIPPER_OPEN_DEG
    };

    printf("move to vertical-up joint pose\r\n");
    servo_move_joint_angles_slow(joint_deg, ARM_MOVE_STEP_DELAY_MS);
    arm_invalidate_last_pose();
}

static void gripper_open(void)
{
    g_gripper_angle_deg = GRIPPER_OPEN_DEG;
    servo_move_angle_slow(5, g_gripper_angle_deg, GRIPPER_STEP_DELAY_MS);
    delay_ms(500);
}

static void gripper_close(void)
{
    g_gripper_angle_deg = GRIPPER_CLOSE_DEG;
    servo_move_angle_slow(5, g_gripper_angle_deg, GRIPPER_STEP_DELAY_MS);
    delay_ms(500);
}

static void arm_identify_end_axes(void)
{
    float joint_deg[SERVO_NUM] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f};

    printf("axis identify demo start\r\n");

    printf("identify joint_deg[3]\r\n");
    joint_deg[3] = 60.0f;
    servo_move_joint_angles_slow(joint_deg, ARM_MOVE_STEP_DELAY_MS);
    delay_ms(800);
    joint_deg[3] = 120.0f;
    servo_move_joint_angles_slow(joint_deg, ARM_MOVE_STEP_DELAY_MS);
    delay_ms(800);
    joint_deg[3] = 90.0f;
    servo_move_joint_angles_slow(joint_deg, ARM_MOVE_STEP_DELAY_MS);
    delay_ms(800);

    printf("identify joint_deg[4]\r\n");
    joint_deg[4] = 60.0f;
    servo_move_joint_angles_slow(joint_deg, ARM_MOVE_STEP_DELAY_MS);
    delay_ms(800);
    joint_deg[4] = 120.0f;
    servo_move_joint_angles_slow(joint_deg, ARM_MOVE_STEP_DELAY_MS);
    delay_ms(800);
    joint_deg[4] = 90.0f;
    servo_move_joint_angles_slow(joint_deg, ARM_MOVE_STEP_DELAY_MS);
    delay_ms(800);

    printf("identify joint_deg[5]\r\n");
    servo_move_angle_slow(5, GRIPPER_CLOSE_DEG, GRIPPER_STEP_DELAY_MS);
    delay_ms(800);
    servo_move_angle_slow(5, GRIPPER_OPEN_DEG, GRIPPER_STEP_DELAY_MS);
    delay_ms(800);
    g_gripper_angle_deg = GRIPPER_OPEN_DEG;

    printf("axis identify demo end\r\n");
}

static uint8_t arm_move_to_pose_ex(const arm4dof_config_t *config,
                                   float x,
                                   float y,
                                   float z,
                                   float pitch_deg,
                                   uint8_t prefer_wrist_perpendicular)
{
    arm4dof_solution_t solution;

    if (config == 0)
    {
        return 0;
    }

    if (arm_prepare_pose_solution(config, x, y, z, pitch_deg, prefer_wrist_perpendicular, &solution) == 0U)
    {
        return 0;
    }

    printf("IK ok: x=%.1f y=%.1f z=%.1f req_pitch=%.1f solve_posture=%.1f\r\n",
           x, y, z, pitch_deg, solution.posture_deg);
    printf("J=%.1f %.1f %.1f %.1f %.1f %.1f\r\n",
           solution.joint_deg[0], solution.joint_deg[1], solution.joint_deg[2],
           solution.joint_deg[3], solution.joint_deg[4], solution.joint_deg[5]);

    arm_execute_solution(&solution, ARM_SETTLE_DELAY_MS);
    arm_store_last_pose(x, y, z, pitch_deg);
    return 1;
}

static uint8_t arm_move_linear_pose_ex(const arm4dof_config_t *config,
                                       float x,
                                       float y,
                                       float z,
                                       float pitch_deg,
                                       uint8_t prefer_wrist_perpendicular)
{
    arm_pose_t start_pose;
    uint16_t pos_segments;
    uint16_t pitch_segments;
    uint16_t segment_count;
    uint16_t i;

    if (config == 0)
    {
        return 0;
    }

    if (g_arm_last_pose_valid == 0U)
    {
        return arm_move_to_pose_ex(config, x, y, z, pitch_deg, prefer_wrist_perpendicular);
    }

    start_pose = g_arm_last_pose;

    pos_segments = arm_div_ceil_u16(arm_maxf(arm_maxf(arm_absf(x - start_pose.x),
                                                     arm_absf(y - start_pose.y)),
                                            arm_absf(z - start_pose.z)),
                                    ARM_LINEAR_STEP_MM);
    pitch_segments = arm_div_ceil_u16(arm_absf(pitch_deg - start_pose.pitch_deg),
                                      ARM_LINEAR_PITCH_STEP_DEG);
    segment_count = (pos_segments >= pitch_segments) ? pos_segments : pitch_segments;

    if (segment_count < 2U)
    {
        return arm_move_to_pose_ex(config, x, y, z, pitch_deg, prefer_wrist_perpendicular);
    }

    printf("linear move: (%0.1f,%0.1f,%0.1f,%0.1f) -> (%0.1f,%0.1f,%0.1f,%0.1f), segments=%u\r\n",
           start_pose.x, start_pose.y, start_pose.z, start_pose.pitch_deg,
           x, y, z, pitch_deg, segment_count);

    for (i = 1U; i <= segment_count; i++)
    {
        float t_linear = (float)i / (float)segment_count;
        float t = (segment_count >= ARM_LINEAR_EASE_MIN_SEGMENTS) ? arm_smoothstep01(t_linear) : t_linear;
        float waypoint_x = start_pose.x + ((x - start_pose.x) * t);
        float waypoint_y = start_pose.y + ((y - start_pose.y) * t);
        float waypoint_z = start_pose.z + ((z - start_pose.z) * t);
        float waypoint_pitch = start_pose.pitch_deg + ((pitch_deg - start_pose.pitch_deg) * t);
        arm4dof_solution_t solution;

        if (arm_prepare_pose_solution(config,
                                      waypoint_x,
                                      waypoint_y,
                                      waypoint_z,
                                      waypoint_pitch,
                                      prefer_wrist_perpendicular,
                                      &solution) == 0U)
        {
            printf("linear waypoint failed: %u/%u\r\n", i, segment_count);
            return 0;
        }

        arm_execute_solution(&solution, (i == segment_count) ? ARM_SETTLE_DELAY_MS : 0U);
        arm_store_last_pose(waypoint_x, waypoint_y, waypoint_z, waypoint_pitch);
    }

    return 1;
}

static uint8_t arm_move_linear_pose(const arm4dof_config_t *config, float x, float y, float z, float pitch_deg)
{
    return arm_move_linear_pose_ex(config, x, y, z, pitch_deg, 0U);
}

static uint8_t arm_pose_is_reachable(const arm4dof_config_t *config,
                                     float x,
                                     float y,
                                     float z,
                                     float pitch_deg)
{
    arm4dof_target_t target;
    arm4dof_solution_t solution;

    if (config == 0)
    {
        return 0;
    }

    target.x = x;
    target.y = y;
    target.z = z;
    target.pitch_deg = pitch_deg;
    target.prefer_wrist_perpendicular = 0U;

    return arm4dof_solve(config, &target, &solution);
}

static uint8_t arm_select_pick_height(const arm4dof_config_t *config,
                                      const desk_point_t *target,
                                      float *pick_grab_z,
                                      float *pick_approach_z)
{
    float z;

    if ((config == 0) || (target == 0) || (pick_grab_z == 0) || (pick_approach_z == 0))
    {
        return 0;
    }

    for (z = TARGET_Z_GRAB; z <= (TARGET_Z_ABOVE + 0.1f); z += PICK_Z_SEARCH_STEP_MM)
    {
        if ((arm_pose_is_reachable(config,
                                   target->x,
                                   target->y,
                                   z,
                                   TARGET_PITCH_GRAB_DEG) != 0U) &&
            (arm_pose_is_reachable(config,
                                   target->x,
                                   target->y,
                                   ((z + PICK_APPROACH_MARGIN_MM) < TARGET_Z_ABOVE) ? (z + PICK_APPROACH_MARGIN_MM) : TARGET_Z_ABOVE,
                                   TARGET_PITCH_GRAB_DEG) != 0U))
        {
            *pick_grab_z = z;
            *pick_approach_z = ((z + PICK_APPROACH_MARGIN_MM) < TARGET_Z_ABOVE) ? (z + PICK_APPROACH_MARGIN_MM) : TARGET_Z_ABOVE;
            return 1;
        }
    }

    return 0;
}

static uint8_t usart_try_read_line(char *line_buf, uint16_t line_buf_size)
{
    uint16_t data_len;
    uint16_t i;

    if ((line_buf == 0) || (line_buf_size < 2U))
    {
        return 0;
    }

    if ((g_openmv_rx_sta & 0x8000U) == 0U)
    {
        return 0;
    }

    __disable_irq();
    data_len = (uint16_t)(g_openmv_rx_sta & 0x3FFFU);

    if (data_len >= (line_buf_size - 1U))
    {
        data_len = line_buf_size - 1U;
    }

    for (i = 0; i < data_len; i++)
    {
        line_buf[i] = (char)g_openmv_rx_buf[i];
    }

    line_buf[data_len] = '\0';
    g_openmv_rx_sta = 0U;
    __enable_irq();

    return 1;
}

static uint8_t parse_openmv_target(const char *line, pixel_point_t *pixel)
{
    int x;
    int y;

    if ((line == 0) || (pixel == 0))
    {
        return 0;
    }

    if (sscanf(line, "%d,%d", &x, &y) == 2)
    {
        pixel->x = (int16_t)x;
        pixel->y = (int16_t)y;
        return 1;
    }

    if (sscanf(line, "X=%d,Y=%d", &x, &y) == 2)
    {
        pixel->x = (int16_t)x;
        pixel->y = (int16_t)y;
        return 1;
    }

    return 0;
}

static uint8_t openmv_line_is_none(const char *line)
{
    if (line == 0)
    {
        return 0;
    }

    if (strcmp(line, "NONE") == 0)
    {
        return 1;
    }

    return 0;
}

static int16_t pixel_abs_diff(int16_t a, int16_t b)
{
    if (a >= b)
    {
        return (int16_t)(a - b);
    }

    return (int16_t)(b - a);
}

static uint8_t pixel_target_is_stable(const pixel_point_t *current,
                                      pixel_point_t *last,
                                      uint8_t *stable_count)
{
    if ((current == 0) || (last == 0) || (stable_count == 0))
    {
        return 0;
    }

    if ((pixel_abs_diff(current->x, last->x) <= TARGET_PIXEL_JITTER_TOLERANCE) &&
        (pixel_abs_diff(current->y, last->y) <= TARGET_PIXEL_JITTER_TOLERANCE))
    {
        if (*stable_count < 255U)
        {
            (*stable_count)++;
        }
    }
    else
    {
        *stable_count = 1U;
        *last = *current;
        return 0;
    }

    *last = *current;
    if (*stable_count >= TARGET_STABLE_COUNT_REQUIRED)
    {
        return 1;
    }

    return 0;
}

static desk_point_t handeye_pixel_to_desk(const handeye_calib_2d_t *calib, const pixel_point_t *pixel)
{
    desk_point_t desk = {0.0f, 0.0f};
    float denom;

    if ((calib == 0) || (pixel == 0))
    {
        return desk;
    }

    denom = (calib->h31 * (float)pixel->x) +
            (calib->h32 * (float)pixel->y) +
            calib->h33;

    if ((denom > -1e-6f) && (denom < 1e-6f))
    {
        return desk;
    }

    desk.x = ((calib->h11 * (float)pixel->x) +
              (calib->h12 * (float)pixel->y) +
              calib->h13) / denom;
    desk.y = ((calib->h21 * (float)pixel->x) +
              (calib->h22 * (float)pixel->y) +
              calib->h23) / denom + CALIB_DESK_Y_BIAS;
    return desk;
}

static uint8_t arm_pick_at_desk_point(const arm4dof_config_t *config, const desk_point_t *target)
{
    float pick_grab_z;
    float pick_approach_z;

    if ((config == 0) || (target == 0))
    {
        return 0;
    }

    if (arm_select_pick_height(config, target, &pick_grab_z, &pick_approach_z) == 0U)
    {
        printf("pick unreachable: x=%.1f y=%.1f no reachable z in %.1f..%.1f\r\n",
               target->x,
               target->y,
               TARGET_Z_GRAB,
               TARGET_Z_ABOVE);
        return 0;
    }

    if (pick_grab_z > (TARGET_Z_GRAB + 0.1f))
    {
        printf("target low unreachable, use lowest reachable grab_z=%.1f approach_z=%.1f\r\n",
               pick_grab_z, pick_approach_z);
    }
    else
    {
        printf("target lowest grab reachable: z=%.1f approach_z=%.1f\r\n",
               pick_grab_z, pick_approach_z);
    }

    printf("target above\r\n");
    if (arm_move_linear_pose_ex(config, target->x, target->y, TARGET_Z_ABOVE, GRIPPER_VERTICAL_DOWN_PITCH_DEG, 1U) == 0)
    {
        return 0;
    }

    printf("target pick approach\r\n");
    if (arm_move_linear_pose_ex(config, target->x, target->y, pick_approach_z, TARGET_PITCH_GRAB_DEG, 1U) == 0)
    {
        return 0;
    }

    printf("target grab\r\n");
    if (arm_move_linear_pose_ex(config, target->x, target->y, pick_grab_z, TARGET_PITCH_GRAB_DEG, 1U) == 0)
    {
        return 0;
    }

    printf("gripper close\r\n");
    gripper_close();
    LED1_TOGGLE();

    printf("target lift\r\n");
    if (arm_move_linear_pose_ex(config, target->x, target->y, pick_approach_z, TARGET_PITCH_GRAB_DEG, 1U) == 0)
    {
        return 0;
    }

    return 1;
}

static uint8_t arm_place_to_drop_zone(const arm4dof_config_t *config)
{
    if (config == 0)
    {
        return 0;
    }

    printf("drop above\r\n");
    if (arm_move_linear_pose(config, DROP_X, DROP_Y, DROP_Z_ABOVE, DROP_PITCH_DEG) == 0)
    {
        return 0;
    }

    printf("drop release\r\n");
    if (arm_move_linear_pose(config, DROP_X, DROP_Y, DROP_Z_RELEASE, DROP_PITCH_DEG) == 0)
    {
        return 0;
    }

    printf("gripper open\r\n");
    gripper_open();
    LED1_TOGGLE();

    printf("drop lift\r\n");
    if (arm_move_linear_pose(config, DROP_X, DROP_Y, DROP_Z_ABOVE, DROP_PITCH_DEG) == 0)
    {
        return 0;
    }

    return 1;
}

static void arm_apply_default_limits(arm4dof_config_t *arm_cfg)
{
    if (arm_cfg == 0)
    {
        return;
    }

    arm_cfg->fixed_joint_deg[0] = 0.0f;
    arm_cfg->fixed_joint_deg[1] = 0.0f;

    arm_cfg->angle_dir[0] = 1;
    arm_cfg->angle_dir[1] = -1;
    arm_cfg->angle_offset_deg[3] = 90.0f;
    arm_cfg->angle_dir[3] = -1;

    arm_cfg->angle_min_deg[0] = 30.0f;
    arm_cfg->angle_max_deg[0] = 150.0f;
    arm_cfg->angle_min_deg[1] = 35.0f;
    arm_cfg->angle_max_deg[1] = 145.0f;
    arm_cfg->angle_min_deg[2] = 20.0f;
    arm_cfg->angle_max_deg[2] = 180.0f;
    arm_cfg->angle_min_deg[3] = 0.0f;
    arm_cfg->angle_max_deg[3] = 180.0f;
    arm_cfg->angle_min_deg[4] = 70.0f;
    arm_cfg->angle_max_deg[4] = 110.0f;
}

/* 应用流程辅助函数 */
static void app_system_init(arm4dof_config_t *arm_cfg)
{
    sys_mpu_config();
    sys_cache_enable();
    HAL_Init();
    sys_stm32_clock_init(300, 6, 2);
    delay_init(600);
    usart_init(115200);
    led_init();

    servo_init();
    arm4dof_load_default_config(arm_cfg);
    arm_apply_default_limits(arm_cfg);
}

static void app_print_startup_banner(const arm4dof_config_t *arm_cfg)
{
    printf("hand-eye pick demo start\r\n");
    printf("OpenMV line format: cx,cy or X=cx,Y=cy\r\n");
    printf("2D calib homography:\r\n");
    printf("desk_x = (%.6f*px + %.6f*py + %.3f) / (%.6f*px + %.6f*py + %.3f)\r\n",
           g_handeye_calib.h11, g_handeye_calib.h12, g_handeye_calib.h13,
           g_handeye_calib.h31, g_handeye_calib.h32, g_handeye_calib.h33);
    printf("desk_y = (%.6f*px + %.6f*py + %.3f) / (%.6f*px + %.6f*py + %.3f)\r\n",
           g_handeye_calib.h21, g_handeye_calib.h22, g_handeye_calib.h23,
           g_handeye_calib.h31, g_handeye_calib.h32, g_handeye_calib.h33);
    printf("arm model: base_h=%.1f upper=%.1f forearm=%.1f wrist=%.1f\r\n",
           arm_cfg->base_height, arm_cfg->upper_arm, arm_cfg->forearm, arm_cfg->wrist);
    printf("J4 map: servo=%.1f %+d*raw, limit %.1f..%.1f\r\n",
           arm_cfg->angle_offset_deg[3],
           arm_cfg->angle_dir[3],
           arm_cfg->angle_min_deg[3],
           arm_cfg->angle_max_deg[3]);

#if OPENMV_UART_DEBUG_ONLY
    printf("debug mode: only receive OpenMV uart, arm motion disabled\r\n");
#else
    printf("run mode: receive target and execute pick-place\r\n");
#endif
}

static void app_prepare_startup_motion(arm4dof_config_t *arm_cfg)
{
    servo_center_all();
    delay_ms(1500);
    gripper_open();

#if AXIS_IDENTIFY_STARTUP_DEMO
    arm_identify_end_axes();
#endif

#if !OPENMV_UART_DEBUG_ONLY
    if (arm_move_linear_pose(arm_cfg, HOME_X, HOME_Y, HOME_Z, HOME_PITCH_DEG) == 0)
    {
        printf("home pose invalid\r\n");
    }
#endif
}

static void app_reset_for_wait_target_leave(uint8_t *target_lock_active, uint8_t *wait_target_leave)
{
    if (target_lock_active != 0)
    {
        *target_lock_active = 0U;
    }

    if (wait_target_leave != 0)
    {
        *wait_target_leave = 1U;
    }
}

/* OpenMV 输入处理辅助函数 */
static void app_handle_none_target(uint8_t *pixel_stable_count,
                                   uint8_t *target_lock_active,
                                   uint8_t *wait_target_leave)
{
    printf("no target\r\n");
    *pixel_stable_count = 0U;
    *target_lock_active = 0U;
    if (*wait_target_leave != 0U)
    {
        *wait_target_leave = 0U;
        printf("target area cleared, ready for next target\r\n");
    }
}

static uint8_t app_try_lock_target(const pixel_point_t *pixel_point,
                                   pixel_point_t *last_pixel_point,
                                   uint8_t *pixel_stable_count,
                                   uint8_t *target_lock_active)
{
    if (*target_lock_active != 0U)
    {
        return 1U;
    }

    if (*pixel_stable_count == 0U)
    {
        *last_pixel_point = *pixel_point;
        *pixel_stable_count = 1U;
        printf("target tracking: 1/%u\r\n", TARGET_STABLE_COUNT_REQUIRED);
        return 0U;
    }

    if (pixel_target_is_stable(pixel_point, last_pixel_point, pixel_stable_count) == 0U)
    {
        printf("target tracking: %u/%u\r\n", *pixel_stable_count, TARGET_STABLE_COUNT_REQUIRED);
        return 0U;
    }

    *target_lock_active = 1U;
    printf("target locked\r\n");
    return 1U;
}

static uint8_t app_execute_pick_cycle(arm4dof_config_t *arm_cfg,
                                      const desk_point_t *desk_point,
                                      uint8_t *target_lock_active,
                                      uint8_t *wait_target_leave,
                                      uint8_t *pixel_stable_count)
{
#if OPENMV_UART_DEBUG_ONLY
    (void)arm_cfg;
    (void)desk_point;
    (void)target_lock_active;
    (void)wait_target_leave;
    (void)pixel_stable_count;
    LED1_TOGGLE();
    return 1U;
#else
    *pixel_stable_count = 0U;
    gripper_open();

    if (arm_move_linear_pose(arm_cfg, HOME_X, HOME_Y, HOME_Z, HOME_PITCH_DEG) == 0)
    {
        app_reset_for_wait_target_leave(target_lock_active, wait_target_leave);
        return 0U;
    }

    printf("prepare grab pose at home\r\n");
    if (arm_move_linear_pose_ex(arm_cfg, HOME_X, HOME_Y, HOME_Z, GRIPPER_VERTICAL_DOWN_PITCH_DEG, 1U) == 0)
    {
        arm_move_to_vertical_up_pose();
        delay_ms(1000);
        app_reset_for_wait_target_leave(target_lock_active, wait_target_leave);
        return 0U;
    }

    if (arm_pick_at_desk_point(arm_cfg, desk_point) == 0)
    {
        printf("pick failed\r\n");
        arm_move_to_vertical_up_pose();
        delay_ms(1000);
        app_reset_for_wait_target_leave(target_lock_active, wait_target_leave);
        return 0U;
    }

    printf("vertical rise above target\r\n");
    if (arm_move_linear_pose_ex(arm_cfg, desk_point->x, desk_point->y, HOME_Z, GRIPPER_VERTICAL_DOWN_PITCH_DEG, 1U) == 0)
    {
        arm_move_to_vertical_up_pose();
        delay_ms(1000);
        app_reset_for_wait_target_leave(target_lock_active, wait_target_leave);
        return 0U;
    }

    printf("horizontal translate to side\r\n");
    if (arm_move_linear_pose_ex(arm_cfg, HOME_X, HOME_Y, HOME_Z, GRIPPER_VERTICAL_DOWN_PITCH_DEG, 1U) == 0)
    {
        arm_move_to_vertical_up_pose();
        delay_ms(1000);
        app_reset_for_wait_target_leave(target_lock_active, wait_target_leave);
        return 0U;
    }

    printf("release\r\n");
    gripper_open();
    LED1_TOGGLE();

    *target_lock_active = 0U;
    *wait_target_leave = 1U;
    printf("pick-place complete, waiting target leave\r\n");
    return 1U;
#endif
}

void app_pick_demo_run(void)
{
    arm4dof_config_t arm_cfg;
    char openmv_line[OPENMV_LINE_BUF_SIZE];
    pixel_point_t pixel_point;
    pixel_point_t last_pixel_point = {0, 0};
    desk_point_t desk_point;
    uint8_t pixel_stable_count = 0U;
    uint8_t target_lock_active = 0U;
    uint8_t wait_target_leave = 0U;

    app_system_init(&arm_cfg);
    app_print_startup_banner(&arm_cfg);
    app_prepare_startup_motion(&arm_cfg);

    while (1)
    {
        if (usart_try_read_line(openmv_line, sizeof(openmv_line)) == 0)
        {
            delay_ms(TARGET_RETRY_DELAY_MS);
            continue;
        }

        printf("rx: %s\r\n", openmv_line);

        if (openmv_line_is_none(openmv_line) != 0U)
        {
            app_handle_none_target(&pixel_stable_count, &target_lock_active, &wait_target_leave);
            continue;
        }

        if (parse_openmv_target(openmv_line, &pixel_point) == 0)
        {
            printf("invalid target line\r\n");
            continue;
        }

        desk_point = handeye_pixel_to_desk(&g_handeye_calib, &pixel_point);
        g_grab_wrist_servo_deg = arm_get_grab_wrist_servo_deg_from_pixel(&pixel_point);

        printf("pixel=(%d,%d) -> desk=(%.1f,%.1f), grab_wrist=%.1f\r\n",
               pixel_point.x, pixel_point.y, desk_point.x, desk_point.y, g_grab_wrist_servo_deg);

        if (wait_target_leave != 0U)
        {
            printf("busy done, waiting target leave before next pick\r\n");
            continue;
        }

        if (app_try_lock_target(&pixel_point,
                                &last_pixel_point,
                                &pixel_stable_count,
                                &target_lock_active) == 0U)
        {
            continue;
        }

        app_execute_pick_cycle(&arm_cfg,
                               &desk_point,
                               &target_lock_active,
                               &wait_target_leave,
                               &pixel_stable_count);
    }
}
