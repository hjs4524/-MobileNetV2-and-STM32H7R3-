#include "./BSP/SERVO/servo.h"
#include "./SYSTEM/delay/delay.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ARM4DOF_RAD_TO_DEG    (180.0f / (float)M_PI)
#define ARM4DOF_DEG_TO_RAD    ((float)M_PI / 180.0f)

#define SERVO_TEST_SETTLE_MS  1200U
#define SERVO_TEST_SWING_MS   1800U

static float g_servo_current_deg[SERVO_NUM] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f};

static float arm4dof_clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static uint16_t servo_angle_to_pulse(uint8_t angle)
{
    if (angle > SERVO_MAX_ANGLE)
    {
        angle = SERVO_MAX_ANGLE;
    }

    return SERVO_MIN_PULSE_US +
           ((uint32_t)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / SERVO_MAX_ANGLE;
}

void servo_init(void)
{
    gtim_pwm_init();
    servo_center_all();
}

void servo_set_angle(uint8_t index, uint8_t angle)
{
    if (index >= SERVO_NUM)
    {
        return;
    }

    g_servo_current_deg[index] = (float)angle;
    gtim_pwm_set_compare(index, servo_angle_to_pulse(angle));
}

void servo_set_all_angle(uint8_t angle)
{
    uint8_t i;

    for (i = 0; i < SERVO_NUM; i++)
    {
        servo_set_angle(i, angle);
    }
}

void servo_center_all(void)
{
    servo_set_all_angle(90);
}

void servo_set_angle_float(uint8_t index, float angle)
{
    angle = arm4dof_clampf(angle, 0.0f, (float)SERVO_MAX_ANGLE);
    servo_set_angle(index, (uint8_t)(angle + 0.5f));
}

void servo_move_angle_slow(uint8_t index, float angle, uint16_t step_delay_ms)
{
    float target_deg;

    if (index >= SERVO_NUM)
    {
        return;
    }

    target_deg = arm4dof_clampf(angle, 0.0f, (float)SERVO_MAX_ANGLE);

    while (g_servo_current_deg[index] < target_deg - 1.0f)
    {
        g_servo_current_deg[index] += 1.0f;
        servo_set_angle(index, (uint8_t)(g_servo_current_deg[index] + 0.5f));
        delay_ms(step_delay_ms);
    }

    while (g_servo_current_deg[index] > target_deg + 1.0f)
    {
        g_servo_current_deg[index] -= 1.0f;
        servo_set_angle(index, (uint8_t)(g_servo_current_deg[index] + 0.5f));
        delay_ms(step_delay_ms);
    }

    servo_set_angle(index, (uint8_t)(target_deg + 0.5f));
}

void servo_set_joint_angles(const float joint_deg[SERVO_NUM])
{
    uint8_t i;

    for (i = 0; i < SERVO_NUM; i++)
    {
        servo_set_angle_float(i, joint_deg[i]);
    }
}

void servo_move_joint_angles_slow(const float joint_deg[SERVO_NUM], uint16_t step_delay_ms)
{
    uint8_t moving;
    uint8_t i;
    float target_deg[SERVO_NUM];

    if (joint_deg == 0)
    {
        return;
    }

    for (i = 0; i < SERVO_NUM; i++)
    {
        target_deg[i] = arm4dof_clampf(joint_deg[i], 0.0f, (float)SERVO_MAX_ANGLE);
    }

    do
    {
        moving = 0U;

        for (i = 0; i < SERVO_NUM; i++)
        {
            if (g_servo_current_deg[i] < target_deg[i] - 1.0f)
            {
                g_servo_current_deg[i] += 1.0f;
                servo_set_angle(i, (uint8_t)(g_servo_current_deg[i] + 0.5f));
                moving = 1U;
            }
            else if (g_servo_current_deg[i] > target_deg[i] + 1.0f)
            {
                g_servo_current_deg[i] -= 1.0f;
                servo_set_angle(i, (uint8_t)(g_servo_current_deg[i] + 0.5f));
                moving = 1U;
            }
            else if ((uint8_t)(g_servo_current_deg[i] + 0.5f) != (uint8_t)(target_deg[i] + 0.5f))
            {
                g_servo_current_deg[i] = target_deg[i];
                servo_set_angle(i, (uint8_t)(target_deg[i] + 0.5f));
                moving = 1U;
            }
        }

        if (moving != 0U)
        {
            delay_ms(step_delay_ms);
        }
    } while (moving != 0U);
}

void servo_load_default_test_ranges(servo_test_range_t ranges[SERVO_NUM])
{
    uint8_t i;

    if (ranges == 0)
    {
        return;
    }

    for (i = 0; i < SERVO_NUM; i++)
    {
        ranges[i].angle_min = 60;
        ranges[i].angle_mid = 90;
        ranges[i].angle_max = 120;
    }

    ranges[0].angle_min = 30;
    ranges[0].angle_mid = 90;
    ranges[0].angle_max = 90;
}

void servo_test_one_with_range(uint8_t index, const servo_test_range_t *range)
{
    if ((index >= SERVO_NUM) || (range == 0))
    {
        return;
    }

    servo_set_angle(index, range->angle_mid);
    delay_ms(SERVO_TEST_SETTLE_MS);

    servo_set_angle(index, range->angle_min);
    delay_ms(SERVO_TEST_SWING_MS);

    servo_set_angle(index, range->angle_max);
    delay_ms(SERVO_TEST_SWING_MS);

    servo_set_angle(index, range->angle_mid);
    delay_ms(SERVO_TEST_SETTLE_MS);
}

void servo_test_all(const servo_test_range_t ranges[SERVO_NUM])
{
    uint8_t i;

    if (ranges == 0)
    {
        return;
    }

    for (i = 0; i < SERVO_NUM; i++)
    {
        servo_test_one_with_range(i, &ranges[i]);
    }

    servo_center_all();
}

void arm4dof_load_default_config(arm4dof_config_t *config)
{
    uint8_t i;
    static const float default_offset[SERVO_NUM] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f};
    static const float default_min[SERVO_NUM] = {0.0f, 15.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    static const float default_max[SERVO_NUM] = {180.0f, 165.0f, 180.0f, 180.0f, 180.0f, 180.0f};
    static const int8_t default_dir[SERVO_NUM] = {1, 1, 1, 1, 1, 1};

    if (config == 0)
    {
        return;
    }

    config->base_height = 130.0f;
    config->upper_arm = 130.0f;
    config->forearm = 130.0f;
    config->wrist = 80.0f;

    config->fixed_joint_deg[0] = 90.0f;
    config->fixed_joint_deg[1] = 90.0f;

    for (i = 0; i < SERVO_NUM; i++)
    {
        config->angle_offset_deg[i] = default_offset[i];
        config->angle_min_deg[i] = default_min[i];
        config->angle_max_deg[i] = default_max[i];
        config->angle_dir[i] = default_dir[i];
    }
}

uint8_t arm4dof_solve(const arm4dof_config_t *config, const arm4dof_target_t *target, arm4dof_solution_t *solution)
{
    float base_rad;
    float plane_r;
    float len;
    float high;
    float raw_joint_deg[SERVO_NUM];
    float selected_j2_deg = 0.0f;
    float selected_j3_deg = 0.0f;
    float selected_j4_deg = 0.0f;
    float selected_posture_deg = 0.0f;
    float preferred_posture_deg;
    float best_score = 0.0f;
    uint16_t posture_index;
    uint8_t i;
    uint8_t found = 0U;

    if ((config == 0) || (target == 0) || (solution == 0))
    {
        return 0;
    }

    base_rad = atan2f(target->y, target->x);
    plane_r = sqrtf((target->x * target->x) + (target->y * target->y));
    len = plane_r;
    high = target->z;
    preferred_posture_deg = 90.0f - target->pitch_deg;
    preferred_posture_deg = arm4dof_clampf(preferred_posture_deg, 0.0f, 180.0f);

    for (posture_index = 0U; posture_index <= 180U; posture_index++)
    {
        float posture_rad;
        float l;
        float h;
        float cos_j3;
        float sin_j3;
        float j3_rad;
        float k1;
        float k2;
        float cos_j2;
        float sin_j2;
        float j2_rad;
        float j4_rad;
        float raw_j2_deg;
        float raw_j3_deg;
        float raw_j4_deg;
        float mapped_joint_deg[SERVO_NUM];
        float candidate_score;
        float wrist_pref_penalty = 0.0f;

        posture_rad = (float)posture_index * ARM4DOF_DEG_TO_RAD;
        l = len - (config->wrist * sinf(posture_rad));
        h = high - (config->wrist * cosf(posture_rad)) - config->base_height;

        cos_j3 = ((l * l) + (h * h) -
                  (config->upper_arm * config->upper_arm) -
                  (config->forearm * config->forearm)) /
                 (2.0f * config->upper_arm * config->forearm);
        if ((cos_j3 < -1.0f) || (cos_j3 > 1.0f))
        {
            continue;
        }

        cos_j3 = arm4dof_clampf(cos_j3, -1.0f, 1.0f);
        sin_j3 = sqrtf(1.0f - (cos_j3 * cos_j3));
        j3_rad = atan2f(sin_j3, cos_j3);

        k2 = config->forearm * sinf(j3_rad);
        k1 = config->upper_arm + (config->forearm * cosf(j3_rad));
        cos_j2 = ((k2 * l) + (k1 * h)) / ((k1 * k1) + (k2 * k2));
        if ((cos_j2 < -1.0f) || (cos_j2 > 1.0f))
        {
            continue;
        }

        cos_j2 = arm4dof_clampf(cos_j2, -1.0f, 1.0f);
        sin_j2 = sqrtf(1.0f - (cos_j2 * cos_j2));
        j2_rad = atan2f(sin_j2, cos_j2);
        j4_rad = posture_rad - j2_rad - j3_rad;

        raw_j2_deg = j2_rad * ARM4DOF_RAD_TO_DEG;
        raw_j3_deg = j3_rad * ARM4DOF_RAD_TO_DEG;
        raw_j4_deg = j4_rad * ARM4DOF_RAD_TO_DEG;

        if ((raw_j2_deg < 0.0f) || (raw_j2_deg > 180.0f) ||
            (raw_j3_deg < 0.0f) || (raw_j3_deg > 180.0f) ||
            (raw_j4_deg < -90.0f) || (raw_j4_deg > 90.0f))
        {
            continue;
        }

        raw_joint_deg[0] = base_rad * ARM4DOF_RAD_TO_DEG;
        raw_joint_deg[1] = raw_j2_deg;
        raw_joint_deg[2] = raw_j3_deg;
        raw_joint_deg[3] = raw_j4_deg;
        raw_joint_deg[4] = config->fixed_joint_deg[0];
        raw_joint_deg[5] = config->fixed_joint_deg[1];

        for (i = 0; i < SERVO_NUM; i++)
        {
            mapped_joint_deg[i] = config->angle_offset_deg[i] +
                                  ((float)config->angle_dir[i] * raw_joint_deg[i]);

            if ((mapped_joint_deg[i] < config->angle_min_deg[i]) ||
                (mapped_joint_deg[i] > config->angle_max_deg[i]))
            {
                break;
            }
        }

        if (i >= SERVO_NUM)
        {
            if (target->prefer_wrist_perpendicular != 0U)
            {
                wrist_pref_penalty = fabsf(fabsf(raw_j4_deg) - 90.0f);
            }

            /* First follow the requested end-effector posture as closely as possible.
             * Then prefer a more downward / crouched branch for tabletop grasping. */
            candidate_score = -(fabsf((float)posture_index - preferred_posture_deg) * 10000.0f) +
                              (wrist_pref_penalty * -1000.0f) +
                              (raw_j2_deg * 100.0f) -
                              raw_j4_deg;

            if ((found == 0U) || (candidate_score > best_score))
            {
                selected_j2_deg = raw_j2_deg;
                selected_j3_deg = raw_j3_deg;
                selected_j4_deg = raw_j4_deg;
                selected_posture_deg = (float)posture_index;
                best_score = candidate_score;
                found = 1U;
            }
        }
    }

    if (found == 0U)
    {
        return 0;
    }

    if (target->prefer_wrist_perpendicular != 0U)
    {
        selected_j4_deg = (selected_j4_deg >= 0.0f) ? 90.0f : -90.0f;
    }

    raw_joint_deg[0] = base_rad * ARM4DOF_RAD_TO_DEG;
    raw_joint_deg[1] = selected_j2_deg;
    raw_joint_deg[2] = selected_j3_deg;
    raw_joint_deg[3] = selected_j4_deg;
    raw_joint_deg[4] = config->fixed_joint_deg[0];
    raw_joint_deg[5] = config->fixed_joint_deg[1];

    for (i = 0; i < SERVO_NUM; i++)
    {
        solution->joint_deg[i] = config->angle_offset_deg[i] +
                                 ((float)config->angle_dir[i] * raw_joint_deg[i]);

        if ((solution->joint_deg[i] < config->angle_min_deg[i]) ||
            (solution->joint_deg[i] > config->angle_max_deg[i]))
        {
            return 0;
        }
    }

    solution->posture_deg = selected_posture_deg;
    return 1;
}

uint8_t arm4dof_move_target(const arm4dof_config_t *config, const arm4dof_target_t *target, arm4dof_solution_t *solution)
{
    arm4dof_solution_t local_solution;

    if (solution == 0)
    {
        solution = &local_solution;
    }

    if (arm4dof_solve(config, target, solution) == 0)
    {
        return 0;
    }

    servo_set_joint_angles(solution->joint_deg);
    return 1;
}
