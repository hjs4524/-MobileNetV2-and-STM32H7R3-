/**
 ****************************************************************************************************
 * @file        gtim.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-05-21
 * @brief       六路PWM硬件驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 */

#include "./BSP/TIMER/gtim.h"

TIM_HandleTypeDef g_gtim_tim2_handle = {0}; /* TIM2负责4路PWM */
TIM_HandleTypeDef g_gtim_tim5_handle = {0}; /* TIM5负责2路PWM */

/* 六路PWM通道与开发板接口固定映射表
 * 这张表只描述“PWM硬件输出”关系，不处理角度语义。
 */
const gtim_pwm_channel_t g_gtim_pwm_channels[GTIM_SERVO_CH_NUM] =
{
    {&g_gtim_tim2_handle, TIM_CHANNEL_1, GPIOA, GPIO_PIN_0,  GPIO_AF1_TIM2},
    {&g_gtim_tim2_handle, TIM_CHANNEL_2, GPIOA, GPIO_PIN_1,  GPIO_AF1_TIM2},
    {&g_gtim_tim2_handle, TIM_CHANNEL_3, GPIOB, GPIO_PIN_10, GPIO_AF1_TIM2},
    {&g_gtim_tim2_handle, TIM_CHANNEL_4, GPIOB, GPIO_PIN_11, GPIO_AF1_TIM2},
    {&g_gtim_tim5_handle, TIM_CHANNEL_3, GPIOA, GPIO_PIN_2,  GPIO_AF2_TIM5},
    {&g_gtim_tim5_handle, TIM_CHANNEL_4, GPIOA, GPIO_PIN_3,  GPIO_AF2_TIM5},
};

static void gtim_pwm_base_init(TIM_HandleTypeDef *htim);
static void gtim_pwm_channel_init(const gtim_pwm_channel_t *pwm_channel);
static uint8_t gtim_is_valid_index(uint8_t index);

static void gtim_pwm_base_init(TIM_HandleTypeDef *htim)
{
    uint32_t psc;

    /* 当前工程按TIM内核时钟300MHz计算。
     * 预分频后得到1MHz计数频率，这样CCR就可以直接按us理解。
     */
    psc = (300000000U / GTIM_PWM_TIMER_TICK_HZ) - 1U;

    htim->Init.Prescaler = psc;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;               /* 向上计数 */
    htim->Init.Period = GTIM_PWM_PERIOD_US - 1U;               /* 20ms周期 */
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;         /* 不额外分频 */
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    HAL_TIM_PWM_Init(htim);
}

static void gtim_pwm_channel_init(const gtim_pwm_channel_t *pwm_channel)
{
    TIM_OC_InitTypeDef tim_oc_struct = {0};

    tim_oc_struct.OCMode = TIM_OCMODE_PWM1;                    /* PWM模式1 */
    tim_oc_struct.Pulse = 1500U;                               /* 默认1.5ms中位脉宽 */
    tim_oc_struct.OCPolarity = TIM_OCPOLARITY_HIGH;            /* 高电平有效 */
    tim_oc_struct.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(pwm_channel->htim, &tim_oc_struct, pwm_channel->channel);
    HAL_TIM_PWM_Start(pwm_channel->htim, pwm_channel->channel);
}

static uint8_t gtim_is_valid_index(uint8_t index)
{
    return (index < GTIM_SERVO_CH_NUM) ? 1U : 0U;
}

void gtim_pwm_init(void)
{
    uint8_t i;

    /* TIM2输出4路，TIM5输出2路 */
    g_gtim_tim2_handle.Instance = TIM2;
    g_gtim_tim5_handle.Instance = TIM5;

    /* 先初始化定时器基准，再逐通道开启PWM输出 */
    gtim_pwm_base_init(&g_gtim_tim2_handle);
    gtim_pwm_base_init(&g_gtim_tim5_handle);

    for (i = 0; i < GTIM_SERVO_CH_NUM; i++)
    {
        gtim_pwm_channel_init(&g_gtim_pwm_channels[i]);
    }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio_init_struct = {0};

    if (htim->Instance == TIM2)
    {
        /* TIM2对应:
         * PA0  -> TIM2_CH1 -> J1_9
         * PA1  -> TIM2_CH2 -> J1_8
         * PB10 -> TIM2_CH3 -> J1_13
         * PB11 -> TIM2_CH4 -> J1_12
         */
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;

        gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        gpio_init_struct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);

        gpio_init_struct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        gpio_init_struct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOB, &gpio_init_struct);
    }
    else if (htim->Instance == TIM5)
    {
        /* TIM5对应:
         * PA2 -> TIM5_CH3 -> J1_19
         * PA3 -> TIM5_CH4 -> J2_49
         */
        __HAL_RCC_TIM5_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        gpio_init_struct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
        gpio_init_struct.Alternate = GPIO_AF2_TIM5;
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);
    }
}

void gtim_pwm_set_compare(uint8_t index, uint16_t compare)
{
    if (!gtim_is_valid_index(index))
    {
        return;
    }

    /* 在1MHz计数基准下，compare数值可直接理解为脉宽us */
    __HAL_TIM_SET_COMPARE(g_gtim_pwm_channels[index].htim, g_gtim_pwm_channels[index].channel, compare);
}

uint16_t gtim_pwm_get_compare(uint8_t index)
{
    if (!gtim_is_valid_index(index))
    {
        return 0U;
    }

    return (uint16_t)__HAL_TIM_GET_COMPARE(g_gtim_pwm_channels[index].htim, g_gtim_pwm_channels[index].channel);
}
