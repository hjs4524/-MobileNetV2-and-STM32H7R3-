/**
 ****************************************************************************************************
 * @file        gtim.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-05-21
 * @brief       六路PWM硬件驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 */

#ifndef __GTIM_H
#define __GTIM_H

#include "./SYSTEM/sys/sys.h"

#define GTIM_SERVO_CH_NUM               6U         /* 当前工程输出6路PWM */
#define GTIM_PWM_TIMER_TICK_HZ          1000000U   /* 1MHz计数基准，1个计数=1us */
#define GTIM_PWM_PERIOD_US              20000U     /* 20ms周期，对应50Hz */

typedef struct
{
    TIM_HandleTypeDef *htim;   /* 所属定时器句柄 */
    uint32_t channel;          /* TIM通道号 */
    GPIO_TypeDef *gpio_port;   /* PWM输出GPIO端口 */
    uint16_t gpio_pin;         /* PWM输出GPIO引脚 */
    uint8_t gpio_af;           /* GPIO复用功能编号 */
} gtim_pwm_channel_t;

extern TIM_HandleTypeDef g_gtim_tim2_handle;
extern TIM_HandleTypeDef g_gtim_tim5_handle;
extern const gtim_pwm_channel_t g_gtim_pwm_channels[GTIM_SERVO_CH_NUM];

void gtim_pwm_init(void);                                       /* 初始化六路PWM硬件，统一输出50Hz */
void gtim_pwm_set_compare(uint8_t index, uint16_t compare);     /* 设置指定通道比较值，compare单位为us */
uint16_t gtim_pwm_get_compare(uint8_t index);                   /* 获取指定通道当前比较值 */

#endif
