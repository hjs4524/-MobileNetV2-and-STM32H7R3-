/**
 ****************************************************************************************************
 * @file        led.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-05-21
 * @brief       LED驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 * 
 * 实验平台:正点原子 H7R3开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 * 
 ****************************************************************************************************
 */

#include "./BSP/LED/led.h"

/**
 * @brief   初始化LED
 * @param   无
 * @retval  无
 */
void led_init(void)
{
    GPIO_InitTypeDef gpio_init_struct = {0};
    
    /* 使能GPIO端口时钟 */
    LED0_GPIO_CLK_ENABLE();
    LED1_GPIO_CLK_ENABLE();
    
    /* 配置LED0控制引脚 */
    gpio_init_struct.Pin = LED0_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED0_GPIO_PORT, &gpio_init_struct);
    
    /* 配置LED1控制引脚 */
    gpio_init_struct.Pin = LED1_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED1_GPIO_PORT, &gpio_init_struct);
    
    /* 关闭LED0、LED1 */
    LED0(1);
    LED1(1);
}
