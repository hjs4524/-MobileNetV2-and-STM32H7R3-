/**
 ****************************************************************************************************
 * @file        sys.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-05-21
 * @brief       系统初始化代码
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

#ifndef __SYS_H
#define __SYS_H

#include "stm32h7rsxx_hal.h"

/* 功能定义 */
#define SYS_SUPPORT_OS 0    /* SYSTEM的OS支持 */

/* 用于配置PLL */
extern RCC_OscInitTypeDef rcc_osc_init_struct;

/* 内存映射开启标志位 */
extern uint8_t sys_memory_mapped_flag;

/* 函数声明 */
void sys_mpu_config(void);                                                  /* 配置MPU */
void sys_cache_enable(void);                                                /* 使能Cache */
uint8_t sys_stm32_clock_init(uint32_t plln, uint32_t pllm, uint32_t pllp);  /* 配置时钟 */

#endif
