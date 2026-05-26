/**
 ****************************************************************************************************
 * @file        delay.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-05-21
 * @brief       延时功能代码
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

#ifndef __DELAY_H
#define __DELAY_H

#include "./SYSTEM/sys/sys.h"

/* 函数声明 */
void delay_init(uint16_t sysclk);   /* 初始化延时 */
void delay_us(uint32_t nus);        /* 微秒级延时 */
void delay_ms(uint16_t nms);        /* 毫秒级延时 */

#endif
