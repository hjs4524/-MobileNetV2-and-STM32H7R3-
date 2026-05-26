#ifndef __USART_H
#define __USART_H

#include "./SYSTEM/sys/sys.h"
#include <stdio.h>

/* USART1: PC debug port on PB14/PB15 */
#define USART1_TX_GPIO_PORT             GPIOB
#define USART1_TX_GPIO_PIN              GPIO_PIN_14
#define USART1_TX_GPIO_AF               GPIO_AF4_USART1
#define USART1_TX_GPIO_CLK_ENABLE()     do { __HAL_RCC_GPIOB_CLK_ENABLE(); } while(0)
#define USART1_RX_GPIO_PORT             GPIOB
#define USART1_RX_GPIO_PIN              GPIO_PIN_15
#define USART1_RX_GPIO_AF               GPIO_AF4_USART1
#define USART1_RX_GPIO_CLK_ENABLE()     do { __HAL_RCC_GPIOB_CLK_ENABLE(); } while(0)
#define USART1_UX                       USART1
#define USART1_UX_IRQn                  USART1_IRQn
#define USART1_UX_CLK_ENABLE()          do { __HAL_RCC_USART1_CLK_ENABLE(); } while(0)

/* USART2: OpenMV port on PD5/PD6 */
#define USART2_TX_GPIO_PORT             GPIOD
#define USART2_TX_GPIO_PIN              GPIO_PIN_5
#define USART2_TX_GPIO_AF               GPIO_AF7_USART2
#define USART2_TX_GPIO_CLK_ENABLE()     do { __HAL_RCC_GPIOD_CLK_ENABLE(); } while(0)
#define USART2_RX_GPIO_PORT             GPIOD
#define USART2_RX_GPIO_PIN              GPIO_PIN_6
#define USART2_RX_GPIO_AF               GPIO_AF7_USART2
#define USART2_RX_GPIO_CLK_ENABLE()     do { __HAL_RCC_GPIOD_CLK_ENABLE(); } while(0)
#define USART2_UX                       USART2
#define USART2_UX_IRQn                  USART2_IRQn
#define USART2_UX_CLK_ENABLE()          do { __HAL_RCC_USART2_CLK_ENABLE(); } while(0)

#define USART_EN_RX                     1
#define RXBUFFERSIZE                    1
#define USART_REC_LEN                   200

extern UART_HandleTypeDef g_uart1_handle;
extern UART_HandleTypeDef g_uart2_handle;

#if USART_EN_RX
extern uint8_t g_uart2_rx_byte[RXBUFFERSIZE];
extern uint8_t g_openmv_rx_buf[USART_REC_LEN];
extern uint16_t g_openmv_rx_sta;
#endif

void usart_init(uint32_t baudrate);

#endif
