#include "./SYSTEM/usart/usart.h"
#if SYS_SUPPORT_OS
#include "os.h"
#endif

int fputc(int c, FILE *stream)
{
    while (!(USART1_UX->ISR & USART_ISR_TC));
    USART1_UX->TDR = (uint8_t)c;
    return c;
}

UART_HandleTypeDef g_uart1_handle = {0};
UART_HandleTypeDef g_uart2_handle = {0};

#if USART_EN_RX
uint8_t g_uart2_rx_byte[RXBUFFERSIZE];
uint8_t g_openmv_rx_buf[USART_REC_LEN];
uint16_t g_openmv_rx_sta = 0;
#endif

static void usart_basic_init(UART_HandleTypeDef *huart, USART_TypeDef *instance, uint32_t baudrate, uint32_t mode)
{
    huart->Instance = instance;
    huart->Init.BaudRate = baudrate;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.Mode = mode;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;
    huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(huart);
}

void usart_init(uint32_t baudrate)
{
    usart_basic_init(&g_uart1_handle, USART1_UX, baudrate, UART_MODE_TX_RX);
    usart_basic_init(&g_uart2_handle, USART2_UX, baudrate, UART_MODE_TX_RX);

#if USART_EN_RX
    HAL_UART_Receive_IT(&g_uart2_handle, g_uart2_rx_byte, sizeof(g_uart2_rx_byte));
#endif
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    RCC_PeriphCLKInitTypeDef clk = {0};
    GPIO_InitTypeDef gpio = {0};

    if (huart->Instance == USART1_UX)
    {
        clk.PeriphClockSelection = RCC_PERIPHCLK_USART1;
        clk.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
        HAL_RCCEx_PeriphCLKConfig(&clk);

        USART1_UX_CLK_ENABLE();
        USART1_TX_GPIO_CLK_ENABLE();
        USART1_RX_GPIO_CLK_ENABLE();

        gpio.Pin = USART1_TX_GPIO_PIN;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = USART1_TX_GPIO_AF;
        HAL_GPIO_Init(USART1_TX_GPIO_PORT, &gpio);

        gpio.Pin = USART1_RX_GPIO_PIN;
        gpio.Alternate = USART1_RX_GPIO_AF;
        HAL_GPIO_Init(USART1_RX_GPIO_PORT, &gpio);

        HAL_NVIC_SetPriority(USART1_UX_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(USART1_UX_IRQn);
    }
    else if (huart->Instance == USART2_UX)
    {
        clk.PeriphClockSelection = RCC_PERIPHCLK_USART234578;
        clk.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PCLK1;
        HAL_RCCEx_PeriphCLKConfig(&clk);

        USART2_UX_CLK_ENABLE();
        USART2_TX_GPIO_CLK_ENABLE();
        USART2_RX_GPIO_CLK_ENABLE();

        gpio.Pin = USART2_TX_GPIO_PIN;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = USART2_TX_GPIO_AF;
        HAL_GPIO_Init(USART2_TX_GPIO_PORT, &gpio);

        gpio.Pin = USART2_RX_GPIO_PIN;
        gpio.Alternate = USART2_RX_GPIO_AF;
        HAL_GPIO_Init(USART2_RX_GPIO_PORT, &gpio);

        HAL_NVIC_SetPriority(USART2_UX_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(USART2_UX_IRQn);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if USART_EN_RX
    if (huart->Instance == USART2_UX)
    {
        if ((g_openmv_rx_sta & 0x8000U) == 0U)
        {
            if ((g_openmv_rx_sta & 0x4000U) != 0U)
            {
                if (g_uart2_rx_byte[0] != 0x0AU)
                {
                    g_openmv_rx_sta = 0U;
                }
                else
                {
                    g_openmv_rx_sta |= 0x8000U;
                }
            }
            else
            {
                if (g_uart2_rx_byte[0] == 0x0DU)
                {
                    g_openmv_rx_sta |= 0x4000U;
                }
                else
                {
                    g_openmv_rx_buf[g_openmv_rx_sta & 0x3FFFU] = g_uart2_rx_byte[0];
                    g_openmv_rx_sta++;
                    if (g_openmv_rx_sta > (USART_REC_LEN - 1U))
                    {
                        g_openmv_rx_sta = 0U;
                    }
                }
            }
        }

        HAL_UART_Receive_IT(&g_uart2_handle, g_uart2_rx_byte, sizeof(g_uart2_rx_byte));
    }
#else
    (void)huart;
#endif
}

void USART1_IRQHandler(void)
{
#if SYS_SUPPORT_OS
    OSIntEnter();
#endif
    HAL_UART_IRQHandler(&g_uart1_handle);
#if SYS_SUPPORT_OS
    OSIntExit();
#endif
}

void USART2_IRQHandler(void)
{
#if SYS_SUPPORT_OS
    OSIntEnter();
#endif
    HAL_UART_IRQHandler(&g_uart2_handle);
#if SYS_SUPPORT_OS
    OSIntExit();
#endif
}
