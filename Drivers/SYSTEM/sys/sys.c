/**
 ****************************************************************************************************
 * @file        sys.c
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

#include "./SYSTEM/sys/sys.h"

/* 用于配置PLL */
RCC_OscInitTypeDef rcc_osc_init_struct = {0};

/* 内存映射开启标志位 */
uint8_t sys_memory_mapped_flag = 0;

/* 使能XSPI1 NOR Flash内存映射 */
static void sys_xspi1_enable_memmapmode(void);

/**
 * @brief   配置MPU
 * @param   无
 * @retval  无
 */
void sys_mpu_config(void)
{
    MPU_Region_InitTypeDef mpu_region_init_struct = {0};
    uint32_t region_index = 0;
    
    /* 关闭MPU */
    HAL_MPU_Disable();
    
    /* 配置背景域（0x00000000~0xFFFFFFFF，4GB） */
    mpu_region_init_struct.Enable = MPU_REGION_ENABLE;
    mpu_region_init_struct.Number = region_index++;
    mpu_region_init_struct.BaseAddress = 0x00000000;
    mpu_region_init_struct.Size = MPU_REGION_SIZE_4GB;
    mpu_region_init_struct.SubRegionDisable = 0x87;
    mpu_region_init_struct.TypeExtField = MPU_TEX_LEVEL0;
    mpu_region_init_struct.AccessPermission = MPU_REGION_NO_ACCESS;
    mpu_region_init_struct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    mpu_region_init_struct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    mpu_region_init_struct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    mpu_region_init_struct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&mpu_region_init_struct);
    
    /* 配置ITCM对应区域（0x00000000~0x0000FFFF，64KB） */
    mpu_region_init_struct.Enable = MPU_REGION_ENABLE;
    mpu_region_init_struct.Number = region_index++;
    mpu_region_init_struct.BaseAddress = 0x00000000;
    mpu_region_init_struct.Size = MPU_REGION_SIZE_64KB;
    mpu_region_init_struct.SubRegionDisable = 0x00;
    mpu_region_init_struct.TypeExtField = MPU_TEX_LEVEL1;
    mpu_region_init_struct.AccessPermission = MPU_REGION_FULL_ACCESS;
    mpu_region_init_struct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    mpu_region_init_struct.IsShareable = MPU_ACCESS_SHAREABLE;
    mpu_region_init_struct.IsCacheable = MPU_ACCESS_CACHEABLE;
    mpu_region_init_struct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&mpu_region_init_struct);
    
    /* 配置DTCM对应区域（0x20000000~0x2000FFFF，64KB） */
    mpu_region_init_struct.Enable = MPU_REGION_ENABLE;
    mpu_region_init_struct.Number = region_index++;
    mpu_region_init_struct.BaseAddress = 0x20000000;
    mpu_region_init_struct.Size = MPU_REGION_SIZE_64KB;
    mpu_region_init_struct.SubRegionDisable = 0x00;
    mpu_region_init_struct.TypeExtField = MPU_TEX_LEVEL1;
    mpu_region_init_struct.AccessPermission = MPU_REGION_FULL_ACCESS;
    mpu_region_init_struct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    mpu_region_init_struct.IsShareable = MPU_ACCESS_SHAREABLE;
    mpu_region_init_struct.IsCacheable = MPU_ACCESS_CACHEABLE;
    mpu_region_init_struct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&mpu_region_init_struct);
    
    /* 配置AXI-SRAM1~4对应区域（0x24000000~0x24071FFF，456KB） */
    mpu_region_init_struct.Enable = MPU_REGION_ENABLE;
    mpu_region_init_struct.Number = region_index++;
    mpu_region_init_struct.BaseAddress = 0x24000000;
    mpu_region_init_struct.Size = MPU_REGION_SIZE_512KB;
    mpu_region_init_struct.SubRegionDisable = 0x00;
    mpu_region_init_struct.TypeExtField = MPU_TEX_LEVEL1;
    mpu_region_init_struct.AccessPermission = MPU_REGION_FULL_ACCESS;
    mpu_region_init_struct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    mpu_region_init_struct.IsShareable = MPU_ACCESS_SHAREABLE;
    mpu_region_init_struct.IsCacheable = MPU_ACCESS_CACHEABLE;
    mpu_region_init_struct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&mpu_region_init_struct);
    
    /* 配置AHB-SRAM1~2对应区域（0x30000000~0x30007FFF，32KB） */
    mpu_region_init_struct.Enable = MPU_REGION_ENABLE;
    mpu_region_init_struct.Number = region_index++;
    mpu_region_init_struct.BaseAddress = 0x30000000;
    mpu_region_init_struct.Size = MPU_REGION_SIZE_32KB;
    mpu_region_init_struct.SubRegionDisable = 0x00;
    mpu_region_init_struct.TypeExtField = MPU_TEX_LEVEL1;
    mpu_region_init_struct.AccessPermission = MPU_REGION_FULL_ACCESS;
    mpu_region_init_struct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    mpu_region_init_struct.IsShareable = MPU_ACCESS_SHAREABLE;
    mpu_region_init_struct.IsCacheable = MPU_ACCESS_CACHEABLE;
    mpu_region_init_struct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&mpu_region_init_struct);
    
    /* 配置FMC LCD对应区域（0x60000000~0x63FFFFFF，64MB） */
    mpu_region_init_struct.Enable = MPU_REGION_ENABLE;
    mpu_region_init_struct.Number = region_index++;
    mpu_region_init_struct.BaseAddress = 0x60000000;
    mpu_region_init_struct.Size = MPU_REGION_SIZE_64MB;
    mpu_region_init_struct.SubRegionDisable = 0x00;
    mpu_region_init_struct.TypeExtField = MPU_TEX_LEVEL0;
    mpu_region_init_struct.AccessPermission = MPU_REGION_FULL_ACCESS;
    mpu_region_init_struct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    mpu_region_init_struct.IsShareable = MPU_ACCESS_SHAREABLE;
    mpu_region_init_struct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    mpu_region_init_struct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&mpu_region_init_struct);
    
    /* 配置XSPI NOR Flash对应区域（0x90000000~0x91FFFFFF，32MB） */
    mpu_region_init_struct.Enable = MPU_REGION_ENABLE;
    mpu_region_init_struct.Number = region_index++;
    mpu_region_init_struct.BaseAddress = 0x90000000;
    mpu_region_init_struct.Size = MPU_REGION_SIZE_32MB;
    mpu_region_init_struct.SubRegionDisable = 0x00;
    mpu_region_init_struct.TypeExtField = MPU_TEX_LEVEL1;
    mpu_region_init_struct.AccessPermission = MPU_REGION_FULL_ACCESS;
    mpu_region_init_struct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    mpu_region_init_struct.IsShareable = MPU_ACCESS_SHAREABLE;
    mpu_region_init_struct.IsCacheable = MPU_ACCESS_CACHEABLE;
    mpu_region_init_struct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&mpu_region_init_struct);
    
    /* 配置XSPI HyperRAM对应区域（0x70000000~0x71FFFFFF，32MB） */
    mpu_region_init_struct.Enable = MPU_REGION_ENABLE;
    mpu_region_init_struct.Number = region_index++;
    mpu_region_init_struct.BaseAddress = 0x70000000;
    mpu_region_init_struct.Size = MPU_REGION_SIZE_32MB;
    mpu_region_init_struct.SubRegionDisable = 0x00;
    mpu_region_init_struct.TypeExtField = MPU_TEX_LEVEL1;
    mpu_region_init_struct.AccessPermission = MPU_REGION_FULL_ACCESS;
    mpu_region_init_struct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    mpu_region_init_struct.IsShareable = MPU_ACCESS_SHAREABLE;
    mpu_region_init_struct.IsCacheable = MPU_ACCESS_CACHEABLE;
    mpu_region_init_struct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&mpu_region_init_struct);
    
    /* 使能MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * @brief   使能Cache
 * @param   无
 * @retval  无
 */
void sys_cache_enable(void)
{
    SCB_EnableICache();
    SCB_EnableDCache();
}

/**
 * @brief   配置时钟
 * @note    当外部晶振频率为25MHz时，推荐值：plln=240 pllm=5 pllp=2
 * @param   plln: PLL1倍频系数（范围：8~420）
 * @param   pllm: PLL1预分频系数（范围：1~63）
 * @param   pllp: PLL1 P输出预分频系数（范围：2~128（偶数））
 * @retval  配置结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t sys_stm32_clock_init(uint32_t plln, uint32_t pllm, uint32_t pllp)
{
    RCC_ClkInitTypeDef rcc_clk_init_struct = {0};
    
    /* 配置系统电源 */
    if (HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY) != HAL_OK)
    {
        return 1;
    }
    
    /* 使能XSPIM接口 */
    HAL_PWREx_EnableXSPIM1();
    HAL_PWREx_EnableXSPIM2();
    
    /* 开启XSPI1、XSPI2 I/O的高速低压（HSLV）模式
     * ！！！特别注意！！！
     * 1、当对应I/O电源电压高于2.7V时不能开启HSLV，否则可能损坏硬件。
     * 2、仅当Flash选项字节中对应XSPI1_HSLV、XSPI2_HSLV或VDDIO_HSLV被设置为1时，软件上的HSLV使能配置才会生效。
     */
    __HAL_RCC_SBS_CLK_ENABLE();
    HAL_SBS_EnableIOSpeedOptimize(SBS_IO_XSPI1_HSLV);
    HAL_SBS_EnableIOSpeedOptimize(SBS_IO_XSPI2_HSLV);
    
    /* 配置内部调节器输出电压 */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
    {
        return 1;
    }
    
    /* 使能VDD33USB电压电平检测器
     * 注意
     * 使用USB HS、USB FS或GPIOM端口时，需先使能VDD33USB电压电平检测器
     */
    if (HAL_PWREx_EnableUSBVoltageDetector() != HAL_OK)
    {
        return 1;
    }
    
    /* 使能AHB-SRAM时钟 */
    __HAL_RCC_SRAM1_CLK_ENABLE();
    __HAL_RCC_SRAM2_CLK_ENABLE();
    
    /* 配置PLL */
    rcc_osc_init_struct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSI48;
    rcc_osc_init_struct.HSEState = RCC_HSE_ON;
    rcc_osc_init_struct.LSIState = RCC_LSI_ON;
    rcc_osc_init_struct.HSI48State = RCC_HSI48_ON;
    rcc_osc_init_struct.PLL1.PLLState = RCC_PLL_ON;
    rcc_osc_init_struct.PLL1.PLLSource = RCC_PLLSOURCE_HSE;
    rcc_osc_init_struct.PLL1.PLLM = pllm;
    rcc_osc_init_struct.PLL1.PLLN = plln;
    rcc_osc_init_struct.PLL1.PLLP = pllp;
    rcc_osc_init_struct.PLL1.PLLQ = 2;
    rcc_osc_init_struct.PLL1.PLLR = 2;
    rcc_osc_init_struct.PLL1.PLLS = 2;
    rcc_osc_init_struct.PLL1.PLLT = 2;
    rcc_osc_init_struct.PLL1.PLLFractional = 0;
    rcc_osc_init_struct.PLL2.PLLState = RCC_PLL_ON;
    rcc_osc_init_struct.PLL2.PLLSource = RCC_PLLSOURCE_HSE;
    rcc_osc_init_struct.PLL2.PLLM = 6;
    rcc_osc_init_struct.PLL2.PLLN = 300;
    rcc_osc_init_struct.PLL2.PLLP = 2;
    rcc_osc_init_struct.PLL2.PLLQ = 12;
    rcc_osc_init_struct.PLL2.PLLR = 6;
    rcc_osc_init_struct.PLL2.PLLS = 3;
    rcc_osc_init_struct.PLL2.PLLT = 6;
    rcc_osc_init_struct.PLL2.PLLFractional = 0;
    rcc_osc_init_struct.PLL3.PLLState = RCC_PLL_ON;
    rcc_osc_init_struct.PLL3.PLLSource = RCC_PLLSOURCE_HSE;
    rcc_osc_init_struct.PLL3.PLLM = 6;
    rcc_osc_init_struct.PLL3.PLLN = 300;
    rcc_osc_init_struct.PLL3.PLLP = 2;
    rcc_osc_init_struct.PLL3.PLLQ = 50;
    rcc_osc_init_struct.PLL3.PLLR = 2;
    rcc_osc_init_struct.PLL3.PLLS = 12;
    rcc_osc_init_struct.PLL3.PLLT = 2;
    rcc_osc_init_struct.PLL3.PLLFractional = 0;
    if (HAL_RCC_OscConfig(&rcc_osc_init_struct) != HAL_OK)
    {
        return 1;
    }
    
    /* 配置CPU、AHB和APB总线时钟 */
    rcc_clk_init_struct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK4 | RCC_CLOCKTYPE_PCLK5;
    rcc_clk_init_struct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    rcc_clk_init_struct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    rcc_clk_init_struct.AHBCLKDivider = RCC_HCLK_DIV2;
    rcc_clk_init_struct.APB1CLKDivider = RCC_APB1_DIV2;
    rcc_clk_init_struct.APB2CLKDivider = RCC_APB2_DIV2;
    rcc_clk_init_struct.APB4CLKDivider = RCC_APB4_DIV2;
    rcc_clk_init_struct.APB5CLKDivider = RCC_APB5_DIV2;
    if (HAL_RCC_ClockConfig(&rcc_clk_init_struct, FLASH_LATENCY_7) != HAL_OK)
    {
        return 1;
    }
    
    /* 使能XSPI1 NOR Flash内存映射 */
    sys_xspi1_enable_memmapmode();
    sys_memory_mapped_flag = 1;
    
    return 0;
}

typedef enum
{
    NORFLASH_MX25UM25645G = 0x00,
    NORFLASH_W25Q128_DUAL = 0x01,
} norflash_type_t;

static void sys_xspi1_init(XSPI_HandleTypeDef *hxspi, norflash_type_t type);
static HAL_StatusTypeDef sys_mx25um25645g_configure(XSPI_HandleTypeDef *hxspi);
static HAL_StatusTypeDef sys_w25q128_dual_configure(XSPI_HandleTypeDef *hxspi);

/**
 * @brief   使能XSPI1 NOR Flash内存映射
 * @param   无
 * @retval  无
 */
static void sys_xspi1_enable_memmapmode(void)
{
    XSPI_HandleTypeDef hxspi1;
    
    sys_xspi1_init(&hxspi1, NORFLASH_MX25UM25645G);
    if (sys_mx25um25645g_configure(&hxspi1) != HAL_OK)
    {
        sys_xspi1_init(&hxspi1, NORFLASH_W25Q128_DUAL);
        sys_w25q128_dual_configure(&hxspi1);
    }
}

/**
 * @brief   使能XSPI1 NOR Flash内存映射
 * @param   hxspi: XSPI句柄指针
 * @param   type: NOR Flash设备类型
 * @arg     NORFLASH_MX25UM25645G: MX25UM25645G
 * @arg     NORFLASH_W25Q128_DUAL: 双W25Q128
 * @retval  无
 */
static void sys_xspi1_init(XSPI_HandleTypeDef *hxspi, norflash_type_t type)
{
    __IO uint8_t *ptr;
    uint32_t index;
    XSPIM_CfgTypeDef xspim_cfg_struct = {0};
    
    ptr = (__IO uint8_t *)hxspi;
    for (index = 0; index < sizeof(XSPI_HandleTypeDef); index++)
    {
        ptr[index] = 0x00;
    }
    
    hxspi->Instance = XSPI1;
    HAL_XSPI_DeInit(hxspi);
    
    hxspi->Instance = XSPI1;
    hxspi->Init.FifoThresholdByte = 4;
    if (type == NORFLASH_MX25UM25645G)
    {
        hxspi->Init.MemoryMode = HAL_XSPI_SINGLE_MEM;
        hxspi->Init.MemoryType = HAL_XSPI_MEMTYPE_MACRONIX;
    }
    else
    {
        hxspi->Init.MemoryMode = HAL_XSPI_DUAL_MEM;
        hxspi->Init.MemoryType = HAL_XSPI_MEMTYPE_APMEM;
    }
    hxspi->Init.MemorySize = HAL_XSPI_SIZE_256MB;
    hxspi->Init.ChipSelectHighTimeCycle = 1;
    hxspi->Init.FreeRunningClock = HAL_XSPI_FREERUNCLK_DISABLE;
    hxspi->Init.ClockMode = HAL_XSPI_CLOCK_MODE_0;
    hxspi->Init.WrapSize = HAL_XSPI_WRAP_NOT_SUPPORTED;
    hxspi->Init.ClockPrescaler = 4 - 1;
    hxspi->Init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE;
    hxspi->Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_DISABLE;
    hxspi->Init.ChipSelectBoundary = HAL_XSPI_BONDARYOF_NONE;
    hxspi->Init.MaxTran = 0;
    hxspi->Init.Refresh = 0;
    hxspi->Init.MemorySelect = HAL_XSPI_CSSEL_NCS1;
    HAL_XSPI_Init(hxspi);
    
    xspim_cfg_struct.nCSOverride = HAL_XSPI_CSSEL_OVR_NCS1;
    xspim_cfg_struct.IOPort = HAL_XSPIM_IOPORT_1;
    HAL_XSPIM_Config(hxspi, &xspim_cfg_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
}

/**
 * @brief   HAL库XSPI初始化回调函数
 * @param   hxspi: XSPI句柄指针
 * @retval  无
 */
void HAL_XSPI_MspInit(XSPI_HandleTypeDef* hxspi)
{
    RCC_PeriphCLKInitTypeDef rcc_periph_clk_init_struct = {0};
    GPIO_InitTypeDef gpio_init_struct = {0};
    
    if (hxspi->Instance == XSPI1)
    {
        rcc_periph_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_XSPI1;
        rcc_periph_clk_init_struct.Xspi1ClockSelection = RCC_XSPI1CLKSOURCE_PLL2S;
        HAL_RCCEx_PeriphCLKConfig(&rcc_periph_clk_init_struct);
        
        __HAL_RCC_XSPIM_CLK_ENABLE();
        __HAL_RCC_XSPI1_CLK_ENABLE();
        __HAL_RCC_GPIOP_CLK_ENABLE();
        __HAL_RCC_GPIOO_CLK_ENABLE();
        
        gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio_init_struct.Alternate = GPIO_AF9_XSPIM_P1;
        HAL_GPIO_Init(GPIOP, &gpio_init_struct);
        
        gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_4;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio_init_struct.Alternate = GPIO_AF9_XSPIM_P1;
        HAL_GPIO_Init(GPIOO, &gpio_init_struct);
    }
}

/**
 * @brief   HAL库XSPI反初始化回调函数
 * @param   hxspi: XSPI句柄指针
 * @retval  无
 */
void HAL_XSPI_MspDeInit(XSPI_HandleTypeDef* hxspi)
{
    if (hxspi->Instance == XSPI1)
    {
        __HAL_RCC_XSPIM_CLK_DISABLE();
        __HAL_RCC_XSPI1_CLK_DISABLE();
        
        HAL_GPIO_DeInit(GPIOP, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
        HAL_GPIO_DeInit(GPIOO, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_4);
    }
}

/**
 * @brief   NOR Flash复位延时
 * @param   无
 * @retval  无
 */
static void sys_norflash_reset_delay(void)
{
    volatile uint32_t i = 10000;
    
    while (--i);
}

/**
 * @brief   配置MX25UM25645G
 * @param   hxspi: XSPI句柄指针
 * @retval  配置结果
 * @arg     HAL_OK: 配置成功
 * @arg     HAL_ERROR: 配置失败
 */
static HAL_StatusTypeDef sys_mx25um25645g_configure(XSPI_HandleTypeDef *hxspi)
{
#define MX25UM25645G_COMMAND_SPI_RSTEN  0x66UL
#define MX25UM25645G_COMMAND_SPI_RST    0x99UL
#define MX25UM25645G_COMMAND_SPI_RDSR   0x05UL
#define MX25UM25645G_COMMAND_SPI_RDID   0x9FUL
#define MX25UM25645G_COMMAND_SPI_RDCR2  0x71UL
#define MX25UM25645G_COMMAND_SPI_WREN   0x06UL
#define MX25UM25645G_COMMAND_SPI_WRCR2  0x72UL

#define MX25UM25645G_COMMAND_OPI_RSTEN  0x6699UL
#define MX25UM25645G_COMMAND_OPI_RST    0x9966UL
#define MX25UM25645G_COMMAND_OPI_RDCR2  0x718EUL
#define MX25UM25645G_COMMAND_OPI_WREN   0x06F9UL
#define MX25UM25645G_COMMAND_OPI_RDSR   0x05FAUL
#define MX25UM25645G_COMMAND_OPI_PP4B   0x12EDUL
#define MX25UM25645G_COMMAND_OPI_8DTRD  0xEE11UL
    
#define MX25UM25645G_IDENTIFICATION     0x3980C2UL
    
    XSPI_RegularCmdTypeDef xspi_regular_cmd_struct = {0};
    XSPI_AutoPollingTypeDef xspi_auto_polling_struct = {0};
    XSPI_MemoryMappedTypeDef xspi_memory_mapped_struct = {0};
    uint8_t data[4];
    
    if (hxspi == NULL)
    {
        return HAL_ERROR;
    }
    
    /* Software Reset */
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = MX25UM25645G_COMMAND_SPI_RSTEN;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_NONE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_NONE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = MX25UM25645G_COMMAND_SPI_RST;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_NONE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_NONE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    sys_norflash_reset_delay();
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = MX25UM25645G_COMMAND_OPI_RSTEN;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_8_LINES;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_16_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_NONE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_NONE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = MX25UM25645G_COMMAND_OPI_RST;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_8_LINES;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_16_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_NONE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_NONE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    sys_norflash_reset_delay();
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = MX25UM25645G_COMMAND_OPI_RSTEN;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_8_LINES;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_16_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_NONE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_NONE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_ENABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = MX25UM25645G_COMMAND_OPI_RST;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_8_LINES;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_16_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_NONE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_NONE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_ENABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    sys_norflash_reset_delay();
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = MX25UM25645G_COMMAND_SPI_RDSR;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_NONE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_1_LINE;
    xspi_regular_cmd_struct.DataLength = 1;
    xspi_regular_cmd_struct.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_auto_polling_struct.MatchValue = 0UL << 0;
    xspi_auto_polling_struct.MatchMask = 1UL << 0;
    xspi_auto_polling_struct.MatchMode = HAL_XSPI_MATCH_MODE_AND;
    xspi_auto_polling_struct.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
    xspi_auto_polling_struct.IntervalTime = 0x10;
    if (HAL_XSPI_AutoPolling(hxspi, &xspi_auto_polling_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    /* Check Identification */
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = MX25UM25645G_COMMAND_SPI_RDID;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_NONE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_1_LINE;
    xspi_regular_cmd_struct.DataLength = 3;
    xspi_regular_cmd_struct.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_XSPI_Receive(hxspi, data, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (*(uint32_t *)data != MX25UM25645G_IDENTIFICATION)
    {
        return HAL_ERROR;
    }
    
    /* Configure DTR-OPI Mode */
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = MX25UM25645G_COMMAND_SPI_RDCR2;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.Address = 0x00000000;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
    xspi_regular_cmd_struct.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
    xspi_regular_cmd_struct.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_1_LINE;
    xspi_regular_cmd_struct.DataLength = 1;
    xspi_regular_cmd_struct.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_XSPI_Receive(hxspi, data, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = MX25UM25645G_COMMAND_SPI_WREN;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_NONE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_NONE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = MX25UM25645G_COMMAND_SPI_RDSR;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_NONE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_1_LINE;
    xspi_regular_cmd_struct.DataLength = 1;
    xspi_regular_cmd_struct.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_auto_polling_struct.MatchValue = 1UL << 1;
    xspi_auto_polling_struct.MatchMask = 1UL << 1;
    xspi_auto_polling_struct.MatchMode = HAL_XSPI_MATCH_MODE_AND;
    xspi_auto_polling_struct.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
    xspi_auto_polling_struct.IntervalTime = 0x10;
    if (HAL_XSPI_AutoPolling(hxspi, &xspi_auto_polling_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    data[0] &= ~(3UL << 0);
    data[0] |= (2UL << 0);
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = MX25UM25645G_COMMAND_SPI_WRCR2;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.Address = 0x00000000;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
    xspi_regular_cmd_struct.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
    xspi_regular_cmd_struct.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_1_LINE;
    xspi_regular_cmd_struct.DataLength = 1;
    xspi_regular_cmd_struct.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_XSPI_Transmit(hxspi, data, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = MX25UM25645G_COMMAND_OPI_RDCR2;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_8_LINES;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_16_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
    xspi_regular_cmd_struct.Address = 0x00000000;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_8_LINES;
    xspi_regular_cmd_struct.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
    xspi_regular_cmd_struct.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_ENABLE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_8_LINES;
    xspi_regular_cmd_struct.DataLength = 1;
    xspi_regular_cmd_struct.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    xspi_regular_cmd_struct.DummyCycles = 4;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_ENABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_XSPI_Receive(hxspi, data, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if ((data[0] & (3UL << 0)) != (2UL << 0))
    {
        return HAL_ERROR;
    }
    
    /* Configure XSPI Clock to 200MHz */
    if (HAL_XSPI_SetClockPrescaler(hxspi, 2 - 1) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    /* Configure XSPI Memory Mapped */
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = MX25UM25645G_COMMAND_OPI_WREN;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_8_LINES;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_16_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_NONE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_NONE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_ENABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = MX25UM25645G_COMMAND_OPI_RDSR;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_8_LINES;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_16_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
    xspi_regular_cmd_struct.Address = 0x00000000;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_8_LINES;
    xspi_regular_cmd_struct.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
    xspi_regular_cmd_struct.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_ENABLE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_8_LINES;
    xspi_regular_cmd_struct.DataLength = 1;
    xspi_regular_cmd_struct.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    xspi_regular_cmd_struct.DummyCycles = 4;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_ENABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_auto_polling_struct.MatchValue = 1UL << 1;
    xspi_auto_polling_struct.MatchMask = 1UL << 1;
    xspi_auto_polling_struct.MatchMode = HAL_XSPI_MATCH_MODE_AND;
    xspi_auto_polling_struct.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
    xspi_auto_polling_struct.IntervalTime = 0x10;
    if (HAL_XSPI_AutoPolling(hxspi, &xspi_auto_polling_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_READ_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = MX25UM25645G_COMMAND_OPI_8DTRD;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_8_LINES;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_16_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_8_LINES;
    xspi_regular_cmd_struct.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
    xspi_regular_cmd_struct.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_ENABLE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_8_LINES;
    xspi_regular_cmd_struct.DataDTRMode = HAL_XSPI_DATA_DTR_ENABLE;
    xspi_regular_cmd_struct.DummyCycles = 20;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_ENABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_WRITE_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = MX25UM25645G_COMMAND_OPI_PP4B;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_8_LINES;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_16_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_8_LINES;
    xspi_regular_cmd_struct.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
    xspi_regular_cmd_struct.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_ENABLE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_8_LINES;
    xspi_regular_cmd_struct.DataDTRMode = HAL_XSPI_DATA_DTR_ENABLE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_ENABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_memory_mapped_struct.TimeOutActivation = HAL_XSPI_TIMEOUT_COUNTER_ENABLE;
    xspi_memory_mapped_struct.TimeoutPeriodClock = 0;
    if (HAL_XSPI_MemoryMapped(hxspi, &xspi_memory_mapped_struct) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/**
 * @brief   配置双W25Q128
 * @param   hxspi: XSPI句柄指针
 * @retval  配置结果
 * @arg     HAL_OK: 配置成功
 * @arg     HAL_ERROR: 配置失败
 */
static HAL_StatusTypeDef sys_w25q128_dual_configure(XSPI_HandleTypeDef *hxspi)
{
#define W25Q128_DUAL_COMMAND_ENABLE_RESET               0x66UL
#define W25Q128_DUAL_COMMAND_RESET_DEVICE               0x99UL
#define W25Q128_DUAL_COMMAND_MANUFACTURER_DEVICE_ID     0x90UL
#define W25Q128_DUAL_COMMAND_READ_STATUS_REGISTER_2     0x35UL
#define W25Q128_DUAL_COMMAND_WRITE_ENABLE               0x06UL
#define W25Q128_DUAL_COMMAND_READ_STATUS_REGISTER_1     0x05UL
#define W25Q128_DUAL_COMMAND_WRITE_STATUS_REGISTER_2    0x31UL
#define W25Q128_DUAL_COMMAND_FAST_READ_QUAD_IO          0xEBUL
#define W25Q128_DUAL_COMMAND_QUAD_INPUT_PAGE_PROGRAM    0x32UL
    
#define W25Q128_DUAL_MANUFACTURER_DEVICE_ID             0x17EFUL
#define BY25FQ128_DUAL_MANUFACTURER_DEVICE_ID           0x1768UL
    
    XSPI_RegularCmdTypeDef xspi_regular_cmd_struct = {0};
    XSPI_AutoPollingTypeDef xspi_auto_polling_struct = {0};
    XSPI_MemoryMappedTypeDef xspi_memory_mapped_struct = {0};
    uint8_t data[4];
    
    if (hxspi == NULL)
    {
        return HAL_ERROR;
    }
    
    /* Software Reset */
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = W25Q128_DUAL_COMMAND_ENABLE_RESET;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_NONE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_NONE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = W25Q128_DUAL_COMMAND_RESET_DEVICE;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_NONE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_NONE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    sys_norflash_reset_delay();
    
    /* Check Identification */
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = W25Q128_DUAL_COMMAND_MANUFACTURER_DEVICE_ID;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.Address = 0x000000UL;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
    xspi_regular_cmd_struct.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
    xspi_regular_cmd_struct.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_1_LINE;
    xspi_regular_cmd_struct.DataLength = 2 * 2;
    xspi_regular_cmd_struct.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_XSPI_Receive(hxspi, data, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (((((uint16_t)data[0] | (data[2] << 8)) != W25Q128_DUAL_MANUFACTURER_DEVICE_ID) || (((uint16_t)data[1] | (data[3] << 8)) != W25Q128_DUAL_MANUFACTURER_DEVICE_ID)) && (((((uint16_t)data[0] | (data[2] << 8)) != BY25FQ128_DUAL_MANUFACTURER_DEVICE_ID) || (((uint16_t)data[1] | (data[3] << 8)) != BY25FQ128_DUAL_MANUFACTURER_DEVICE_ID))))
    {
        return HAL_ERROR;
    }
    
    /* Configure Quad Mode */
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = W25Q128_DUAL_COMMAND_READ_STATUS_REGISTER_2;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_NONE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_1_LINE;
    xspi_regular_cmd_struct.DataLength = 1 * 2;
    xspi_regular_cmd_struct.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_XSPI_Receive(hxspi, data, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = W25Q128_DUAL_COMMAND_WRITE_ENABLE;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_NONE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_NONE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = W25Q128_DUAL_COMMAND_READ_STATUS_REGISTER_1;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_NONE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_1_LINE;
    xspi_regular_cmd_struct.DataLength = 1 * 2;
    xspi_regular_cmd_struct.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_auto_polling_struct.MatchValue = (1UL << 1) | ((1UL << 1) << 8);
    xspi_auto_polling_struct.MatchMask = (1UL << 1) | ((1UL << 1) << 8);
    xspi_auto_polling_struct.MatchMode = HAL_XSPI_MATCH_MODE_AND;
    xspi_auto_polling_struct.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
    xspi_auto_polling_struct.IntervalTime = 0x10;
    if (HAL_XSPI_AutoPolling(hxspi, &xspi_auto_polling_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    *(uint16_t *)data &= ~((1UL << 1) | ((1UL << 1) << 8));
    *(uint16_t *)data |= (1UL << 1) | ((1UL << 1) << 8);
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = W25Q128_DUAL_COMMAND_WRITE_STATUS_REGISTER_2;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_NONE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_1_LINE;
    xspi_regular_cmd_struct.DataLength = 1 * 2;
    xspi_regular_cmd_struct.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_XSPI_Transmit(hxspi, data, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = W25Q128_DUAL_COMMAND_READ_STATUS_REGISTER_1;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_NONE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_1_LINE;
    xspi_regular_cmd_struct.DataLength = 1 * 2;
    xspi_regular_cmd_struct.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_auto_polling_struct.MatchValue = (0UL << 0) | ((0UL << 0) << 8);
    xspi_auto_polling_struct.MatchMask = (1UL << 0) | ((1UL << 0) << 8);
    xspi_auto_polling_struct.MatchMode = HAL_XSPI_MATCH_MODE_AND;
    xspi_auto_polling_struct.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
    xspi_auto_polling_struct.IntervalTime = 0x10;
    if (HAL_XSPI_AutoPolling(hxspi, &xspi_auto_polling_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = W25Q128_DUAL_COMMAND_READ_STATUS_REGISTER_2;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_NONE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_1_LINE;
    xspi_regular_cmd_struct.DataLength = 1 * 2;
    xspi_regular_cmd_struct.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_XSPI_Receive(hxspi, data, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if ((*(uint16_t *)data & ((1UL << 1) | ((1UL << 1) << 8))) != ((1UL << 1) | ((1UL << 1) << 8)))
    {
        return HAL_ERROR;
    }
    
    /* Configure XSPI Memory Mapped */
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = W25Q128_DUAL_COMMAND_WRITE_ENABLE;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_NONE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_NONE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = W25Q128_DUAL_COMMAND_READ_STATUS_REGISTER_1;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_NONE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_1_LINE;
    xspi_regular_cmd_struct.DataLength = 1 * 2;
    xspi_regular_cmd_struct.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_auto_polling_struct.MatchValue = (1UL << 1) | ((1UL << 1) << 8);
    xspi_auto_polling_struct.MatchMask = (1UL << 1) | ((1UL << 1) << 8);
    xspi_auto_polling_struct.MatchMode = HAL_XSPI_MATCH_MODE_AND;
    xspi_auto_polling_struct.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
    xspi_auto_polling_struct.IntervalTime = 0x10;
    if (HAL_XSPI_AutoPolling(hxspi, &xspi_auto_polling_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_READ_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = W25Q128_DUAL_COMMAND_FAST_READ_QUAD_IO;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_4_LINES;
    xspi_regular_cmd_struct.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
    xspi_regular_cmd_struct.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_4_LINES;
    xspi_regular_cmd_struct.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    xspi_regular_cmd_struct.DummyCycles = 6;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_regular_cmd_struct.OperationType = HAL_XSPI_OPTYPE_WRITE_CFG;
    xspi_regular_cmd_struct.IOSelect = HAL_XSPI_SELECT_IO_7_0;
    xspi_regular_cmd_struct.Instruction = W25Q128_DUAL_COMMAND_QUAD_INPUT_PAGE_PROGRAM;
    xspi_regular_cmd_struct.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    xspi_regular_cmd_struct.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    xspi_regular_cmd_struct.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    xspi_regular_cmd_struct.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
    xspi_regular_cmd_struct.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
    xspi_regular_cmd_struct.AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
    xspi_regular_cmd_struct.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    xspi_regular_cmd_struct.DataMode = HAL_XSPI_DATA_4_LINES;
    xspi_regular_cmd_struct.DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    xspi_regular_cmd_struct.DummyCycles = 0;
    xspi_regular_cmd_struct.DQSMode = HAL_XSPI_DQS_DISABLE;
    if (HAL_XSPI_Command(hxspi, &xspi_regular_cmd_struct, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    xspi_memory_mapped_struct.TimeOutActivation = HAL_XSPI_TIMEOUT_COUNTER_ENABLE;
    xspi_memory_mapped_struct.TimeoutPeriodClock = 0;
    if (HAL_XSPI_MemoryMapped(hxspi, &xspi_memory_mapped_struct) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}
