/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/06/06
 * Description        : Main program body.
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
//如果使用shell命令行终端的时候请用uart1,如果用于调试打印的话用uart4
#include "ch32v30x.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <rthw.h>
#include "drivers/pin.h"
#include <board.h>
#include <rtdbg.h>
#include <u8g2_port.h>
#include <qpid.h>

#include "USART.h"
#include "KEY.h"
#include "DAC.h"
#include "PWM.h"

/* Global typedef */


/* Global define */
#define WDT_DEVICE_NAME    "wdt"    /* 看门狗设备名称 */
static rt_device_t wdg_dev; /* 看门狗设备句柄 */
/* ADC参考电压 */
#define VREF 3.3
/* 电源电压 */
#define VCC 3.3
/* 补偿校准数据 */
#define V0_COMP 0.880   // 0.0325倍电压档位校准系数（实测10V显示11.37V，校准后=10/11.37）
#define V1_COMP 1.000   // 0.0947倍电压档位
#define V2_COMP 1.000   // 0.6175倍电压档位
#define YIF1_COMP 0.92  // MOS管1電流採樣補償（二次校准：0.94A/1.91A实测拟合，增益已收敛）
#define YIF2_COMP 0.92  // MOS管2電流採樣補償（二次校准，3A点验证一致）
#define YIF1_OFFSET 0.054  // MOS管1电流零点偏移（三次校准：1.04A/2.05A实测拟合，恒定-0.07偏差已修正）
#define YIF2_OFFSET 0.054  // MOS管2电流零点偏移（同YIF1）
#define DAC1_COMP  1.00    // DAC1(VREF)输出补偿系数
#define IREF2_COMP  1.00  // IREF2输出补偿系数
#define DAC2_COMP   1.00   // DAC2(IREF1)输出补偿
/* ADC採樣平均值計算次數 */
#define ADC_count 3
/* 一阶低通滤波器滤波系数*/
#define dPower1 0.5
/* 引脚编号，通过查看驱动文件drv_gpio.c确定 */
#define OLED_I2C_PIN_SCL    22  //PB6
#define OLED_I2C_PIN_SDA    23  //PB7
#define LED2                59  //PD11
#define LED1                60  //PD12
#define MCU_G0              62  //PD14
#define MCU_G1              63  //PD15

/* Global Variable */
u8g2_t u8g2;                    // u8g2的结构体变量
rt_uint16_t AD_Value[4];           //ADC采样数据

// 定义模式页面的枚举变量
enum mode_type
{
    menu = 0, // 菜单
    CC,       // 恒流
    CV,       // 恒压
    CR,       // 恒阻
    CW        // 恒功率
};

volatile uint8_t Eload_Out = 0;                 // 电子负载输出开启/关闭状态
volatile uint8_t mode = menu;                   // 当前模式
volatile uint8_t voltage_dw = 0;                // 电压采样档位，0为0.0325倍，2为0.6175倍，1为0.0947倍
volatile double YVF, YIF1, YIF2, YIF, VBAT;     // 当前电压电流
volatile double ISET, VSET, RSET, PSET;         // 电流、电压、电阻、功率设置值
volatile uint32_t YVF_SUM, YIF1_SUM, YIF2_SUM, VBAT_SUM;  // 电压电流算平均值用的计算总和
volatile uint8_t AVG_count = 0;                 // 电流平均值计算累加计数值
volatile uint8_t YVF_AVG_count = 0;             // 电压平均值计算累加计数值
volatile uint8_t VBAT_count = 0;                // 电池电压平均值计算累加计数值
volatile uint8_t Key_ONOFF = 0;                 // 电子负载开启关闭按钮是否被按下状态

static qpid_t qpid_CC;                          // PID控制数据指针
static qpid_t qpid_CV;                          // PID控制数据指针
static qpid_t qpid_CR;                          // PID控制数据指针
static qpid_t qpid_CW;                          // PID控制数据指针
static double I_SET,V_SET, R_SET, P_SET;

/* IV扫描相关 */
#define IV_POINTS 200                           // IV扫描采样点数
volatile uint8_t IV_scan_flag = 0;             // IV扫描触发标志
volatile uint8_t IV_scanning = 0;              // 扫描进行中标志，暂停thread7串口发送
static uint16_t IV_volt[IV_POINTS];            // 电压数据（×100，单位0.01V）
static uint16_t IV_curr[IV_POINTS];            // 电流数据（×1000，单位0.001A）

/* 电池检测相关 */
volatile uint8_t bat_test_flag = 0;            // 电池检测触发标志


/* 函数声明 */
void OLED_Init(void);
static int IWDG_Init();
static void thread1_sysLED_entry(void *parameter);      //系统状态灯线程(LED1)             优先级30
static void thread2_OLED_entry(void *parameter);
static void thread3_ADC_entry(void *parameter);         //核心:ADC采集,电压电流采集,PID运算     优先级18
static void thread4_HMI_GetDate_entry(void *parameter); //处理串口屏(HMI)输入                                 优先级23
static void thread5_ONOFF_entry(void *parameter);       //处理电子负载输出开关逻辑                          优先级 15
static void thread7_HMI_Display_entry(void *parameter); //刷新串口屏显示数据                                     优先级 25
static void thread8_FAN_entry(void *parameter);         //根据功率自动控制散热风扇                          优先级 26
static void thread9_CWCR_entry(void *parameter);        //恒功率/恒阻模式的后台计算                        优先级19
static void thread10_BlueTooth_entry(void *parameter);  //处理蓝牙模块输入(逻辑同串口屏)      优先级23
static void thread11_ESP32_Push_entry(void *parameter); //定时向ESP32推送JSON数据              优先级27
void CW_mode(void);
void CR_mode(void);
void IV_scan(void);
void bat_test(void);
void key123(void);
void Thread_Init(void);
void SYS_Init(void);
void PID(void);


/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    rt_kprintf("MCU: CH32V307\n");
    rt_kprintf("SysClk: %dHz\n", SystemCoreClock);
    SYS_Init();

    while (1)
    {
        rt_thread_mdelay(2000);
        // 过流和过功率保护
        if (YIF > 10 | YVF * YIF > 200)//功率不是200吗
        {
            if (Eload_Out == 1)         //只有"输出正在开着"时才需要保护
            {
                Key_ONOFF = 1;          //触发"开关切换"
            }
        }
    }
}

/* 系统初始化 */
void SYS_Init(void)
{
    UART_Init();   // 串口3初始化
    HMILCD_Send("page 0");   // 切到启动页 通过串口3发送到串口屏
    Dac_Init();     // 初始化DAC
    PWM_Init();     // 初始化PWM

    Thread_Init();   // 创建线程
    //IWDG_Init();    // 暂时禁用看门狗，排查问题
}

/* 线程初始化 */
void Thread_Init(void)
{
    rt_thread_t tid = NULL; //定义一个线程控制块指针
    /* 创建线程 */
    tid = rt_thread_create("SYS_LED", thread1_sysLED_entry, NULL, 512, 30, 5);
    //创建一个名为SYS_LED的线程，入口函数为thread1_sysLED_entry，参数为NULL，栈大小为256字节，优先级为30，时间片为5个tick
    if (tid != RT_NULL) // 判断线程是否创建成功
    {
        if (rt_thread_startup(tid) == RT_EOK) // 启动线程
            LOG_D("thread sysLED create success");
    }
    else
    {
        LOG_E("thread1 sysLED create failed...");
    }
    /*
     tid = rt_thread_create("OLED_Display", thread2_OLED_entry, NULL, 2048, 25, 30);
     if (tid != RT_NULL)
     {
     if (rt_thread_startup(tid) == RT_EOK) // 启动线程
     LOG_D("thread2 OLED create success");
     }
     else
     {
     LOG_E("thread2_OLED create failed...");
     }*/

    tid = rt_thread_create("ADC", thread3_ADC_entry, NULL, 1536, 18, 30);
    if (tid != RT_NULL)
    {
        if (rt_thread_startup(tid) == RT_EOK) // 启动线程
            LOG_D("thread3 ADC create success");
    }
    else
    {
        LOG_E("thread3 ADC create failed...");
    }

    tid = rt_thread_create("HMI_GetDate", thread4_HMI_GetDate_entry, NULL, 2048, 23, 30);
    if (tid != RT_NULL)
    {
        if (rt_thread_startup(tid) == RT_EOK) // 启动线程
            LOG_D("thread4 HMI_GetDate create success");
    }
    else
    {
        LOG_E("thread4 HMI_GetDate create failed...");
    }

    tid = rt_thread_create("ONOFF", thread5_ONOFF_entry, NULL, 1024, 15, 25);
    if (tid != RT_NULL)
    {
        if (rt_thread_startup(tid) == RT_EOK) // 启动线程
            LOG_D("thread5 ONOFF create success");
    }
    else
    {
        LOG_E("thread5 ONOFF create failed...");
    }
    tid = rt_thread_create("KEY", thread6_KEY_entry, NULL, 512, 20, 20);
    if (tid != RT_NULL)
    {
        if (rt_thread_startup(tid) == RT_EOK) // 启动线程
            LOG_D("thread6 KEY create success");
    }
    else
    {
        LOG_E("thread6 KEY create failed...");
    }

    tid = rt_thread_create("HMI_Display", thread7_HMI_Display_entry, NULL, 2048, 25, 30);
    if (tid != RT_NULL)
    {
        if (rt_thread_startup(tid) == RT_EOK) // 启动线程
            LOG_D("thread7 HMI_Display create success");
    }
    else
    {
        LOG_E("thread7 HMI_Display create failed...");
    }

    tid = rt_thread_create("FAN", thread8_FAN_entry, NULL, 512, 26, 15);
    if (tid != RT_NULL)
    {
        if (rt_thread_startup(tid) == RT_EOK) // 启动线程
            LOG_D("thread8 FAN create success");
    }
    else
    {
        LOG_E("thread8 FAN create failed...");
    }

    tid = rt_thread_create("CWCR", thread9_CWCR_entry, NULL, 2048, 19, 15);
    if (tid != RT_NULL)
    {
        if (rt_thread_startup(tid) == RT_EOK) // 启动线程
            LOG_D("thread9 CWCR create success");
    }
    else
    {
        LOG_E("thread9 CWCR create failed...");
    }

    tid = rt_thread_create("BlueTooth", thread10_BlueTooth_entry, NULL, 2048, 23, 30);
    if (tid != RT_NULL)
    {
        if (rt_thread_startup(tid) == RT_EOK)
            LOG_D("thread10 BlueTooth create success");
    }
    else
    {
        LOG_E("thread10 BlueTooth create failed...");
    }

    tid = rt_thread_create("ESP32Push", thread11_ESP32_Push_entry, NULL, 1024, 27, 20);
    if (tid != RT_NULL)
    {
        if (rt_thread_startup(tid) == RT_EOK)
            LOG_D("thread11 ESP32Push create success");
    }
    else
    {
        LOG_E("thread11 ESP32Push create failed...");
    }

}

/* 一阶低通滤波
 * 返回值：iData 经过一阶滤波后的采样值 */
double lowV1(double com1)
{
    static double iLastData1;    //上一次值
    double iData1;               //本次计算值
    iData1 = (com1 * dPower1) + (1 - dPower1) * iLastData1; //计算
    iLastData1 = iData1;                                     //存贮本次数据
    return iData1;                                         //返回数据
}
double lowV2(double com1)
{
    static double iLastData2;    //上一次值
    double iData1;               //本次计算值
    iData1 = (com1 * dPower1) + (1 - dPower1) * iLastData2; //计算
    iLastData2 = iData1;                                     //存贮本次数据
    return iData1;                                         //返回数据
}
u16 lowV3(u16 com1)
{
    static u16 iLastData3;    //上一次值
    u16 iData1;               //本次计算值
    iData1 = (com1 * dPower1) + (1 - dPower1) * iLastData3; //计算
    iLastData3 = iData1;                                     //存贮本次数据
    return iData1;                                         //返回数据
}
u16 lowV4(u16 com1)
{
    static u16 iLastData3;    //上一次值
    u16 iData1;               //本次计算值
    iData1 = (com1 * 0.1) + (1 - 0.1) * iLastData3; //计算
    iLastData3 = iData1;                                     //存贮本次数据
    return iData1;                                         //返回数据
}

static void idle_hook(void)
{
    /* 在空闲线程的回调函数里喂狗 */
    rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_KEEPALIVE, NULL);
    //rt_kprintf("feed the dog!\n ");
}

static int IWDG_Init()
{
    rt_err_t ret = RT_EOK;
    rt_uint32_t timeout = 1; /* 溢出时间，单位：秒 */
    /* 根据设备名称查找看门狗设备，获取设备句柄 */
    wdg_dev = rt_device_find(WDT_DEVICE_NAME);
    if (!wdg_dev)
    {
        rt_kprintf("find %s failed!\n", WDT_DEVICE_NAME);
        return RT_ERROR;
    }
    /* 初始化设备 */
    rt_device_init(wdg_dev);
    /* 设置看门狗溢出时间 */
    ret = rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &timeout);
    if (ret != RT_EOK)
    {
        rt_kprintf("set %s timeout failed!\n", WDT_DEVICE_NAME);
        return RT_ERROR;
    }
    /* 启动看门狗 */
    ret = rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_START, RT_NULL);
    if (ret != RT_EOK)
    {
        rt_kprintf("start %s failed!\n", WDT_DEVICE_NAME);
        return -RT_ERROR;
    }
    /* 设置空闲线程回调函数 */
    rt_thread_idle_sethook(idle_hook);

    return ret;
}

/* 线程 1 的入口函数,系统运行状态灯闪烁 */
static void thread1_sysLED_entry(void *parameter)
{
    /* LED1引脚为输出模式 */
    rt_pin_mode(LED1, PIN_MODE_OUTPUT);
    /* 默认低电平 */
    rt_pin_write(LED1, PIN_LOW);
    while (1)
    {
        /* 线程 1 采用低优先级运行，一直闪烁LED1 */
        rt_pin_write(LED1, !rt_pin_read(LED1));
        rt_thread_mdelay(500);
    }
}

/* 线程 2 的入口函数,OLED屏显示信息 */
/*static void thread2_OLED_entry(void *parameter)
{
    // Initialization
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_rtthread_hw_i2c, u8x8_gpio_and_delay_rtthread);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);

    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    while (1)
    {
        char String[26];
        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFont(&u8g2, u8g2_font_wqy15_t_chinese3); // 设置中文字符集

        float V0 = AD_Value[0] * VREF / 4096.0;
        sprintf(String, "AD0:%d V:%d.%d%d%d", AD_Value[0], (uint8_t) V0, (uint16_t)(V0 * 10.0) % 10,
                (uint16_t)(V0 * 100.0) % 100 % 10, (uint16_t)(V0 * 1000.0) % 1000 % 100 % 10); // 格式化字符串输出到字符串变量
        u8g2_DrawStr(&u8g2, 0, 15, String);

        float V1 = AD_Value[1] * VREF / 4096.0;
        sprintf(String, "AD1:%d V:%d.%d%d%d", AD_Value[1], (uint8_t) V1, (uint16_t)(V1 * 10.0) % 10,
                (uint16_t)(V1 * 100.0) % 100 % 10, (uint16_t)(V1 * 1000.0) % 1000 % 100 % 10); // 格式化字符串输出到字符串变量
        u8g2_DrawStr(&u8g2, 0, 31, String);

        float V2 = AD_Value[2] * VREF / 4096.0;
        sprintf(String, "AD2:%d V:%d.%d%d%d", AD_Value[2], (uint8_t) V2, (uint16_t)(V2 * 10.0) % 10,
                (uint16_t)(V2 * 100.0) % 100 % 10, (uint16_t)(V2 * 1000.0) % 1000 % 100 % 10); // 格式化字符串输出到字符串变量
        u8g2_DrawStr(&u8g2, 0, 47, String);

        float V3 = AD_Value[3] * VREF / 4096.0;
        sprintf(String, "AD3:%d V:%d.%d%d%d", AD_Value[3], (uint8_t) V3, (uint16_t)(V3 * 10.0) % 10,
                (uint16_t)(V3 * 100.0) % 100 % 10, (uint16_t)(V3 * 1000.0) % 1000 % 100 % 10); // 格式化字符串输出到字符串变量
        u8g2_DrawStr(&u8g2, 0, 63, String);
        u8g2_SendBuffer(&u8g2); // 发送缓冲区数据
        rt_thread_mdelay(100);  // 延时100毫秒
    }
}*/

/* 线程 3 的入口函数,ADC数据处理ADC0是对被测电压(YVF)进行采样ADC1对MOS管1的电流采样(YIF1)ADC2是对MOS管2的电流采样(YIF2)ADC3是对电池电压进行采样(VBAT)*/
static void thread3_ADC_entry(void *parameter)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE); //使能GPIOA时钟和ADC
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);      //启用DMA1时钟

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);       //ADC时钟分配配置，6分频（72Mhz/6=12Mhz），ADC时钟频率不能大于14Mhz

    GPIO_InitTypeDef GPIO_InitStructure = { 0 };                //定义结构体配置GPIO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;    //设置GPIO口
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //GPIO模式为模拟输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5); //配置ADC规则组，在规则组的序列1写入通道0，采样时间55.5个周期
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5); //配置ADC规则组，在规则组的序列2写入通道1，采样时间55.5个周期
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5); //配置ADC规则组，在规则组的序列3写入通道2，采样时间55.5个周期
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_239Cycles5); //配置ADC规则组，在规则组的序列4写入通道3，采样时间55.5个周期

    ADC_InitTypeDef ADC_InitStructure = { 0 };
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  //配置ADC为独立模式
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;        //多通道模式下开启扫描模式
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  //设置开启连续转换模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //设置转换不是由外部触发启动，软件触发启动
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //设置ADC数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 4;                //规则转换的ADC通道的数目
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_Cmd(ADC1, ENABLE);      //使能ADC1

    ADC_ResetCalibration(ADC1); //重置ADC1校准寄存器。

    while (ADC_GetResetCalibrationStatus(ADC1)); //等待复位校准结束

    ADC_StartCalibration(ADC1); //开启AD校准

    while (ADC_GetCalibrationStatus(ADC1));      //等待校准结束

    DMA_DeInit(DMA1_Channel1); //复位DMA控制器
    DMA_InitTypeDef DMA_InitStructure;                      //定义结构体配置DMA
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32) &ADC1->RDATAR; //配置外设地址为ADC数据寄存器地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32) AD_Value;          //配置存储器地址为读取ADC值地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;              //配置数据源为外设，即DMA传输方式为外设到存储器
    DMA_InitStructure.DMA_BufferSize = 4;                           //设置DMA数据缓冲区大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                           //设置DMA外设递增模式关闭
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;         //设置DMA存储器递增模式开启
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //设置外设数据大小为半字，即两个字节
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;         //设置存储器数据大小为半字，即两个字节
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;     //设置DMA模式为循环传输模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; //设置DMA传输通道优先级为高，当使用一 DMA通道时，优先级设置不影响
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;        //因为此DMA传输方式为外设到存储器，因此禁用存储器到存储器传输方式
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);        //初始化DMA1的通道1，ADC1的硬件触发接在DMA1的通道1上，所以必须使用DMA1通道1

    DMA_Cmd(DMA1_Channel1, ENABLE); //启动DMA1通道1
    ADC_DMACmd(ADC1, ENABLE);       // 使能ADC DMA 请求
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);       // 由于没有采用外部触发，所以使用软件触发ADC转换

    qpid_init(&qpid_CC);            // 初始化PID控制数据
    qpid_set_lmt(&qpid_CC, 0, 10);  // 設置PID限值
    qpid_set_ratio(&qpid_CC, 1, 0.001, 0.1);    // 设置控制比率系数

    qpid_init(&qpid_CV);                // 初始化PID控制数据
    qpid_set_lmt(&qpid_CV, 31.0, 100);  // 設置PID限值
    qpid_set_ratio(&qpid_CV, 0.35, 0.005, 0.001);    // 设置控制比率系数

    qpid_init(&qpid_CW);                // 初始化PID控制数据
    qpid_set_lmt(&qpid_CW, 0.1, 200);  // 設置PID限值
    qpid_set_ratio(&qpid_CW, 0.5, 0.003, 0.001);    // 设置控制比率系数

    qpid_init(&qpid_CR);                // 初始化PID控制数据
    qpid_set_lmt(&qpid_CR, 0.1, 1000);  // 設置PID限值
    qpid_set_ratio(&qpid_CR, 0.35, 0.003, 0.001);    // 设置控制比率系数

    // 初始化档位控制引脚，固定档位0（三档切换电路设计有缺陷，Q3/Q4导通时直接拉低PA0）
    rt_pin_mode(MCU_G0, PIN_MODE_OUTPUT);
    rt_pin_mode(MCU_G1, PIN_MODE_OUTPUT);
    rt_pin_write(MCU_G0, PIN_LOW);
    rt_pin_write(MCU_G1, PIN_LOW);
    voltage_dw = 0;
    rt_kprintf("thread3 start\n");

    /*滑动平均滤波:用来对ADC采样值做多次累加取平均值,减少噪声干扰*/
    while (1)
    {
        /*负载电压部分*/
        if (YVF_AVG_count < ADC_count)
        {
            YVF_SUM += AD_Value[0];
            YVF_AVG_count++;
        }
        if (YVF_AVG_count == ADC_count)
        {
            YVF = (double)YVF_SUM / YVF_AVG_count //先计算平均值(3次的均值)
                    * VREF / 4096.0               //转成电压:3.3V/4096(12位ADC)
                    / 0.03219;                    //除以分压比,还原真实电压.分压比校准（Q3/Q4未焊，固定档位0）
            if (YVF < 0.1)
                YVF = 0;
            YVF_AVG_count = 0;
            YVF_SUM = 0;
        }
//        为什么除以0.03219？
//        硬件分压电路把实际电压缩小了，0.03219就是分压比（约1/31），除回去就是真实电压。

        /*电流部分*/
        if (AVG_count < ADC_count)
        {
            YIF1_SUM += AD_Value[1]; // MOS管1电流累加
            YIF2_SUM += AD_Value[2]; // MOS管2电流累加
            AVG_count++;
        }
        if (AVG_count == ADC_count)
        {
            YIF1 = YIF1_SUM / AVG_count //平均值
                    * VREF / 4096.0     //转成电压
                    / 50                //除以运放增益(放大了50倍,还原回去)
                    / 0.01              //采样电阻10毫欧
                    * YIF1_COMP         //乘以校准系数
                    + YIF1_OFFSET;      //加上零点偏移校准
            YIF2 = YIF2_SUM / AVG_count * VREF / 4096.0 / 50 / 0.01 * YIF2_COMP + YIF2_OFFSET;//同上
            YIF = YIF1 + YIF2;
            if (YIF < 0.008)
                YIF = 0;
            AVG_count = 0;
            YIF1_SUM = 0;
            YIF2_SUM = 0;
            PID();
        }
//        电流怎么算出来的？
//        采样电阻10mΩ串在MOS管源极，电流流过产生压降，运放放大50倍后送ADC。
//        所以：真实电流 = ADC电压 ÷ 50 ÷ 0.01
        if (VBAT_count < 5)
        {
            VBAT_SUM += lowV4(AD_Value[3]); // 电池电压采样值累加
            VBAT_count++;
        }
        if (VBAT_count == 5)
        {
            VBAT = VBAT_SUM / VBAT_count * VREF / 4096.0 / 0.3535;
            VBAT_count = 0;
            VBAT_SUM = 0;
        }
        rt_thread_mdelay(5);
    }
}


/* 线程 4 的入口函数，对串口屏发来的数据进行处理 */
static void thread4_HMI_GetDate_entry(void *parameter)
{
    HMILCD_Send("CC.x0.val=0"); // 屏幕显示的电流设定值清零
    HMILCD_Send("CV.x0.val=0"); // 屏幕显示的电压设定值清零
    HMILCD_Send("CR.x0.val=0"); // 屏幕显示的电阻设定值清零
    HMILCD_Send("CW.x0.val=0"); // 屏幕显示的功率设定值清零
    while (1)
    {
        if (Serial3_RxFlag == 1)
        {
            if (Serial3_RxPacket[0] == 0x01) // 当前是主菜单页面
            {
                if (Serial3_RxPacket[1] == 0x10) // 恒流按钮按下
                {
                    HMILCD_Send("page CC"); // 切换到恒流模式页面
                    mode = CC;                    // 设置当前模式恒流模式
                }
                else if (Serial3_RxPacket[1] == 0x11) // 恒压按钮按下
                {
                    HMILCD_Send("page CV"); // 切换到恒压页面
                    mode = CV;                    // 设置当前模式为恒压模式
                }
                else if (Serial3_RxPacket[1] == 0x12) // 恒阻按钮按下
                {
                    HMILCD_Send("page CR"); // 切换到恒阻页面
                    mode = CR;                    // 设置当前模式为恒阻模式
                }
                else if (Serial3_RxPacket[1] == 0x13) // 恒功率按钮按下
                {
                    HMILCD_Send("page CW"); // 切换到恒功率页面
                    mode = CW;                    // 设置当前模式为恒功率模式
                }
                else if (Serial3_RxPacket[1] == 0x14) // 电池检测按钮按下
                {
                    bat_test_flag = 1;
                }
            }
            else if (Serial3_RxPacket[0] == 0x02) // 当前是恒流模式页面
            {
                if (Serial3_RxPacket[1] == 0x10) // 菜单按钮按下
                {
                    HMILCD_Send("CC.t1.txt=\"OFF\"");  // 屏幕右上角标题框显示OFF
                    HMILCD_Send("CC.b1.txt=\"开启\""); // 屏幕右下角按钮显示开启
                    Eload_Out = 0;                           // 负载输出状态设置为关闭
                    DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
                    DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
                    PWM_SetCCR4(0);  // 设置IREF2
                    HMILCD_Send("page menu");          // 切换到菜单页面
                    mode = menu;
                }
                else if (Serial3_RxPacket[1] == 0x11) // 开启按钮按下并且当前负载输出状态为关闭
                {
                    Key_ONOFF = 1;
                }
                else if (Serial3_RxPacket[1] == 0x12) // IV扫描按钮按下
                {
                    if (ISET > 0.01) // 设定电流有效时触发扫描
                        IV_scan_flag = 1;
                }
            }
            else if (Serial3_RxPacket[0] == 0x03) // 当前是恒压模式页面
            {
                if (Serial3_RxPacket[1] == 0x10) // 菜单按钮按下
                {
                    HMILCD_Send("CV.t1.txt=\"OFF\"");
                    HMILCD_Send("CV.b1.txt=\"开启\"");
                    Eload_Out = 0;
                    DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
                    DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
                    PWM_SetCCR4(0);  // 设置IREF2
                    HMILCD_Send("page menu");
                    mode = menu;
                }
                if (Serial3_RxPacket[1] == 0x11) // 开启按钮按下并且当前负载输出状态为关闭
                {
                    Key_ONOFF = 1;
                }
            }
            else if (Serial3_RxPacket[0] == 0x04) // 当前是恒阻模式页面
            {
                if (Serial3_RxPacket[1] == 0x10) // 菜单按钮按下
                {
                    HMILCD_Send("CR.t1.txt=\"OFF\"");
                    HMILCD_Send("CR.b1.txt=\"开启\"");
                    Eload_Out = 0;
                    DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
                    DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
                    PWM_SetCCR4(0);  // 设置IREF2
                    HMILCD_Send("page menu");
                    mode = menu;
                }
                if (Serial3_RxPacket[1] == 0x11) // 开启按钮按下并且当前负载输出状态为关闭
                {
                    Key_ONOFF = 1;
                }
            }
            else if (Serial3_RxPacket[0] == 0x05) // 当前是恒功率模式页面
            {
                if (Serial3_RxPacket[1] == 0x10) // 菜单按钮按下
                {
                    HMILCD_Send("CW.t1.txt=\"OFF\"");
                    HMILCD_Send("CW.b1.txt=\"开启\"");
                    Eload_Out = 0;
                    DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
                    DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
                    PWM_SetCCR4(0);  // 设置IREF2
                    HMILCD_Send("page menu");
                    mode = menu;
                }
                if (Serial3_RxPacket[1] == 0x11) // 开启按钮按下并且当前负载输出状态为关闭
                {
                    Key_ONOFF = 1;
                }
            }
            else if (Serial3_RxPacket[0] == 0x06) // 当前是BAT电池检测页面
            {
                if (Serial3_RxPacket[1] == 0x10) // 返回菜单按钮按下
                {
                    HMILCD_Send("page menu");
                    mode = menu;
                }
            }

            else if (Serial3_RxPacket[0] == 0xAA) // 当前为数字键盘页面
            {
                char *temp = Serial3_RxPacket;
                temp++;                      // 地址自增1
                uint16_t temp2 = atoi(temp); // 字符串转整形   比如("500"->500)
                if (mode == CC)  //恒流模式
                {
                    if (temp2 > 1000)//限幅
                        temp2 = 1000;
                    ISET = temp2 / 100.0; //转换成实际电流 (500->5.00A)
                    HMILCD_Send("CC.x0.val=%d", temp2); //数值回显到屏幕上

                    if (Eload_Out == 1)         //如果负载开启状态，立即计算并设置DAC
                    {
                        DAC_SetChannel1Data(DAC_Align_12b_R, 0);
                        if (ISET <= 2.5)//为什么要分 ISET <= 2.5 和 ISET > 2.5？这是 “双量程自动切换”！
                        {
                            // 设置DAC2输出值，控制恒流，+0.5是为了四舍五入
                            DAC_SetChannel2Data(DAC_Align_12b_R,
                                    (uint16_t)(ISET * 0.01 * 50 * 4096 / VREF * DAC2_COMP + 0.5));
                            PWM_SetCCR4(0);  // 设置IREF2
                        }
                        else
                        {
                            // 设置DAC2输出值，控制恒流，+0.5是为了四舍五入
                            DAC_SetChannel2Data(DAC_Align_12b_R,
                                    (uint16_t)(ISET / 2.0 * 0.01 * 50 * 4096 / VREF * DAC2_COMP + 0.5));
                            PWM_SetCCR4((uint16_t)(ISET / 2.0 * 0.01 * 50 / VCC * 50000 * IREF2_COMP + 0.5)); // 设置IREF2
                        }

                    }

                }
                else if (mode == CV)
                {
                    VSET = temp2 / 100.0;
                    HMILCD_Send("CV.x0.val=%d", temp2);
                    if (Eload_Out == 1)
                    {
                        DAC_SetChannel2Data(DAC_Align_12b_R, 4095); // 设置IREF1
                        PWM_SetCCR4(50000);  // 设置IREF2
                        if (voltage_dw == 0)
                        {
                            DAC_SetChannel1Data(DAC_Align_12b_R,
                                    (uint16_t)(VSET * 0.05225 * 4096 / VREF * DAC1_COMP + 0.5)); // 设置DAC1输出值，控制恒压
                        }
                        else if (voltage_dw == 1)
                        {
                            DAC_SetChannel1Data(DAC_Align_12b_R,
                                    (uint16_t)(VSET * 0.0947 * 4096 / VREF * DAC1_COMP + 0.5)); // 设置DAC1输出值，控制恒压
                        }
                        else if (voltage_dw == 2)
                        {
                            uint16_t vset_pwm = (uint16_t)(VSET * 0.6175 * 4096 / VREF * DAC1_COMP + 0.5);
                            if (vset_pwm > 4065)
                                vset_pwm = 4095;
                            DAC_SetChannel1Data(DAC_Align_12b_R, vset_pwm); // 设置DAC1输出值，控制恒压
                        }
                    }
                }
                else if (mode == CR)
                {
                    RSET = temp2 / 100.0;
                    HMILCD_Send("CR.x0.val=%d", temp2);
                    //CR_mode();
                }
                else if (mode == CW)
                {
                    PSET = temp2 / 100.0;
                    HMILCD_Send("CW.x0.val=%d", temp2);
                    //CW_mode();
                }
            }
            Serial3_RxFlag = 0;
        }
        key123();
        rt_thread_mdelay(35);
    }

}

/*恒功率模式*/
void CW_mode(void)
{
    double Ptemp = PSET / YVF;
    if (Ptemp > 10)
        Ptemp = 10;
    if (Eload_Out == 1)
    {
        if (Ptemp <= 2.5)
        {
            DAC_SetChannel2Data(DAC_Align_12b_R, (uint16_t)(Ptemp * 0.01 * 50 * 4096 / VREF * DAC2_COMP + 0.5));
            PWM_SetCCR4(0);
        }
        else
        {
            DAC_SetChannel2Data(DAC_Align_12b_R, (uint16_t)(Ptemp / 2.0 * 0.01 * 50 * 4096 / VREF * DAC2_COMP + 0.5));
            PWM_SetCCR4((uint16_t)(Ptemp / 2.0 * 0.01 * 50 / VCC * 50000 * IREF2_COMP + 0.5));
        }
    }
    else
    {
        DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
        DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
        PWM_SetCCR4(0);  // 设置IREF2
    }
}

/*恒阻模式*/
void CR_mode(void)
{
    double Rtemp = YVF / RSET;
    if (Rtemp > 10)
        Rtemp = 10;
    if (Eload_Out == 1)
    {
        if (Rtemp <= 2.5)
        {
            DAC_SetChannel2Data(DAC_Align_12b_R, (uint16_t)(Rtemp * 0.01 * 50 * 4096 / VREF * DAC2_COMP + 0.5));
            PWM_SetCCR4(0);
        }
        else
        {
            DAC_SetChannel2Data(DAC_Align_12b_R, (uint16_t)(Rtemp / 2.0 * 0.01 * 50 * 4096 / VREF * DAC2_COMP + 0.5));
            PWM_SetCCR4((uint16_t)(Rtemp / 2.0 * 0.01 * 50 / VCC * 50000 * IREF2_COMP + 0.5));
        }
    }
    else
    {
        DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
        DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
        PWM_SetCCR4(0);  // 设置IREF2
    }
}

/* IV特性曲线扫描
 * 在CC模式下，DAC2从0逐步增加到ISET对应值，每步20ms采集一次电压电流
 * 200个采样点存入数组后通过UART3发送到HMI屏波形控件显示 */
void IV_scan(void)
{
    if (mode != CC || ISET < 0.01)
    {
        rt_kprintf("IV_scan skip: mode=%d ISET=%d\n", mode, (int)(ISET * 100));
        return;
    }
    rt_kprintf("IV_scan start: ISET=%d\n", (int)(ISET * 100));

    // 扫描开始前先把电流降到0，从零开始扫
    DAC_SetChannel1Data(DAC_Align_12b_R, 0);
    DAC_SetChannel2Data(DAC_Align_12b_R, 0);
    PWM_SetCCR4(0);
    rt_thread_mdelay(100); // 等待稳定
    uint16_t dac2_max;
    uint32_t pwm_max;
    if (ISET <= 2.5)
    {
        dac2_max = (uint16_t)(ISET * 0.01 * 50 * 4096 / VREF * DAC2_COMP + 0.5);
        pwm_max = 0;
    }
    else
    {
        dac2_max = (uint16_t)(ISET / 2.0 * 0.01 * 50 * 4096 / VREF * DAC2_COMP + 0.5);
        pwm_max = (uint32_t)(ISET / 2.0 * 0.01 * 50 / VCC * 50000 * IREF2_COMP + 0.5);
    }

    // 逐步扫描，采集200个点
    for (int i = 0; i < IV_POINTS; i++)
    {
        uint16_t dac2_val = (uint16_t)((uint32_t)dac2_max * (i + 1) / IV_POINTS);
        uint32_t pwm_val  = (uint32_t)((uint64_t)pwm_max  * (i + 1) / IV_POINTS);
        DAC_SetChannel2Data(DAC_Align_12b_R, dac2_val);
        if (ISET > 2.5)
            PWM_SetCCR4(pwm_val);
        rt_thread_mdelay(20); // 等待电路稳定
        IV_volt[i] = (uint16_t)(YVF * 100 + 0.5); // 电压×100存储
        IV_curr[i] = (uint16_t)(YIF * 1000 + 0.5); // 电流×1000存储
    }

    // 扫描完成，关闭输出
    DAC_SetChannel1Data(DAC_Align_12b_R, 4095);
    DAC_SetChannel2Data(DAC_Align_12b_R, 0);
    PWM_SetCCR4(0);

    // 打印前5个和后5个采样点，确认数据正确
    rt_kprintf("volt[0]=%d volt[99]=%d volt[199]=%d\n", IV_volt[0], IV_volt[99], IV_volt[199]);
    rt_kprintf("curr[0]=%d curr[99]=%d curr[199]=%d\n", IV_curr[0], IV_curr[99], IV_curr[199]);

    // 切换到IV曲线页面
    HMILCD_Send("page IV");
    rt_thread_mdelay(500);

    // 绘图区域：左上(50,10) 右下(420,250)，宽370，高240
    // X轴 = 电流(0~ISET)，Y轴 = 电压(自动量程)，原点在左下角(50,250)
    #define IV_X0  50
    #define IV_Y0  250
    #define IV_W   370
    #define IV_H   240

    // 计算电压实际范围，自动量程
    uint16_t volt_min = IV_volt[0], volt_max_v = IV_volt[0];
    for (int i = 1; i < IV_POINTS; i++)
    {
        if (IV_volt[i] < volt_min) volt_min = IV_volt[i];
        if (IV_volt[i] > volt_max_v) volt_max_v = IV_volt[i];
    }
    // 上下各留10%余量，避免曲线贴边
    uint16_t volt_range = volt_max_v - volt_min;
    if (volt_range < 10) volt_range = 10; // 最小10（0.1V），防止除零
    uint16_t v_bottom = (volt_min > volt_range/5) ? (volt_min - volt_range/5) : 0;
    uint16_t v_top    = volt_max_v + volt_range/5;

    // 画坐标轴（白色=65535）
    HMILCD_Send("line %d,%d,%d,%d,65535", IV_X0, IV_Y0, IV_X0+IV_W, IV_Y0);
    rt_thread_mdelay(10);
    HMILCD_Send("line %d,%d,%d,%d,65535", IV_X0, IV_Y0, IV_X0, IV_Y0-IV_H);
    rt_thread_mdelay(10);

    // 画IV曲线（黄色=65504），加3点均值滤波减少锯齿
    uint16_t px_prev = 0, py_prev = 0;
    for (int i = 0; i < IV_POINTS; i++)
    {
        // 5点均值滤波减少锯齿
        uint32_t v_avg = 0;
        uint8_t cnt = 0;
        for (int j = i-2; j <= i+2; j++)
        {
            if (j >= 0 && j < IV_POINTS) { v_avg += IV_volt[j]; cnt++; }
        }
        v_avg /= cnt;

        uint16_t px = IV_X0 + (uint16_t)((uint32_t)i * IV_W / (IV_POINTS - 1));
        uint16_t py;
        if (v_avg <= v_bottom)
            py = IV_Y0;
        else if (v_avg >= v_top)
            py = IV_Y0 - IV_H;
        else
            py = IV_Y0 - (uint16_t)((uint32_t)(v_avg - v_bottom) * IV_H / (v_top - v_bottom));

        if (i == 0)
        {
            px_prev = px;
            py_prev = py;
        }
        else
        {
            HMILCD_Send("line %d,%d,%d,%d,65504", px_prev, py_prev, px, py);
            rt_thread_mdelay(8);
            px_prev = px;
            py_prev = py;
        }
    }

    // 曲线画完后再画刻度，避免被覆盖
    // Y轴刻度
    HMILCD_Send("xstr %d,%d,40,16,7,65535,0,2,1,3,\"%d.%dV\"",
        IV_X0-42, IV_Y0-IV_H+2, v_top/100, (v_top%100)/10);
    rt_thread_mdelay(10);
    HMILCD_Send("xstr %d,%d,40,16,7,65535,0,2,1,3,\"%d.%dV\"",
        IV_X0-42, IV_Y0-8, v_bottom/100, (v_bottom%100)/10);
    rt_thread_mdelay(10);
    uint16_t v_mid = (v_top + v_bottom) / 2;
    HMILCD_Send("xstr %d,%d,40,16,7,65535,0,2,1,3,\"%d.%dV\"",
        IV_X0-42, IV_Y0-IV_H/2-8, v_mid/100, (v_mid%100)/10);
    rt_thread_mdelay(10);

    // X轴刻度
    HMILCD_Send("xstr %d,%d,25,16,7,65535,0,0,1,3,\"0A\"", IV_X0, IV_Y0+2);
    rt_thread_mdelay(10);
    int iset_half = (int)(ISET * 10 / 2 + 0.5);
    HMILCD_Send("xstr %d,%d,40,16,7,65535,0,0,1,3,\"%d.%dA\"",
        IV_X0+IV_W/2-15, IV_Y0+2, iset_half/10, iset_half%10);
    rt_thread_mdelay(10);
    HMILCD_Send("xstr %d,%d,25,16,7,65535,0,0,1,3,\"%dA\"",
        IV_X0+IV_W-15, IV_Y0+2, (int)(ISET+0.5));
    rt_thread_mdelay(10);

    // 通过Serial5把IV数据发给ESP32，ESP32转发给网页
    // 分包发送：先发开始帧，再逐点发，最后发结束帧
    Serial5_Send("{\"type\":\"ivs\",\"n\":%d}", IV_POINTS);
    rt_thread_mdelay(5);
    for (int i = 0; i < IV_POINTS; i++)
    {
        Serial5_Send("{\"type\":\"ivp\",\"idx\":%d,\"i\":%d,\"v\":%d}",
            i, IV_curr[i], IV_volt[i]);
        rt_thread_mdelay(2); // 给ESP32和串口缓冲区留时间
    }
    Serial5_Send("{\"type\":\"ive\"}");
    rt_kprintf("IV data sent to ESP32: %d points\n", IV_POINTS);
}

/*
 * 电池内阻测量 + 健康度评估
 * 原理：
 *   OCV  = 开路电压（0电流时）
 *   V1   = 0.5A轻载时电压
 *   V2   = 2.0A重载时电压
 *   Ri   = (V1 - V2) / (2.0 - 0.5)  单位Ω，×1000得mΩ
 *   SOH  = 根据OCV和Ri查表估算
 */
void bat_test(void)
{
    IV_scanning = 1; // 暂停thread7串口发送

    // ---- 1. 关闭负载，测开路电压 OCV ----
    Eload_Out = 0;
    DAC_SetChannel1Data(DAC_Align_12b_R, 4095);
    DAC_SetChannel2Data(DAC_Align_12b_R, 0);
    PWM_SetCCR4(0);
    rt_thread_mdelay(500); // 等待电路稳定
    uint32_t ocv_sum = 0;
    for (int i = 0; i < 8; i++)
    {
        ocv_sum += (uint32_t)(YVF * 100 + 0.5);
        rt_thread_mdelay(50);
    }
    uint16_t ocv = (uint16_t)(ocv_sum / 8); // 单位0.01V

    // ---- 2. 施加0.5A轻载，测V1 ----
    // 0.5A <= 2.5A，只用DAC2，DAC1=0开启MOS管
    uint16_t dac2_05A = (uint16_t)(0.5 * 0.01 * 50 * 4096 / VREF * DAC2_COMP + 0.5);
    DAC_SetChannel1Data(DAC_Align_12b_R, 0);
    DAC_SetChannel2Data(DAC_Align_12b_R, dac2_05A);
    PWM_SetCCR4(0);
    rt_thread_mdelay(500); // 等待稳定
    uint32_t v1_sum = 0;
    for (int i = 0; i < 8; i++)
    {
        v1_sum += (uint32_t)(YVF * 100 + 0.5);
        rt_thread_mdelay(50);
    }
    uint16_t v1 = (uint16_t)(v1_sum / 8); // 单位0.01V

    // ---- 3. 施加2.0A重载，测V2 ----
    // 2.0A <= 2.5A，只用DAC2
    uint16_t dac2_20A = (uint16_t)(2.0 * 0.01 * 50 * 4096 / VREF * DAC2_COMP + 0.5);
    DAC_SetChannel2Data(DAC_Align_12b_R, dac2_20A);
    rt_thread_mdelay(500); // 等待稳定
    uint32_t v2_sum = 0;
    for (int i = 0; i < 8; i++)
    {
        v2_sum += (uint32_t)(YVF * 100 + 0.5);
        rt_thread_mdelay(50);
    }
    uint16_t v2 = (uint16_t)(v2_sum / 8); // 单位0.01V

    // ---- 4. 测完关闭负载 ----
    DAC_SetChannel1Data(DAC_Align_12b_R, 4095);
    DAC_SetChannel2Data(DAC_Align_12b_R, 0);
    PWM_SetCCR4(0);

    // ---- 5. 计算内阻 Ri ----
    // Ri(mΩ) = (V1 - V2) / (2.0 - 0.5) * 1000
    //        = (v1 - v2) * 0.01V / 1.5A * 1000
    //        = (v1 - v2) * 10 / 15  (整数运算，单位mΩ)
    uint16_t ri_mohm = 0;
    if (v1 > v2)
        ri_mohm = (uint16_t)((uint32_t)(v1 - v2) * 100 / 15); // (v1-v2)单位0.01V，×100/15 = ÷1.5A，结果单位mΩ
    // 防止异常值（超过1000mΩ视为无效）
    if (ri_mohm > 1000) ri_mohm = 1000;

    // ---- 6. 计算SOH ----
    // OCV < 380（3.80V）说明电量不足，测量结果不可靠，提示充电
    // SOH 只用 Ri 判断，OCV 仅用于检查电量是否充足
    uint8_t soh = 0;
    uint8_t low_charge = (ocv < 350); // 电量不足标志（OCV < 3.50V）

    if (ri_mohm < 80)
        soh = 100;
    else if (ri_mohm < 120)
        soh = 80;
    else if (ri_mohm < 200)
        soh = 60;
    else if (ri_mohm < 300)
        soh = 40;
    else
        soh = 20;

    // 健康状态文字
    const char *health_str;
    if (low_charge)         health_str = "请先充满电";
    else if (soh >= 80)     health_str = "优秀";
    else if (soh >= 60)     health_str = "良好";
    else if (soh >= 40)     health_str = "老化";
    else                    health_str = "报废";

    rt_kprintf("bat_test: OCV=%d.%02dV V1=%d.%02dV V2=%d.%02dV Ri=%dmohm SOH=%d%%\n",
        ocv/100, ocv%100, v1/100, v1%100, v2/100, v2%100, ri_mohm, soh);

    // ---- 7. 切换到BAT页面显示结果 ----
    HMILCD_Send("page BAT");
    rt_thread_mdelay(300);

    // 显示OCV
    HMILCD_Send("BAT.t_ocv.txt=\"%d.%02dV\"", ocv/100, ocv%100);
    rt_thread_mdelay(20);
    // 显示V1（0.5A时）
    HMILCD_Send("BAT.t_v1.txt=\"%d.%02dV\"", v1/100, v1%100);
    rt_thread_mdelay(20);
    // 显示V2（2A时）
    HMILCD_Send("BAT.t_v2.txt=\"%d.%02dV\"", v2/100, v2%100);
    rt_thread_mdelay(20);
    // 显示内阻
    HMILCD_Send("BAT.t_ri.txt=\"%dmohm\"", ri_mohm);
    rt_thread_mdelay(20);
    // 显示SOH百分比
    HMILCD_Send("BAT.t_soh.txt=\"%d%%\"", soh);
    rt_thread_mdelay(20);
    // 显示健康状态
    HMILCD_Send("BAT.t_health.txt=\"%s\"", health_str);
    rt_thread_mdelay(20);

    // 发送电池检测结果到ESP32网页
    Serial5_Send("{\"type\":\"bat\",\"ocv\":%d,\"v1\":%d,\"v2\":%d,\"ri\":%d,\"soh\":%d,\"health\":\"%s\"}",
        ocv, v1, v2, ri_mohm, soh, health_str);

    mode = menu; // 测试完成后模式回到menu，等待用户返回
    IV_scanning = 0; // 恢复thread7
}

/* PID控制  */
void PID(void)
{
    if (mode == CC && Eload_Out == 1)
    {
        qpid_set_dst(&qpid_CC, ISET);
        I_SET = qpid_cal_pos(&qpid_CC, YIF);
        if (ISET <= 2.5)
        {
            DAC_SetChannel2Data(DAC_Align_12b_R, (uint16_t)(I_SET * 0.01 * 50 * 4096 / VREF * DAC2_COMP + 0.5));
        }
        else
        {
            DAC_SetChannel2Data(DAC_Align_12b_R, (uint16_t)(I_SET / 2.0 * 0.01 * 50 * 4096 / VREF * DAC2_COMP + 0.5));
            PWM_SetCCR4((uint16_t)(I_SET / 2.0 * 0.01 * 50 / VCC * 50000 * IREF2_COMP + 0.5));
        }
    }
    else if (mode == CV && Eload_Out == 1)
    {
        // CV 模式开环控制，DAC1 直接由 VSET 计算，硬件运放负反馈保证精度
        if (voltage_dw == 0)
            DAC_SetChannel1Data(DAC_Align_12b_R, (uint16_t)(VSET * 0.03219 * 4096 / VREF * DAC1_COMP + 0.5));
        else if (voltage_dw == 1)
            DAC_SetChannel1Data(DAC_Align_12b_R, (uint16_t)(VSET * 0.0947 * 4096 / VREF * DAC1_COMP + 0.5));
        else if (voltage_dw == 2)
            DAC_SetChannel1Data(DAC_Align_12b_R, (uint16_t)(VSET * 0.6175 * 4096 / VREF * DAC1_COMP + 0.5));
    }
    // CR/CW 由 thread9 的 CR_mode()/CW_mode() 前馈控制
}

/* 按键处理函数 */
void key123(void)
{
    if (key[2] == 1)    // 按键2，切换键
    {
        if (mode == menu)
        {
            HMILCD_Send("page CC"); // 切换到恒流模式页面
            mode = CC;              // 设置当前模式恒流模式
        }
        else if (mode == CC)
        {
            if (Eload_Out == 1)
            {
                HMILCD_Send("CC.t1.txt=\"OFF\"");
                HMILCD_Send("CC.b1.txt=\"开启\"");
                Eload_Out = 0;
                DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
                DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
                PWM_SetCCR4(0);  // 设置IREF2
            }
            HMILCD_Send("page CV"); // 切换到恒压模式页面
            mode = CV;              // 设置当前模式恒压模式
        }
        else if (mode == CV)
        {
            if (Eload_Out == 1)
            {
                HMILCD_Send("CV.t1.txt=\"OFF\"");
                HMILCD_Send("CV.b1.txt=\"开启\"");
                Eload_Out = 0;
                DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
                DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
                PWM_SetCCR4(0);  // 设置IREF2
            }
            HMILCD_Send("page CR"); // 切换到恒阻模式页面
            mode = CR;              // 设置当前模式恒阻模式
        }
        else if (mode == CR)
        {
            if (Eload_Out == 1)
            {
                HMILCD_Send("CR.t1.txt=\"OFF\"");
                HMILCD_Send("CR.b1.txt=\"开启\"");
                Eload_Out = 0;
                DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
                DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
                PWM_SetCCR4(0);  // 设置IREF2
            }
            HMILCD_Send("page CW"); // 切换到恒功率模式页面
            mode = CW;              // 设置当前模式恒功率模式
        }
        else if (mode == CW)
        {
            if (Eload_Out == 1)
            {
                HMILCD_Send("CW.t1.txt=\"OFF\"");
                HMILCD_Send("CW.b1.txt=\"开启\"");
                Eload_Out = 0;
                DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
                DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
                PWM_SetCCR4(0);  // 设置IREF2
            }
            HMILCD_Send("page menu"); // 切换到菜单页面
            mode = menu;              // 设置当前模式菜单模式
        }
        key[2] = 0;
    }
    if (key[3] == 1)    // 按键3，菜单键
    {
        if (Eload_Out == 1)
            Key_ONOFF = 1;
        rt_thread_mdelay(15);
        HMILCD_Send("page menu"); // 切换到菜单页面
        mode = menu;              // 设置当前模式菜单模式
        key[3] = 0;
    }
}

/* 线程 5 的入口函数，电子负载开启关闭按键处理 */
static void thread5_ONOFF_entry(void *parameter)
{
    /* LED2引脚为输出模式 */
    rt_pin_mode(LED2, PIN_MODE_OUTPUT);
    /* 默认高电平 */
    rt_pin_write(LED2, PIN_HIGH);
    while (1)
    {
        if (Key_ONOFF == 1 | key[1] == 1) // 开启按钮按下
        {
            if (mode == CC) // 恒流模式
            {
                if (Eload_Out == 0) // 当前负载输出状态为关闭时
                {
                    HMILCD_Send("CC.t1.txt=\"ON\"");   // 屏幕右上角标题框显示ON
                    HMILCD_Send("CC.b1.txt=\"关闭\"");    // 屏幕右下角按钮显示关闭
                    qpid_init(&qpid_CC);
                    qpid_set_lmt(&qpid_CC, 0, 10);
                    qpid_set_ratio(&qpid_CC, 1, 0.001, 0.1);
                    qpid_set_dst(&qpid_CC, ISET);
                    Eload_Out = 1;                           // 负载输出状态设置为开启
                    DAC_SetChannel1Data(DAC_Align_12b_R, 0);
                    if (ISET <= 2.5)
                    {
                        DAC_SetChannel2Data(DAC_Align_12b_R,
                                (uint16_t)(ISET * 0.01 * 50 * 4096 / VREF * DAC2_COMP + 0.5));
                        PWM_SetCCR4(0);  // 设置IREF2
                    }
                    else
                    {
                        DAC_SetChannel2Data(DAC_Align_12b_R,
                                (uint16_t)(ISET / 2.0 * 0.01 * 50 * 4096 / VREF * DAC2_COMP + 0.5));
                        PWM_SetCCR4((uint16_t)(ISET / 2.0 * 0.01 * 50 / VCC * 50000 * IREF2_COMP + 0.5));  // 设置IREF2
                    }
                }
                else if (Eload_Out == 1) // 当前负载输出状态为开启时
                {
                    HMILCD_Send("CC.t1.txt=\"OFF\"");
                    HMILCD_Send("CC.b1.txt=\"开启\"");
                    Eload_Out = 0;
                    DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
                    DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
                    PWM_SetCCR4(0);  // 设置IREF2
                }
            }
            else if (mode == CV) // 恒压模式
            {
                if (Eload_Out == 0) // 当前负载输出状态为关闭时
                {
                    HMILCD_Send("CV.t1.txt=\"ON\"");
                    HMILCD_Send("CV.b1.txt=\"关闭\"");
                    Eload_Out = 1;
                    DAC_SetChannel2Data(DAC_Align_12b_R, 4095); // 设置IREF1
                    PWM_SetCCR4(50000);  // 设置IREF2
                    if (voltage_dw == 0)
                    {
                        DAC_SetChannel1Data(DAC_Align_12b_R, (uint16_t)(VSET * 0.03219 * 4096 / VREF * DAC1_COMP + 0.5));
                        // 设置DAC1输出值，控制恒压
                    }
                    else if (voltage_dw == 1)
                    {
                        DAC_SetChannel1Data(DAC_Align_12b_R, (uint16_t)(VSET * 0.0947 * 4096 / VREF * DAC1_COMP + 0.5));
                        // 设置DAC1输出值，控制恒压
                    }
                    else if (voltage_dw == 2)
                    {
                        DAC_SetChannel1Data(DAC_Align_12b_R, (uint16_t)(VSET * 0.6175 * 4096 / VREF * DAC1_COMP + 0.5));
                        // 设置DAC1输出值，控制恒压
                    }
                }
                else if (Eload_Out == 1) // 当前负载输出状态为开启时
                {
                    HMILCD_Send("CV.t1.txt=\"OFF\"");
                    HMILCD_Send("CV.b1.txt=\"开启\"");
                    Eload_Out = 0;
                    DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
                    DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
                    PWM_SetCCR4(0);  // 设置IREF2
                }
            }
            else if (mode == CR) // 恒阻模式
            {
                if (Eload_Out == 0) // 当前负载输出状态为关闭时
                {
                    HMILCD_Send("CR.t1.txt=\"ON\"");
                    HMILCD_Send("CR.b1.txt=\"关闭\"");
                    Eload_Out = 1;
                    DAC_SetChannel1Data(DAC_Align_12b_R, 0);
                    DAC_SetChannel2Data(DAC_Align_12b_R, 0);  // 先清零，防止残留值冲击
                    PWM_SetCCR4(0);
                    qpid_init(&qpid_CR);                      // 重置PID状态
                    qpid_set_lmt(&qpid_CR, 0.1, 1000);
                    qpid_set_ratio(&qpid_CR, 0.35, 0.003, 0.001);
                    CR_mode();
                }
                else if (Eload_Out == 1) // 当前负载输出状态为开启时
                {
                    HMILCD_Send("CR.t1.txt=\"OFF\"");
                    HMILCD_Send("CR.b1.txt=\"开启\"");
                    Eload_Out = 0;
                    DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
                    DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
                    PWM_SetCCR4(0);  // 设置IREF2
                }
            }
            else if (mode == CW) // 恒功率模式
            {
                if (Eload_Out == 0) // 当前负载输出状态为关闭时
                {
                    HMILCD_Send("CW.t1.txt=\"ON\"");
                    HMILCD_Send("CW.b1.txt=\"关闭\"");
                    Eload_Out = 1;
                    DAC_SetChannel1Data(DAC_Align_12b_R, 0);
                    DAC_SetChannel2Data(DAC_Align_12b_R, 0);  // 先清零，防止残留值冲击
                    PWM_SetCCR4(0);
                    qpid_init(&qpid_CW);                      // 重置PID状态
                    qpid_set_lmt(&qpid_CW, 0.1, 200);
                    qpid_set_ratio(&qpid_CW, 0.5, 0.003, 0.001);
                    CW_mode();
                }
                else if (Eload_Out == 1) // 当前负载输出状态为开启时
                {
                    HMILCD_Send("CW.t1.txt=\"OFF\"");
                    HMILCD_Send("CW.b1.txt=\"开启\"");
                    Eload_Out = 0;
                    DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
                    DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
                    PWM_SetCCR4(0);  // 设置IREF2
                }
            }
            Key_ONOFF = 0;
            key[1] = 0;
        }

        if (Eload_Out == 0)
        {
            rt_pin_write(LED2, SET);
        }
        else if (Eload_Out == 1)
        {
            rt_pin_write(LED2, RESET);
        }
        rt_thread_mdelay(35);
    }

}

/* 线程 7 的入口函数，串口屏显示参数 */
static void thread7_HMI_Display_entry(void *parameter)
{
    while (1)
    {
        if (mode != menu && IV_scanning == 0) // 扫描期间不发送，避免串口冲突
        {
            double V = lowV1(YVF);
            double I = lowV2(YIF);
            HMILCD_Send("x1.val=%d", (uint16_t)(V * 100));
            rt_thread_mdelay(100);
            HMILCD_Send("x2.val=%d", (uint16_t)(I * 1000));
            rt_thread_mdelay(100);
            HMILCD_Send("x3.val=%d", (uint32_t)(I * V * 100));
            rt_thread_mdelay(100);
            if (I > 0.01)
                HMILCD_Send("x4.val=%d", (uint32_t)(V / I * 100));
            else
                HMILCD_Send("x4.val=0");
            rt_thread_mdelay(100);
            HMILCD_Send("x5.val=%d", (uint32_t)(VBAT * 100));
        }
        rt_thread_mdelay(500);
    }
}

/* 线程 8 的入口函数，散热风扇控制 */
/* 4线PWM风扇低电平有效：CCR越小高电平越少低电平越多，转速越快 */
/* SetCCR(100)=CCR500=全高=最慢, SetCCR(0)=CCR0=全低=满转 */
static void thread8_FAN_entry(void *parameter)
{
    while (1)
    {
        uint16_t P = (uint16_t)(YIF * YVF + 0.5);
        if (P >= 13)
        {
            FAN_PWM_ON();
            if (P < 20)
                FAN_PWM_SetCCR(80);      // 20W: 低速
            else if (P < 25)
                FAN_PWM_SetCCR(70);
            else if (P < 30)
                FAN_PWM_SetCCR(60);
            else if (P < 35)
                FAN_PWM_SetCCR(50);
            else if (P < 40)
                FAN_PWM_SetCCR(40);
            else if (P < 45)
                FAN_PWM_SetCCR(30);
            else if (P < 50)
                FAN_PWM_SetCCR(20);
            else if (P < 60)
                FAN_PWM_SetCCR(10);
            else
                FAN_PWM_SetCCR(0);       // >=60W: 满转
        }
        else if (P <= 8)
        {
            FAN_PWM_ON();
            FAN_PWM_SetCCR(100);         // 功率低时最低转速
        }
        rt_thread_mdelay(200);
    }
}

/* 线程 9 的入口函数，恒功率和恒阻模式控制  */
static void thread9_CWCR_entry(void *parameter)
{
    while (1)
    {
        if (IV_scan_flag == 1)
        {
            IV_scan_flag = 0;
            IV_scan();
        }
        if (bat_test_flag == 1)
        {
            bat_test_flag = 0;
            bat_test();
        }
        if (mode == CW)
        {
            CW_mode();
        }
        if (mode == CR)
        {
            CR_mode();
        }
        rt_thread_mdelay(50);
    }
}

/* 线程 10 的入口函数，对蓝牙发来的数据进行处理 */
static void thread10_BlueTooth_entry(void *parameter)
{
    HMILCD_Send("CC.x0.val=0"); // 屏幕显示的电流设定值清零
    HMILCD_Send("CV.x0.val=0"); // 屏幕显示的电压设定值清零
    HMILCD_Send("CR.x0.val=0"); // 屏幕显示的电阻设定值清零
    HMILCD_Send("CW.x0.val=0"); // 屏幕显示的功率设定值清零
    while (1)
    {
        if (Serial5_RxFlag == 1)
        {
            if (Serial5_RxPacket[0] == 0x01) // 当前是主菜单页面
            {
                if (Serial5_RxPacket[1] == 0x10) // 恒流按钮按下
                {
                    if (mode == CV)
                    {
                        HMILCD_Send("CV.t1.txt=\"OFF\"");
                        HMILCD_Send("CV.b1.txt=\"开启\"");

                    }
                    else if (mode == CR)
                    {
                        HMILCD_Send("CR.t1.txt=\"OFF\"");
                        HMILCD_Send("CR.b1.txt=\"开启\"");

                    }
                    else if (mode == CW)
                    {
                        HMILCD_Send("CW.t1.txt=\"OFF\"");
                        HMILCD_Send("CW.b1.txt=\"开启\"");

                    }
                    DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
                    DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
                    PWM_SetCCR4(0);  // 设置IREF2
                    Eload_Out = 0;
                    HMILCD_Send("page CC"); // 切换到恒流模式页面
                    mode = CC;                    // 设置当前模式恒流模式
                }
                else if (Serial5_RxPacket[1] == 0x11) // 恒压按钮按下
                {
                    if (mode == CC)
                    {
                        HMILCD_Send("CC.t1.txt=\"OFF\"");
                        HMILCD_Send("CC.b1.txt=\"开启\"");

                    }
                    else if (mode == CR)
                    {
                        HMILCD_Send("CR.t1.txt=\"OFF\"");
                        HMILCD_Send("CR.b1.txt=\"开启\"");

                    }
                    else if (mode == CW)
                    {
                        HMILCD_Send("CW.t1.txt=\"OFF\"");
                        HMILCD_Send("CW.b1.txt=\"开启\"");

                    }
                    DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
                    DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
                    PWM_SetCCR4(0);  // 设置IREF2
                    Eload_Out = 0;
                    HMILCD_Send("page CV"); // 切换到恒压页面
                    mode = CV;                    // 设置当前模式为恒压模式
                }
                else if (Serial5_RxPacket[1] == 0x12) // 恒阻按钮按下
                {
                    if (mode == CC)
                    {
                        HMILCD_Send("CC.t1.txt=\"OFF\"");
                        HMILCD_Send("CC.b1.txt=\"开启\"");

                    }
                    else if (mode == CV)
                    {
                        HMILCD_Send("CV.t1.txt=\"OFF\"");
                        HMILCD_Send("CV.b1.txt=\"开启\"");

                    }
                    else if (mode == CW)
                    {
                        HMILCD_Send("CW.t1.txt=\"OFF\"");
                        HMILCD_Send("CW.b1.txt=\"开启\"");

                    }
                    DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
                    DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
                    PWM_SetCCR4(0);  // 设置IREF2
                    Eload_Out = 0;
                    HMILCD_Send("page CR"); // 切换到恒阻页面
                    mode = CR;                    // 设置当前模式为恒阻模式
                }
                else if (Serial5_RxPacket[1] == 0x13) // 恒功率按钮按下
                {
                    if (mode == CV)
                    {
                        HMILCD_Send("CV.t1.txt=\"OFF\"");
                        HMILCD_Send("CV.b1.txt=\"开启\"");

                    }
                    else if (mode == CR)
                    {
                        HMILCD_Send("CR.t1.txt=\"OFF\"");
                        HMILCD_Send("CR.b1.txt=\"开启\"");

                    }
                    else if (mode == CC)
                    {
                        HMILCD_Send("CC.t1.txt=\"OFF\"");
                        HMILCD_Send("CC.b1.txt=\"开启\"");

                    }
                    DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
                    DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
                    PWM_SetCCR4(0);  // 设置IREF2
                    Eload_Out = 0;
                    HMILCD_Send("page CW"); // 切换到恒功率页面
                    mode = CW;                    // 设置当前模式为恒功率模式
                }
            }
            else if (Serial5_RxPacket[0] == 0x08)    // 返回菜单
            {
                if (mode == CC)
                {
                    HMILCD_Send("CC.t1.txt=\"OFF\"");  // 屏幕右上角标题框显示OFF
                    HMILCD_Send("CC.b1.txt=\"开启\""); // 屏幕右下角按钮显示开启
                    Eload_Out = 0;                           // 负载输出状态设置为关闭
                    DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
                    DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
                    PWM_SetCCR4(0);  // 设置IREF2
                    HMILCD_Send("page menu");          // 切换到菜单页面
                    mode = menu;
                }
                else if (mode == CV)
                {
                    HMILCD_Send("CV.t1.txt=\"OFF\"");
                    HMILCD_Send("CV.b1.txt=\"开启\"");
                    Eload_Out = 0;
                    DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
                    DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
                    PWM_SetCCR4(0);  // 设置IREF2
                    HMILCD_Send("page menu");
                    mode = menu;
                }
                else if (mode == CR)
                {
                    HMILCD_Send("CR.t1.txt=\"OFF\"");
                    HMILCD_Send("CR.b1.txt=\"开启\"");
                    Eload_Out = 0;
                    DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
                    DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
                    PWM_SetCCR4(0);  // 设置IREF2
                    HMILCD_Send("page menu");
                    mode = menu;
                }
                else if (mode == CW)
                {
                    HMILCD_Send("CW.t1.txt=\"OFF\"");
                    HMILCD_Send("CW.b1.txt=\"开启\"");
                    Eload_Out = 0;
                    DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
                    DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流
                    PWM_SetCCR4(0);  // 设置IREF2
                    HMILCD_Send("page menu");
                    mode = menu;
                }
            }
            else if (Serial5_RxPacket[0] == 0x09)    // 开启按钮
            {
                Key_ONOFF = 1;
            }
            else if (Serial5_RxPacket[0] == 0x0A)    // 关闭负载（网页控制）
            {
                Eload_Out = 0;
                DAC_SetChannel1Data(DAC_Align_12b_R, 4095);
                DAC_SetChannel2Data(DAC_Align_12b_R, 0);
                PWM_SetCCR4(0);
            }
            else if (Serial5_RxPacket[0] == 0x0B)    // 电池检测（网页触发）
            {
                bat_test_flag = 1;
            }

            else if (Serial5_RxPacket[0] == 0xAA) // 当前为数字键盘页面
            {
                char *temp = Serial5_RxPacket;
                temp++;                      // 地址自增1
                uint16_t temp2 = atoi(temp); // 字符串转整形
                if (mode == CC)
                {
                    if (temp2 > 1000)
                        temp2 = 1000;
                    ISET = temp2 / 100.0;
                    HMILCD_Send("CC.x0.val=%d", temp2);

                    if (Eload_Out == 1)
                    {
                        DAC_SetChannel1Data(DAC_Align_12b_R, 0);
                        if (ISET <= 2.5)
                        {
                            // 设置DAC2输出值，控制恒流，+0.5是为了四舍五入
                            DAC_SetChannel2Data(DAC_Align_12b_R,
                                    (uint16_t)(ISET * 0.01 * 50 * 4096 / VREF * DAC2_COMP + 0.5));
                            PWM_SetCCR4(0);  // 设置IREF2
                        }
                        else
                        {
                            // 设置DAC2输出值，控制恒流，+0.5是为了四舍五入
                            DAC_SetChannel2Data(DAC_Align_12b_R,
                                    (uint16_t)(ISET / 2.0 * 0.01 * 50 * 4096 / VREF * DAC2_COMP + 0.5));
                            PWM_SetCCR4((uint16_t)(ISET / 2.0 * 0.01 * 50 / VCC * 50000 * IREF2_COMP + 0.5)); // 设置IREF2
                        }

                    }

                }
                else if (mode == CV)
                {
                    VSET = temp2 / 100.0;
                    HMILCD_Send("CV.x0.val=%d", temp2);
                    if (Eload_Out == 1)
                    {
                        DAC_SetChannel2Data(DAC_Align_12b_R, 4095); // 设置IREF1
                        PWM_SetCCR4(50000);  // 设置IREF2
                        if (voltage_dw == 0)
                        {
                            DAC_SetChannel1Data(DAC_Align_12b_R,
                                    (uint16_t)(VSET * 0.05225 * 4096 / VREF * DAC1_COMP + 0.5)); // 设置DAC1输出值，控制恒压
                        }
                        else if (voltage_dw == 1)
                        {
                            DAC_SetChannel1Data(DAC_Align_12b_R,
                                    (uint16_t)(VSET * 0.0947 * 4096 / VREF * DAC1_COMP + 0.5)); // 设置DAC1输出值，控制恒压
                        }
                        else if (voltage_dw == 2)
                        {
                            uint16_t vset_pwm = (uint16_t)(VSET * 0.6175 * 4096 / VREF * DAC1_COMP + 0.5);
                            if (vset_pwm > 4065)
                                vset_pwm = 4095;
                            DAC_SetChannel1Data(DAC_Align_12b_R, vset_pwm); // 设置DAC1输出值，控制恒压
                        }
                    }
                }
                else if (mode == CR)
                {
                    RSET = temp2 / 100.0;
                    HMILCD_Send("CR.x0.val=%d", temp2);
                    CR_mode();
                }
                else if (mode == CW)
                {
                    PSET = temp2 / 100.0;
                    HMILCD_Send("CW.x0.val=%d", temp2);
                    CW_mode();
                }
            }
            // 0x02：网页设置数值命令，格式：0x40 0x02 + mode_byte + 5位ASCII数字 + 0xFF 0xFF 0xFF
            else if (Serial5_RxPacket[0] == 0x02)
            {
                uint8_t mb  = Serial5_RxPacket[1];
                char numstr[6];
                memcpy(numstr, &Serial5_RxPacket[2], 5);
                numstr[5] = '\0';
                uint16_t val = (uint16_t)atoi(numstr); // ×100整数
                rt_kprintf("SET cmd: mb=0x%02X numstr=%s val=%d\n", mb, numstr, val);

                // 先切换模式和页面（与0x01命令逻辑一致）
                DAC_SetChannel1Data(DAC_Align_12b_R, 4095);
                DAC_SetChannel2Data(DAC_Align_12b_R, 0);
                PWM_SetCCR4(0);
                Eload_Out = 0;

                if (mb == 0x10) // CC
                {
                    if (val > 1000) val = 1000;
                    ISET = val / 100.0;
                    HMILCD_Send("page CC");
                    rt_thread_mdelay(50);
                    HMILCD_Send("CC.x0.val=%d", val);
                    mode = CC;
                }
                else if (mb == 0x11) // CV
                {
                    if (val > 3000) val = 3000;
                    VSET = val / 100.0;
                    HMILCD_Send("page CV");
                    rt_thread_mdelay(50);
                    HMILCD_Send("CV.x0.val=%d", val);
                    mode = CV;
                }
                else if (mb == 0x12) // CR
                {
                    if (val > 99900) val = 99900;
                    RSET = val / 100.0;
                    HMILCD_Send("page CR");
                    rt_thread_mdelay(50);
                    HMILCD_Send("CR.x0.val=%d", val);
                    mode = CR;
                }
                else if (mb == 0x13) // CW
                {
                    if (val > 20000) val = 20000;
                    PSET = val / 100.0;
                    HMILCD_Send("page CW");
                    rt_thread_mdelay(50);
                    HMILCD_Send("CW.x0.val=%d", val);
                    mode = CW;
                }
            }
            Serial5_RxFlag = 0;
        }
        rt_thread_mdelay(40);
    }

}

/* 线程11：每200ms向ESP32推送一次JSON数据 */
static void thread11_ESP32_Push_entry(void *parameter)
{
    while (1)
    {
        if (IV_scanning == 0) // IV扫描或电池检测期间不推送，避免串口冲突
        {
            // 模式编号：0=CC 1=CV 2=CR 3=CW 4=menu
            uint8_t m = (mode == CC) ? 0 :
                        (mode == CV) ? 1 :
                        (mode == CR) ? 2 :
                        (mode == CW) ? 3 : 4;
            // 电压×100、电流×100、功率×100、内阻×10，全部整数，网页端除以100
            Serial5_Send("{\"v\":%d,\"i\":%d,\"p\":%d,\"m\":%d,\"o\":%d}",
                (int)(YVF * 100 + 0.5),
                (int)(YIF * 100 + 0.5),
                (int)(YVF * YIF * 100 + 0.5),
                m,
                (int)Eload_Out);
        }
        rt_thread_mdelay(200);
    }
}
