/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-07-17     cd       the first version
 */
#include "PWM.h"

void PWM_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);    // 启用TIM4时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM8, ENABLE);
    // 启用GPIOB、GPIOC、TIM8时钟

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);    // 启用AFIO时钟
    //  GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);
    //  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;                    // 定义结构体，配置GPIO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;         // 设置GPIO口为复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;               // 设置GPIO口
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       // 设置GPIO口速度50Mhz
    GPIO_Init(GPIOB, &GPIO_InitStructure);                  // 初始化GPIOB

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;               // PC6 先占位，后面统一设置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

    TIM_InternalClockConfig(TIM4);                          // 设置TIM4使用内部时钟
    TIM_InternalClockConfig(TIM8);                          // 设置TIM8使用内部时钟

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;              // 定义结构体，配置定时器
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     // 设置1分频（不分频）
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 设置计数模式为向上计数
    TIM_TimeBaseInitStructure.TIM_Period = 50000 - 1;       //ARR，设置最大计数值，达到最大值触发更新事件，因为从0开始计数，所以计数60000次是60000-1
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;        //PSC，设置时钟预分频，2-1就是每 时钟频率(144Mhz)/1=144Mhz
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;    // 重复计数器（高级定时器才有，所以设置0）
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);     // 初始化TIM4定时器

    TIM_TimeBaseInitStructure.TIM_Period = 500 - 1;       //ARR，设置最大计数值，达到最大值触发更新事件，因为从0开始计数，所以计数500次是500-1
    TIM_TimeBaseInitStructure.TIM_Prescaler = 12 - 1;       //PSC，设置时钟预分频，2-1就是每 时钟频率(144Mhz)/12=12Mhz
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure);     // 初始化TIM4定时器

    /**
     * @PWM频率：   Freq = CK_PSC（时钟频率）/(PSC+1)/(ARR+1)
     * @PWM占空比：  Duty = CCR/((ARR+1)
     * @PWM分辨率：  Reso = 1/(ARR+1)
     */

    TIM_OCInitTypeDef TIM_OCInitStructure;                          // 定义结构体，配置捕获/比较寄存器
    TIM_OCStructInit(&TIM_OCInitStructure);                         // 给结构体赋初始值
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;               // 输出比较模式，配置为PWM模式1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;       // TIM4高极性，CC电流控制
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   // 输出使能
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);                        // 初始化TIM4 OC4（IREF2）

    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;        // TIM8低极性，风扇PWM控制
    TIM_OC1Init(TIM8, &TIM_OCInitStructure);                        // 初始化TIM8 OC1（风扇）

    TIM_Cmd(TIM4, ENABLE);          // 开启定时器
    TIM_Cmd(TIM8, ENABLE);          // 开启定时器
    TIM_CtrlPWMOutputs(TIM8, ENABLE);       // 开启TIM8的PWM输出
    TIM_SetCompare1(TIM8, 100 * 5);         // 初始CCR=500，低极性下输出低电平，风扇最低速待机
    // PC6 切换为复用推挽，输出PWM
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void PWM_SetCCR4(uint16_t Compare)      //设置占空比
{
    TIM_SetCompare4(TIM4, Compare);     //设置CCR的值
}

void FAN_PWM_SetCCR(uint16_t Compare)      //设置占空比
{
    TIM_SetCompare1(TIM8, (uint16_t)(Compare * 5));         //设置CCR的值
}

void FAN_PWM_ON(void)
{
    TIM_CtrlPWMOutputs(TIM8, ENABLE);   // 开启TIM8的PWM输出
    GPIO_InitTypeDef GPIO_InitStructure;                    // 定义结构体，配置GPIO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;// 设置GPIO口为复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;// 设置GPIO口速度50Mhz
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;// 设置GPIO口
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void FAN_PWM_OFF(void)
{
    TIM_CtrlPWMOutputs(TIM8, DISABLE);   // 关闭TIM8的PWM输出
    GPIO_InitTypeDef GPIO_InitStructure;                    // 定义结构体，配置GPIO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;// 设置GPIO口为推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;// 设置GPIO口速度50Mhz
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;// 设置GPIO口
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_WriteBit(GPIOC,GPIO_Pin_6,RESET);

}
