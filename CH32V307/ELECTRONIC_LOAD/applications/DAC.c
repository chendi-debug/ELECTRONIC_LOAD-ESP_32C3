/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-07-15     zeruns       the first version
 */
#include "DAC.h"

void Dac_Init(void)
{
    // 定义GPIO和DAC结构体变量
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };
    DAC_InitTypeDef DAC_InitType = { 0 };

    // 使能GPIOA和DAC时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

    // 配置PA4和PA5为模拟输入模式，速度为50MHz
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // 设置PA4和PA5为高电平
    GPIO_SetBits(GPIOA, GPIO_Pin_4);
    GPIO_SetBits(GPIOA, GPIO_Pin_5);

    // 配置DAC为无触发模式，无波形生成，低频屏蔽寄存器掩码为0，输出缓冲禁用
    DAC_InitType.DAC_Trigger = DAC_Trigger_None;
    DAC_InitType.DAC_WaveGeneration = DAC_WaveGeneration_None;
    DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
    DAC_InitType.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
    // 初始化DAC通道1和通道2
    DAC_Init(DAC_Channel_1, &DAC_InitType);
    DAC_Init(DAC_Channel_2, &DAC_InitType);
    // 使能DAC通道1和通道2
    DAC_Cmd(DAC_Channel_1, ENABLE);
    DAC_Cmd(DAC_Channel_2, ENABLE);

    // 设置DAC通道1的数据，右对齐12位格式
    DAC_SetChannel1Data(DAC_Align_12b_R, 4095); // DAC1输出高电平，关闭恒压
    // 设置DAC通道2的数据W，右对齐12位格式
    DAC_SetChannel2Data(DAC_Align_12b_R, 0);    // DAC2输出低电平，关闭恒流

    LOG_D("DAC initialize success");
}
