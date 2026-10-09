/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-07-14     zeruns       the first version
 */
#ifndef __KEY_H
#define __KEY_H

#include <rtthread.h> // 引入 RT-Thread 操作系统的头文件
#include "ch32v30x.h"
#include "drivers/pin.h"
#include <rtdbg.h>

extern uint8_t KEY_Status[4][3]; // 记录各按键状态
extern uint8_t key[4];           // 记录各按键是否稳定按下，1表示按键已按下，0表示按键没被按下

void thread6_KEY_entry(void *parameter);

#endif /* APPLICATIONS_KEY_H_ */
