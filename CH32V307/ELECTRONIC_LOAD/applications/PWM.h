/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-07-17     cd       the first version
 */
#ifndef APPLICATIONS_PWM_H_
#define APPLICATIONS_PWM_H_

#include "ch32v30x.h"
#include <rtthread.h>

void PWM_Init(void);
void PWM_SetCCR4(uint16_t Compare);
void FAN_PWM_SetCCR(uint16_t Compare);
void FAN_PWM_ON(void);
void FAN_PWM_OFF(void);

#endif /* APPLICATIONS_PWM_H_ */
