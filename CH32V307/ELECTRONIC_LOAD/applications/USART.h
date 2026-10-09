#ifndef __USART_H
#define __USART_H

#include <rtthread.h> // 引入 RT-Thread 操作系统的头文件
#include <rtdevice.h>
#include "ch32v30x.h"
#include <stdio.h>
#include <stdarg.h>

extern rt_uint8_t Serial3_RxPacket[];
extern rt_uint8_t Serial3_RxFlag;
extern rt_uint8_t Serial5_RxPacket[];
extern rt_uint8_t Serial5_RxFlag;

void HMILCD_Send(char *format, ...);
void Serial5_Send(char *format, ...);
int UART_Init(void);

#endif
