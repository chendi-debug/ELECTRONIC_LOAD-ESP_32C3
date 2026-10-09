#include "KEY.h"

/* 引脚编号，通过查看驱动文件drv_gpio.c确定 */
#define PD10    58
#define PD9     57
#define PD8     56

#define KEY1 rt_pin_read(PD10)
#define KEY2 rt_pin_read(PD9)
#define KEY3 rt_pin_read(PD8)

// 定义按键状态的枚举变量类型
typedef enum
{
    KS_RELEASE = 0, // 按键松开
    KS_SHAKE,       // 按键抖动
    KS_PRESS,       // 稳定按下
} KEY_STATUS;

// 当前循环结束的(状态机的)状态
#define g_keyStatus 0
// 当前状态(每次循环后与g_keyStatus保持一致)
#define g_nowKeyStatus 1
// 上次状态(用于记录前一状态以区分状态的来源)
#define g_lastKeyStatus 2
// 按键数量
#define keynum 3

uint8_t KEY_Status[keynum+1][3]; // 记录各按键状态
uint8_t key[keynum+1];           // 记录各按键是否稳定按下，1表示按键已按下，0表示按键没被按下

// 按键状态机程序
void key_status_check(uint8_t key_num, uint8_t KEY)
{
    switch (KEY_Status[key_num][g_keyStatus])//switch ([哪个按键][状态检测])
    {
    // 松开的时候
    case KS_RELEASE:
    {
        // 检测到低电平，先进行消抖
        if (KEY == 0)
        {
            KEY_Status[key_num][g_keyStatus] = KS_SHAKE;
        }
    }
        break;

        // 抖动
    case KS_SHAKE:
    {
        if (KEY == 1)
        {
            KEY_Status[key_num][g_keyStatus] = KS_RELEASE;
        }
        else
        {
            KEY_Status[key_num][g_keyStatus] = KS_PRESS;
        }
    }
        break;

        // 稳定短按
    case KS_PRESS:
    {
        // 检测到高电平，先进行消抖
        if (KEY == 1)
        {
            KEY_Status[key_num][g_keyStatus] = KS_SHAKE;
        }
    }
        break;

    default:
        break;
    }

    if (KEY_Status[key_num][g_keyStatus] != KEY_Status[key_num][g_nowKeyStatus])
    {
        // 当前状态为松开 并且 前一次状态为按下
        if ((KEY_Status[key_num][g_keyStatus] == KS_RELEASE) && (KEY_Status[key_num][g_lastKeyStatus] == KS_PRESS))
        {
            key[key_num] = 1;
        }
        KEY_Status[key_num][g_lastKeyStatus] = KEY_Status[key_num][g_nowKeyStatus];
        KEY_Status[key_num][g_nowKeyStatus] = KEY_Status[key_num][g_keyStatus];
    }
}

/* 线程 6 的入口函数，按键扫描 */
void thread6_KEY_entry(void *parameter)
{
    /* PD10引脚为上拉输入模式 */
    rt_pin_mode(PD10, PIN_MODE_INPUT_PULLUP);
    /* 默认高电平 */
    rt_pin_write(PD10, PIN_HIGH);

    rt_pin_mode(PD9, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(PD8, PIN_MODE_INPUT_PULLUP);
    rt_pin_write(PD9, PIN_HIGH);
    rt_pin_write(PD8, PIN_HIGH);

    while (1)
    {
        key_status_check(1, KEY1);
        key_status_check(2, KEY2);
        key_status_check(3, KEY3);
        rt_thread_mdelay(20);//作用:1.硬件消抖，2.让出CPU使用权  运行态-->挂起态
    }
}


