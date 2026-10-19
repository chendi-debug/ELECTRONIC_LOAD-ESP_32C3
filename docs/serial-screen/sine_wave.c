/*
 * 正弦波显示示例代码 - 适用于 TJC4827T143_011 串口屏
 * 屏幕分辨率: 480x272
 * 通信协议: USART HMI (与 Nextion 兼容)
 * 波特率: 115200 (可根据实际情况修改)
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>

/* 配置参数 */
#define UART_DEVICE_NAME    "uart3"     // 串口设备名称
#define UART_BAUDRATE       115200      // 波特率
#define SINE_WAVE_POINTS    100         // 正弦波采样点数
#define WAVE_AMPLITUDE      100         // 波形幅度 (像素)
#define WAVE_CENTER_Y       136         // 波形中心Y坐标 (272/2)
#define WAVE_SPEED          10          // 波形更新速度 (ms)

/* 全局变量 */
static rt_device_t serial_device = RT_NULL;

/* 向串口屏发送指令 (自动添加结束符 0xFF 0xFF 0xFF) */
void TJC_Send(const char *format, ...)
{
    char buffer[128];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // 发送指令内容
    rt_device_write(serial_device, 0, buffer, rt_strlen(buffer));

    // 发送结束符
    uint8_t end_data[3] = {0xFF, 0xFF, 0xFF};
    rt_device_write(serial_device, 0, end_data, 3);
}

/* 初始化串口屏 */
rt_err_t TJC_Init(void)
{
    serial_device = rt_device_find(UART_DEVICE_NAME);
    if (!serial_device)
    {
        rt_kprintf("找不到串口设备: %s\n", UART_DEVICE_NAME);
        return RT_ERROR;
    }

    rt_device_open(serial_device, RT_DEVICE_FLAG_INT_RX);

    // 配置波特率
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = UART_BAUDRATE;
    rt_device_control(serial_device, RT_DEVICE_CTRL_CONFIG, &config);

    rt_thread_mdelay(200); // 等待屏幕就绪

    // 初始化屏幕
    TJC_Send("bkcmd=0");           // 关闭返回数据
    TJC_Send("page 0");            // 切换到第0页

    // 清屏为黑色
    TJC_Send("cls BLACK");

    rt_kprintf("串口屏初始化完成\n");
    return RT_EOK;
}

/* 绘制坐标轴 */
void Draw_Axis(void)
{
    // 绘制X轴 (水平中线)
    // line x1,y1,x2,y2,color
    TJC_Send("line 10,%d,470,%d,65535", WAVE_CENTER_Y, WAVE_CENTER_Y); // 白色X轴

    // 绘制Y轴
    TJC_Send("line 10,20,10,252,65535"); // 白色Y轴

    // 绘制刻度线
    for (int i = 0; i <= 4; i++)
    {
        int x = 10 + i * 115;
        TJC_Send("line %d,%d,%d,%d,33808", x, WAVE_CENTER_Y - 5, x, WAVE_CENTER_Y + 5); // 灰色刻度
    }

    // 显示标题
    TJC_Send("xstr 180,5,120,20,0,65535,0,1,1,3,\"Sine Wave\"");
}

/* 绘制正弦波 */
void Draw_SineWave(float phase_offset)
{
    int prev_x = 0, prev_y = 0;

    for (int i = 0; i < SINE_WAVE_POINTS; i++)
    {
        // 计算当前点坐标
        float angle = (float)i / SINE_WAVE_POINTS * 2.0f * 3.14159f + phase_offset;
        int x = 10 + (int)((float)i / SINE_WAVE_POINTS * 460);
        int y = WAVE_CENTER_Y - (int)(sinf(angle) * WAVE_AMPLITUDE);

        // 限制Y坐标范围
        if (y < 20)  y = 20;
        if (y > 252) y = 252;

        if (i > 0)
        {
            // 用青色绘制波形线段
            TJC_Send("line %d,%d,%d,%d,2016", prev_x, prev_y, x, y); // 绿色波形
        }

        prev_x = x;
        prev_y = y;
    }
}

/* 清除波形区域 */
void Clear_WaveArea(void)
{
    // 用黑色填充波形区域
    TJC_Send("fill 11,21,460,231,0"); // fill x,y,w,h,color
}

/* 正弦波动画线程 */
static void sine_wave_thread(void *param)
{
    float phase = 0.0f;

    // 初始化串口屏
    if (TJC_Init() != RT_EOK)
        return;

    // 绘制坐标轴 (只绘制一次)
    Draw_Axis();

    while (1)
    {
        // 清除上一帧波形
        Clear_WaveArea();

        // 绘制新一帧波形
        Draw_SineWave(phase);

        // 更新相位，实现动态滚动效果
        phase += 0.2f;
        if (phase > 2.0f * 3.14159f)
            phase -= 2.0f * 3.14159f;

        rt_thread_mdelay(WAVE_SPEED);
    }
}

/* 启动正弦波显示 */
int sine_wave_start(void)
{
    rt_thread_t thread = rt_thread_create(
        "sine_wave",
        sine_wave_thread,
        RT_NULL,
        2048,       // 栈大小
        20,         // 优先级
        20          // 时间片
    );

    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
        rt_kprintf("正弦波线程已启动\n");
        return RT_EOK;
    }

    rt_kprintf("正弦波线程创建失败\n");
    return RT_ERROR;
}

/* 导出为 MSH 命令，可在串口终端输入 sine_wave 启动 */
MSH_CMD_EXPORT(sine_wave_start, start sine wave display on TJC screen);

