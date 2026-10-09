#include "USART.h"

#define ONE_DATA_MAXLEN                  100         /* 不定长数据的最大长度 */

static struct rt_semaphore UART3_rx_sem;  // 定义一个静态的信号量变量 UART3_rx_sem
static struct rt_semaphore UART5_rx_sem;
static rt_device_t serial3;          // 定义一个静态的串口设备变量 serial3
static rt_device_t serial5;
rt_uint8_t Serial3_RxPacket[ONE_DATA_MAXLEN];   // 定义一个字符数组 Serial3_RxPacket，用于存储不定长数据
rt_uint8_t Serial3_RxFlag;                      // 定义一个变量，用于串口3接收标志位
rt_uint8_t Serial5_RxPacket[ONE_DATA_MAXLEN];   // 定义一个字符数组 Serial3_RxPacket，用于存储不定长数据
rt_uint8_t Serial5_RxFlag;                      // 定义一个变量，用于串口3接收标志位

/* uart3接收数据回调函数 */
static rt_err_t UART3_rx_ind(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    if (size > 0) // 如果接收到的数据大小大于 0
    {
        rt_sem_release(&UART3_rx_sem); // 释放信号量，表示有数据可读
    }
    return RT_EOK; // 返回正常状态
}

/* uart5接收数据回调函数 */
static rt_err_t UART5_rx_ind(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    if (size > 0) // 如果接收到的数据大小大于 0
    {
        rt_sem_release(&UART5_rx_sem); // 释放信号量，表示有数据可读
    }
    return RT_EOK; // 返回正常状态
}

/*usart3接收函数*/
static char uart3_get_char(void)
{
    char ch; // 定义一个字符变量 ch

    while (rt_device_read(serial3, 0, &ch, 1) == 0) // 0-没有读到任何数据 1-成功读到一个数据 只要没有字符返回就一直循环
    {
        rt_sem_control(&UART3_rx_sem, RT_IPC_CMD_RESET, RT_NULL); // 重置信号量为 0
        rt_sem_take(&UART3_rx_sem, RT_WAITING_FOREVER);           // 等待信号量，表示等待数据到来
    }
    return ch; // 返回读到的字符
}

static char uart5_get_char(void)
{
    char ch; // 定义一个字符变量 ch

    while (rt_device_read(serial5, 0, &ch, 1) == 0) // 循环读取串口设备，直到读到一个字符
    {
        rt_sem_control(&UART5_rx_sem, RT_IPC_CMD_RESET, RT_NULL); // 重置信号量为 0
        rt_sem_take(&UART5_rx_sem, RT_WAITING_FOREVER); // 等待信号量，表示等待数据到来
    }
    return ch; // 返回读到的字符
}

/* uart3数据解析线程 */
static void UART3_data_parsing(void)
{
    static char pRxPacket = 0; // 定义一个静态的字符变量 pRxPacket，用于记录数据的长度
    static rt_uint8_t RxState = 0; //静态变量，函数进入时只初始化一次

    while (1) // 循环执行以下操作
    {
        rt_uint8_t RxData = uart3_get_char(); // 调用函数获取一个字符
        //状态0:等待帧头0x40
        if (RxState == 0)
        {
            //如果收到帧头并且之前没有处理过任何数据
            if (RxData == 0x40 && Serial3_RxFlag == 0)
            {
                RxState = 1;
                pRxPacket = 0;
                continue;
            }
        }
        //状态1:接收真正的数据内容
        else if (RxState == 1)
        {
            //收到0xFF->代表帧尾开始,切换到状态2
            if (RxData == 0xFF)
            {
                RxState = 2;
                continue;
            }
            //否则把字符存到缓冲区里面,下标加1
            else
            {
                Serial3_RxPacket[pRxPacket] = RxData;
                pRxPacket++;
                continue;
            }
        }
        //状态2:等待第二个帧尾
        else if (RxState == 2)
        {
            if (RxData == 0xFF)
            {
                RxState = 3;
                continue;
            }
        }
        //状态3:等待第三个帧尾
        else if (RxState == 3)
        {
            if (RxData == 0xFF)
            {
                RxState = 0;//重置为初始状态,准备接收下一帧
                Serial3_RxPacket[pRxPacket] = '\0';//给数据加结束符(变成字符串)
                Serial3_RxFlag = 1;//接收标志位为1
                rt_kprintf("UART3_RX:%s\n", Serial3_RxPacket);
                continue;
            }
        }
        pRxPacket = (pRxPacket >= ONE_DATA_MAXLEN - 1) ? ONE_DATA_MAXLEN - 1 : pRxPacket; // 如果 pRxPacket 超过了最大长度，就将其限制在最大长度减一（留出一个空位给空字符）
        rt_thread_mdelay(20);
    }
}

/* uart5数据解析线程 */
static void UART5_data_parsing(void)
{
    static char pRxPacket = 0; // 定义一个静态的字符变量 pRxPacket，用于记录数据的长度
    static rt_uint8_t RxState = 0; //静态变量，函数进入时只初始化一次

    while (1) // 循环执行以下操作
    {
        rt_uint8_t RxData = uart5_get_char(); // 调用函数获取一个字符
        if (RxState == 0)
        {
            if (RxData == 0x40 && Serial5_RxFlag == 0)
            {
                RxState = 1;
                pRxPacket = 0;
                continue;
            }
        }
        else if (RxState == 1)
        {
            if (RxData == 0xFF)
            {
                RxState = 2;
                continue;
            }
            else
            {
                Serial5_RxPacket[pRxPacket] = RxData;
                pRxPacket++;
                continue;
            }
        }
        else if (RxState == 2)
        {
            if (RxData == 0xFF)
            {
                RxState = 3;
                continue;
            }
        }
        else if (RxState == 3)
        {
            if (RxData == 0xFF)
            {
                RxState = 0;
                Serial5_RxPacket[pRxPacket] = '\0';
                Serial5_RxFlag = 1;
                rt_kprintf("UART5_RX:%s\n", Serial5_RxPacket);
                continue;
            }
        }
        pRxPacket = (pRxPacket >= ONE_DATA_MAXLEN - 1) ? ONE_DATA_MAXLEN - 1 : pRxPacket; // 如果 pRxPacket 超过了最大长度，就将其限制在最大长度减一（留出一个空位给空字符）
        rt_thread_mdelay(30);
    }
}

void HMILCD_Send(char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    rt_device_write(serial3, 0, String, rt_strlen(String));
    uint8_t end_data[3] = { 0xFF, 0xFF, 0xFF };
    rt_device_write(serial3, 0, end_data, 3);
}

/* 向ESP32发送格式化字符串，末尾自动加换行符 */
void Serial5_Send(char *format, ...)
{
    char String[130];  // 多2字节给\n和\0
    va_list arg;
    va_start(arg, format);
    vsnprintf(String, 128, format, arg);  // 最多写128字节，防止越界
    va_end(arg);
    rt_size_t len = rt_strlen(String);
    String[len]     = '\n';
    String[len + 1] = '\0';
    len++;
    for (rt_size_t i = 0; i < len; i++)
    {
        uint32_t timeout = 10000;
        while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET)
        {
            if (--timeout == 0) return;
        }
        USART_SendData(UART5, (uint8_t)String[i]);
    }
}

int UART_Init(void)
{
    rt_err_t ret = RT_EOK; // 定义一个错误码变量 ret，并初始化为正常状态

    /* 查找系统中的串口设备 */
    serial3 = rt_device_find("uart3"); // 调用函数查找 uart3 对应的串口设备，并将其赋值给 serial3 变量
    if (!serial3) // 如果没有找到串口设备
    {
        rt_kprintf("find uart3 failed!\n"); // 打印错误信息到控制台
        return RT_ERROR; // 返回错误状态
    }

    /* 查找系统中的串口设备 */
    serial5 = rt_device_find("uart5"); // 调用函数查找 uart5 对应的串口设备，并将其赋值给 serial5 变量
    if (!serial5) // 如果没有找到串口设备
    {
        rt_kprintf("find uart5 failed!\n"); // 打印错误信息到控制台
        return RT_ERROR; // 返回错误状态
    }

    rt_sem_init(&UART3_rx_sem, "UART3_rx_sem", 0, RT_IPC_FLAG_FIFO);
    rt_device_open(serial3, RT_DEVICE_FLAG_INT_RX);
    rt_device_set_rx_indicate(serial3, UART3_rx_ind);
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 921600;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART3, &USART_InitStructure);

    rt_sem_init(&UART5_rx_sem, "UART5_rx_sem", 0, RT_IPC_FLAG_FIFO);
    rt_device_open(serial5, RT_DEVICE_FLAG_INT_RX);
    rt_device_set_rx_indicate(serial5, UART5_rx_ind);
    USART_InitStructure.USART_BaudRate = 921600;
    USART_Init(UART5, &USART_InitStructure);

    /* 创建 UART3 线程，命名为 UART3，执行函数为 UART3_data_parsing，参数为空，栈大小为 1024 字节，优先级为 25，时间片为 25 个 tick */
    rt_thread_t thread = rt_thread_create("UART3", (void (*)(void *parameter)) UART3_data_parsing, RT_NULL, 1024, 23,
            30);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread); // 调用函数启动线程
    }
    else // 否则（线程创建失败）
    {
        ret = RT_ERROR; // 将 ret 设置为错误状态
    }

    /* 创建 UART5 线程，命名为 UART5，执行函数为 UART5_data_parsing，参数为空，栈大小为 1024 字节，优先级为 25，时间片为 25 个 tick */
    rt_thread_t thread5 = rt_thread_create("UART5", (void (*)(void *parameter)) UART5_data_parsing, RT_NULL, 1024, 25,
            30);
    /* 创建成功则启动线程 */
    if (thread5 != RT_NULL)
    {
        rt_thread_startup(thread5); // 调用函数启动线程
    }
    else // 否则（线程创建失败）
    {
        ret = RT_ERROR; // 将 ret 设置为错误状态
    }

    return ret; // 返回 ret 的值
}

