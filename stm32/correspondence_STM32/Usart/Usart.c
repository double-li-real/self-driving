#include "sys.h"
#include "usart.h"

// 加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
struct __FILE
{
    int handle;
};

FILE __stdout;
_sys_exit(int x)
{
    x = x;
}
int fputc(int ch, FILE *f)
{
    while((USART1->SR&0X40)==0);//循环发送,直到发送完毕
    USART1->DR = (u8) ch;
    return ch;
}
#endif

#if EN_USART1_RX   //如果使能了接收

u8 USART_RX_BUF[USART_REC_LEN];   //接收缓冲
u8 USART_RX_CNT = 0;              //接收计数
u8 CAR_buff[4] = {0,0,0,0};       //解析出的数据帧: 方向A,速度A,方向B,速度B
volatile u8 uart_rec_flag = 0;    //收到一帧完整数据标志
volatile u32 uart_byte_cnt = 0;    //已收到的字节计数(调试用)

void uart_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);  //使能USART1,GPIOA时钟

    //USART1_TX   GPIOA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      //复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //USART1_RX   GPIOA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;         //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //USART 初始化设置: 115200, 8位数据, 1停止位, 无校验
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  //开启串口接收中断
    USART_Cmd(USART1, ENABLE);                      //使能串口1
}

// 串口1中断服务程序: 按 0xFC帧头 + 4字节数据 + 0xFD帧尾 解析一帧
void USART1_IRQHandler(void)
{
    u8 Res;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)   //判断中断类型
    {
        Res = USART_ReceiveData(USART1);                    //读取收到的数据
        uart_byte_cnt++;                                     //调试: 字节计数+1
        if(USART_RX_CNT >= 6) USART_RX_CNT = 0;             //防止超长数据越界,重新同步帧头
        USART_RX_BUF[USART_RX_CNT] = Res;

        if(USART_RX_BUF[0] == 0xFC)   //寻找帧头
            USART_RX_CNT++;
        else
            USART_RX_CNT = 0;

        if(USART_RX_CNT == 6 && USART_RX_BUF[5] == 0xFD)   //找到帧尾: 一帧完整数据
        {
            USART_RX_CNT = 0;
            CAR_buff[0] = USART_RX_BUF[1];   //方向A(左轮)
            CAR_buff[1] = USART_RX_BUF[2];   //速度A(左轮 0~150)
            CAR_buff[2] = USART_RX_BUF[3];   //方向B(右轮)
            CAR_buff[3] = USART_RX_BUF[4];   //速度B(右轮 0~150)
            memset(USART_RX_BUF, 0, 6);
            uart_rec_flag = 1;               //串口帧标志
        }
    }
    USART_ClearFlag(USART1, USART_FLAG_RXNE); //清除接收标志
}

#endif
