#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "sys.h"

#define USART_REC_LEN  20      // 接收帧缓冲最大长度
#define EN_USART1_RX   1       // 使能(1)/禁止(0) 串口1接收

extern u8 USART_RX_BUF[USART_REC_LEN];   // 串口接收原始缓冲
extern u8 USART_RX_CNT;                  // 接收计数(用于帧头/帧尾同步)

// 解析出的数据帧: [方向A, 速度A, 方向B, 速度B]   (0=正转 1=反转 / 0~150)
extern u8 CAR_buff[4];
extern volatile u8 uart_rec_flag;        // 收到一帧完整数据的标志
extern volatile u32 uart_byte_cnt;        // 已收到的字节计数(调试用, 判断是否收到数据)

void uart_init(u32 bound);

#endif