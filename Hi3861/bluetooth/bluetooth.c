/*
 * Copyright (c) 2020 Huawei Device Co., Ltd.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_uart.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "hi_io.h"

// ==================== 小车运动控制 (UART2 -> STM32) ====================
uint8_t uart_sendbuf[20];

/***
 * 向 STM32 发送数据帧: [0xFC][左方向][左速度][右方向][右速度][0xFD]
 * motorA/motorB: 速度值(0~150), 负值=反转
 */
void stm32motor_control(int motorA, int motorB)
{
    uint8_t A_dir = 0;
    uint8_t B_dir = 0;

    if (motorA < 0) { A_dir = 1; motorA = -motorA; } else { A_dir = 0; }
    if (motorB < 0) { B_dir = 1; motorB = -motorB; } else { B_dir = 0; }

    if (motorA > 150) motorA = 150;
    if (motorB > 150) motorB = 150;

    uart_sendbuf[0] = 0xFC;       // 帧头
    uart_sendbuf[1] = A_dir;      // 左轮方向  0正转 1反转
    uart_sendbuf[2] = motorA;     // 左轮速度
    uart_sendbuf[3] = B_dir;      // 右轮方向
    uart_sendbuf[4] = motorB;     // 右轮速度
    uart_sendbuf[5] = 0xFD;       // 帧尾

    // ===== 调试: 每次向 STM32 发送都打印帧内容(便于观测) =====
    printf("[STM32] send frame: FC %02X %02X %02X %02X FD (A=%d B=%d)\r\n",
           (unsigned int)uart_sendbuf[1], (unsigned int)uart_sendbuf[2],
           (unsigned int)uart_sendbuf[3], (unsigned int)uart_sendbuf[4],
           (A_dir == 1) ? -motorA : motorA, (B_dir == 1) ? -motorB : motorB);
    unsigned int ret = UartWrite(WIFI_IOT_UART_IDX_2, (unsigned char *)uart_sendbuf, 6);
    printf("[STM32] UartWrite ret=%u (0=成功)\r\n", ret);
}

void car_backward(void) { stm32motor_control(-100, -100); }  // 后退
void car_forward(void)  { stm32motor_control(50, 50); }    // 前进
void car_left(void)     { stm32motor_control(50, 150); }    // 左转
void car_right(void)    { stm32motor_control(150, 50); }    // 右转
void car_stop(void)     { stm32motor_control(0, 0); }      // 停止

// ==================== 指令解析 ====================
static void HandleBtCommand(char cmd)
{
    // 小写转大写, 容错
    if (cmd >= 'a' && cmd <= 'z')
    {
        cmd -= 32;
    }

    printf("[BT] cmd: %c\r\n", cmd);
    switch (cmd)
    {
        case 'O': car_stop(); break;                        // 停止
        case 'W': car_forward(); break;                     // 前进
        case 'A': car_left(); break;                        // 左转
        case 'D': car_right(); break;                       // 右转
        case 'S': car_backward(); break;                    // 后退
        case 'I': stm32motor_control(100, 100); break;      // 速度100直行
        case 'K': stm32motor_control(150, 150); break;      // 速度150直行
        default:
            break;   // 忽略回车换行等无效字符
    }
}

// ==================== 蓝牙串口接收任务 (UART1 @ 9600) ====================
#define BT_RX_BUF_SIZE 64

static void BluetoothRecvTask(void)
{
    uint8_t recvBuf[BT_RX_BUF_SIZE] = { 0 };

    printf("[BT] recv task start\r\n");
    while (1)
    {
        int len = UartRead(WIFI_IOT_UART_IDX_1, recvBuf, BT_RX_BUF_SIZE - 1);
        if (len > 0)
        {
            recvBuf[len] = '\0';
            printf("[BT] recv(%d): %s hex:", len, (char *)recvBuf);
            for (int i = 0; i < len; i++)
            {
                printf("%02X ", (unsigned int)recvBuf[i]);
            }
            printf("\r\n");

            // 逐字符处理指令(只处理字母, 忽略回车换行等干扰)
            for (int i = 0; i < len; i++)
            {
                if ((recvBuf[i] >= 'A' && recvBuf[i] <= 'Z') ||
                    (recvBuf[i] >= 'a' && recvBuf[i] <= 'z'))
                {
                    HandleBtCommand((char)recvBuf[i]);
                }
            }
            memset(recvBuf, 0, sizeof(recvBuf));
        }
        usleep(20000);   // 20ms 轮询一次
    }
}

// ==================== 主入口 ====================
static void CarRemoteControl(void)
{
    printf("[car] init start\r\n");
    GpioInit();

    // ========== 实验版: 先把两个UART的所有GPIO复用设好, 再依次初始化 ==========
    // 1) 设好全部引脚复用
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_0, WIFI_IOT_IO_FUNC_GPIO_0_UART1_TXD);   // 蓝牙模块 TX
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_1, WIFI_IOT_IO_FUNC_GPIO_1_UART1_RXD);   // 蓝牙模块 RX
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD); // STM32 TX
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD); // STM32 RX

    // 2) 依次初始化 UART1(蓝牙模块, 9600)
    WifiIotUartAttribute uart_attr1 = {
        .baudRate = 9600,   // 蓝牙串口模块波特率(按模块实际配置调整)
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    unsigned int ret1 = UartInit(WIFI_IOT_UART_IDX_1, &uart_attr1, NULL);
    printf("[car] UART1 init ret=%u (0=成功) <- bluetooth module\r\n", ret1);

    // 3) 再初始化 UART2(STM32, 115200)
    WifiIotUartAttribute uart_attr2 = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    unsigned int ret2 = UartInit(WIFI_IOT_UART_IDX_2, &uart_attr2, NULL);
    printf("[car] UART2 init ret=%u (0=成功) -> STM32\r\n", ret2);

    // ========== 创建蓝牙接收任务 ==========
    osThreadAttr_t attr;
    attr.name = "bt_recv";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 4;
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)BluetoothRecvTask, NULL, &attr) == NULL)
    {
        printf("[car] Failed to create bt_recv task!\r\n");
    }

    printf("[car] init done, waiting for bluetooth command...\r\n");
    printf("[car] W=前进 A=左转 D=右转 S=后退 O=停止 I=速度100 K=速度150\r\n");
}

APP_FEATURE_INIT(CarRemoteControl);
