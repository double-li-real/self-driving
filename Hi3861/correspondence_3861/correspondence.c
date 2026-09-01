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
#include <unistd.h>
#include <memory.h>
#include "wifiiot_uart.h"
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"
#include "wifiiot_pwm.h"
#include "hi_pwm.h"
#include "hi_uart.h"
#include "wifiiot_gpio_ex.h"

static void thread1(void);
//static void thread2(void);

uint8_t uart_sendbuf[20];
osMutexId_t mutex_id;

/***通信协议***/
/*
函数功能 ：发送至stm32的数据协议
参数     ： 电机实际转速的一百倍，例如：设置转速为1rad/s，则传入100
*/
void stm32motor_control(int motorA, int motorB)
{
    uint8_t A_dir = 0;
    uint8_t B_dir = 0;

    //小车运动方向 前进（正转）：0    后退（反转）  1
    if(motorA < 0){
        A_dir = 1;
        motorA = -motorA;
    }else{
        A_dir = 0;
    }
    
    if(motorB < 0){
        B_dir = 1;
        motorB = -motorB;
    }else{
        B_dir = 0;
    }
    
    //限制幅度 -150 ~ 150
    if (motorA > 150)
    {
        motorA = 150;
    }
    if (motorB > 150)
    {
        motorB = 150;
    }

    // 数据协议
    uart_sendbuf[0] = 0xFC;       // 帧头
    uart_sendbuf[1] = A_dir;      // 左轮方向    0正转，1反转
    uart_sendbuf[2] = motorA;     // 左轮速度
    uart_sendbuf[3] = B_dir;      // 右轮方向    0正转，1反转
    uart_sendbuf[4] = motorB;     // 右轮速度
    uart_sendbuf[5] = 0xFD;       // 帧尾
    
    UartWrite(WIFI_IOT_UART_IDX_2, (unsigned char *)uart_sendbuf, 6);
}

// 小车后退
void car_backward(void)
{
    stm32motor_control(-100, -100);
}

// 小车前进
void car_forward(void)
{
    stm32motor_control(100, 100);
}

// 小车左转
void car_left(void)
{
    stm32motor_control(50, 150);
}

// 小车右转
void car_right(void)
{
    stm32motor_control(150, 50);
}

// 小车停止
void car_stop(void)
{
    stm32motor_control(0, 0);
}

/*****任务一*****/
static void thread1(void)
{
    printf("[correspondence] thread1 start (motion sequence)\r\n");
    while (1)
    {
        // ========== 运动序列: 改这里即可改变小车运动 ==========
        car_forward();                 // 前进 2秒
        printf("[correspondence] send FORWARD(100,100)\r\n");
        usleep(2000000);

        car_left();                    // 左转 2秒
        printf("[correspondence] send LEFT(50,150)\r\n");
        usleep(2000000);

        car_right();                   // 右转 2秒
        printf("[correspondence] send RIGHT(150,50)\r\n");
        usleep(2000000);

        car_backward();                // 后退 2秒
        printf("[correspondence] send BACKWARD(-100,-100)\r\n");
        usleep(2000000);

        car_stop();                    // 停止 1秒
        printf("[correspondence] send STOP(0,0)\r\n");
        usleep(1000000);
        // =====================================================
    }
}

/*****任务二*****/
/***static void thread2(void)
{
    sleep(1); // 休眠1秒
    printf("[correspondence] thread2 start (LEFT)\r\n");
    while (1)
    {
        // 获取互斥锁
        osMutexAcquire(mutex_id, osWaitForever);
        car_left();       // 左转
        printf("[correspondence] send LEFT(50,150)\r\n");
        usleep(1000000);  // 延时1s

        // // 释放互斥锁
        osMutexRelease(mutex_id);
    }
}***/

static void correspondence(void)
{
    printf("[correspondence] init start\r\n");
    GpioInit(); // GPIO功能初始化
    
    /***********************通讯串口初始化***********************/
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD); // GPIO_11复用为UART2_TXD
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD); // GPIO_12复用为UART2_RX

    /*****************串口参数*****************/
    WifiIotUartAttribute uart_attr2 = {
        // 波特率：115200
        .baudRate = 115200,
        // 数据位：8bits
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    UartInit(WIFI_IOT_UART_IDX_2, &uart_attr2, NULL);
    printf("[correspondence] UART2 init ok (115200,8N1)\r\n");

    // 先创建互斥锁, 再创建任务 (避免任务启动时锁还未创建导致 osMutexAcquire(NULL) 崩溃)
    mutex_id = osMutexNew(NULL);
    if (mutex_id == NULL)
    {
        printf("Falied to create Mutex!\n");
    }
    printf("[correspondence] mutex created\r\n");

    osThreadAttr_t attr;
    attr.attr_bits = 0U;         // 设置osThraedJoin是否可以使用
    attr.cb_mem = NULL;          // 控制块指针设置
    attr.cb_size = 0U;           // 控制块指针大小
    attr.stack_mem = NULL;       // 任务栈设置
    attr.stack_size = 1024 * 4;  // 任务栈大小
    
    // 创建任务1
    attr.name = "thread1";       // 创建任务名称
    attr.priority = 25;          // 任务优先级
    if (osThreadNew((osThreadFunc_t)thread1, NULL, &attr) == NULL)
    {
        printf("Falied to create thread1!\n");
    }
    printf("[correspondence] thread1 created\r\n");
    
    // 创建任务2
    /***attr.name = "thread2";       // 创建任务名称
    attr.priority = 25;          // 任务优先级
    if (osThreadNew((osThreadFunc_t)thread2, NULL, &attr) == NULL)
    {
        printf("Falied to create thread2!\n");
    }
    printf("[correspondence] thread2 created, init done\r\n");***/
}

APP_FEATURE_INIT(correspondence); // 启动任务
