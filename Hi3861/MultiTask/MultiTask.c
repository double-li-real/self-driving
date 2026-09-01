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

/*
 * Copyright (c) 2020 Huawei Device Co., Ltd.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * 综合实验：多任务联动（红外寻线 + 舵机测距 + 消息队列 + 蓝牙UART）
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"
#include "hi_io.h"
#include "hi_time.h"

/******************** 硬件引脚宏定义 ********************/
#define GPIO_L 13          // 红外左
#define GPIO_R 14          // 红外右
#define GPIO_SERVO 2       // 舵机
#define GPIO_TRIG 7        // 超声波触发
#define GPIO_ECHO 8        // 超声波回响
#define UART_IDX WIFI_IOT_UART_IDX_1

/******************** 消息队列定义 ********************/
typedef struct {
    char *Buf;
    uint8_t Idx;
} MSGQUEUE_OBJ_t;

osMessageQueueId_t mid_MsgQueue;
MSGQUEUE_OBJ_t msg;
MSGQUEUE_OBJ_t msg_rx;

/******************** 时间与互斥量 ********************/
osMutexId_t mutex_id;
#define TICK_15S 1500      // 15秒对应的tick数（100tick=1s）
#define ANGLE_LEFT 1000    // 舵机左转45度
#define ANGLE_RIGHT 2000   // 舵机右转45度

/******************** 模块基础函数（整合简化版） ********************/
/* 红外对管寻线 */
void Read_IR(void) {
    WifiIotGpioValue val;
    GpioGetInputVal(GPIO_L, &val);
    if (val == WIFI_IOT_GPIO_VALUE0) printf("IR: Left Black\r\n");
    else printf("IR: Left White\r\n");
    
    GpioGetInputVal(GPIO_R, &val);
    if (val == WIFI_IOT_GPIO_VALUE0) printf("IR: Right Black\r\n");
    else printf("IR: Right White\r\n");
}

/* 舵机驱动 */
void Set_Servo_Angle(uint32_t duty) {
    GpioSetOutputVal(GPIO_SERVO, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(duty);
    GpioSetOutputVal(GPIO_SERVO, WIFI_IOT_GPIO_VALUE0);
    hi_udelay(20000 - duty);
}

/* 超声波测距 */
float Get_Distance(void) {
    static unsigned long start_time, time;
    WifiIotGpioValue value = WIFI_IOT_GPIO_VALUE0;
    unsigned int flag = 0;

    GpioSetDir(GPIO_ECHO, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(GPIO_TRIG, WIFI_IOT_GPIO_DIR_OUT);

    GpioSetOutputVal(GPIO_TRIG, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(GPIO_TRIG, WIFI_IOT_GPIO_VALUE0);

    while (1) {
        GpioGetInputVal(GPIO_ECHO, &value);
        if (value == WIFI_IOT_GPIO_VALUE1 && flag == 0) {
            start_time = hi_get_us();
            flag = 1;
        }
        if (value == WIFI_IOT_GPIO_VALUE0 && flag == 1) {
            time = hi_get_us() - start_time;
            break;
        }
    }
    return time * 0.034 / 2;
}

/******************** 四大核心任务 ********************/
/* 任务1：前15秒红外寻线 */
static void Task_1_Infrared(void) {
    while (1) {
        // 15秒后挂起，等待任务4接管
        if (osKernelGetTickCount() > TICK_15S) {
            osDelay(1000); 
            continue;
        }
        Read_IR();
        osDelay(100); // 交替运行延时
    }
}

/* 任务2：前15秒舵机左右旋转测距（与任务1交替） */
static void Task_2_Servo_Ultrasonic(void) {
    while (1) {
        if (osKernelGetTickCount() > TICK_15S) {
            osDelay(1000); 
            continue;
        }
        
        osMutexAcquire(mutex_id, osWaitForever);
        // 舵机右转，测距
        for (int i = 0; i < 10; i++) Set_Servo_Angle(ANGLE_RIGHT);
        float dist1 = Get_Distance();
        msg.Idx = 1;
        msg.Buf = (char *)"Right";
        osMessageQueuePut(mid_MsgQueue, &msg, 0U, 0U);
        
        // 舵机左转，测距
        for (int i = 0; i < 10; i++) Set_Servo_Angle(ANGLE_LEFT);
        float dist2 = Get_Distance();
        msg.Idx = 2;
        msg.Buf = (char *)"Left";
        osMessageQueuePut(mid_MsgQueue, &msg, 0U, 0U);
        
        // 发送测距数据
        char buf[32];
        sprintf(buf, "Dist: %.1f cm, %.1f cm", dist1, dist2);
        msg.Idx = 3;
        msg.Buf = buf;
        osMessageQueuePut(mid_MsgQueue, &msg, 0U, 0U);
        osMutexRelease(mutex_id);
        
        osDelay(200); // 交替运行延时
    }
}

/* 任务3：消息队列处理与串口打印（全时运行） */
static void Task_3_Queue_Print(void) {
    while (1) {
        if (osMessageQueueGet(mid_MsgQueue, &msg_rx, NULL, osWaitForever) == osOK) {
            printf("[Task3] ID:%d, Data: %s\r\n", msg_rx.Idx, msg_rx.Buf);
        }
    }
}

/* 任务4：15秒后开启蓝牙/UART通信 */
static void Task_4_Bluetooth_UART(void) {
    // 前15秒挂起
    while (osKernelGetTickCount() < TICK_15S) {
        osDelay(100);
    }

    // 初始化UART1 (蓝牙模块通常连接UART)
    WifiIotUartAttribute uart_attr = {
    .baudRate = 9600,
    .dataBits = 8,
    .stopBits = 1,
    .parity = 0,
    };
    UartInit(UART_IDX, &uart_attr, NULL);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_0, WIFI_IOT_IO_FUNC_GPIO_0_UART1_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_1, WIFI_IOT_IO_FUNC_GPIO_1_UART1_RXD);

    printf("Bluetooth/UART Communication Started!\r\n");
    uint8_t uart_buff[256] = {0};

    while (1) {
        // 接收蓝牙数据
        int len = UartRead(UART_IDX, uart_buff, sizeof(uart_buff) - 1);
        if (len > 0) {
            uart_buff[len] = '\0';
            msg.Idx = 100;
            msg.Buf = (char *)uart_buff;
            osMessageQueuePut(mid_MsgQueue, &msg, 0U, 0U);
        }
        osDelay(100);
    }
}

/******************** 主入口与任务创建 ********************/
static void MultiTask_Linkage_Entry(void) {
    GpioInit();
    
    // 初始化硬件引脚
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(GPIO_L, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(GPIO_R, WIFI_IOT_GPIO_DIR_IN);
    
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_IO_FUNC_GPIO_2_GPIO);
    GpioSetDir(GPIO_SERVO, WIFI_IOT_GPIO_DIR_OUT);

    // 创建互斥锁与消息队列
    mutex_id = osMutexNew(NULL);
    mid_MsgQueue = osMessageQueueNew(16, sizeof(MSGQUEUE_OBJ_t), NULL);

    // 创建任务
    osThreadAttr_t attr = {0};
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 4;
    attr.priority = 25;  // 任务1、2、3同优先级，通过延时交替

    attr.name = "Task1_IR";
    osThreadNew((osThreadFunc_t)Task_1_Infrared, NULL, &attr);

    attr.name = "Task2_Servo";
    osThreadNew((osThreadFunc_t)Task_2_Servo_Ultrasonic, NULL, &attr);

    attr.stack_size = 1024 * 8;
    attr.name = "Task3_Queue";
    osThreadNew((osThreadFunc_t)Task_3_Queue_Print, NULL, &attr);

    attr.stack_size = 1024 * 4;
    attr.name = "Task4_Bluetooth";
    osThreadNew((osThreadFunc_t)Task_4_Bluetooth_UART, NULL, &attr);
}

APP_FEATURE_INIT(MultiTask_Linkage_Entry);
