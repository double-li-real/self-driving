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
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "hal_bsp_ap3216c.h"
#include "wifiiot_gpio.h"      // 新增：GPIO控制
#include "wifiiot_gpio_ex.h"   // 新增：GPIO引脚定义

#define LED_PIN WIFI_IOT_IO_NAME_GPIO_9
/**
 * 功能说明：
 * ir: 人体红外传感器 (Infrared)
 * als: 光强传感器 (Ambient Light)
 * ps: 接近传感器 (Proximity)
 */
void Task1(void)
{
    // 1. 初始化 GPIO (用于控制LED)    
    GpioInit();  
    IoSetFunc(LED_PIN, WIFI_IOT_IO_FUNC_GPIO_9_GPIO); // 设置引脚复用为GPIO   
    GpioSetDir(LED_PIN, WIFI_IOT_GPIO_DIR_OUT);       // 设置为输出模式   
    GpioSetOutputVal(LED_PIN, 0);                     // 初始状态：灭 (假设低电平灭)   
    // 2. 初始化传感器 (调用你目录里的驱动)
    AP3216C_Init(); 
    
    printf("=== Smart Light System Started ===\n");

    uint16_t ir = 0, als = 0, ps = 0;

    // 2. 循环读取数据
    while (1)
    {
    AP3216C_ReadData(&ir, &als, &ps);
    
    // 条件：环境光弱 (als < 100) 且 有人靠近 (ps > 50)
    if (als < 100 && ps > 50) 
    {
        // 满足条件：开灯 (低电平点亮还是高电平点亮取决于你的硬件，通常是低电平 0)
        GpioSetOutputVal(WIFI_IOT_IO_NAME_GPIO_9, 0); 
        printf("[AUTO] Dark & Someone Near -> LED ON\n");
    }
    else
    {
        // 不满足条件：关灯
        GpioSetOutputVal(WIFI_IOT_IO_NAME_GPIO_9, 1); 
    }

    printf("人体红外传感器=%d, 光强传感器=%d, 接近传感器=%d\n", ir, als, ps);
    sleep(1);
    }
}

/**
 * 入口函数：创建线程
 */
static void i2c_ap3216c_demo(void)
{
    osThreadAttr_t options;
    
    // 配置线程属性
    options.name       = "thread_1";
    options.attr_bits  = 0;
    options.cb_mem     = NULL;
    options.cb_size    = 0;
    options.stack_mem  = NULL;
    options.stack_size = 1024; // 栈大小
    options.priority   = osPriorityNormal; // 优先级

    osThreadId_t Task1_ID;

    // 创建线程
    Task1_ID = osThreadNew((osThreadFunc_t)Task1, NULL, &options);

    if (Task1_ID != NULL)
    {
        printf("ID = %d, Create Task1_ID is OK!\n", Task1_ID);
    }
    else
    {
        printf("Create Task1 Failed!\n");
    }
}

APP_FEATURE_INIT(i2c_ap3216c_demo);
