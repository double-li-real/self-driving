#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "hal_bsp_sht20.h" // 确保这个头文件在你的 include 路径中

// (2) 变量创建
osSemaphoreId_t sem1;

// (4) 任务函数实现
void thread1(void)
{
    while (1)
    {
        // 1秒中释放两次sem1信号量，使得Thread2和Thread3能同步执行
        // 此处若只释放一次信号量，则Thread2和Thread3会交替运行。
        osSemaphoreRelease(sem1);
        osSemaphoreRelease(sem1);
        
        printf("\n");
        printf("Thread1 释放信号量!\n");
        
        osDelay(300); // 延时3秒 (假设系统Tick为10ms, 300*10ms = 3000ms)
    }
}

void thread2(void)
{
    float temperature = 0, humidity = 0;
    
    printf("i2c_sht20_demo() !\n");
    SHT20_Init(); // SHT20初始化
    
    while (1)
    {
        // 等待sem1信号量
        osSemaphoreAcquire(sem1, osWaitForever);
        
        SHT20_ReadData(&temperature, &humidity);
        printf("temperature = %.2f    humidity = %.2f\r\n", temperature, humidity);
        printf("Thread2 得到信号量!\n");
        
        osDelay(1); // 延时10ms
    }
}

void thread3(void)
{
    while (1)
    {
        // 等待sem1信号量
        osSemaphoreAcquire(sem1, osWaitForever);
        
        printf("Thread3 得到信号量!\n");
        
        osDelay(1); // 延时10ms
    }
}

// (3) 任务创建 (作为入口函数)
static void i2c_sht20_demo(void)
{
    osThreadAttr_t attr;
    
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 4; // 栈大小 4KB
    
    // 创建 Thread1
    attr.name = "thread1";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)thread1, NULL, &attr) == NULL)
    {
        printf("Failed to create thread1!\n");
    }
    
    // 创建 Thread2
    attr.name = "thread2";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)thread2, NULL, &attr) == NULL)
    {
        printf("Failed to create thread2!\n");
    }
    
    // 创建 Thread3
    attr.name = "thread3";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)thread3, NULL, &attr) == NULL)
    {
        printf("Failed to create thread3!\n");
    }
    
    // 创建信号量：初始值为0，最大值为4
    sem1 = osSemaphoreNew(4, 0, NULL);
    if (sem1 == NULL)
    {
        printf("Failed to create Semaphore1!\n");
    }
}

APP_FEATURE_INIT(i2c_sht20_demo);
