#include <stdio.h>
#include <string.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "hal_bsp_ssd1306.h"

void Task1(void);   // 前置声明

static void i2c_ssd1306_demo(void)
{
    osThreadAttr_t options;
    options.name = "thread_1";
    options.attr_bits = 0;
    options.cb_mem = NULL;
    options.cb_size = 0;
    options.stack_mem = NULL;
    options.stack_size = 2048;          // ① 1024->2048，防 sprintf+OLED 栈溢出
    options.priority = osPriorityNormal;

    osThreadId_t Task1_ID;
    Task1_ID = osThreadNew((osThreadFunc_t)Task1, NULL, &options);
    if (Task1_ID != NULL)
    {
        printf("ID = %p, Create Task1_ID is OK!\r\n", (void *)Task1_ID);  // ② %d->%p
    }
}

void Task1(void)
{
    uint8_t displayBuff[20] = {0};
    uint8_t hour = 16, min = 0, sec = 0;

    SSD1306_Init();
    SSD1306_CLS();
    SSD1306_ShowStr(0, 0, (uint8_t *)"   QST CAR   ", 16);
    SSD1306_ShowStr(0, 3, (uint8_t *)"2025:10:08", 16);

    while (1)
    {
        sec++;
        if (sec > 59) { sec = 0; min++; }
        if (min > 59) { min = 0; hour++; }
        if (hour > 23) { hour = 0; }

        memset(displayBuff, 0, sizeof(displayBuff));
        sprintf((char *)displayBuff, "%02d:%02d:%02d", hour, min, sec);
        SSD1306_ShowStr(0, 2, (uint8_t *)displayBuff, 16);

        osDelay(1000);   // ③ sleep(1)->osDelay(1000)，CMSIS标准API，单位ms
    }
}

APP_FEATURE_INIT(i2c_ssd1306_demo);
