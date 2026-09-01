#include "stm32f10x.h"
#include "sys.h"
#include "motor.h"
#include "encoder.h"
#include "control_system.h"

int main(void)
{
    Stm32_Clock_Init(9);                 //外部时钟8MHz, 9倍频 -> 72MHz
    MY_NVIC_PriorityGroupConfig(2);      //中断优先级分组
    uart_init(115200);                   //串口1初始化为115200(接收Hi3861的数据帧)
    JTAG_Set(JTAG_SWD_DISABLE);          //关闭JTAG接口
    JTAG_Set(SWD_ENABLE);                //打开SWD接口, 可利用主板的SWD接口调试

    Encoder_Init_TIM2();                 //初始化左电机编码器
    Encoder_Init_TIM3();                 //初始化右电机编码器
    PWM_Init(7199, 9);                   //定时器PWM初始化(电机驱动)
    colorful_led_Init();                 //炫彩灯初始化(含倒车灯)

    SysTick_Config(72000000/1000);       //滴答定时器, 每1ms触发一次中断

    printf("STM32双机通信已启动\r\n");
    /*主程序: 控制逻辑由SysTick+串口中断完成 */
    {
        volatile u32 dbg_tick = 0;
        u32 last_cnt = 0;
        while (1)
        {
            delay_ms(100);
            if (++dbg_tick % 5 == 0)   //每500ms打印一次心跳
            {
                printf("[心跳] bytes=%d frames=%d flag=%d Lc=%d Rc=%d\r\n",
                       (int)uart_byte_cnt, (int)frame_cnt, (int)uart_rec_flag,
                       L_coder, R_coder);
            }
            if (frame_cnt != last_cnt)   //刚解析到新帧: 打印帧内容
            {
                last_cnt = frame_cnt;
                printf("[帧] dA=%d sA=%d dB=%d sB=%d (0.01转/s)\r\n",
                       (int)last_frame[0], (int)last_frame[1],
                       (int)last_frame[2], (int)last_frame[3]);
            }
        }
    }
}
