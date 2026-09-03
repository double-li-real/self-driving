/*
 * Auto_Drive_2min: 教室自主行驶2分钟 - 超声波避障 + 红外黑胶带禁入区
 *
 * 需求: 小车放在教室里自动行驶约2分钟, 途中不撞障碍、不进入黑胶带圈禁入区,
 *       行驶过程连续匀速, 只在遇到障碍/黑线时停下处理。
 *
 * 方案(QST小车板硬件):
 *   超声波 HC-SR04 装在 GPIO2 舵机云台上, 可左/中/右转向测距 (TRIG=GPIO7,ECHO=GPIO8)
 *   红外对管 GPIO13=左/GPIO14=右 (循迹模块TC_OUT_L/R, 与STM32 PA11/12并联)
 *   电机: Hi3861 经 UART2(GPIO11/12) 发协议帧指挥 STM32 (115200, 0xFC+dirA+spdA+dirB+spdB+0xFD)
 *
 * 行为:
 *   上电: 红外白地校准1s(车放白地) -> 云台居中 -> 开始2分钟行驶
 *   [前进] 给油起步(150/115/90各0.12s再落巡航70)后连续匀速前进:
 *      - 每轮重发前进帧; 每轮测一次前方
 *      - 距离过滤: <3cm杂波忽略; <6cm 单次立即判障; 其余按 <12cm(重)/<30cm(轻) 累计,
 *        总分>=3 才判障(连续确认, 消除偶发假回波造成的"动一下停一下")
 *      - 避险/绕行后有冷却期(期间只前进不判障), 避免刚起步又误判
 *      - 前方>60cm时周期转动舵机测左/右(观察两侧, 测完回中)
 *      - 每10ms查一次红外, 压线即绕行
 *   [遇障] 反转强刹 -> 后退 -> 云台扫左右选较空侧 -> 转向90/180 -> 回到前进
 *   120秒到: 停车结束
 *
 * 实车调参都在本文件顶部宏。
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

/* ================= 引脚 ================= */
#define PIN_SG90         WIFI_IOT_IO_NAME_GPIO_2   /* 舵机云台 */
#define PIN_TRIG         7                         /* 超声波 TRIG */
#define PIN_ECHO         8                         /* 超声波 ECHO */
#define UART_STM32       WIFI_IOT_UART_IDX_2       /* 与STM32通信 */
#define PIN_IR_L         WIFI_IOT_IO_NAME_GPIO_13  /* 红外左 */
#define PIN_IR_R         WIFI_IOT_IO_NAME_GPIO_14  /* 红外右 */

/* ================= 舵机角度脉宽 us(实车标定): 左/中/右 ================= */
#define SG90_LEFT        2200
#define SG90_MID         1650
#define SG90_RIGHT       1100

/* ================= 运动参数(按实车微调) ================= */
#define SPEED_FWD        70                        /* 前进速度(巡航) */
#define SPEED_BRAKE      100                       /* 反转制动力 */
#define SPEED_BACK       80                        /* 后退速度 */
#define SPEED_TURN       100                       /* 转向速度(原地差速) */
#define BRAKE_MS         250                       /* 强力制动时长 */
#define BACKUP_MS        700                       /* 遇障后后退时长 */
#define TURN_90_MS       1500                      /* 转向较空侧时长(约90度) */
#define TURN_180_MS      3000                      /* 两侧都堵调头180度 */
#define REST_MS          200                       /* 转向后停顿 */

/* ============ 起步动力补偿(静止起步要人推: STM32增量PID从0起步力矩小) ============ */
#define KICK_HI          150
#define KICK_MID         115
#define KICK_LO          90
#define KICK_MS          120                       /* 每档持续时间ms */

/* ============ 红外黑胶带参数 ============ */
#define TAPE_BACK_MS     300
#define TAPE_PIVOT_MS    700
#define TAPE_COOLDOWN_MS 400                       /* 绕行后短暂冷却, 防刚恢复又触发 */

/* ================= 避障判定阈值 ================= */
#define STOP_TRIGGER_CM  30                        /* 前方低于此值累计(轻) */
#define EMERGENCY_CM     12                        /* 前方低于此值累计(重) */
#define SIDE_MIN_CM      35                        /* 转向侧最小可用距离 */
#define FAR_CM           300                       /* 超过此距离视为畅通 */
#define NOISE_MIN_CM     3                         /* 小于此值视为杂波, 忽略 */
#define RETRIG_COOLDOWN_MS 900                     /* 避险后的冷却期: 只前进不判障 */

/* ================= 时序/周期观察参数 ================= */
#define RUN_TOTAL_MS     120000
#define SAMPLE_PAD_US    40000                     /* 测距间隔填充 */
#define SCAN_PULSES      30                        /* 避障扫描舵机脉冲数 */
#define SCAN_SETTLE_MS   200                       /* 扫描后等舵机到位 */
#define OBS_PULSES       12                        /* 巡航中观察左右: 舵机脉冲数(短扫) */
#define OBS_SETTLE_MS    150
#define SIDE_OBS_CM      60                        /* 前方超过该距离才周期观察左右 */
#define SIDE_OBS_CYCLE   12                        /* 每12轮(~1s)观察一次左右 */

/* ================= 舵机驱动(软件PWM, 20ms周期) ================= */
static void sg90_pulse(unsigned int duty)
{
    GpioSetDir(PIN_SG90, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetOutputVal(PIN_SG90, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(duty);
    GpioSetOutputVal(PIN_SG90, WIFI_IOT_GPIO_VALUE0);
    hi_udelay(20000 - duty);
}
static void sg90_goto(unsigned int duty, int pulses)
{
    int i;
    for (i = 0; i < pulses; i++) {
        sg90_pulse(duty);
    }
}

/* ================= 超声波测距(HC-SR04, 返回cm) ================= */
static float hcsr04_get_distance(void)
{
    WifiIotGpioValue value = WIFI_IOT_GPIO_VALUE0;
    uint64_t t0, t1;
    float cm;

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_7, WIFI_IOT_IO_FUNC_GPIO_7_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_IO_FUNC_GPIO_8_GPIO);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_7, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_GPIO_DIR_IN);

    GpioSetOutputVal(PIN_TRIG, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(PIN_TRIG, WIFI_IOT_GPIO_VALUE0);

    t0 = hi_get_us();
    while (1) {
        GpioGetInputVal(PIN_ECHO, &value);
        if (value == WIFI_IOT_GPIO_VALUE1) {
            break;
        }
        if ((hi_get_us() - t0) > 30000) {
            return (float)FAR_CM;
        }
    }
    t1 = hi_get_us();
    while (1) {
        GpioGetInputVal(PIN_ECHO, &value);
        if (value == WIFI_IOT_GPIO_VALUE0) {
            break;
        }
        if ((hi_get_us() - t1) > 40000) {
            return (float)FAR_CM;
        }
    }
    t1 = hi_get_us() - t1;
    cm = (float)t1 * 0.034f / 2.0f;
    if (cm > FAR_CM) {
        cm = (float)FAR_CM;
    }
    return cm;
}

/* ================= UART2 电机协议 ================= */
static uint8_t uart_sendbuf[6];
static void stm32motor_control(int motorA, int motorB)
{
    uint8_t A_dir = (motorA < 0) ? 1 : 0;
    uint8_t B_dir = (motorB < 0) ? 1 : 0;
    if (motorA < 0) motorA = -motorA;
    if (motorB < 0) motorB = -motorB;
    if (motorA > 150) motorA = 150;
    if (motorB > 150) motorB = 150;
    uart_sendbuf[0] = 0xFC;
    uart_sendbuf[1] = A_dir;
    uart_sendbuf[2] = (uint8_t)motorA;
    uart_sendbuf[3] = B_dir;
    uart_sendbuf[4] = (uint8_t)motorB;
    uart_sendbuf[5] = 0xFD;
    UartWrite(UART_STM32, uart_sendbuf, 6);
}
static void car_forward(void)  { stm32motor_control(SPEED_FWD, SPEED_FWD); }
static void car_brake(void)    { stm32motor_control(-SPEED_BRAKE, -SPEED_BRAKE); }
static void car_backward(void) { stm32motor_control(-SPEED_BACK, -SPEED_BACK); }
static void car_left(void)     { stm32motor_control(-SPEED_TURN, SPEED_TURN); }
static void car_right(void)    { stm32motor_control(SPEED_TURN, -SPEED_TURN); }
static void car_stop(void)     { stm32motor_control(0, 0); }

/* 给油起步: 高档起步再逐档降速到巡航 */
static void kick_start_forward(void)
{
    stm32motor_control(KICK_HI, KICK_HI);
    usleep(KICK_MS * 1000);
    stm32motor_control(KICK_MID, KICK_MID);
    usleep(KICK_MS * 1000);
    stm32motor_control(KICK_LO, KICK_LO);
    usleep(KICK_MS * 1000);
    car_forward();
}

/* ================= 红外对管: 黑胶带禁入区检测 ================= */
static int g_white_level = 1;        /* 校准: 该电平=白色地面 */
static uint32_t g_cooldown_ms = 0;   /* 冷却: 避险/绕行后只前进不判障 */

static void ir_pins_init(void)
{
    IoSetFunc(PIN_IR_L, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(PIN_IR_R, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(PIN_IR_L, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(PIN_IR_R, WIFI_IOT_GPIO_DIR_IN);
}

static int ir_left_pin(void)  { WifiIotGpioValue v; GpioGetInputVal(PIN_IR_L, &v); return (int)v; }
static int ir_right_pin(void) { WifiIotGpioValue v; GpioGetInputVal(PIN_IR_R, &v); return (int)v; }

/* 开机白地校准(1s): 多数电平记为白色, 黑=反相 (车务必放白地再上电) */
static void ir_calibrate(void)
{
    int i;
    long zeros = 0, ones = 0;
    WifiIotGpioValue v;

    for (i = 0; i < 50; i++) {
        GpioGetInputVal(PIN_IR_L, &v);
        if (v == WIFI_IOT_GPIO_VALUE0) zeros++; else ones++;
        GpioGetInputVal(PIN_IR_R, &v);
        if (v == WIFI_IOT_GPIO_VALUE0) zeros++; else ones++;
        usleep(20000);
    }
    g_white_level = (zeros > ones) ? 0 : 1;
    printf("[IR] calibrated: white_level=%d(该电平=白) raw L=%d R=%d\r\n",
           g_white_level, ir_left_pin(), ir_right_pin());
}

/* 去抖判定: 需要连续2次读到与白色相反才算黑(滤掉瞬间抖动) */
static int ir_left_black(void)
{
    static int cnt = 0;
    if (ir_left_pin() != g_white_level) {
        if (cnt < 3) cnt++;
    } else {
        cnt = 0;
    }
    return (cnt >= 2) ? 1 : 0;
}
static int ir_right_black(void)
{
    static int cnt = 0;
    if (ir_right_pin() != g_white_level) {
        if (cnt < 3) cnt++;
    } else {
        cnt = 0;
    }
    return (cnt >= 2) ? 1 : 0;
}

/* 返回: 0=无黑线 1=左黑 2=右黑 3=双侧黑 */
static int ir_tape_state(void)
{
    int l = ir_left_black();
    int r = ir_right_black();
    if (l && r) return 3;
    if (l) return 1;
    if (r) return 2;
    return 0;
}

static int tape_pivot_left_prev = 1;   /* 双侧压线后转向交替, 防死循环 */

/* 压到黑线: 停车 -> 转离禁入区 -> 重新起步 */
static void do_tape_avoid(int tape)
{
    printf("[Tape] tape=%d detected, stop!\r\n", tape);
    car_brake();
    usleep(60000);
    car_stop();
    usleep(60000);

    if (tape == 3) {                       /* 双侧都黑: 正在压线, 后退脱离 */
        printf("[Tape] both black, back & turn\r\n");
        car_backward();
        usleep(TAPE_BACK_MS * 1000);
        car_stop();
        usleep(80000);
        if (tape_pivot_left_prev) { car_left(); tape_pivot_left_prev = 0; }
        else                      { car_right(); tape_pivot_left_prev = 1; }
        usleep(TAPE_PIVOT_MS * 1000);
    } else if (tape == 1) {                /* 左黑 -> 右转离开 */
        printf("[Tape] left black -> turn RIGHT\r\n");
        car_right();
        usleep(TAPE_PIVOT_MS * 1000);
    } else {                               /* 右黑 -> 左转离开 */
        printf("[Tape] right black -> turn LEFT\r\n");
        car_left();
        usleep(TAPE_PIVOT_MS * 1000);
    }
    car_stop();
    usleep(60000);

    g_cooldown_ms = (uint32_t)(hi_get_us() / 1000) + TAPE_COOLDOWN_MS;
    kick_start_forward();                  /* 重新起步继续行驶 */
}

/* ================= 巡航中周期观察左右(舵机转左测->转右测->回中) ================= */
static void observe_sides(void)
{
    int dl2, dr2;
    sg90_goto(SG90_LEFT, OBS_PULSES);
    usleep(OBS_SETTLE_MS * 1000);
    dl2 = (int)hcsr04_get_distance();

    sg90_goto(SG90_RIGHT, OBS_PULSES);
    usleep(OBS_SETTLE_MS * 1000);
    dr2 = (int)hcsr04_get_distance();

    sg90_goto(SG90_MID, OBS_PULSES);
    usleep(OBS_SETTLE_MS * 1000);
    printf("[Auto] obs left=%d right=%d cm\r\n", dl2, dr2);
}

/* ================= 时间判断 ================= */
static int time_up(uint64_t t_start)
{
    return ((hi_get_us() - t_start) / 1000) >= RUN_TOTAL_MS;
}

/* ================= 主任务 ================= */
static void *AutoDriveTask(void *arg)
{
    (void)arg;
    float d = 0, dl = 0, dr = 0;
    int trig_cnt = 0;
    int side_cyc = 0;
    uint32_t diag = 0;
    uint64_t t_start;

    sg90_goto(SG90_MID, 50);
    usleep(300000);

    t_start = hi_get_us();
    printf("[Auto] 2min auto drive start!\r\n");

    while (1) {
        /* ============ 前进阶段 ============ */
        kick_start_forward();       /* 给油起步 */
        trig_cnt = 0;
        printf("[Auto] forward...\r\n");

        while (1) {
            uint32_t now = (uint32_t)(hi_get_us() / 1000);

            car_forward();          /* 每轮重发前进帧, 保证连续匀速 */

            /* 冷却期(刚避险/绕行完): 只前进不判障, 防起步又误判 */
            if (now < g_cooldown_ms) {
                usleep(20000);
                if (ir_tape_state() != 0) {   /* 黑线仍要处理 */
                    do_tape_avoid(ir_tape_state());
                    trig_cnt = 0;
                }
                continue;
            }

            /* 云台居中保持, 测前方 */
            sg90_pulse(SG90_MID);
            d = hcsr04_get_distance();

            /* 距离分级判障: <3cm杂波忽略; <6cm立即; <12cm重; <30cm轻; 累计>=3判障 */
            if (d < NOISE_MIN_CM) {
                /* 杂波忽略 */
            } else if (d < 6.0f) {
                trig_cnt = 5;                /* 极近: 单次立即 */
            } else if (d < EMERGENCY_CM) {
                trig_cnt += 2;
            } else if (d < STOP_TRIGGER_CM) {
                trig_cnt += 1;
            } else {
                trig_cnt = 0;
            }
            if (trig_cnt >= 3) {
                break;                       /* 连续确认判障 */
            }

            /* 诊断: 每约0.6秒打印一次距离与红外原始电平(方便现场测感应) */
            if (++diag % 8 == 0) {
                printf("[Auto] dist=%d cm | IR raw L=%d R=%d white=%d\r\n",
                       (int)d, ir_left_pin(), ir_right_pin(), g_white_level);
            }

            if (time_up(t_start)) {
                break;
            }

            /* 前方够远时, 周期转动舵机观察左右(测完回中) */
            if (d >= SIDE_OBS_CM) {
                if (++side_cyc >= SIDE_OBS_CYCLE) {
                    side_cyc = 0;
                    observe_sides();
                }
            }

            /* 测距间隔内每10ms查一次红外, 黑线不漏检 */
            {
                int tape = 0;
                int k;
                for (k = 0; k < (SAMPLE_PAD_US / 10000); k++) {
                    usleep(10000);
                    tape = ir_tape_state();
                    if (tape != 0) {
                        break;
                    }
                }
                if (tape != 0) {
                    do_tape_avoid(tape);
                    trig_cnt = 0;
                    continue;                /* 绕行后重新进入前进测量 */
                }
            }
        }
        if (time_up(t_start)) {
            break;
        }

        /* ============ 遇障: 强力制动 -> 后退 ============ */
        printf("[Auto] blocked dist=%d cm\r\n", (int)d);
        car_brake();
        usleep(10000);
        car_brake();
        usleep(10000);
        car_brake();
        usleep(BRAKE_MS * 1000);

        printf("[Auto] backup...\r\n");
        car_backward();
        usleep(BACKUP_MS * 1000);
        car_stop();
        usleep(100000);

        /* ============ 云台扫左右, 选较空一侧 ============ */
        sg90_goto(SG90_LEFT, SCAN_PULSES);
        usleep(SCAN_SETTLE_MS * 1000);
        dl = hcsr04_get_distance();
        printf("[Auto] left=%d cm\r\n", (int)dl);

        sg90_goto(SG90_RIGHT, SCAN_PULSES);
        usleep(SCAN_SETTLE_MS * 1000);
        dr = hcsr04_get_distance();
        printf("[Auto] right=%d cm\r\n", (int)dr);

        sg90_goto(SG90_MID, SCAN_PULSES);

        if (dl < SIDE_MIN_CM && dr < SIDE_MIN_CM) {
            printf("[Auto] both blocked, turn around 180!\r\n");
            car_left();
            usleep(TURN_180_MS * 1000);
        } else if (dl >= dr) {
            printf("[Auto] turn LEFT (L=%d R=%d)\r\n", (int)dl, (int)dr);
            car_left();
            usleep(TURN_90_MS * 1000);
        } else {
            printf("[Auto] turn RIGHT (L=%d R=%d)\r\n", (int)dl, (int)dr);
            car_right();
            usleep(TURN_90_MS * 1000);
        }
        car_stop();
        usleep(REST_MS * 1000);

        /* 转向后冷却, 给新方向留出前进时间再判障 */
        g_cooldown_ms = (uint32_t)(hi_get_us() / 1000) + RETRIG_COOLDOWN_MS;
    }

    /* 2分钟到: 停车帧连发3次 */
    car_stop();
    usleep(10000);
    car_stop();
    usleep(10000);
    car_stop();
    printf("[Auto] 2 MIN DONE, car stopped!\r\n");
    return NULL;
}

/* ================= 入口 ================= */
static void auto_drive_demo(void)
{
    osThreadAttr_t attr;
    WifiIotUartAttribute uattr;

    GpioInit();

    /* 红外对管 GPIO13/14: 初始化 + 白地校准(请把车放白色地面再上电) */
    ir_pins_init();
    ir_calibrate();

    /* 舵机 GPIO2 输出 */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_IO_FUNC_GPIO_2_GPIO);
    GpioSetDir(PIN_SG90, WIFI_IOT_GPIO_DIR_OUT);

    /* 超声波 GPIO7/8 */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_7, WIFI_IOT_IO_FUNC_GPIO_7_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_IO_FUNC_GPIO_8_GPIO);

    /* UART2 与 STM32 通信 */
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);
    memset(&uattr, 0, sizeof(uattr));
    uattr.baudRate = 115200;
    uattr.dataBits = 8;
    uattr.stopBits = 1;
    uattr.parity = 0;
    UartInit(UART_STM32, &uattr, NULL);

    /* 创建自主行驶任务 */
    memset(&attr, 0, sizeof(attr));
    attr.name = "AutoDrive";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 4096;
    attr.priority = osPriorityNormal;
    osThreadNew((osThreadFunc_t)AutoDriveTask, NULL, &attr);

    printf("Auto Drive 2min ready!\r\n");
}

APP_FEATURE_INIT(auto_drive_demo);
