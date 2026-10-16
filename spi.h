#ifndef SPI_H
#define SPI_H

#include <stm32f1xx_hal.h>

/* ===========================================================================
 * SPI2 总线驱动（软件分层中的"总线层"）
 * ---------------------------------------------------------------------------
 * 本层只干一件事：把字节搬出去、把字节搬回来。它不认识陀螺仪，也不认识
 * 寄存器地址 —— 那些属于器件层 xv7001bb.c。
 * 分层的好处：换一颗 SPI 器件只改器件层；换 SPI 口、改波特率只改本文件；
 *             主程序里的业务逻辑一个字都不用动。
 *
 * 硬件连接（来自原理图 Schematic_LB_RackRail_Gyro-V5.pdf 网表）：
 *     PB12  SS / CS      片选      普通 GPIO 输出，软件控制，低电平选中
 *     PB13  SCLK         时钟      SPI2 复用功能
 *     PB14  MISO / SA0   收数据    SPI2 复用功能（主机侧作输入）
 *     PB15  MOSI / SDA   发数据    SPI2 复用功能
 *
 * 模块开关：本层不自己决定用不用，是否被调用由 Project1_10_13.c 顶部的
 *           MODULE_SPI_TEST_ENABLE / MODULE_GYRO_ENABLE 两个开关决定 ——
 *           两个开关都是 0 时，SPI_Init() 根本不会被调用
 * =========================================================================== */

/* 片选引脚。片选不交给 SPI 硬件自动管理，而是当普通 GPIO 手动控制，
   为什么这么做见 SPI_CS() 的说明 */
#define SPI_CS_GPIO_PORT        GPIOB
#define SPI_CS_PIN              GPIO_PIN_12

/* SCLK 波特率分频：SPI2 挂在 APB1 上，PCLK1 = 36MHz（主频 72MHz 再二分频）
 *     分频 8  ->  SCLK = 36MHz / 8 = 4.5MHz   ← 当前采用值（先求稳）
 *     分频 4  ->  SCLK = 36MHz / 4 = 9MHz     （器件手册上限 10MHz，余量较薄）
 * ★ 换速度、换主频只改这一个宏，spi.c 里不用动 */
#define SPI_SCLK_PRESCALER      SPI_BAUDRATEPRESCALER_8

/* 单次收发故障等待上限（ms）：正常几字节传输远短于 1ms；5ms 仍给 HAL
   足够余量，同时避免原 100ms 超时一次阻塞十个 10ms 采样周期。
   这是故障上限，不是周期保证；SPI 故障时任务仍可能错过采样期限。 */
#define SPI_TIMEOUT_MS          5U

/* 初始化 SPI2 外设和它的 4 个引脚：任何一次 SPI 收发之前调用一次，
   调用位置在 main() 里、创建任务之前
 * ★ 本函数只配外设，不创建总线锁 —— 锁由 SPI_BusMutexInit() 单独创建 */
HAL_StatusTypeDef SPI_Init(void);

/* 创建 SPI2 总线锁（互斥量），供 SPI_CS() 在多任务之间互斥使用
 * @retval HAL_OK 成功；HAL_ERROR 调用顺序不对（SPI 未初始化）或内存不足
 * ★ 调用位置必须严格照抄 main() 里的顺序：
 *        SPI_Init()  ->  ...（含 XV7001bb_Init() 等所有 HAL_Delay()）...
 *        ->  SPI_BusMutexInit()  ->  创建任务  ->  osKernelStart()
 *   提前到任何一个 HAL_Delay() 之前调用，都会把 SysTick 中断挡掉，
 *   使后面的 HAL_Delay() 死等（原理见 spi.c 里本函数的详细注释） */
HAL_StatusTypeDef SPI_BusMutexInit(void);

/* 片选控制：level = 0 选中器件（SS 拉低），level = 1 释放器件（SS 拉高）
 * ★ 必须成对使用，把 SPI_Transfer() 夹在中间：
 *       SPI_CS(0);  SPI_Transfer(tx, rx, len);  SPI_CS(1);
 *   一次通信中间绝对不能拉高，否则器件会把本次通信作废（手册 5-1）
 * ★ 本函数同时兼管 SPI2 总线锁：拉低片选时取锁、拉高片选时放锁。
 *   所以"一次通信"和"一次加锁"范围完全一致 —— 课题六 task_gyro 每 10ms
 *   读数据时，总线锁仍可保护其它明确启用的 SPI 任务
 * ★ 锁没创建（SPI_BusMutexInit() 之前）或调度器还没启动时，本函数不加锁，
 *   直接操作片选 —— 初始化阶段是单线程，本来也不需要互斥 */
void SPI_CS(uint8_t level);

/* 收发一串字节（全双工：发出去多少，就同时收回来多少）
 *     tx、rx 都非空  ->  边发边收（读寄存器走这条）
 *     rx 传 NULL     ->  只发（写寄存器、发命令走这条）
 *     tx 传 NULL     ->  只收（主机模式下收不到数据，原因见 spi.c）
 * ★ 本函数不碰片选，调用方负责用 SPI_CS() 把整串字节包在同一个低电平里 */
HAL_StatusTypeDef SPI_Transfer(const uint8_t *tx, uint8_t *rx, uint16_t len);

/* 发送单个字节，自带一次片选 —— 课题四.2 的示波器测试专用
 * ★ 不能用它读写 XV7001BB 寄存器：寄存器操作要求"地址 + 数据"多个字节在
 *   同一个片选低电平内连续走完，必须改用 SPI_CS() + SPI_Transfer() */
HAL_StatusTypeDef SPI_SendByte(uint8_t data);

#endif /* SPI_H */