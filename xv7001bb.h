#ifndef XV7001BB_H
#define XV7001BB_H

#include <stm32f1xx_hal.h>

/* ===========================================================================
 * XV7001BB 陀螺仪（内含温度传感器）驱动接口 —— 课题五
 * ---------------------------------------------------------------------------
 * 器件    ：SEIKO EPSON XV7001BB，量程 ±100 °/s，模拟部分 + DSP 输出的数字陀螺仪
 * 手册    ：XV7001BB_AE_Ver.1.00（工程目录里的 XV7001BB_AE_Ver100.pdf）
 * 总线    ：SPI2，4 线模式
 *               PB12 = SS（软件片选）、PB13 = SCLK
 *               PB14 = MISO / SA0、PB15 = MOSI / SDA
 *           SPI 参数：Mode 0（CPOL = 0 / CPHA = 0），MSB 先出，SCLK 上限 10MHz
 * 地址字节：bit7    = 1 读 / 0 写
 *           bit6:5  = 从机地址，本板只有一颗器件、多从机功能关闭，固定 00
 *           bit4:0  = 寄存器地址 A4~A0
 * 通信约定：一次读/写过程中 SS 必须一直保持低电平，地址字节和数据字节之间
 *           不允许拉高，否则本次通信会被器件作废（手册 5-1）
 *
 * 本文件（器件层）负责：寄存器地址、数据格式、上电时序、单位换算
 * 它不管：字节怎么搬（那是 spi.c）、什么时候读、读来干什么（那是主程序）
 *
 * 模块开关：是否使用本驱动由 Project1_10_13.c 顶部的 MODULE_GYRO_ENABLE 决定
 * =========================================================================== */

/* ---- 用户命令寄存器地址（手册 Table 6 User command register） ------------- */
#define XV7001BB_REG_DSPCTL1       0x01U    /* R/W  DSP 设置 1                  */
#define XV7001BB_REG_DSPCTL2       0x02U    /* R/W  DSP 设置 2                  */
#define XV7001BB_REG_DSPCTL3       0x03U    /* R/W  DSP 设置 3                  */
#define XV7001BB_REG_STSRD         0x04U    /* R    状态读取                    */
#define XV7001BB_REG_SLPIN         0x05U    /* C    进入睡眠（只有命令、无数据） */
#define XV7001BB_REG_SLPOUT        0x06U    /* C    退出睡眠 / 待机              */
#define XV7001BB_REG_STBY          0x07U    /* C    进入待机                    */
#define XV7001BB_REG_TEMPRD        0x08U    /* R    温度传感器数据读取          */
#define XV7001BB_REG_SWRST         0x09U    /* C    软件复位                    */
#define XV7001BB_REG_DATACCON      0x0AU    /* R    角速度数据读取              */
#define XV7001BB_REG_OUTCTL1       0x0BU    /* R/W  角速度数据读取控制          */
#define XV7001BB_REG_AUTOC         0x0CU    /* C    零速率电平校准（AutoC）      */
#define XV7001BB_REG_DSPRES        0x0DU    /* C    数字滤波器复位              */
#define XV7001BB_REG_MEMLOAD       0x1BU    /* C    参考数据寄存器重载          */
#define XV7001BB_REG_TSDATAFORMAT  0x1CU    /* R/W  温度传感器数据格式          */
#define XV7001BB_REG_IFCTL         0x1FU    /* R/W  串行接口设置                */
/* ★ 其余地址（0x00、0x0E~0x1A、0x1D、0x1E）是保留寄存器，手册要求不得写入，
   误写可能让器件永久损坏 —— 所以下面所有接口都会把地址限制在低 5 位 */

/* ---- StsRd（0x04）状态寄存器（手册 Table 6.4） ---------------------------- */
#define XV7001BB_STS_PROCOK        0x08U    /* bit3: 1 = 温度数据已可读取       */
#define XV7001BB_STS_PREST_MASK    0x07U    /* bit2:0 = 器件状态标志            */
#define XV7001BB_STS_PREST_POR     0x04U    /*   100: 刚上电（POR）             */
#define XV7001BB_STS_PREST_STBY    0x02U    /*   010: 待机                     */
#define XV7001BB_STS_PREST_SLEEP   0x00U    /*   000: 睡眠                     */
#define XV7001BB_STS_PREST_SLPOUT  0x01U    /*   001: 已退出睡眠（正常工作）    */

/* ---- OutCtl1（0x0B）角速度数据读取控制（手册 Table 6.11） ---------------- */
#define XV7001BB_OUTCTL1_FORMAT_16BIT   0x00U   /* bit2 = 0：16 bit 输出        */
#define XV7001BB_OUTCTL1_FORMAT_24BIT   0x04U   /* bit2 = 1：24 bit 输出        */
#define XV7001BB_OUTCTL1_SEL_RATE       0x01U   /* bit1:0 = 01：输出角速度数据  */
#define XV7001BB_OUTCTL1_DEFAULT        0x01U   /* 复位后的默认值               */

/* ---- DspCtl3（0x03）DSP 设置 3（手册 Table 6.3） ------------------------- */
#define XV7001BB_DSPCTL3_ENCALIBCMD 0x40U   /* bit6: 1 = 允许执行 AutoC 校准命令；
												复位后默认 0，此时 AutoC 是空操作 */
/* ---- TsDataFormat（0x1C）温度数据格式（手册 Table 6.15） ----------------- */
#define XV7001BB_TSDATA_8BIT       0x00U    /* bit6:5 = 00：8 bit 温度输出      */
#define XV7001BB_TSDATA_10BIT      0x20U    /* bit6:5 = 01：10 bit 温度输出     */
#define XV7001BB_TSDATA_12BIT      0x40U    /* bit6:5 = 10：12 bit（默认值）    */

/* ---- IFCtl（0x1F）串行接口设置（手册 Table 6.16） ------------------------ */
#define XV7001BB_IFCTL_I2C_EN      0x01U    /* bit0 = 1：使能 I²C 接口；
											   本工程只用 SPI，初始化必须把它清 0 */
#define XV7001BB_IFCTL_SPISEL      0x02U    /* bit1 = 1：3 线 SPI；
											   清 0 = 4 线 SPI（本工程用 4 线） */
#define XV7001BB_IFCTL_SPI_4WIRE   0x00U    /* 关 I²C + 4 线 SPI，
											   就是初始化要写进去的值 */

/* ---- 换算系数（手册 Table 3.5 / Table 3.6） ------------------------------ */
/* ★ 实测灵敏度是 70 LSB/(°/s)，不是手册标称的 280。
 * 依据：转台精确转 90°，本固件按 280 换算积分出来只有 22.5°，正好差 4 倍；
 *       按 70 换算后 90°/180°/360° 的读数与真实角度一致（360° 残差 1.3°≈0.36%）。
 * 为什么差 4 倍：器件的 16 bit 字是"14 bit 数据左对齐"，低 2 位恒为 0，
 *       按 16 bit 全宽除以 280 就小了 4 倍；也可能是所用档位的量程/灵敏度与手册
 *       标称值不同。两种情况下正确的原始码→角速度比例都是"每 (°/s) 70 个码"。
 * ★ 改这个常数会让所有 °/s 数值放大 4 倍，工程里所有以 °/s 为单位的阈值/门限
 *   必须同步 ×4，否则物理行为会静默改变（角度积分由 GYRO_ANGLE_SCALE_CALIBRATION
 *   从 4.0f 改成 1.0f 抵消）。 */
#define XV7001BB_SCALE_16BIT       70.0f        /* 16 bit 输出：实测 70 LSB/(°/s)；手册标称 280 */
#define XV7001BB_SCALE_24BIT       17920.0f     /* 24 bit 输出：按同一比例 71680/4；本工程未使用 */
#define XV7001BB_TEMP_LSB_PER_C    16.0f        /* 12 bit 输出：16 LSB/°C       */
#define XV7001BB_TEMP_CODE_25C     400.0f       /* 12 bit 输出：+25°C 对应 400  */

/* ---- 量程与上电时序（手册 Chapter 3 / Table 3.4） ------------------------ */
/* 按 70 LSB/(°/s) 换算，±100 °/s 对应 ±7000 码。 */
#define XV7001BB_RANGE_DPS         100.0f       /* 测量范围 ±100 °/s            */

/* 数据有效性门限（只用来挡"物理上不可能"的读数，比如 SPI 位错把码打飞）。
 * ★ 为什么不能卡在 ±XV7001BB_RANGE_DPS：
 *   卡在量程会把"快转时恰好饱和"的样本判成坏数据，而坏样本的代价极大 ——
 *   固件丢弃一拍会连带丢掉下一拍的 dt，等于丢 20ms；以 100 °/s 计一次就丢 2°，
 *   而且滤波状态被重置。宁可把饱和值积分进去，也不能丢时间。
 *   16 bit 码上限 32767 → 468 °/s，所以 400 已经是"几乎不会触发"的门限：
 *   只有码值 > 28000 才会被判无效，那种读数一定是链路出了问题。 */
#define XV7001BB_VALID_RATE_DPS    400.0f       /* 有效读数上限（超出判为坏数据）*/
#define XV7001BB_T_IF_MS           10U          /* tIF  ：串口可通信，手册要求 ≥1ms */
#define XV7001BB_T_STA_MS          200U         /* tSTA ：角速度数据可读，手册 ≤200ms */

/* ===========================================================================
 * 对外接口
 * =========================================================================== */

/* ---------------------------------------------------------------------------
 * b) 初始化：复位寄存器 -> 退出上电态 -> 关 I²C（只用 4 线 SPI）-> 等数据稳定
 * @retval HAL_OK 表示与器件通信正常
 * @note   必须在 SPI_Init() 之后调用；内部用 HAL_Delay 等待约 210ms，
 *         所以要放在 main() 里、启动调度器之前（不能放进任务里阻塞别人）
 * --------------------------------------------------------------------------- */
HAL_StatusTypeDef XV7001bb_Init(void);

/* ---------------------------------------------------------------------------
 * a) 写入数据：向寄存器 addr 写 1 个字节
 * @param addr 寄存器地址（低 5 位有效，见 XV7001BB_REG_xxx）
 * @param data 要写入的字节
 * --------------------------------------------------------------------------- */
HAL_StatusTypeDef XV7001bb_WriteData(uint8_t addr, uint8_t data);

/* ---------------------------------------------------------------------------
 * c) 读取寄存器：从寄存器 addr 读 1 个字节
 * @param addr 寄存器地址（低 5 位有效）
 * @param data 读出的数据存放处
 * --------------------------------------------------------------------------- */
HAL_StatusTypeDef XV7001bb_ReadReg(uint8_t addr, uint8_t *data);

/* ---------------------------------------------------------------------------
 * d) 读取温度：返回温度传感器原始码
 * @param raw 原始码（默认 12 bit 输出，2 的补码）
 * @note   12 bit 格式下：第 1 字节 = D[11:4]，第 2 字节 = D[3:0]（低 4 位恒为 0），
 *         拼成 12 bit 左对齐值后要右移 4 位才算真正的温度码
 * --------------------------------------------------------------------------- */
HAL_StatusTypeDef XV7001bb_ReadTmp(int16_t *raw);

/* ---------------------------------------------------------------------------
 * e) 读取角速度：返回角速度原始码
 * @param raw 原始码（默认 16 bit 输出，2 的补码）
 * @note   16 bit 格式下：第 1 字节 = D[15:8]，第 2 字节 = D[7:0]
 * --------------------------------------------------------------------------- */
HAL_StatusTypeDef XV7001bb_ReadAngle(int16_t *raw);

/* ---- 单位换算（把原始码变成能直接看的物理量） ---------------------------- */

/* 温度原始码 -> 摄氏度：25°C 对应 400 LSB，温度系数 16 LSB/°C */
float XV7001bb_TmpToCelsius(int16_t raw);

/* 角速度原始码 -> °/s：16 bit 输出时灵敏度 280 LSB/(°/s) */
float XV7001bb_AngleToDps(int16_t raw);

/* 读取温度并直接换算成摄氏度（ReadTmp + TmpToCelsius 的组合） */
HAL_StatusTypeDef XV7001bb_ReadTmpCelsius(float *celsius);

/* 读取角速度并直接换算成 °/s（ReadAngle + AngleToDps 的组合） */
HAL_StatusTypeDef XV7001bb_ReadAngleDps(float *dps);

/* ---- 器件控制命令（C 类命令：只发地址、不带数据） ------------------------ */

/* 读状态寄存器 StsRd(0x04)，位定义见上面 XV7001BB_STS_xxx */
HAL_StatusTypeDef XV7001bb_ReadStatus(uint8_t *status);

/* 只发送命令地址，用于 SLPIN / SLPOUT / STBY / SWRST / AUTOC / DSPRES / MEMLOAD */
HAL_StatusTypeDef XV7001bb_SendCmd(uint8_t cmd);

HAL_StatusTypeDef XV7001bb_SoftReset(void);     /* 软件复位：寄存器回默认值   */
HAL_StatusTypeDef XV7001bb_SleepIn(void);       /* 进入睡眠（省电）           */
HAL_StatusTypeDef XV7001bb_SleepOut(void);      /* 退出睡眠 / 待机，复位 DSP  */
HAL_StatusTypeDef XV7001bb_Standby(void);       /* 进入待机（检测电路停摆）   */
HAL_StatusTypeDef XV7001bb_AutoCalib(void);     /* 零速率电平校准（AutoC）    */
HAL_StatusTypeDef XV7001bb_ResetDsp(void);      /* 数字滤波器复位             */
HAL_StatusTypeDef XV7001bb_MemLoad(void);       /* 参考数据寄存器重载         */

/* 使能 / 禁止零速率校准命令（读-改-写 DspCtl3 的 bit6 EnCalibCmd）
 * ★ 手册 6-12 明确要求：执行 AutoC 之前必须先把这个位置 1，否则命令是空操作。
 *   正确顺序：XV7001bb_SetCalibCmd(1) -> XV7001bb_AutoCalib() -> 按需禁止 */
HAL_StatusTypeDef XV7001bb_SetCalibCmd(uint8_t enable);

#endif /* XV7001BB_H */