/**
  ******************************************************************************
  * @file    Project1_10_13.c
  * @brief   陀螺仪课题主程序（STM32F103C8T6 + FreeRTOS + HAL）
  * @note    代码按"层"组织，改哪一层就只动哪个文件：
  *             本文件          模块开关、任务调度、课题自检
  *             spi.c / spi.h       SPI2 总线层：只负责把字节搬出去、搬回来
  *             xv7001bb.c/.h       陀螺仪器件层：寄存器地址、数据格式、上电时序
  *             can.c / can.h        CAN 驱动层：收发帧、诊断计数
  *         硬件：MCU STM32F103C8T6（8MHz 晶振 ×9 = 72MHz）
  *               SPI2 -> XV7001BB 陀螺仪（PB12/PB13/PB14/PB15）
  *               CAN1 -> PCA82C251 收发器（PA11/PA12，1Mbps）
  *               PB9  -> 指示灯（低电平点亮）
  ******************************************************************************
  */

#include <stm32f1xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include <limits.h>
#include "spi.h"
#include "can.h"
#include "xv7001bb.h"

/* ===========================================================================
 * 【模块开关】—— 整个工程的功能总闸：1 = 启用，0 = 屏蔽
 * ---------------------------------------------------------------------------
 * 为什么要集中放一段开关：
 *     课题是一步步做的，同一个工程里同时存在 SPI、CAN、陀螺仪几套代码。
 *     验收某一项时，其它模块的收发会干扰观察（例如 SPI 每 0.5s 的波形会盖住
 *     想看的 CAN 波形），调试时也分不清到底是谁在动总线。
 *     把每个模块的开、关集中放在这里：
 *         MODULE_xxx_ENABLE = 1  ->  初始化该外设 + 建任务 + 周期收发
 *         MODULE_xxx_ENABLE = 0  ->  不初始化、不建任务，代码原样留着
 *     ★ 置 0 只是把相关代码从编译中排除，函数和注释都还在，改回 1 立刻恢复。
 *
 * 常用组合（改完必须重新编译烧写）：
 *     课题二  只闪灯        ：LED = 1，其余全 0
 *     课题三/四.2 只测 SPI ：LED = 1，SPI_TEST = 1，CAN 两个 = 0
 *     课题四.5~四.8 只测 CAN：LED = 1，SPI_TEST = 0，CAN_TX / CAN_RX = 1
 *                            ★ 关掉 SPI 测试，CAN 的波形和时序才干净
 *     课题六  角度采样 + CAN：GYRO = 1，TASK6_CAN = 1，旧测试和巡检均关闭
 *                            ★ 旧接口巡检会写寄存器/发命令，不得与采样并行
 * =========================================================================== */
#define MODULE_LED_ENABLE           0U      /* 课题二   ：PB9 指示灯闪烁           */
#define MODULE_SPI_TEST_ENABLE      0U      /* 课题四.2 ：SPI2 每 0.5s 发 0xAA     */
#define MODULE_CAN_TX_ENABLE        0U      /* 课题四.5~四.7：CAN 每 0.5s 发 0xAA  */
#define MODULE_CAN_RX_ENABLE        0U      /* 课题四.8 ：CAN 接收轮询任务         */
#define MODULE_GYRO_ENABLE          1U      /* 课题六   ：XV7001BB 10ms 采样/积分   */
#define MODULE_GYRO_CHECK_ENABLE    0U      /* 课题五旧巡检：会写寄存器和发命令，默认关闭 */
#define MODULE_TASK6_CAN_ENABLE     1U      /* 课题六   ：角度/温度 CAN TX 与命令 RX */

/* ---------------------------------------------------------------------------
 * 【后续新增代码的统一写法】—— 每加一个独立功能（新芯片、新通信、新任务）都照做：
 *     1) 在上面加一个 MODULE_新功能_ENABLE 开关，并写上它属于哪个课题；
 *     2) 初始化外设、创建任务、任务里的收发语句，全部用
 *            #if (MODULE_新功能_ENABLE != 0U)
 *                ...
 *            #endif
 *        包起来，做到"关掉就完全不参与编译、不占用 CPU"；
 *     3) 开关之间如果抢同一个外设或引脚，用下面的 #error 在编译期挡住，
 *        不要等到烧进板子才发现冲突；
 *     4) 开关旁边补一句中文注释，写清这个模块对应哪个课题、开关会影响到什么。
 * --------------------------------------------------------------------------- */

/* ---- 开关一致性检查：编译期就把矛盾组合挡下来，省得烧进去才发现 ---- */
#if ((MODULE_GYRO_ENABLE != 0U) || (MODULE_GYRO_CHECK_ENABLE != 0U)) && \
	(MODULE_SPI_TEST_ENABLE != 0U)
#error "SPI 测试和陀螺仪都用 SPI2 总线，SPI 不能重入：请把 MODULE_SPI_TEST_ENABLE 置 0"
#endif

#if (MODULE_GYRO_ENABLE != 0U) && (MODULE_GYRO_CHECK_ENABLE != 0U)
#error "课题六采样与旧陀螺仪接口巡检不能同时启用，巡检会影响采样"
#endif

#if (MODULE_TASK6_CAN_ENABLE != 0U) && (MODULE_GYRO_ENABLE == 0U)
#error "课题六 CAN 协议依赖陀螺仪采样，请同时启用 MODULE_GYRO_ENABLE"
#endif

#if (MODULE_LED_ENABLE == 0U) && (MODULE_SPI_TEST_ENABLE == 0U) && \
	(MODULE_CAN_TX_ENABLE == 0U) && (MODULE_CAN_RX_ENABLE == 0U) && \
	(MODULE_GYRO_ENABLE == 0U) && (MODULE_GYRO_CHECK_ENABLE == 0U) && \
	(MODULE_TASK6_CAN_ENABLE == 0U)
#error "所有模块都被屏蔽了，程序不会做任何事：请至少打开一个模块"
#endif

/* ---- 引脚与周期参数 ------------------------------------------------------ */

/* 课题二：PB9 指示灯，低电平点亮 */
#define LED_GPIO_PORT               GPIOB
#define LED_PIN                     GPIO_PIN_9
#define LED_BLINK_INTERVAL_MS       500U    /* 每 500ms 翻转一次：亮 0.5s、灭 0.5s */

/* 任务周期 */
#define TASK_MAIN_PERIOD_MS         500U    /* 主任务周期：SPI / CAN 都是 0.5s 发一次 */
#define CAN_RX_POLL_PERIOD_MS       10U     /* CAN 接收轮询周期                       */
#define CAN_TX_PERIOD_MS            100U    /* 课题六测量帧发送周期                   */
#define GYRO_SAMPLE_PERIOD_MS       10U     /* 角速度采样周期                         */
#define GYRO_TEMP_PERIOD_MS         1000U   /* 温度读取降频，避免干扰角速度采样       */
#define GYRO_STABILIZE_MS           1000U   /* 上电稳定等待时间                       */
#define GYRO_CALIBRATION_SAMPLES    200U    /* 软件零偏校准有效样本数                 */
#define GYRO_CALIBRATION_MAX_RANGE_DPS 2.0f /* 校准样本最大峰峰波动，超出则判失败     */
#define GYRO_CALIBRATION_MAX_BIAS_DPS  10.0f/* 校准均值过大时判为不静止/异常           */
#define GYRO_RX_MAX_FRAMES_PER_POLL 4U     /* 每轮最多排空的 CAN 帧数，避免饿死采样   */
#define GYRO_LOWPASS_TAU_SECONDS    0.03183099f /* 一阶低通约 5Hz 时间常数            */
#define GYRO_ANGLE_SCALE_CALIBRATION 4.0f  /* 实测 90° 显示约 22.5°，按比例校正积分角度 */

/* ---- 角度零漂抑制（课题六增强，总开关见下方 GYRO_DRIFT_SUPPRESS_ENABLE）----
 * 陀螺仪只输出角速度，残余零偏哪怕只有 1 个 LSB（1/280 ≈ 0.0036 °/s），
 * 积分成角度后一小时也能漂十几度，所以"校准一次"并不够，必须持续抑制。
 * 这里用两个互补手段，都要先判断"当前是否静止"：
 *   1) ZUPT 静止锁定：确认静止后本周期不做积分，角度完全不动 —— 直接切断漂移
 *      的累积路径，这是对"静止时零漂"最直接有效的手段。
 *   2) 偏置慢跟踪  ：确认静止后，用很慢的一阶跟踪把残余零偏收进 bias，
 *      使静止解锁后的角速度已经接近 0，避免漂移重新累积。
 * 两者都只在偏置补偿后的角速度足够小、且相邻样本足够平稳时才生效；静止判定
 * 期间仍然照常采样和滤波，只是不把噪声积分进角度，不是"死区丢数据"，正常转动
 * （远大于阈值）完全不受影响。
 * ★ 代价写在明处：比 GYRO_ZUPT_MAX_RATE_DPS 更慢的转动会被判成静止而不计入角度。
 *   若发现慢速转动被吃掉，把 GYRO_DRIFT_SUPPRESS_ENABLE 置 0 即可整体关闭，
 *   角度退回到"纯积分"行为。 */
#define GYRO_DRIFT_SUPPRESS_ENABLE  1U      /* 1 = 启用 ZUPT + 偏置慢跟踪          */
#define GYRO_ZUPT_MAX_RATE_DPS      0.30f   /* 判定静止的最大角速度幅值（°/s）     */
#define GYRO_ZUPT_MAX_STEP_DPS      0.10f   /* 相邻样本跳变上限，挡住抖动和尖峰    */
#define GYRO_ZUPT_CONFIRM_SAMPLES   50U     /* 连续满足静止判据的样本数（10ms → 0.5s）*/
#define GYRO_BIAS_TRACK_GAIN        0.0025f /* 静止时偏置跟踪步长（≈4s 时间常数）  */
#define GYRO_DRIFT_WINDOW_MS        1000U   /* 角度漂移率统计窗口，便于 Watch 观察 */

/* 课题六测试协议：经典 CAN、11 位标准数据帧、小端序。
 * 0x321 / DLC=8 测量帧：
 *   byte[0..3] 有符号 int32 角度，单位 0.01 度（centidegree）
 *   byte[4..5] 有符号 int16 温度，单位 0.01 摄氏度
 *   byte[6]    bit0=角度/采样有效，bit1=温度有效，
 *              bit2=零偏校准完成，bit3=零偏校准失败，bit4=判定静止（角度已锁定）
 *   byte[7]    最近两次采样间隔，单位 ms，最大 255
 * 0x323 / DLC=4 角速度帧（与 0x321 同一次快照发出，两者严格同源）：
 *   byte[0..3] 有符号 int32 滤波后角速度，单位 0.001 °/s（milli-°/s），小端
 *              上位机读取后除以 1000 即得 °/s，例如 1234 → 1.234 °/s
 *              ★ 送的是 gyroRateFilteredDps（零偏补偿 + 中值 + 低通之后的值），
 *                正是参与积分的那一路，便于和 0x321 的角度对照排查
 * 上电后先等待 1s，再要求传感器保持静止并采集 200 个样本；校准检查峰峰波动
 * 不超过 2°/s 且平均偏置不超过 10°/s，不满足时进入失败状态。
 * 0x322 / DLC=1 命令帧：0x01=重新零偏校准，0x02=角度清零。
 * 命令由 task_can_rx 置标志，实际状态只由 task_gyro 修改。
 * ★ 0x321 与 0x323 每 100ms 连发一次，共用同一次临界区快照，发送结果分别判断。 */
#define CAN_GYRO_DATA_STD_ID        0x321U
#define CAN_GYRO_RATE_STD_ID        0x323U
#define CAN_GYRO_COMMAND_STD_ID     0x322U
#define CAN_GYRO_CMD_RECALIBRATE    0x01U
#define CAN_GYRO_CMD_ZERO_ANGLE     0x02U
#define CAN_GYRO_DATA_DLC           8U
#define CAN_GYRO_RATE_DLC           4U
#define CAN_GYRO_RATE_SCALE         1000.0f /* 角速度帧比例：0.001 °/s 每 LSB */

/* 课题指定的测试数据：SPI 和 CAN 各发 1 个字节 0xAA */
#define SPI_TEST_DATA               0xAAU
#define CAN_TEST_DATA               0xAAU

/* ---- 失败码：出错卡死时，在 Watch 窗口看 failCode 就知道卡在哪一步 ------- */
#define FAIL_SPI_INIT               1U      /* main() 里 SPI_Init() 失败            */
#define FAIL_CAN_INIT               2U      /* main() 里 CAN_Init() 失败            */
#define FAIL_TASK_CREATE            3U      /* 创建任务失败（内存不够）             */
#define FAIL_GYRO_INIT              4U      /* main() 里 XV7001bb_Init() 失败       */
#define FAIL_SPI_SEND               5U      /* 任务里 SPI 发送失败                  */
#define FAIL_SPI_MUTEX_INIT         6U      /* main() 里 SPI_BusMutexInit() 失败    */

/* ★ failCode 一直为 0、程序却停在原地不往下走 —— 这是"卡死"而不是"报错"，
   最常见的原因是某个 HAL_Delay() 死等（SysTick 中断被屏蔽，HAL_GetTick()
   不再增长）。陀螺仪初始化阶段的延时都靠 SysTick，所以第 3.5 步的
   SPI_BusMutexInit() 必须放在它之后，顺序不能提前（见下面的注释） */

/* ---- 诊断变量（都在 Watch 窗口直接观察，不需要打断点） ------------------ */

/* 时钟诊断：sysClockIs72M = 1 表示 HSE 8MHz + PLL ×9 配置成功（PCLK1 = 36MHz）；
   = 0 表示外部晶振起振失败、系统退回 HSI 8MHz —— 此时 SPI 和 CAN 的
   波特率都会偏，是排查通信问题时要先看的一个数 */
volatile uint8_t  sysClockIs72M = 0U;
volatile uint32_t sysClockHz = 0U;
volatile uint32_t sysPclk1Hz = 0U;

/* 初始化失败码：0 = 一切正常 */
volatile uint8_t  failCode = 0U;

/* 课题五 陀螺仪自检结果：
	 gyroInitOk       : 1 = main() 里 XV7001bb_Init() 成功（初始化本身）
	 ------ 下面这些由 task_main 每轮巡检刷新（每 500ms 一轮）------
	 gyroStatus       : 状态寄存器 StsRd(0x04) 的原始值
	 gyroPreSts       : StsRd 的器件状态位，正常工作时应为 001（已退出睡眠）
	 gyroProCOK       : StsRd 的 bit3，1 = 温度数据已可读取
	 ------ 每个功能接口各自的"是否正确"标志：1 = 通过，0 = 不通过 ------
	 gyroOkInit       : b) 初始化 —— 用 IFCtl(关 I²C、4 线 SPI) + preSts 复核
	 gyroOkWriteData  : a) 写入数据 —— 写回原值后回读比对
	 gyroOkReadReg    : c) 读取寄存器
	 gyroOkReadTmp    : d) 读取温度（含换算与范围检查）
	 gyroOkReadAngle  : e) 读取角速度（含换算与范围检查）
	 gyroOkAux        : 辅助接口（多寄存器读、组合读+换算、读-改-写命令位）
	 gyroOkCmd        : 命令类接口 SendCmd（只在第 1 轮测一次，避免扰动滤波器）
	 gyroCheckAllOk   : 上面所有接口是否全部通过；1 = 全部正确
	 gyroCheckFailStep: 最近一轮巡检第一个出错的步骤（0 = 全部通过），
						1 = 读状态寄存器   2 = 读寄存器     3 = 写数据/回读比对
						4 = 读配置寄存器   5 = 初始化复核   6 = 读温度
						7 = 读角速度       8 = 辅助接口     9 = 命令类接口
	 gyroCheckFailCount / gyroCheckRoundCount : 失败轮数 / 已巡检轮数
	 gyroRegWriteBack : OutCtl1 的回读值（应与写入值相同）
	 gyroCfgIfCtl / gyroCfgTsFormat / gyroCfgDspCtl1~3 : 读回来的配置寄存器
	 gyroTempRaw      : 温度原始码        gyroTempCelsius : 温度换算值（°C）
	 gyroRateRaw      : 角速度原始码      gyroRateDps     : 角速度换算值（°/s）
	 gyroReadCount / gyroErrCount : 周期读取任务的成功 / 失败次数 */
volatile uint8_t  gyroInitOk = 0U;
volatile uint8_t  gyroStatus = 0U;
volatile uint8_t  gyroPreSts = 0U;
volatile uint8_t  gyroProCOK = 0U;
volatile uint8_t  gyroOkInit = 0U;
volatile uint8_t  gyroOkWriteData = 0U;
volatile uint8_t  gyroOkReadReg = 0U;
volatile uint8_t  gyroOkReadTmp = 0U;
volatile uint8_t  gyroOkReadAngle = 0U;
volatile uint8_t  gyroOkAux = 0U;
volatile uint8_t  gyroOkCmd = 0U;
volatile uint8_t  gyroCheckAllOk = 0U;
volatile uint8_t  gyroCheckFailStep = 0U;
volatile uint32_t gyroCheckFailCount = 0U;
volatile uint32_t gyroCheckRoundCount = 0U;
volatile uint8_t  gyroRegWriteBack = 0U;
volatile uint8_t  gyroCfgIfCtl = 0U;
volatile uint8_t  gyroCfgTsFormat = 0U;
volatile uint8_t  gyroCfgDspCtl1 = 0U;
volatile uint8_t  gyroCfgDspCtl2 = 0U;
volatile uint8_t  gyroCfgDspCtl3 = 0U;
volatile int16_t  gyroTempRaw = 0;
volatile int16_t  gyroRateRaw = 0;
volatile float    gyroTempCelsius = 0.0f;
volatile float    gyroRateDps = 0.0f;
volatile uint32_t gyroReadCount = 0U;
volatile uint32_t gyroErrCount = 0U;

/* 课题六采样与校准监视量：校准状态 0=稳定等待、1=采样、2=完成、3=失败。 */
volatile float gyroBiasDps = 0.0f;
volatile float gyroRateCorrectedDps = 0.0f;
volatile float gyroRateFilteredDps = 0.0f;
volatile float gyroAccumulatedAngleDeg = 0.0f;
volatile float gyroCalibrationRangeDps = 0.0f;
volatile uint8_t gyroCalibrationState = 0U;
volatile uint8_t gyroDataValid = 0U;
volatile uint8_t gyroTempValid = 0U;
volatile uint32_t gyroCalibrationSampleCount = 0U;
volatile uint32_t gyroCalibrationFailCount = 0U;
volatile uint32_t gyroSampleIntervalMs = 0U;
volatile uint32_t gyroOverrunCount = 0U;
/* 课题六 CAN 发送计数：Ok/Err 是 0x321 与 0x323 两帧的合计，RateOk/RateErr 只统计
 * 0x323 角速度帧。两个计数放在一起对比，就能看出是哪一帧发送失败。 */
volatile uint32_t gyroCanTxOkCount = 0U;
volatile uint32_t gyroCanTxErrCount = 0U;
volatile uint32_t gyroCanTxRateOkCount = 0U;
volatile uint32_t gyroCanTxRateErrCount = 0U;
volatile uint32_t gyroCanRxCommandCount = 0U;
volatile uint32_t gyroCanRxProtocolErrCount = 0U;
volatile uint8_t gyroRequestRecalibration = 0U;
volatile uint8_t gyroRequestZeroAngle = 0U;

/* 零漂抑制监视量：gyroZuptState 0 = 运动中（正常积分）、1 = 判定静止（角度已锁定，
 * 此时角度不会再漂）。gyroAngleDriftDps 是最近 1s 的角度变化率，静止时它越接近 0
 * 说明零漂压得越好；gyroBiasTrackDeltaDps 是慢跟踪累计吃掉的零偏量。 */
volatile uint8_t  gyroZuptState = 0U;
volatile uint32_t gyroZuptHoldCount = 0U;
volatile uint32_t gyroZuptFrozenSamples = 0U;
volatile uint32_t gyroBiasTrackUpdateCount = 0U;
volatile float    gyroBiasTrackDeltaDps = 0.0f;
volatile float    gyroAngleDriftDps = 0.0f;

/* ---- 函数声明 ------------------------------------------------------------ */
static void SystemClock_Config(void);
static void task_main(void const *argument);
static void FatalError(uint8_t code);
static void Error_Handler(void);

#if ((MODULE_CAN_RX_ENABLE != 0U) || (MODULE_TASK6_CAN_ENABLE != 0U))
static void task_can_rx(void const *argument);
#endif

#if ((MODULE_GYRO_ENABLE != 0U) || (MODULE_GYRO_CHECK_ENABLE != 0U))
static void task_gyro(void const *argument);
#endif

#if (MODULE_TASK6_CAN_ENABLE != 0U)
static void task_can_tx(void const *argument);
#endif

#if (MODULE_GYRO_CHECK_ENABLE != 0U)
static void GyroCheckFail(uint8_t step);
static void GyroInterfaceCheck(void);
#endif

#if (MODULE_LED_ENABLE != 0U)
static void task_led(void const *argument);
#endif

/**
  * @brief  程序入口：初始化时钟和各外设 -> 创建总线锁 -> 创建任务 -> 启动调度器
  * @note   初始化顺序不能随意调换：
  *         时钟必须最先配好（SPI 波特率、CAN 位时序、FreeRTOS tick 都依赖它），
  *         陀螺仪必须在 SPI 之后、调度器之前（它的初始化要阻塞式等待 210ms），
  *         SPI_BusMutexInit() 必须排在陀螺仪之后、创建任务之前 ——
  *         它一执行 SysTick 就会被屏蔽（FreeRTOS 临界区在调度器启动前的固有
  *         行为），排在 HAL_Delay() 前面就会死等，详见 spi.c 里的详细说明
  */
int main(void)
{
	/* ---- 第 1 步：HAL 库初始化（SysTick、NVIC 中断分组等） ---- */
	HAL_Init();

	/* ---- 第 2 步：系统时钟切到 72MHz（HSE 8MHz × 9，APB1 二分频 -> PCLK1 36MHz）----
	   这一组时钟是 SPI 波特率、CAN 位时序、FreeRTOS tick 三处的共同前提，
	   切不过去时 sysClockIs72M 会停在 0，通信波特率将全部偏掉 */
	SystemClock_Config();
	sysClockHz = HAL_RCC_GetHCLKFreq();
	sysPclk1Hz = HAL_RCC_GetPCLK1Freq();

	/* ---- 第 3 步：按模块开关逐个初始化外设 ---- */

#if ((MODULE_SPI_TEST_ENABLE != 0U) || (MODULE_GYRO_ENABLE != 0U) || \
	(MODULE_GYRO_CHECK_ENABLE != 0U))
	/* SPI 测试、课题六采样和可选旧巡检共用 SPI2 底层总线 */
	if (SPI_Init() != HAL_OK)
	{
		FatalError(FAIL_SPI_INIT);
	}
#endif

#if ((MODULE_CAN_TX_ENABLE != 0U) || (MODULE_CAN_RX_ENABLE != 0U) || \
	(MODULE_TASK6_CAN_ENABLE != 0U))
	/* 旧 CAN 测试或课题六收发启用时，初始化 CAN1 和接收滤波器 */
	if (CAN_Init() != HAL_OK)
	{
		FatalError(FAIL_CAN_INIT);
	}
#endif

#if ((MODULE_GYRO_ENABLE != 0U) || (MODULE_GYRO_CHECK_ENABLE != 0U))
	/* XV7001BB_Init() 内含约 210ms HAL_Delay，必须在创建 FreeRTOS 对象前完成。
	   旧接口巡检默认关闭，避免对课题六采样中的寄存器/命令产生干扰。 */
	if (XV7001bb_Init() != HAL_OK)
	{
		FatalError(FAIL_GYRO_INIT);
	}
	gyroInitOk = 1U;
#endif

#if ((MODULE_SPI_TEST_ENABLE != 0U) || (MODULE_GYRO_ENABLE != 0U) || \
	(MODULE_GYRO_CHECK_ENABLE != 0U))
	/* ---- 第 3.5 步：创建 SPI2 总线锁（供多任务互斥使用） ----
	   ★ 位置很关键，必须夹在"陀螺仪初始化之后"和"创建任务之前"中间：
	     1) 陀螺仪之后 —— SPI_BusMutexInit() 内部会进 FreeRTOS 临界区，
	        而调度器启动前临界区嵌套计数复位不了，中断屏蔽位 BASEPRI 会一直
	        保持屏蔽状态（configMAX_SYSCALL_INTERRUPT_PRIORITY = 0x50），
	        SysTick 再也进不了中断、HAL_GetTick() 不再增长。
	        陀螺仪初始化里有两处 HAL_Delay()，靠的正是 SysTick 计时，
	        一旦把本函数提到它前面，这两处就地死等 ——
	        现象是 failCode 和 gyroInitOk 都停在 0（卡住，不是报错）。
	        详细原理见 spi.c 里 SPI_BusMutexInit() 的注释。
	     2) 创建任务之前 —— 任务一跑起来就要用 SPI2 总线锁，锁必须先建好。
	     本函数之后 main() 里只剩创建任务和启动调度器，没有任何延时，
	     紧接着 osKernelStart() 会把临界区计数复位并重新开中断 */
	if (SPI_BusMutexInit() != HAL_OK)
	{
		FatalError(FAIL_SPI_MUTEX_INIT);
	}
#endif

	/* ---- 第 4 步：创建任务（每个任务都由对应的模块开关控制） ---- */

	/* 主任务：SPI / CAN 的周期发送，两个功能各自用 #if 控制，开关关掉就什么都不发 */
	osThreadDef(MAIN_TASK, task_main, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	if (osThreadCreate(osThread(MAIN_TASK), NULL) == NULL)
	{
		FatalError(FAIL_TASK_CREATE);
	}

#if ((MODULE_CAN_RX_ENABLE != 0U) || (MODULE_TASK6_CAN_ENABLE != 0U))
	/* CAN 接收任务：有界排空 FIFO0，并解析课题六命令帧 */
	osThreadDef(CAN_RX_TASK, task_can_rx, osPriorityNormal, 0,
				(configMINIMAL_STACK_SIZE * 2U));
	if (osThreadCreate(osThread(CAN_RX_TASK), NULL) == NULL)
	{
		FatalError(FAIL_TASK_CREATE);
	}
#endif

#if (MODULE_GYRO_ENABLE != 0U)
	/* 陀螺仪任务：高于通信任务优先级，按 10ms 周期采样、校准并积分 */
	osThreadDef(GYRO_TASK, task_gyro, osPriorityAboveNormal, 0,
				(configMINIMAL_STACK_SIZE * 2U));
	if (osThreadCreate(osThread(GYRO_TASK), NULL) == NULL)
	{
		FatalError(FAIL_TASK_CREATE);
	}
#endif

#if (MODULE_TASK6_CAN_ENABLE != 0U)
	/* CAN 测量发送任务：发送一致快照，不在此任务访问 SPI */
	osThreadDef(CAN_TX_TASK, task_can_tx, osPriorityNormal, 0,
				(configMINIMAL_STACK_SIZE * 2U));
	if (osThreadCreate(osThread(CAN_TX_TASK), NULL) == NULL)
	{
		FatalError(FAIL_TASK_CREATE);
	}
#endif

#if (MODULE_LED_ENABLE != 0U)
	/* 指示灯任务：每 0.5s 翻转 PB9，用来证明系统还在正常运行 */
	osThreadDef(LED_TASK, task_led, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	if (osThreadCreate(osThread(LED_TASK), NULL) == NULL)
	{
		FatalError(FAIL_TASK_CREATE);
	}
#endif

	/* ---- 第 5 步：启动调度器，从此只在任务之间切换，不再返回 ---- */
	osKernelStart();

	/* 能走到这里说明调度器都退出了，属于致命错误 */
	FatalError(FAIL_TASK_CREATE);
}

/* ---------------------------------------------------------------------------
 * FreeRTOS 的时基：HAL 的毫秒计数和 FreeRTOS 的 tick 用同一个中断
 * --------------------------------------------------------------------------- */
void SysTick_Handler(void)
{
	HAL_IncTick();
	osSystickHandler();
}

/* ---------------------------------------------------------------------------
 * 系统时钟配置：HSE 8MHz 外部晶振 -> PLL ×9 -> SYSCLK 72MHz
 *     AHB = 72MHz，APB1 = 36MHz（SPI2 和 CAN1 都在 APB1 上），APB2 = 72MHz
 * @note  若外部晶振起振失败则直接返回，系统退回 HSI 8MHz 继续跑，
 *        此时 sysClockIs72M = 0 —— 通信不通时先看这个标志
 * --------------------------------------------------------------------------- */
static void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		sysClockIs72M = 0U;
		return;
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
								  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		sysClockIs72M = 0U;
		return;
	}

	sysClockIs72M = 1U;
}

/* ---------------------------------------------------------------------------
 * 主任务：保留旧 SPI/CAN 测试和陀螺仪接口巡检代码，由独立开关单独控制
 * @note  课题六采样数据由 task_gyro 负责，绝不在此任务执行可能影响传感器的巡检
 *        SPI 发送失败当成硬故障（配置错了才会失败），进 FatalError；
 *        CAN 发送失败只累加 canTxErrCount，不进 FatalError —— 总线上没有
 *        第二个节点应答是正常现象，要让其它任务继续跑，方便反复观察
 * --------------------------------------------------------------------------- */
static void task_main(void const *argument)
{
	(void)argument;

	for (;;)
	{
#if (MODULE_GYRO_CHECK_ENABLE != 0U)
		/* 旧接口巡检会写寄存器/发送命令，仅通过专用开关显式启用。 */
		GyroInterfaceCheck();
#endif

#if (MODULE_SPI_TEST_ENABLE != 0U)
		/* ---- 课题四.2 / 四.3：SPI2 每 0.5s 发送 1 个字节 0xAA ----
		   示波器探头分别放 PB12(片选) / PB13(SCLK) / PB15(MOSI)，
		   能看到"片选拉低 -> 8 个时钟 -> 片选拉高"这一组完整波形 */
		if (SPI_SendByte(SPI_TEST_DATA) != HAL_OK)
		{
			FatalError(FAIL_SPI_SEND);
		}
#endif

#if (MODULE_CAN_TX_ENABLE != 0U)
		/* ---- 课题四.5~四.7：CAN 每 0.5s 以标准帧 ID 0x123 发送 1 字节 0xAA ----
		   回环自检还是正常总线模式由 can.h 的 CAN_LOOPBACK_TEST 决定，
		   Watch 窗口看 canMode：1 = 回环自检，0 = 正常总线模式 */
		(void)CAN_SendByte(CAN_TEST_DATA);
#endif

		osDelay(TASK_MAIN_PERIOD_MS);
	}
}

#if ((MODULE_CAN_RX_ENABLE != 0U) || (MODULE_TASK6_CAN_ENABLE != 0U))
/* 有界排空 CAN FIFO0 并解析完整帧；FIFO 空闲不是通信故障。 */
static void task_can_rx(void const *argument)
{
	uint16_t stdId;
	uint8_t data[8];
	uint8_t len;
	uint8_t framesRead;
	HAL_StatusTypeDef status;

	(void)argument;

	for (;;)
	{
		for (framesRead = 0U; framesRead < GYRO_RX_MAX_FRAMES_PER_POLL; framesRead++)
		{
			status = CAN_ReceiveFrame(&stdId, data, &len);
			if (status == HAL_TIMEOUT)
			{
				break;
			}
			if (status != HAL_OK)
			{
				gyroCanRxProtocolErrCount++;
				continue;
			}

			if (len > 0U)
			{
				canRxData = data[0];
				canRxReady = 1U;
			}

#if (MODULE_TASK6_CAN_ENABLE != 0U)
			/* 0x321 / 0x323 是本机自己的发送 ID：如果被回环或上位机转发回 FIFO，
			 * 直接丢弃即可，不能算成协议错误，否则会把正常回显计成故障。 */
			if ((stdId == CAN_GYRO_DATA_STD_ID) || (stdId == CAN_GYRO_RATE_STD_ID))
			{
				continue;
			}

			if ((stdId == CAN_GYRO_COMMAND_STD_ID) && (len == 1U))
			{
				if (data[0] == CAN_GYRO_CMD_RECALIBRATE)
				{
					taskENTER_CRITICAL();
					gyroRequestRecalibration = 1U;
					taskEXIT_CRITICAL();
					gyroCanRxCommandCount++;
				}
				else if (data[0] == CAN_GYRO_CMD_ZERO_ANGLE)
				{
					taskENTER_CRITICAL();
					gyroRequestZeroAngle = 1U;
					taskEXIT_CRITICAL();
					gyroCanRxCommandCount++;
				}
				else
				{
					gyroCanRxProtocolErrCount++;
				}
			}
			else
			{
				gyroCanRxProtocolErrCount++;
			}
#else
			(void)stdId;
#endif
		}

		osDelay(CAN_RX_POLL_PERIOD_MS);
	}
}
#endif

#if (MODULE_GYRO_ENABLE != 0U)
static float GyroMedianOfThree(float a, float b, float c)
{
	if (a > b)
	{
		float temp = a;
		a = b;
		b = temp;
	}
	if (b > c)
	{
		float temp = b;
		b = c;
		c = temp;
	}
	if (a > b)
	{
		float temp = a;
		a = b;
		b = temp;
	}
	return b;
}

#if (GYRO_DRIFT_SUPPRESS_ENABLE != 0U)
/* 小工具：取浮点绝对值。只为 fabsf 引入 <math.h> 不划算，这里手写一行。 */
static float GyroAbsF(float value)
{
	return (value < 0.0f) ? -value : value;
}
#endif

/* 10ms 周期角速度采样、静止零偏校准和梯形积分。温度每秒读取一次。
 * 偏置校正后的角速度先过 3 点中值抑制孤立尖峰，再过约 5Hz 一阶 IIR，
 * 滤波后的角速度才参与梯形积分。
 * 积分之前还有一道"零漂抑制"：判定静止（ZUPT）时锁定角度并把残余零偏慢慢
 * 收进 bias，避免静止时角度持续单向漂移，详见文件上方 GYRO_DRIFT_SUPPRESS_* 说明。
 * SPI 正常传输远短于周期，但超时仍可能延长本周期，这不代表硬实时保证。 */
static void task_gyro(void const *argument)
{
	const TickType_t periodTicks = pdMS_TO_TICKS(GYRO_SAMPLE_PERIOD_MS);
	TickType_t lastWakeTick = xTaskGetTickCount();
	uint32_t taskStartMs = HAL_GetTick();
	uint32_t lastAttemptMs = 0U;
	uint32_t lastSuccessfulReadMs = 0U;
	uint32_t previousValidMs = 0U;
	uint32_t lastTempMs = taskStartMs;
	uint32_t sampleIntervalMs = 0U;
	uint32_t attemptIntervalMs;
	uint32_t elapsedMs;
	uint32_t sampleStartMs;
	float calibrationSum = 0.0f;
	float calibrationMin = 0.0f;
	float calibrationMax = 0.0f;
	float calibrationMean;
	float biasDps = 0.0f;
	float correctedRateDps = 0.0f;
	float filteredRateDps = 0.0f;
	float previousFilteredRateDps = 0.0f;
	float medianRateDps;
	float filterAlpha;
	float filterSamples[3] = {0.0f, 0.0f, 0.0f};
	float accumulatedAngleDeg = 0.0f;
	float measuredRateDps = 0.0f;
	float tempCelsius = 0.0f;
	float dtSeconds;
	uint32_t calibrationCount = 0U;
	uint8_t havePreviousRate = 0U;
	uint8_t dataValid;
	uint8_t tempValid = 0U;
	uint8_t calibrationState = 0U; /* 0=稳定等待、1=采样、2=完成、3=失败 */
	uint8_t requestRecalibration;
	uint8_t requestZeroAngle;
	uint8_t readTemp;
	uint8_t filterIndex = 0U;
	uint8_t filterPrimed = 0U;
	uint32_t driftWindowStartMs = 0U;      /* 漂移统计窗口起点，0 = 尚未开始          */
	uint32_t driftWindowElapsedMs = 0U;
	float driftWindowBaseAngle = 0.0f;     /* 窗口起点的角度，用来算变化率            */
	float angleDriftDps = 0.0f;            /* 最近一个窗口的角度变化率，单位 °/s      */
#if (GYRO_DRIFT_SUPPRESS_ENABLE != 0U)
	uint32_t zuptCounter = 0U;             /* 连续满足静止判据的样本计数              */
	uint32_t zuptFrozenSamples = 0U;       /* 被 ZUPT 锁定、没有积分的周期数          */
	uint32_t biasTrackUpdates = 0U;        /* 偏置慢跟踪执行次数                      */
	uint8_t zuptActive = 0U;               /* 1 = 判定静止，本周期锁定角度            */
	uint8_t havePreviousCorrected = 0U;    /* 是否已有关键的上一拍补偿后角速度        */
	float previousCorrectedRateDps = 0.0f; /* 上一拍补偿后角速度，用于跳变判据        */
	float biasTrackBaseDps = 0.0f;         /* 校准结束时的零偏，用于统计跟踪增量      */
#endif

	(void)argument;

	for (;;)
	{
		vTaskDelayUntil(&lastWakeTick, periodTicks);

		/* CAN 请求通过单字节标志交接，只有本任务改变校准和积分状态。 */
		taskENTER_CRITICAL();
		requestRecalibration = gyroRequestRecalibration;
		requestZeroAngle = gyroRequestZeroAngle;
		gyroRequestRecalibration = 0U;
		gyroRequestZeroAngle = 0U;
		taskEXIT_CRITICAL();

		if (requestZeroAngle != 0U)
		{
			accumulatedAngleDeg = 0.0f;
			havePreviousRate = 0U;
			/* 角度清零后漂移统计窗口一起重来，避免把清零前的变化算进去 */
			driftWindowStartMs = 0U;
			angleDriftDps = 0.0f;
		}

		if (requestRecalibration != 0U)
		{
			calibrationCount = 0U;
			calibrationSum = 0.0f;
			calibrationMin = 0.0f;
			calibrationMax = 0.0f;
			calibrationState = 1U;
			havePreviousRate = 0U;
			filterPrimed = 0U;
			driftWindowStartMs = 0U;
			angleDriftDps = 0.0f;
		}

		sampleStartMs = HAL_GetTick();
		if (lastAttemptMs != 0U)
		{
			attemptIntervalMs = sampleStartMs - lastAttemptMs;
			if (attemptIntervalMs > GYRO_SAMPLE_PERIOD_MS)
			{
				gyroOverrunCount++;
			}
		}
		lastAttemptMs = sampleStartMs;

		if ((calibrationState == 0U) &&
			((sampleStartMs - taskStartMs) >= GYRO_STABILIZE_MS))
		{
			calibrationState = 1U;
		}

		readTemp = ((sampleStartMs - lastTempMs) >= GYRO_TEMP_PERIOD_MS) ? 1U : 0U;
		if (XV7001bb_ReadAngleDps(&measuredRateDps) == HAL_OK)
		{
			dataValid = ((measuredRateDps <= XV7001BB_RANGE_DPS) &&
						 (measuredRateDps >= -XV7001BB_RANGE_DPS)) ? 1U : 0U;
		}
		else
		{
			dataValid = 0U;
		}
		if (dataValid != 0U)
		{
			if (lastSuccessfulReadMs != 0U)
			{
				sampleIntervalMs = sampleStartMs - lastSuccessfulReadMs;
			}
			lastSuccessfulReadMs = sampleStartMs;
			gyroReadCount++;
			correctedRateDps = measuredRateDps - biasDps;

			if (calibrationState == 1U)
			{
				if (calibrationCount == 0U)
				{
					calibrationMin = measuredRateDps;
					calibrationMax = measuredRateDps;
				}
				else
				{
					if (measuredRateDps < calibrationMin)
					{
						calibrationMin = measuredRateDps;
					}
					if (measuredRateDps > calibrationMax)
					{
						calibrationMax = measuredRateDps;
					}
				}
				calibrationSum += measuredRateDps;
				calibrationCount++;

				if (calibrationCount >= GYRO_CALIBRATION_SAMPLES)
				{
					calibrationMean = calibrationSum / (float)calibrationCount;
					if (((calibrationMax - calibrationMin) <= GYRO_CALIBRATION_MAX_RANGE_DPS) &&
						((calibrationMean <= GYRO_CALIBRATION_MAX_BIAS_DPS) &&
						 (calibrationMean >= -GYRO_CALIBRATION_MAX_BIAS_DPS)))
					{
						biasDps = calibrationMean;
						calibrationState = 2U;
						accumulatedAngleDeg = 0.0f;
						correctedRateDps = measuredRateDps - biasDps;
						filterSamples[0] = correctedRateDps;
						filterSamples[1] = correctedRateDps;
						filterSamples[2] = correctedRateDps;
						filteredRateDps = correctedRateDps;
						filterIndex = 0U;
						filterPrimed = 1U;
						/* 校准刚结束、角度已归零：漂移统计窗口重新开始 */
						driftWindowStartMs = 0U;
						angleDriftDps = 0.0f;
#if (GYRO_DRIFT_SUPPRESS_ENABLE != 0U)
						/* 以刚校准好的零偏作为慢跟踪基准，静止判定从零计时 */
						biasTrackBaseDps = biasDps;
						biasTrackUpdates = 0U;
						zuptCounter = 0U;
						zuptActive = 0U;
						havePreviousCorrected = 0U;
						previousCorrectedRateDps = 0.0f;
#endif
					}
					else
					{
						calibrationState = 3U;
						gyroCalibrationFailCount++;
					}
					havePreviousRate = 0U;
				}
			}
			else if (calibrationState == 2U)
			{
				elapsedMs = (havePreviousRate != 0U) ?
					(sampleStartMs - previousValidMs) : 0U;

#if (GYRO_DRIFT_SUPPRESS_ENABLE != 0U)
				/* --- 零漂抑制第 1 步：静止判定（ZUPT 判据）--------------------
				 * 判据 = 偏置补偿后的角速度足够小，且相邻样本不跳变。
				 * 必须连续满足 GYRO_ZUPT_CONFIRM_SAMPLES 个样本（默认 0.5s）才算
				 * 静止：这样正常转动、以及转动中刚好路过 0 的那一瞬间都不会被误判。
				 * 一旦出现真实转动，计数立刻清零，重新计时。 */
				if ((GyroAbsF(correctedRateDps) <= GYRO_ZUPT_MAX_RATE_DPS) &&
					((havePreviousCorrected == 0U) ||
					 (GyroAbsF(correctedRateDps - previousCorrectedRateDps) <=
					  GYRO_ZUPT_MAX_STEP_DPS)))
				{
					if (zuptCounter < GYRO_ZUPT_CONFIRM_SAMPLES)
					{
						zuptCounter++;
					}
				}
				else
				{
					zuptCounter = 0U;
				}
				zuptActive = (zuptCounter >= GYRO_ZUPT_CONFIRM_SAMPLES) ? 1U : 0U;

				if (zuptActive != 0U)
				{
					/* --- 零漂抑制第 2 步：残余零偏慢跟踪 -----------------------
					 * 确认静止后，让 bias 朝当前补偿后角速度走一小步（约 4s 时间
					 * 常数），把 1~2 个 LSB 的残余零偏慢慢吃掉。步长足够小，所以
					 * 不会在角度曲线上留下台阶。
					 * 跟踪后立刻用新 bias 重算 corrected，保证下面滤波和积分拿到
					 * 的是同一时刻的一致值。 */
					biasDps += GYRO_BIAS_TRACK_GAIN * correctedRateDps;
					correctedRateDps = measuredRateDps - biasDps;
					biasTrackUpdates++;
				}
				previousCorrectedRateDps = correctedRateDps;
				havePreviousCorrected = 1U;
#endif

				if (filterPrimed == 0U)
				{
					filterSamples[0] = correctedRateDps;
					filterSamples[1] = correctedRateDps;
					filterSamples[2] = correctedRateDps;
					filteredRateDps = correctedRateDps;
					filterIndex = 0U;
					filterPrimed = 1U;
				}
				else
				{
					filterSamples[filterIndex] = correctedRateDps;
					filterIndex = (uint8_t)((filterIndex + 1U) % 3U);
					medianRateDps = GyroMedianOfThree(
						filterSamples[0], filterSamples[1], filterSamples[2]);
					if (elapsedMs > 0U)
					{
						dtSeconds = (float)elapsedMs / 1000.0f;
						filterAlpha = dtSeconds / (GYRO_LOWPASS_TAU_SECONDS + dtSeconds);
						filteredRateDps += filterAlpha * (medianRateDps - filteredRateDps);
					}
				}

				if (havePreviousRate != 0U)
				{
#if (GYRO_DRIFT_SUPPRESS_ENABLE != 0U)
					if (zuptActive != 0U)
					{
						/* --- 零漂抑制第 3 步：静止锁定 -----------------------------
						 * 已判定静止，本周期不积分：角度保持不变，残余零偏就
						 * 不会继续被累积成角度漂移。滤波状态照常更新，所以
						 * 一旦真的开始转动，角度能立刻平滑接上，不会跳变。 */
						zuptFrozenSamples++;
					}
					else
#endif
					{
						dtSeconds = (float)elapsedMs / 1000.0f;
						accumulatedAngleDeg +=
							((previousFilteredRateDps + filteredRateDps) * 0.5f) *
							dtSeconds * GYRO_ANGLE_SCALE_CALIBRATION;
					}
				}
				previousFilteredRateDps = filteredRateDps;
				previousValidMs = sampleStartMs;
				havePreviousRate = 1U;

				/* --- 零漂抑制效果监视：每 1s 算一次角度变化率 ------------------
				 * 静止时该值越接近 0°/s，说明零漂压得越好（可以直接在 Watch 里
				 * 看 gyroAngleDriftDps）；转动时它等于真实角速度，属正常现象。 */
				if (driftWindowStartMs == 0U)
				{
					driftWindowStartMs = sampleStartMs;
					driftWindowBaseAngle = accumulatedAngleDeg;
				}
				else
				{
					driftWindowElapsedMs = sampleStartMs - driftWindowStartMs;
					if (driftWindowElapsedMs >= GYRO_DRIFT_WINDOW_MS)
					{
						angleDriftDps = (accumulatedAngleDeg - driftWindowBaseAngle) *
							1000.0f / (float)driftWindowElapsedMs;
						driftWindowBaseAngle = accumulatedAngleDeg;
						driftWindowStartMs = sampleStartMs;
					}
				}
			}
			else
			{
				havePreviousRate = 0U;
			}
		}
		else
		{
			gyroErrCount++;
			dataValid = 0U;
			havePreviousRate = 0U;
			filterPrimed = 0U;
			/* 本次读取失败：静止判定和漂移统计窗口一起作废，恢复后重新计时，
			 * 避免用一段不连续的数据去判断"是否静止" */
			driftWindowStartMs = 0U;
#if (GYRO_DRIFT_SUPPRESS_ENABLE != 0U)
			zuptCounter = 0U;
			zuptActive = 0U;
			havePreviousCorrected = 0U;
#endif
		}

		if (readTemp != 0U)
		{
			lastTempMs = sampleStartMs;
			if (XV7001bb_ReadTmpCelsius(&tempCelsius) == HAL_OK)
			{
				tempValid = 1U;
			}
			else
			{
				tempValid = 0U;
				gyroErrCount++;
			}
		}

		taskENTER_CRITICAL();
		gyroRateDps = measuredRateDps;
		gyroRateCorrectedDps = correctedRateDps;
		gyroRateFilteredDps = filteredRateDps;
		gyroBiasDps = biasDps;
		gyroAccumulatedAngleDeg = accumulatedAngleDeg;
		gyroCalibrationState = calibrationState;
		gyroCalibrationSampleCount = calibrationCount;
		gyroCalibrationRangeDps =
			(calibrationCount > 0U) ? (calibrationMax - calibrationMin) : 0.0f;
		gyroDataValid = dataValid;
		gyroSampleIntervalMs = sampleIntervalMs;
#if (GYRO_DRIFT_SUPPRESS_ENABLE != 0U)
		/* 零漂抑制监视量（开关关闭时这些量保持 0，表示未启用） */
		gyroZuptState = zuptActive;
		gyroZuptHoldCount = zuptCounter;
		gyroZuptFrozenSamples = zuptFrozenSamples;
		gyroBiasTrackUpdateCount = biasTrackUpdates;
		gyroBiasTrackDeltaDps = biasDps - biasTrackBaseDps;
#endif
		gyroAngleDriftDps = angleDriftDps;
		if (readTemp != 0U)
		{
			if (tempValid != 0U)
			{
				gyroTempCelsius = tempCelsius;
			}
			gyroTempValid = tempValid;
		}
		taskEXIT_CRITICAL();
	}
}

#endif

#if (MODULE_TASK6_CAN_ENABLE != 0U)
/* 把浮点物理量按协议比例转成有符号整数，防止超量程时整数溢出。 */
static int32_t GyroScaleToInt32(float value, float scale)
{
	float scaled = value * scale;

	if (scaled >= 2147483000.0f)
	{
		return INT32_MAX;
	}
	if (scaled <= -2147483000.0f)
	{
		return INT32_MIN;
	}
	return (int32_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static int16_t GyroScaleToInt16(float value, float scale)
{
	float scaled = value * scale;

	if (scaled >= 32767.0f)
	{
		return INT16_MAX;
	}
	if (scaled <= -32768.0f)
	{
		return INT16_MIN;
	}
	return (int16_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

/* 100ms 发送一致快照；测量任务在临界区更新同一组字段，避免撕裂多字节数据。
 * 每个周期连发两帧，两帧共用同一次临界区快照，所以角度、温度、角速度严格同源：
 *   0x321 / DLC=8  角度 + 温度 + 状态位 + 采样间隔（协议不变）
 *   0x323 / DLC=4  滤波后角速度，单位 0.001 °/s，小端
 * 两帧的发送返回值分别判断、分别计数，便于区分是哪一帧出问题。 */
static void task_can_tx(void const *argument)
{
	uint8_t payload[8];
	uint8_t ratePayload[4];
	uint8_t statusFlags;
	uint8_t sampleInterval;
	uint32_t angleBits;
	uint32_t rateBits;
	uint16_t tempBits;
	int32_t angleCentiDegrees;
	int32_t rateMilliDps;
	int16_t tempCentiDegrees;
	float angleDegrees;
	float temperatureCelsius;
	float filteredRateDps;
	uint8_t dataValid;
	uint8_t tempValid;
	uint8_t calibrationState;
	uint8_t zuptState;
	uint32_t intervalMs;

	(void)argument;

	for (;;)
	{
		/* 一次临界区同时取走三路数据的快照：角度、温度、滤波后角速度。
		 * 三个量来自 task_gyro 的同一拍，所以 0x321 和 0x323 反映的是同一时刻。 */
		taskENTER_CRITICAL();
		angleDegrees = gyroAccumulatedAngleDeg;
		temperatureCelsius = gyroTempCelsius;
		filteredRateDps = gyroRateFilteredDps;
		dataValid = gyroDataValid;
		tempValid = gyroTempValid;
		calibrationState = gyroCalibrationState;
		zuptState = gyroZuptState;
		intervalMs = gyroSampleIntervalMs;
		taskEXIT_CRITICAL();

		angleCentiDegrees = GyroScaleToInt32(angleDegrees, 100.0f);
		tempCentiDegrees = GyroScaleToInt16(temperatureCelsius, 100.0f);
		/* 角速度乘 1000 后四舍五入成整数：GyroScaleToInt32 内部按"四舍五入、
		 * 远离零"处理（正数 +0.5、负数 -0.5），±100 °/s 满量程对应 ±100000，
		 * 远未触及 int32 上限，不会溢出。 */
		rateMilliDps = GyroScaleToInt32(filteredRateDps, CAN_GYRO_RATE_SCALE);
		angleBits = (uint32_t)angleCentiDegrees;
		rateBits = (uint32_t)rateMilliDps;
		tempBits = (uint16_t)tempCentiDegrees;
		statusFlags = 0U;
		if ((dataValid != 0U) && (calibrationState == 2U))
		{
			statusFlags |= 0x01U;
		}
		if (tempValid != 0U)
		{
			statusFlags |= 0x02U;
		}
		if (calibrationState == 2U)
		{
			statusFlags |= 0x04U;
		}
		else if (calibrationState == 3U)
		{
			statusFlags |= 0x08U;
		}
		if (zuptState != 0U)
		{
			/* bit4：零漂抑制判定为静止（角度已锁定，不再漂移） */
			statusFlags |= 0x10U;
		}
		sampleInterval = (intervalMs > 255U) ? 255U : (uint8_t)intervalMs;

		payload[0] = (uint8_t)(angleBits & 0xFFU);
		payload[1] = (uint8_t)((angleBits >> 8) & 0xFFU);
		payload[2] = (uint8_t)((angleBits >> 16) & 0xFFU);
		payload[3] = (uint8_t)((angleBits >> 24) & 0xFFU);
		payload[4] = (uint8_t)(tempBits & 0xFFU);
		payload[5] = (uint8_t)((tempBits >> 8) & 0xFFU);
		payload[6] = statusFlags;
		payload[7] = sampleInterval;

		/* 帧 1：0x321 角度/温度/状态帧，DLC=8 */
		if (CAN_SendFrame(CAN_GYRO_DATA_STD_ID, payload, CAN_GYRO_DATA_DLC) == HAL_OK)
		{
			gyroCanTxOkCount++;
		}
		else
		{
			gyroCanTxErrCount++;
		}

		/* 帧 2：0x323 滤波后角速度帧，DLC=4，小端 int32，单位 0.001 °/s */
		ratePayload[0] = (uint8_t)(rateBits & 0xFFU);
		ratePayload[1] = (uint8_t)((rateBits >> 8) & 0xFFU);
		ratePayload[2] = (uint8_t)((rateBits >> 16) & 0xFFU);
		ratePayload[3] = (uint8_t)((rateBits >> 24) & 0xFFU);

		if (CAN_SendFrame(CAN_GYRO_RATE_STD_ID, ratePayload, CAN_GYRO_RATE_DLC) == HAL_OK)
		{
			gyroCanTxOkCount++;
			gyroCanTxRateOkCount++;
		}
		else
		{
			gyroCanTxErrCount++;
			gyroCanTxRateErrCount++;
		}
		osDelay(CAN_TX_PERIOD_MS);
	}
}
#endif

#if (MODULE_GYRO_CHECK_ENABLE != 0U)
/* ---------------------------------------------------------------------------
 * 记录巡检失败的步骤号（只记第一个出错的步骤，后面的不覆盖它）
 * @param step 步骤号，见 GyroInterfaceCheck() 里的编号
 * --------------------------------------------------------------------------- */
static void GyroCheckFail(uint8_t step)
{
	if (gyroCheckFailStep == 0U)
	{
		gyroCheckFailStep = step;
	}
}

/* ---------------------------------------------------------------------------
 * 课题五：在 task_main 里巡检 XV7001BB —— 读状态寄存器 + 逐个确认每个功能接口
 * @note  每轮主任务（500ms）跑一遍，结果全部写进全局变量，Watch 窗口逐条核对：
 *            gyroStatus / gyroPreSts / gyroProCOK  ← 状态寄存器 StsRd(0x04)
 *            gyroOkInit       b) 初始化      （用 IFCtl 与器件状态复核）
 *            gyroOkWriteData  a) 写入数据    （写回原值后回读比对）
 *            gyroOkReadReg    c) 读取寄存器
 *            gyroOkReadTmp    d) 读取温度    （含换算与范围检查）
 *            gyroOkReadAngle  e) 读取角速度  （含换算与范围检查）
 *            gyroOkAux        辅助接口       （多寄存器读、组合读+换算、读-改-写）
 *            gyroOkCmd        命令类接口     （只在第 1 轮测一次）
 *            gyroCheckAllOk   上面 7 个标志是否全部为 1
 *        ★ 巡检失败不做死循环：只累加 gyroCheckFailCount 并记下 gyroCheckFailStep，
 *          CAN / LED 继续跑，方便反复观察、反复试
 *        ★ SPI2 是共享总线，本函数与 task_gyro 之间由 spi.c 的总线锁互斥
 * --------------------------------------------------------------------------- */
static void GyroInterfaceCheck(void)
{
	uint8_t status = 0U;
	uint8_t statusAfter = 0U;
	uint8_t outCtl1 = 0U;
	uint8_t writeBack = 0U;
	uint8_t dspCtl1 = 0U;
	uint8_t dspCtl2 = 0U;
	uint8_t dspCtl3 = 0U;
	uint8_t tsFormat = 0U;
	uint8_t ifCtl = 0U;
	uint8_t dspCtl3Again = 0U;
	uint8_t calibEnable = 0U;
	uint8_t readOk = 0U;
	uint8_t cfgReadOk = 0U;
	int16_t tempRaw = 0;
	int16_t rateRaw = 0;
	float celsius = 0.0f;
	float dps = 0.0f;
	float celsius2 = 0.0f;
	float dps2 = 0.0f;

	gyroCheckRoundCount++;
	gyroCheckFailStep = 0U;

	/* ---- 第 1 步：读状态寄存器 StsRd(0x04)，解析它的两个位域 ----
	   proCOK(bit3)   = 1 表示温度数据已可读取
	   preSts(bit2:0) = 器件状态，正常工作时应为 001（已退出睡眠） */
	if (XV7001bb_ReadStatus(&status) != HAL_OK)
	{
		GyroCheckFail(1U);
	}

	gyroStatus = status;
	gyroPreSts = (uint8_t)(status & XV7001BB_STS_PREST_MASK);
	gyroProCOK = ((status & XV7001BB_STS_PROCOK) != 0U) ? 1U : 0U;

	/* ---- 第 2 步：c) 读寄存器 —— 读 OutCtl1(0x0B)，顺便把原值留给第 3 步 ---- */
	if (XV7001bb_ReadReg(XV7001BB_REG_OUTCTL1, &outCtl1) != HAL_OK)
	{
		gyroOkReadReg = 0U;
		readOk = 0U;
		GyroCheckFail(2U);
	}
	else
	{
		gyroOkReadReg = 1U;
		readOk = 1U;
	}

	/* ---- 第 3 步：a) 写数据 —— 把读到的值原样写回去，再读回来比对 ----
	   只写"读到的同一个值"，所以不会改动器件配置；比对一致才说明写通道通。
	   writeBack 先塞一个必定不同的值，避免通信失败时比对假通过 */
	writeBack = (uint8_t)(~outCtl1);

	if (readOk == 0U)
	{
		/* 原值都没读到，写回测试不能做：拿不准的值写进去会改掉器件配置，
		   这里直接判不通过，等下一轮再测 */
		gyroOkWriteData = 0U;
	}
	else if (XV7001bb_WriteData(XV7001BB_REG_OUTCTL1, outCtl1) != HAL_OK)
	{
		gyroOkWriteData = 0U;
		GyroCheckFail(3U);
	}
	else if ((XV7001bb_ReadReg(XV7001BB_REG_OUTCTL1, &writeBack) != HAL_OK) ||
			 (writeBack != outCtl1))
	{
		gyroOkWriteData = 0U;	/* 写进去的和读出来的不一致 -> 写通道不可靠 */
		GyroCheckFail(3U);
	}
	else
	{
		gyroOkWriteData = 1U;
	}

	gyroRegWriteBack = writeBack;

	/* ---- 第 4 步：读一圈配置寄存器（DspCtl1/2/3、TsDataFormat、IFCtl），
	   确认 c) 接口对不同地址都成立，同时把值放进 Watch 窗口备查 ---- */
	if ((XV7001bb_ReadReg(XV7001BB_REG_DSPCTL1, &dspCtl1) == HAL_OK) &&
		(XV7001bb_ReadReg(XV7001BB_REG_DSPCTL2, &dspCtl2) == HAL_OK) &&
		(XV7001bb_ReadReg(XV7001BB_REG_DSPCTL3, &dspCtl3) == HAL_OK) &&
		(XV7001bb_ReadReg(XV7001BB_REG_TSDATAFORMAT, &tsFormat) == HAL_OK) &&
		(XV7001bb_ReadReg(XV7001BB_REG_IFCTL, &ifCtl) == HAL_OK))
	{
		gyroCfgDspCtl1 = dspCtl1;
		gyroCfgDspCtl2 = dspCtl2;
		gyroCfgDspCtl3 = dspCtl3;
		gyroCfgTsFormat = tsFormat;
		gyroCfgIfCtl = ifCtl;
		cfgReadOk = 1U;
	}
	else
	{
		cfgReadOk = 0U;
		GyroCheckFail(4U);
	}

	/* ---- 第 5 步：b) 初始化复核 ----
	   Init() 把 IFCtl 写成了"4 线 SPI + 关闭 I²C"，所以读回来 bit1、bit0 都该是 0；
	   再加上器件必须停在 001（已退出睡眠）—— 两条都对，才算初始化确实生效 */
	if ((cfgReadOk != 0U) &&
		((gyroCfgIfCtl & (XV7001BB_IFCTL_SPISEL | XV7001BB_IFCTL_I2C_EN)) == 0U) &&
		(gyroPreSts == XV7001BB_STS_PREST_SLPOUT))
	{
		gyroOkInit = 1U;
	}
	else
	{
		gyroOkInit = 0U;
		GyroCheckFail(5U);
	}

	/* ---- 第 6 步：d) 读温度 —— 读原始码、换算成摄氏度、再做范围检查 ----
	   常温下应在 -40 ~ +100°C 之间；读到 0 或者离谱的值，说明字节拼错了 */
	if (XV7001bb_ReadTmp(&tempRaw) != HAL_OK)
	{
		gyroOkReadTmp = 0U;
		GyroCheckFail(6U);
	}
	else
	{
		gyroTempRaw = tempRaw;
		celsius = XV7001bb_TmpToCelsius(tempRaw);
		gyroTempCelsius = celsius;

		if ((celsius >= -40.0f) && (celsius <= 100.0f))
		{
			gyroOkReadTmp = 1U;
		}
		else
		{
			gyroOkReadTmp = 0U;		/* 值不在合理范围 -> 读回来的字节不对 */
			GyroCheckFail(6U);
		}
	}

	/* ---- 第 7 步：e) 读角速度 —— 读原始码、换算成 °/s、再做范围检查 ----
	   静止时应远小于量程 ±100°/s，超出量程说明读回来的字节不对 */
	if (XV7001bb_ReadAngle(&rateRaw) != HAL_OK)
	{
		gyroOkReadAngle = 0U;
		GyroCheckFail(7U);
	}
	else
	{
		gyroRateRaw = rateRaw;
		dps = XV7001bb_AngleToDps(rateRaw);
		gyroRateDps = dps;

		if ((dps >= -XV7001BB_RANGE_DPS) && (dps <= XV7001BB_RANGE_DPS))
		{
			gyroOkReadAngle = 1U;
		}
		else
		{
			gyroOkReadAngle = 0U;
			GyroCheckFail(7U);
		}
	}

	/* ---- 第 8 步：辅助接口 ----
	   (1) ReadTmpCelsius / ReadAngleDps：读+换算的组合接口，值也要在合理范围
	   (2) SetCalibCmd：内部是"读 DspCtl3 -> 改 bit6 -> 写回"的读-改-写。
	       这里刻意传入器件当前的值（原样写回），既验证了这条路径，
	       又不会真正改动校准使能位 —— 改了会影响陀螺仪读数。
	       ★ DspCtl3 是配置寄存器（手册标 R/W，里面是 EnCalibCmd + SelFs 等设置位），
	         不是 SLPIN/AUTOC 那类"发一次就执行"的命令寄存器，
	         所以把读到的原值写回去是安全、无副作用的 */
	calibEnable = ((dspCtl3 & XV7001BB_DSPCTL3_ENCALIBCMD) != 0U) ? 1U : 0U;

	if ((cfgReadOk != 0U) &&
		(XV7001bb_ReadTmpCelsius(&celsius2) == HAL_OK) &&
		(celsius2 >= -40.0f) && (celsius2 <= 100.0f) &&
		(XV7001bb_ReadAngleDps(&dps2) == HAL_OK) &&
		(dps2 >= -XV7001BB_RANGE_DPS) && (dps2 <= XV7001BB_RANGE_DPS) &&
		(XV7001bb_SetCalibCmd(calibEnable) == HAL_OK) &&
		(XV7001bb_ReadReg(XV7001BB_REG_DSPCTL3, &dspCtl3Again) == HAL_OK) &&
		(dspCtl3Again == dspCtl3))
	{
		gyroOkAux = 1U;
	}
	else
	{
		gyroOkAux = 0U;
		GyroCheckFail(8U);
	}

	/* ---- 第 9 步：命令类接口 SendCmd（只发地址、不带数据的命令） ----
	   只在第 1 轮测一次：用 Sleep-out（Init 里用过的同一条命令）确认命令通道能通，
	   并且器件执行后仍停在正常工作态。
	   ★ 只测一次的原因：这条命令会复位数字滤波器，每轮都发会影响陀螺仪读数 */
	if (gyroCheckRoundCount == 1U)
	{
		if ((XV7001bb_SleepOut() == HAL_OK) &&
			(XV7001bb_ReadStatus(&statusAfter) == HAL_OK) &&
			((statusAfter & XV7001BB_STS_PREST_MASK) == XV7001BB_STS_PREST_SLPOUT))
		{
			gyroOkCmd = 1U;
		}
		else
		{
			gyroOkCmd = 0U;
			GyroCheckFail(9U);
		}
	}

	/* ---- 汇总：7 个接口标志全为 1，才算"每个功能接口都正确" ---- */
	if ((gyroOkInit != 0U) && (gyroOkWriteData != 0U) && (gyroOkReadReg != 0U) &&
		(gyroOkReadTmp != 0U) && (gyroOkReadAngle != 0U) && (gyroOkAux != 0U) &&
		(gyroOkCmd != 0U))
	{
		gyroCheckAllOk = 1U;
	}
	else
	{
		gyroCheckAllOk = 0U;
		gyroCheckFailCount++;
	}
}
#endif

#if (MODULE_LED_ENABLE != 0U)
/* ---------------------------------------------------------------------------
 * 指示灯任务：PB9 每 0.5s 翻转一次（低电平点亮）
 * @note  它不依赖任何通信外设，所以只要灯在闪，就说明 MCU 和调度器是好的
 * --------------------------------------------------------------------------- */
static void task_led(void const *argument)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	(void)argument;

	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = LED_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);

	for (;;)
	{
		HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
		osDelay(LED_BLINK_INTERVAL_MS);
	}
}
#endif

/* ---------------------------------------------------------------------------
 * 致命错误处理：先记下失败码，再关中断死循环
 * @param code 失败码，见文件开头的 FAIL_xxx 定义
 * @note  卡死时用调试器暂停，在 Watch 窗口看 failCode 就知道是哪一步失败了
 * --------------------------------------------------------------------------- */
static void FatalError(uint8_t code)
{
	failCode = code;
	Error_Handler();
}

static void Error_Handler(void)
{
	__disable_irq();
	for (;;)
	{
	}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
	(void)file;
	(void)line;
	Error_Handler();
}
#endif