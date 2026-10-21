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
/* 课题六测量帧发送周期。
 * ★ 2026-10-20 由 100 改为 10：积分本来就在 100 Hz 跑，上报却只有 10 Hz，
 *   一次 180° 往返（加速 200 ms + 匀速 + 减速 200 ms）只能看到 2 个斜坡采样点，
 *   起动/停止丢掉的那 0.2~0.5° 在曲线上根本分辨不出来。
 *   1 Mbps 下每 10 ms 两帧（0x321 + 0x323）约 26 kbit/s，占用可以忽略。
 *   要回到旧节奏只改这一行即可；采样与积分周期（GYRO_SAMPLE_PERIOD_MS）不受影响。 */
#define CAN_TX_PERIOD_MS            10U     /* 课题六测量帧发送周期                   */
#define CAN_TX_MAILBOX_TIMEOUT_MS   2U      /* 4 帧共享 3 邮箱，等待有界且可被采样任务抢占 */
#define GYRO_SAMPLE_PERIOD_MS       10U     /* 角速度采样周期                         */
/* 单拍读取失败后，允许下一个有效样本"跨接"的最大空洞长度（ms）。
 * 只要空洞不超过这个值，就按真实 elapsedMs 把中间那段时间一起积起来；
 * 超过则判定采样已不可信，积分/ZUPT/滤波状态整体作废重来。
 * 取值要远大于正常抖动（正常 10ms，掉一拍 20ms），又远小于"丢了一秒角度"的量级。 */
#define GYRO_MAX_BRIDGE_MS          50U
#define GYRO_TEMP_PERIOD_MS         1000U   /* 温度读取降频，避免干扰角速度采样       */
#define GYRO_STABILIZE_MS           1000U   /* 上电稳定等待时间                       */
#define GYRO_CALIBRATION_SAMPLES    6000U   /* 60 s 估计；后续验证样本不参与零偏拟合 */
#define GYRO_CALIBRATION_VALIDATION_SAMPLES 2000U
#define GYRO_CALIBRATION_VALIDATION_BLOCK_SAMPLES 500U
/* 每个独立 5 s 窗口均值限值：若持续 200 s，偏差预算为 0.3 度。
 * 这是校准质量门限，不是未来温漂或机械静止的保证。 */
#define GYRO_CALIBRATION_MAX_RESIDUAL_DPS 0.0015f
#define GYRO_CALIBRATION_MAX_RANGE_DPS 8.0f /* 校准样本最大峰峰波动，超出则判失败（×4 换算自旧值 2.0，见 XV7001BB_SCALE_16BIT）*/
#define GYRO_CALIBRATION_MAX_BIAS_DPS  40.0f/* 校准均值过大时判为不静止/异常（×4 换算自旧值 10.0）*/
/* 校准判失败后的自动重试间隔。之前失败即停在 state 3 不再重试，桌面轻微震动就会
 * 让角度永久显示 --，必须重新上电才能恢复；现在等这么久自动重来一轮。 */
#define GYRO_CALIBRATION_RETRY_DELAY_MS 2000U
#define GYRO_RX_MAX_FRAMES_PER_POLL 4U     /* 每轮最多排空的 CAN 帧数，避免饿死采样   */
#define GYRO_LOWPASS_TAU_SECONDS    0.03183099f /* 一阶低通约 5Hz 时间常数            */
/* 中值不保持积分面积；默认仅校准和静止判定用中值，积分走线性低通。
 * 置 1 可回到旧积分链做 A/B。线性链不剔除量程内尖峰，仍需检查原始日志。 */
#ifndef GYRO_INTEGRATION_MEDIAN3_ENABLE
#define GYRO_INTEGRATION_MEDIAN3_ENABLE 0U
#endif
#if (GYRO_INTEGRATION_MEDIAN3_ENABLE > 1U)
#error "GYRO_INTEGRATION_MEDIAN3_ENABLE must be 0 or 1"
#endif
/* 往复测量基线：保留器件实测 /70，不用来源未确认的电机反馈拟合标度。 */
#define GYRO_ANGLE_SCALE_CALIBRATION 1.0f
/* 0：校准后固定零偏，整批往复（包括起步与中间停靠）不自动追踪。
 * 1：旧跟踪算法，仅供 A/B；它会把未解锁起步段的真实速度吸收到零偏。
 * 温漂需要在批次之间、确认机械静止后显式重新校准，不自动清零。 */
#define GYRO_BIAS_TRACK_ENABLE       0U
/* 连续测量不根据静止判据丢弃角度，允许显示真实静止漂移。
 * 置 1 恢复旧冻结/25 拍回补；静止判定本身仍用于校准保护和状态上报。 */
#ifndef GYRO_ZUPT_FREEZE_ENABLE
#define GYRO_ZUPT_FREEZE_ENABLE      0U
#endif
#if (GYRO_ZUPT_FREEZE_ENABLE > 1U)
#error "GYRO_ZUPT_FREEZE_ENABLE must be 0 or 1"
#endif

/* ---- 角度零漂抑制（课题六增强，总开关见下方 GYRO_DRIFT_SUPPRESS_ENABLE）----
 * 陀螺仪只输出角速度，残余零偏哪怕只有 1 个 LSB（1/70 ≈ 0.0143 °/s），
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
#define GYRO_ZUPT_MAX_RATE_DPS      40.00f  /* 判定静止的最大角速度幅值（°/s）
 * ★ 沿革：3.00（驱动小 4 倍的旧读数）→ 12（换算到真实 °/s）→ 20 → 40（2026-10-21）。
 * ★ 40 °/s 是极大的死区：比 40 °/s 慢的转动会被判成静止、不计入角度。
 *   按台架换算（100 mm/s ↔ 114.6 °/s）相当于 35 mm/s 以下的匀速运动。
 *   滞回判据（见下面 GYRO_ZUPT_JUDGE_HYSTERESIS）把这个门限同时用于
 *   入场和出场，所以死区不会因为抗尖峰而变大。
 * ★ 抬高门限的代价靠"起步坡道补偿"兜住：解锁时把冻结区间最后 0.25s
 *   的增量补回，40 °/s 在 573 °/s² 下坡道只有 0.07s，完全在缓冲内。
 * ★ 抬门限只能治标：真正的病根是 10ms 原值上出现 ~40 °/s 的单拍尖峰
 *   （2865 码量级，不是真实角振动）。门限必须明显高于该尖峰才能稳住锁，
 *   否则每次尖峰都会打断 0.5s 确认窗口。 */
#define GYRO_ZUPT_MAX_STEP_DPS      80.00f  /* 入场判据的相邻样本跳变上限
 * ★ 入场用的是 3 拍中值，只要连续两拍中值都满足 |c| <= 40，则 |Δc| <= 80
 *   必然成立，所以这个判据在当前取值下永远不生效（数学冗余）。
 *   保留它只为 GYRO_ZUPT_JUDGE_MEDIAN3 = 0U 的 A/B 分支（那时判据是原值，
 *   跳变判据才真正起作用）。 */
#define GYRO_ZUPT_CONFIRM_SAMPLES   50U     /* 连续满足静止判据的样本数（10ms → 0.5s）*/
/* 判定信号选择：0 = 直接用本拍补偿后角速度（原行为）
 *               1 = 用最近 3 拍的中值
 * 为什么默认 1：实测电机使能静止 5 分钟、31077 拍里只有 6 拍 |值| 超过 3.0°/s，
 * 全是孤立单拍尖峰；而每出现 1 拍，原判据就把 50 拍计数清零，白丢 0.5s 锁定。
 * 3 拍中值会把孤立尖峰完全吃掉（它前后两拍都快照原样输出），真实转动只多
 * 1 拍（10ms）延迟。想回到原行为便于 A/B 对比时，把它改成 0U 即可。 */
#define GYRO_ZUPT_JUDGE_MEDIAN3     1U
/* ---- ZUPT 判据滞回（GYRO_ZUPT_JUDGE_HYSTERESIS = 1U 时启用）---------------
 * 入场（未锁定 → 锁定）：连续 GYRO_ZUPT_CONFIRM_SAMPLES 拍满足
 *                         |judgeRateDps| <= GYRO_ZUPT_MAX_RATE_DPS
 *   judgeRateDps 由上面的 GYRO_ZUPT_JUDGE_MEDIAN3 决定（默认 3 拍中值）。
 * 出场（已锁定 → 未锁定）：两档判据，任一成立即解锁 ——
 *   tier-1：本拍 |correctedRateDps| > GYRO_ZUPT_EXIT_FAST_DPS      → 立刻解锁
 *   tier-2：连续 GYRO_ZUPT_EXIT_CONFIRM_SAMPLES 拍
 *           |correctedRateDps| > GYRO_ZUPT_MAX_RATE_DPS            → 解锁
 *
 * ★ 为什么两侧必须用不同信号（这是本判据的核心）：
 *   - 只用中值（MEDIAN3 = 1、无滞回）：抗尖峰，但"退出锁定"也晚 1 拍，
 *     高速段 200 °/s × 10 ms = 2° 的角度会被当成静止丢掉。
 *   - 只用原值（MEDIAN3 = 0、无滞回）：退出零延迟，但单拍 bit10 位错
 *     （1024 码 = 14.6 °/s）会打断 0.5 s 确认窗口，静止时平均 50 s 掉一次锁。
 *   - 滞回：入场抗尖峰（中值 + 50 拍连击），出场零延迟（原值 + 连击确认）。
 *     孤立单拍尖峰最多让 exitCounter 变成 1，下一拍就复位；
 *     真实转动连续超阈值，2 拍（10 ms）即解锁。
 * ★ 两侧共用 GYRO_ZUPT_MAX_RATE_DPS，所以慢转死区**没有变大**。
 * ★ tier-1 为什么取 80 °/s：实测单拍尖峰量级已到 ~41 °/s
 *   （gyroDbgJudgePeakHoldDps 实测 40.93），tier-1 必须明显高于入场门限，
 *   否则 40~80 这一档全被 tier-2 的两拍确认吃掉；>80 °/s 的转动一定是
 *   真实运动，没必要再等一拍（等一拍就是白丢 10 ms，高速下可达 2°）。
 * ★ 置 0U 回到"单一判据 + 立即清零"的原行为，便于 A/B 对比。 */
#define GYRO_ZUPT_JUDGE_HYSTERESIS      1U
#define GYRO_ZUPT_EXIT_FAST_DPS         80.0f
#define GYRO_ZUPT_EXIT_CONFIRM_SAMPLES  2U
/* ---- 静止锁定结束时的"起步坡道补偿" --------------------------------------
 * ★ 它解决的是"每次起步都要丢掉一段坡道"这个问题。
 *   锁定期间角度被冻结，可平台并不是瞬间停下的：从静止起转时，角速度从 0
 *   一路升到判据门限之前，全部落在锁定区间里、被整段丢掉。设加速度为 a，
 *     丢失角度 ≈ 门限² / (2a)。
 *   上位机默认参数（100 mm/s、500 mm/s²、D = 100 mm）换算成角运动是
 *     速度 ≈ 114.6 °/s，加速度 ≈ 573 °/s²；
 *   tier-1 在 80 °/s 解锁 → 丢 80²/(2×573) ≈ 5.58°，
 *   tier-2 在 ~40 °/s 解锁 → 丢 40²/(2×573) ≈ 1.40°。
 *   ★ 这两个数都是"没有坡道补偿时"的损失；补偿能全部收回（见下）。
 * ★ 注意方向：这个损失只出现在每段运动的**起点**（终点减速时 ZUPT 要连续
 *   50 拍不超阈值才锁定，减速坡道早就积进去了），所以它是**有方向**的。
 *   往返一次时，去程与回程各丢一次、方向相反，两者会部分抵消；于是最终
 *   误差取决于"远端有没有重新锁定"，而重锁与否只看远端停顿是否超过 0.5s。
 *   这就是"停久一点 / 停短一点结果不一样、178 和 179 差出 0.6°"的来源。
 *
 * 补偿办法：锁定期间把"本来该积进去"的增量（与积分分支用逐字相同的梯形公式）
 * 存进一个 0.25s 的环形缓冲；解锁的那一刻，把这 0.25s 的和补回角度。
 *   - 为什么是 0.25s：默认加速度 573 °/s² 下 80 °/s 的坡道 0.14s，
 *     0.25s 能完整覆盖"加速度 ≥ 320 °/s²"的全部坡道；加速度更低的
 *     极缓起动只能补回前 0.25s（补回的仍是同一公式的精确增量，
 *     只是不完整，不会引入新的标度误差）；
 *   - 为什么不是全部锁定区间：更早的部分是真正的静止，补回去等于把零偏当角度；
 *   - 为什么用同一公式：补回的量就是"当初不冻结时会积进去的量"，逐位可比，
 *     所以补完等于"这段时间从未冻结"，不会引入新的标度误差；
 *   - 门限 GYRO_ZUPT_RAMP_RECOVER_MIN_DEG 挡误解锁：静止时 0.25s 的冻结积分
 *     只有 残余零偏×0.25s ≈ 0.002°，远低于 0.02°，不会污染 0.00000°。
 *   - 效果：起步不再丢坡道，往返误差与"远端停多久"解耦。 */
#define GYRO_ZUPT_RAMP_RECOVER_ENABLE   GYRO_ZUPT_FREEZE_ENABLE
#define GYRO_ZUPT_RAMP_RECOVER_SAMPLES  25U     /* 10 ms × 25 = 0.25s 环形缓冲    */
#define GYRO_ZUPT_RAMP_RECOVER_MIN_DEG  0.02f   /* 小于此值判为噪声，不补         */

/* ---- 运动段零偏回溯修正（retro correct）----------------------------------
 * ★ 它解决的是"往返残差随行程变长而变大"这个问题：
 *     角度误差 ≈ 零偏估计误差 ε × 运动时长 T。
 *   T 从 90° 的 1.0s、180° 的 1.8s 一直到 360° 的 3.3s，所以同一个 ε 在
 *   360° 上的误差是 90° 的 3.3 倍。用户实测"转一圈停在一两度、回来有时
 *   加倍"，就是 ε 在去程/回程同号的表现（同号 ⇒ 相加 ⇒ 加倍）。
 *
 * ★ 为什么"提高校准精度"解决不了：校准只能估计**运动开始前**的零偏。
 *   陀螺零偏随温度/时间缓慢漂移，运动段内那 3.3s 里真实零偏是多少，
 *   段初的估计无从得知。于是"去程按旧零偏积完，回程才能用上新零偏"，
 *   同向残差必然留下。
 *
 * ★ 本算法把"已经积错的那一段"补回来，而且不需要任何采样缓冲（RAM 只有
 *   16.5KB/20KB，存 360° 行程的原始码要 2~3KB，放不下）：
 *     1) 运动开始（ZUPT 1→0）：记下段起点零偏 b0 和"段内实际积分时长"的
 *        累加器 t（只在真正做积分的拍上累加 elapsedMs，所以自动排除 ZUPT
 *        冻结拍和掉拍跨接，与实际积分严格同口径）。
 *     2) 运动结束（ZUPT 0→1）后进入静止：用静止样本重新估计零偏
 *        b1 = (b0 × W + Σ原始码/70) / (W + N)，W = GYRO_RETRO_BIAS_PRIOR_WEIGHT，
 *        N 最多 GYRO_RETRO_BIAS_MAX_SAMPLES 拍。这里用**原始未补偿**的
 *        measuredRateDps 累加，避免和偏置慢跟踪互相污染。
 *     3) 修正量 = −(b1 − b0) × t × (1 − RAMP/2)。RAMP = 0 表示"整段零偏恒为
 *        b1"（段内零偏稳定的场景最优）；RAMP = 1 表示"整段从 b0 线性过渡到
 *        b1"（段内均匀漂移的场景最优）；RAMP = 0.5 是两者折中。
 *     4) 修正量按 GYRO_RETRO_SLEW_DPS 限速平滑加到角度上。限速的原因：修正
 *        量直接阶跃会在静止曲线上留下一个 1~2° 的台阶，肉眼可见；限速后
 *        它摊成 0.3~1s 的平滑滑动，而静止时角度本来就在 ZUPT 锁定下不动，
 *        这点滑动不会被误读成真实运动。
 *     5) 累积到 GYRO_RETRO_CREEP_MAX_SAMPLES 拍后冻结，避免长时间静止时
 *        角度一直跟着零偏估计游走。
 *
 * ★ 与"每站重新校准（0x322=01）"互补而不重复：重校准只改 bias，影响的是
 *   **后续**积分；回溯修正改的是**已经积完**的那一段角度。两者修的是
 *   同一误差的前后两半，不会叠加两次。
 * ★ 置 0U 可整体关闭，角度退回"只靠段初零偏"的行为，便于 A/B 对比。 */
#define GYRO_RETRO_CORRECT_ENABLE       0U
#if (GYRO_ZUPT_FREEZE_ENABLE == 0U) && \
	((GYRO_BIAS_TRACK_ENABLE != 0U) || (GYRO_RETRO_CORRECT_ENABLE != 0U))
#error "Continuous measurement requires fixed bias and no retroactive correction"
#endif
#define GYRO_RETRO_BIAS_PRIOR_WEIGHT    10.0f  /* 段初零偏的等效样本数（先验权重）。
                                                * 段末静止 250 拍时 N/(W+N) ≈ 0.96，
                                                * 即"旧零偏只占 4% 权重"，段末估计基本
                                                * 说了算；又不会因为几十拍坏样本翻车。 */
#define GYRO_RETRO_BIAS_MAX_SAMPLES     500U   /* 静止块平均窗口上限（5 s）      */
#define GYRO_RETRO_BIAS_MAX_RATE_DPS    3.0f   /* 参与块平均的 |补偿后角速度| 上限，
                                                * 挡住减速尾巴和轻微振动          */
#define GYRO_RETRO_BIAS_RAMP            0.2f   /* 0=整段用 b1（段内零偏稳定时最优），
                                                * 1=整段 b0→b1 线性（段内均匀漂移时最优）。
                                                * 系数 = 1 − RAMP/2 = 0.9：往返同号残差
                                                * 是"段内零偏稳定"的特征，取小值一次修掉
                                                * 九成；留 0.1 的余量防过度修正。 */
#define GYRO_RETRO_SLEW_DPS             1.5f   /* 修正量平滑速率上限（°/s）       */
#define GYRO_RETRO_CREEP_MAX_SAMPLES    250U   /* 平滑修正最多跟 250 拍（2.5 s）  */
#define GYRO_RETRO_MIN_SEGMENT_MS       100U   /* 段内积分时长短于此值不修正      */
#define GYRO_RETRO_MIN_STATIC_SAMPLES   80U    /* 静止样本不足 0.8s 就放弃本次修正 */
#define GYRO_RETRO_MIN_CORRECTION_DEG   0.03f  /* 修正量小于此值直接丢掉（噪声级） */
#define GYRO_RETRO_MAX_CORRECTION_DEG   5.0f   /* 安全阀：修正量超过此值判为异常段
                                                * （多半是静止样本里混进了坏数据），
                                                * 整段放弃，宁可不修也不能把好角度
                                                * 一次推歪好几度。 */
#define GYRO_BIAS_TRACK_GAIN        0.0025f /* 外层：静止时偏置跟踪步长（≈4s 收敛时间常数）*/
/* 内层平均：先对"补偿后角速度"做一级 τ≈0.5s 的滑动平均，再喂给外层积分器。
 * ★ 为什么必须有它：单级积分器的零偏抖动 σ ≈ σ_noise × √(g/2)。实测静止噪声
 *   σ_noise ≈ 1.9 °/s，只用一个 g = 0.0025 会得到 σ_bias ≈ 0.067 °/s ——
 *   看起来就是"零偏一直在跳"（±0.2 峰峰），而且这 0.067 °/s 会直接注入角速度、
 *   在转动时随机游走进角度里（几秒就有 0.1~0.2°）。
 *   加一级内层平均后 σ_bias ≈ σ_noise × √(g_outer × g_inner / 2) ≈ 0.007 °/s，
 *   而收敛速度仍由外层 g 决定（τ≈4s），两者互不影响。 */
#define GYRO_BIAS_TRACK_AVG_GAIN    0.02f   /* 内层：τ ≈ 0.5s 的输入平滑          */
#define GYRO_DRIFT_WINDOW_MS        1000U   /* 角度漂移率统计窗口，便于 Watch 观察 */

/* ---- 【临时调参】10ms 原始数据统计与触发波形捕获 ---------------------------
 * 为什么需要它：ZUPT 判据用的是"零偏补偿后、滤波前"的 10ms 原始角速度，
 * 而上位机 0x323 收到的是 100ms 一次的滤波后值。上位机曲线看起来平，不代表
 * 10ms 判据没被尖峰打断；只盯着上位机曲线永远调不出正确的阈值。
 * 打开后全部变量都是 volatile 全局量，直接加进 VisualGDB 的 Watch / Live Watch：
 *   gyroDbgRatePeakDps      最近 1s 内 |补偿后角速度| 的峰值  <- 要的"10ms 峰值"
 *   gyroDbgRatePeakHoldDps  上电以来 |补偿后角速度| 的最大值  <- 抓偶发尖峰那一刻
 *   gyroDbgRateMeanDps      最近 1s 的平均值（看残余零偏方向/大小）
 *   gyroDbgRateAbsMeanDps   最近 1s 的 |值| 平均（看抖动强度）
 *   gyroDbgStepPeakDps      最近 1s 内相邻样本最大跳变（>24 多半是 SPI 数据损坏）
 *   gyroDbgStepPeakHoldDps  上电以来相邻样本最大跳变
 *   gyroDbgFilteredPeakDps  最近 1s 内 |滤波后角速度| 的峰值（和 raw 峰值对比）
 *   gyroDbgJudgePeakDps     最近 1s 内判据信号（中值后）的峰值 <- 定 ZUPT 阈值看这个
 *   gyroDbgJudgePeakHoldDps 上电以来判据信号的峰值
 *   gyroDbgJudgeFailCount   最近 1s 内判据信号 > GYRO_ZUPT_MAX_RATE_DPS 的样本数
 *   gyroDbgHoldDropCount    静止计时被打断的累计次数（真正的"掉出锁定"）
 *   gyroDbgRateDropCount    其中因角速度超阈值掉的次数
 *   gyroDbgStepDropCount    其中因相邻跳变超阈值掉的次数
 *   gyroDbgFastExitCount    滞回判据 tier-1 解锁次数（> GYRO_ZUPT_EXIT_FAST_DPS）
 *   gyroDbgSlowExitCount    滞回判据 tier-2 解锁次数（连击确认）
 *   gyroDbgRampRecoverCount 起步坡道补偿执行次数
 *   gyroDbgRampRecoverDegSum 起步坡道累计补回的角度（°）
 *   gyroDbgRateFailCount    最近 1s 内 |值| > GYRO_ZUPT_MAX_RATE_DPS 的样本数
 *   gyroDbgStepFailCount    最近 1s 内跳变 > GYRO_ZUPT_MAX_STEP_DPS 的样本数
 *   gyroDbgZuptLostDeg      ZUPT 冻结期间丢掉的角度（°，手动清零）
 *                           ← 判"慢转被吃掉多少"，和 accumulatedAngleDeg 同量纲
 *   gyroDbgHoldRunMax       上电以来 zuptCounter 达到过的最大值（< 50 永远进不了静止）
 *   gyroDbgHistOver[i]      |值| 超过 4/8/20/40/60/80 dps 的累计样本数（i = 0..5）
 *   gyroDbgHistSamples      累计参与统计的样本数（60s ≈ 6000）
 *   ↑ gyroDbgHistOver[i] / gyroDbgHistSamples 就是该阈值的"单样本误判率 p"，
 *     50 连击成功率 ≈ (1-p)^50：p = 1% 时只有 60%，p = 5% 时只剩 7.7%。
 *   gyroDbgCapture[]        触发波形：|值| 超过 gyroDbgCaptureTriggerDps 时冻结最近
 *                           128 个样本（1.28s），触发后不再写入；把
 *                           gyroDbgCaptureFrozen 写 0 可重新武装。
 * 所有 "*Hold"、hist 和 capture 都只在手动写 0 时清零。
 * ★ 调完参数务必把 GYRO_ZUPT_DEBUG_STATS_ENABLE 改回 0U 再发布。
 * ------------------------------------------------------------------------- */
#define GYRO_ZUPT_DEBUG_STATS_ENABLE 0U

#if (GYRO_ZUPT_DEBUG_STATS_ENABLE != 0U) && (GYRO_DRIFT_SUPPRESS_ENABLE != 0U)
#define GYRO_ZUPT_DEBUG_ACTIVE 1U
#else
#define GYRO_ZUPT_DEBUG_ACTIVE 0U
#endif

/* 课题六测试协议：经典 CAN、11 位标准数据帧、小端序。
 * 0x321 / DLC=8 测量帧：
 *   byte[0..3] 有符号 int32 角度，单位 0.00001 度（1e-5 度，raw = 角度 × 100000）
 *              上位机除以 100000 即得角度，例如 9000000 → 90.00000°，
 *              1234567 → 12.34567°。int32 上限对应 ±21474.83°
 *   byte[4..5] 有符号 int16 温度，单位 0.01 摄氏度（温度分辨率保持 0.01℃）
 *   byte[6]    bit0=角度/采样有效，bit1=温度有效，
 *              bit2=零偏校准完成，bit3=零偏校准失败，bit4=判定静止，
 *              bit5=校准降级，bit6=连续积分（即使 bit4=1 也不冻结角度）
 *   byte[7]    最近两次采样间隔，单位 ms，最大 255
 * 0x323 / DLC=4 角速度帧（与 0x321 同一次快照发出，两者严格同源）：
 *   byte[0..3] 有符号 int32 滤波后角速度，单位 0.00001 °/s（1e-5 °/s），小端
 *              上位机除以 100000 即得 °/s，例如 123400 → 1.234 °/s，
 *              1234567 → 12.34567 °/s。int32 上限对应 ±21474.83 °/s，
 *              而陀螺仪满量程只有 ±100 °/s，余量充足不会饱和
 *              ★ 送的是 gyroRateFilteredDps（零偏补偿 + 可选中值 + 低通之后的值），
 *                正是参与积分的那一路，便于和 0x321 的角度对照排查
 * 0x324 / DLC=8 原始采样帧（与 0x321/0x323 同一次快照发出，用于离线分析滤波器）：
 *   byte[0..1] 有符号 int16 陀螺仪原始 SPI 码（16 bit 补码，小端）
 *              按实测 70 LSB/(°/s) 换算，1 LSB ≈ 0.014286 °/s
 *   byte[2..5] 无符号 uint32 本次采样的实际时刻（HAL_GetTick()，ms，小端）
 *              ★ 这是"读完 SPI 那一刻"的时间戳，不是任务唤醒时刻，
 *                用它算 dt 才能反映真实采样间隔（含任务抖动与 SPI 耗时）
 *   byte[6..7] 有符号 int16 当前零偏，单位 0.001 °/s（raw = 零偏 × 1000）
 *              零偏包含校准均值和静止时的慢跟踪修正，是下方 corrected 的减数
 * 0x325 / DLC=8 滤波明细帧：
 *   byte[0..3] 有符号 int32 滤波后角速度，单位 0.00001 °/s（与 0x323 同值）
 *   byte[4..7] 无符号 uint32 采样时刻（ms），与 0x324 的 byte[2..5] 相同。
 *              ★ 两帧都带同一时间戳，上位机据此配对；缺帧时不会错配到上一拍。
 * ★ 0x324/0x325 只在产生新样本时发送；读取失败的周期不发，避免把上一拍的值
 *   重复记成新样本。正常 10ms 一拍，两帧共 16 字节，1 Mbps 下占用可忽略。
 * 上电后先等待 1s，再要求传感器保持静止并采集 200 个样本；校准检查峰峰波动
 * 不超过 2°/s 且平均偏置不超过 10°/s。
 * ★ 校准判据分三档，避免一次震动把角度永久锁死（旧版失败后停在 state 3 不再重试，
 *   桌面轻微震动就会让上位机一直显示 --，必须重新上电）：
 *     1) 峰峰值 ≤ 2°/s          -> 正常校准；
 *     2) 峰峰值超标但均值 ≤ 10°/s -> 降级校准（bit5），零偏取均值，角度照常有效；
 *     3) 均值也超限             -> 本轮失败，2s 后自动重试（gyroCalibrationRetryCount 计数）。
 * 0x322 / DLC=1 命令帧：0x01=重新零偏校准，0x02=角度清零。
 * 命令由 task_can_rx 置标志，实际状态只由 task_gyro 修改。
 * ★ 0x321/0x323/0x324/0x325 每 10ms 连发一次，共用同一次临界区快照，发送结果分别判断。
 *   0x324/0x325 只在产生新样本时发送（见 gyroSampleSequence 的说明）。
 * ★ 角度比例是板端与上位机的硬约定，必须成对修改，否则角度整体差整数倍且不报错：
 *     板端 CAN_GYRO_ANGLE_SCALE  <-->  上位机 GyroProtocol.AngleLsbPerDegree
 *   2026-10-16：比例由 100（0.01°）提高到 100000（0.00001°），对应上位机 5 位小数显示。
 *   注意 gyroAccumulatedAngleDeg 是 float（约 7 位有效十进制位），角度越大 ULP 越大：
 *   ±180° 内 ULP ≤ 1.5e-5°（约 1.5 个 LSB）末位可信，累计到几千度后末位会抖动，
 *   这是浮点累加的固有特性，不是协议或上位机的问题。
 * ★ 角速度比例同理，改一处必须同步改另一处，否则角速度整体差整数倍：
 *     板端 CAN_GYRO_RATE_SCALE  <-->  上位机 GyroProtocol.RateLsbPerDps
 *   2026-10-16：比例由 1000（0.001 °/s）提高到 100000（0.00001 °/s），与角度对齐 5 位小数。
 *   注意角速度的真实精度下限在传感器：XV7001BB 实测灵敏度 70 LSB/(°/s)，
 *   即原始码 1 个 LSB ≈ 0.0143 °/s，比 0.00001 °/s 的协议分辨率粗 1429 倍。
 *   所以 5 位小数的后几位反映的是零偏补偿量与滤波输出（连续量），
 *   不是传感器的独立测量分辨率——看绝对值请以 0.001 °/s 量级为准。 */
#define CAN_GYRO_DATA_STD_ID        0x321U
#define CAN_GYRO_RATE_STD_ID        0x323U
#define CAN_GYRO_COMMAND_STD_ID     0x322U
#define CAN_GYRO_RAW_STD_ID         0x324U
#define CAN_GYRO_RAWDETAIL_STD_ID   0x325U
#define CAN_GYRO_CMD_RECALIBRATE    0x01U
#define CAN_GYRO_CMD_ZERO_ANGLE     0x02U
#define CAN_GYRO_DATA_DLC           8U
#define CAN_GYRO_RATE_DLC           4U
#define CAN_GYRO_RAW_DLC            8U
#define CAN_GYRO_RAWDETAIL_DLC      8U
#define CAN_GYRO_ANGLE_SCALE        100000.0f /* 角度帧比例：0.00001° 每 LSB（1e-5 度） */
#define CAN_GYRO_RATE_SCALE         100000.0f /* 角速度帧比例：0.00001 °/s 每 LSB（1e-5 °/s） */
#define CAN_GYRO_BIAS_SCALE         1000.0f   /* 零偏帧比例：0.001 °/s 每 LSB（int16 饱和） */

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
volatile int16_t  gyroRateRaw = 0;volatile float    gyroTempCelsius = 0.0f;
volatile float    gyroRateDps = 0.0f;
volatile uint32_t gyroIntegrationGapCount = 0U; /* >50 ms 的不可恢复积分空洞 */
volatile uint8_t gyroAngleIntegrityLost = 0U; /* 锁存到显式建立新角度原点 */
volatile uint32_t gyroRejectedCalibrationCount = 0U;
volatile uint32_t gyroReadCount = 0U;
volatile uint32_t gyroErrCount = 0U;
/* |角速度| 超过量程 (±100 °/s) 的样本数。这些样本仍然参与积分（丢时间的代价更大），
 * 只在这里计数，用来判断"快转时是否打满量程"。正常测量它应该很小。 */
volatile uint32_t gyroSaturatedCount = 0U;
/* 课题六采样与校准监视量：校准状态 0=稳定等待、1=采样、2=完成、3=失败（会自动重试）。 */
volatile float gyroBiasDps = 0.0f;volatile float gyroRateCorrectedDps = 0.0f;
volatile float gyroRateFilteredDps = 0.0f;
volatile uint8_t gyroIntegrationMedianEnabled = GYRO_INTEGRATION_MEDIAN3_ENABLE;
volatile uint8_t gyroAngleFreezeEnabled = GYRO_ZUPT_FREEZE_ENABLE;
volatile float gyroAccumulatedAngleDeg = 0.0f;volatile float gyroCalibrationRangeDps = 0.0f;
volatile uint8_t gyroCalibrationState = 0U;
volatile uint8_t gyroDataValid = 0U;
volatile uint8_t gyroTempValid = 0U;
volatile uint32_t gyroCalibrationSampleCount = 0U;
volatile uint32_t gyroCalibrationFailCount = 0U;
volatile uint32_t gyroCalibrationRetryCount = 0U;
volatile uint32_t gyroCalibrationValidationSamples = 0U;
volatile float gyroCalibrationValidationResidualDps = 0.0f;
volatile float gyroCalibrationValidationPeakDps = 0.0f;
volatile uint8_t gyroCalibrationValidationPassed = 0U;
/* 峰峰值、采样完整性或独立验证不合格时标记降级；bit5 对外报告。
 * 均值仍可用于显示，但固定零偏不会自行收敛，不能判精度合格。 */
volatile uint8_t gyroCalibrationDegraded = 0U;
volatile uint32_t gyroCalibrationDegradedCount = 0U;
volatile uint32_t gyroSampleIntervalMs = 0U;
volatile uint32_t gyroOverrunCount = 0U;
/* 离线分析用的采样快照（0x324/0x325 两帧的数据源）：
 *   gyroRawCode       本次采样的陀螺仪原始 SPI 码（16 bit 补码）
 *   gyroSampleTickMs  读完 SPI 那一刻的 HAL tick（ms），即"实际采样时间戳"
 *   gyroSampleSequence 每成功读到一个新样本 +1；task_can_tx 靠它判断
 *                      "这一拍有没有新数据"，读取失败时不上报上一拍的旧值
 * 三者与 gyroBiasDps / gyroRateFilteredDps 在同一个临界区快照里发布，
 * 保证一帧记录里的原始码、零偏、滤波值属于同一拍。 */
volatile int16_t  gyroRawCode = 0;
volatile uint32_t gyroSampleTickMs = 0U;
volatile uint32_t gyroSampleSequence = 0U;
/* 课题六 CAN 发送计数：Ok/Err 是 0x321/0x323/0x324/0x325 四帧的合计，
 * RateOk/RateErr 只统计 0x323 角速度帧，RawOk/RawErr 只统计 0x324/0x325
 * 原始采样帧。分开计数就能看出是哪一帧发送失败。 */
volatile uint32_t gyroCanTxOkCount = 0U;
volatile uint32_t gyroCanTxErrCount = 0U;
volatile uint32_t gyroCanTxRateOkCount = 0U;
volatile uint32_t gyroCanTxRateErrCount = 0U;
volatile uint32_t gyroCanTxRawOkCount = 0U;
volatile uint32_t gyroCanTxRawErrCount = 0U;
volatile uint32_t gyroCanRawPairDropCount = 0U;
volatile uint32_t gyroCanSkippedSampleCount = 0U;
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

/* 运动段零漂回溯修正的监视量（GYRO_RETRO_CORRECT_ENABLE 关闭时保持 0）：
 *   gyroRetroEstimateDps      = 最近一段运动结束后，用静止样本重新估出的零偏
 *   gyroRetroLastCorrectionDeg= 最近一次的段角度修正量（正 = 角度被加回来）
 *   gyroRetroCorrectionDeg    = 上电以来累计补进角度的量
 *   gyroRetroSettleCount      = 已完成结算的段数
 * 若 LastCorrection 长期在 ±0.05° 内、SettleCount 在涨，说明零偏稳定、无需修正；
 * 若 |LastCorrection| 持续 0.5° 以上，说明段初零偏确实偏了，修正在起作用。 */
volatile float    gyroRetroEstimateDps = 0.0f;
volatile float    gyroRetroLastCorrectionDeg = 0.0f;
volatile float    gyroRetroCorrectionDeg = 0.0f;
volatile uint32_t gyroRetroSettleCount = 0U;
volatile uint32_t gyroRetroRejectCount = 0U;   /* 被 GYRO_RETRO_MAX_CORRECTION_DEG 挡下的段数 */

#if (GYRO_ZUPT_DEBUG_ACTIVE != 0U)
/* 调参统计的滚动窗口（每秒刷新一次，Watch 里读到的就是"最近一秒"） */
volatile float    gyroDbgRatePeakDps = 0.0f;
volatile float    gyroDbgRateMeanDps = 0.0f;
volatile float    gyroDbgRateAbsMeanDps = 0.0f;
volatile float    gyroDbgStepPeakDps = 0.0f;
volatile float    gyroDbgFilteredPeakDps = 0.0f;
volatile uint32_t gyroDbgWindowSamples = 0U;
volatile uint32_t gyroDbgRateFailCount = 0U;
volatile uint32_t gyroDbgStepFailCount = 0U;
/* 上电以来的峰值与最长连续静止样本数（手动写 0 清零） */
volatile float    gyroDbgRatePeakHoldDps = 0.0f;
volatile float    gyroDbgStepPeakHoldDps = 0.0f;
volatile float    gyroDbgFilteredPeakHoldDps = 0.0f;
volatile uint32_t gyroDbgHoldRunMax = 0U;
/* 静止计时被打断的次数（zuptCounter 从非 0 掉回 0，每次都记一次）。
 * 一个是"一分钟掉几次"，另外两个分开记是谁掉的：角速度超阈值 / 相邻跳变。 */
volatile uint32_t gyroDbgHoldDropCount = 0U;
volatile uint32_t gyroDbgRateDropCount = 0U;
volatile uint32_t gyroDbgStepDropCount = 0U;
/* 滞回判据的两档出场计数（只在 GYRO_ZUPT_JUDGE_HYSTERESIS = 1 时有意义）：
 *   gyroDbgFastExitCount  tier-1 单拍解锁次数（> 80 °/s，真实高速转动）
 *   gyroDbgSlowExitCount  tier-2 连击解锁次数（40~80 °/s，慢转或持续噪声）
 * 静止台上跑一分钟，这两个数都应该是 0；只有真正转动时才会增长。 */
volatile uint32_t gyroDbgFastExitCount = 0U;
volatile uint32_t gyroDbgSlowExitCount = 0U;
/* 起步坡道补偿的生效情况：
 *   gyroDbgRampRecoverCount   补偿执行次数（每次解锁最多 1 次）
 *   gyroDbgRampRecoverDegSum  累计补回的角度（°，手动写 0 清零）
 * 一轮 90° 往返正常应看到 Count 增加 1~2、DegSum 增加 0.3~2.6（取决于加速度）。
 * 若 Count 从不增长，说明解锁时坡道已整段落在 0.25s 缓冲之外。 */
volatile uint32_t gyroDbgRampRecoverCount = 0U;
volatile float    gyroDbgRampRecoverDegSum = 0.0f;
/* 判定信号本身（开了中值就是中值，没开就是本拍原值）的统计：
 * 调 GYRO_ZUPT_MAX_RATE_DPS 要看这一组，而不是看 raw 峰值。 */
volatile float    gyroDbgJudgePeakDps = 0.0f;
volatile float    gyroDbgJudgePeakHoldDps = 0.0f;
volatile uint32_t gyroDbgJudgeFailCount = 0U;
/* ZUPT 冻结期间"丢掉"的角度（°，手动写 0 清零）。
 * 累加方式和积分分支完全相同（同样的梯形公式），所以：
 *   accumulatedAngleDeg + gyroDbgZuptLostDeg = 关掉 ZUPT 时的纯积分角度。
 * 判读：静止时它只有"残余零偏×时间"，很小；慢速转动（< ZUPT 阈值）时它就是
 * 被吃掉的那部分角度。整圈 360° 少读 1.3°，这里就应该读到 ≈ -1.3。 */
volatile float    gyroDbgZuptLostDeg = 0.0f;
/* 累计分布：|补偿后角速度| 超过 4/8/20/40/60/80 °/s 的样本数（手动写 0 清零） */
#define GYRO_DBG_HIST_BINS 6U
volatile uint32_t gyroDbgHistOver[GYRO_DBG_HIST_BINS] = {0U, 0U, 0U, 0U, 0U, 0U};
volatile uint32_t gyroDbgHistSamples = 0U;
/* 触发波形捕获：循环保存最近 128 个补偿后角速度（°/s，10ms 一个点）。
 * gyroDbgCapture 是环形缓冲（读起来要绕圈），触发时把它一次性按时间顺序
 * 拷进 gyroDbgCaptureOrdered：Ordered[0] 最旧，Ordered[127] 最新（就是触发点），
 * 直接顺序看/顺序复制即可，不用自己算回绕。
 * 触发点下标 = gyroDbgCaptureTriggerIndex（定长 128，与触发位置无关）。 */
#define GYRO_DBG_CAPTURE_LEN 128U
volatile float    gyroDbgCapture[GYRO_DBG_CAPTURE_LEN];
volatile float    gyroDbgCaptureOrdered[GYRO_DBG_CAPTURE_LEN];
volatile uint16_t gyroDbgCaptureIndex = 0U;
volatile uint8_t  gyroDbgCaptureOrderedValid = 0U;
volatile uint8_t  gyroDbgCaptureFrozen = 0U;
volatile float    gyroDbgCaptureTriggerDps = 32.0f;
#endif

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
 * 积分默认用偏置补偿后的原值经过约 5Hz 线性低通，避免中值改变积分面积。
 * 校准和静止判定仍独立使用中值；旧积分中值链可通过宏启用做 A/B。
 * 每成功读到一个样本就记下原始码与"读完 SPI 那一刻"的 tick，并通过
 * gyroSampleSequence +1 通知 task_can_tx 发送 0x324/0x325（离线分析用）。
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
	double calibrationSum = 0.0;
	double calibrationValidationSum = 0.0;
	double calibrationValidationBlockSum = 0.0;
	float calibrationValidationPeak = 0.0f;
	float calibrationValidationResidual = 0.0f;
	uint8_t calibrationDataFault = 0U;
	uint8_t calibrationValidationPassed = 0U;
	float calibrationMin = 0.0f;
	float calibrationMax = 0.0f;
	float calibrationMean = 0.0f;
	float biasDps = 0.0f;
	float correctedRateDps = 0.0f;
	float filteredRateDps = 0.0f;
	float previousFilteredRateDps = 0.0f;
	float integrationInputDps;
	float filterAlpha;
	float filterSamples[3] = {0.0f, 0.0f, 0.0f};
	float accumulatedAngleDeg = 0.0f;
	float measuredRateDps = 0.0f;
	float tempCelsius = 0.0f;
	float dtSeconds;
	int16_t rawCode = 0;                  /* 本拍陀螺仪原始 SPI 码                  */
	uint32_t sampleTickMs = 0U;           /* 读完 SPI 那一刻的 HAL tick（ms）        */
	uint32_t sampleSequence = 0U;         /* 成功样本序号，供 CAN 帧判断新旧         */
	uint32_t calibrationCount = 0U;
	float calibrationFilterSamples[3] = {0.0f, 0.0f, 0.0f};
	uint8_t calibrationFilterIndex = 0U;
	float calibrationSample = 0.0f;        /* 3 拍中值后的校准输入（抗孤立尖峰）      */
	uint32_t calibrationFailMs = 0U;      /* 本轮校准判失败的时刻，用于自动重试计时  */
	uint8_t calibrationDegraded = 0U;     /* 1 = 本轮零偏是降级均值法得到的          */
	uint8_t stableEnough;
	uint8_t meanUsable;
	uint8_t havePreviousRate = 0U;
	uint8_t dataValid;
	uint8_t tempValid = 0U;
	uint8_t calibrationState = 0U; /* 0=稳定等待、1=采样、2=完成、3=失败（自动重试） */
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
	float biasTrackBaseDps = 0.0f;         /* 校准结束时的零偏，用于统计跟踪增量      */
	float biasTrackAvgDps = 0.0f;           /* 内层平滑后的跟踪输入（减小零偏抖动）    */
	uint8_t biasTrackPrimed = 0U;           /* 内层平滑是否已用当前残余初始化          */
	float judgeRateDps = 0.0f;             /* 本拍实际参与判据的信号（可能是中值）    */
	float previousJudgeRateDps = 0.0f;     /* 上一拍参与判据的信号，用于跳变判据      */
	uint8_t havePreviousJudge = 0U;        /* 是否已有上一拍参与判据的信号            */
#if (GYRO_ZUPT_JUDGE_HYSTERESIS != 0U)
	uint8_t zuptExitCounter = 0U;          /* 锁定时连续超阈值的拍数（出场确认）      */
#endif
#if ((GYRO_ZUPT_RAMP_RECOVER_ENABLE != 0U) || (GYRO_RETRO_CORRECT_ENABLE != 0U))
	uint8_t zuptWasActive = 0U;            /* 上一拍的锁定状态，用来抓 1→0 的跳变      */
#endif
#if (GYRO_ZUPT_RAMP_RECOVER_ENABLE != 0U)
	/* 起步坡道补偿缓冲：锁定期间"本来会积进去"的角度增量，解锁时整体补回。
	 * 用 static 放在 .bss 而不是任务栈上：这是 100 字节，任务栈只有 1KB，
	 * 而它只在 task_gyro 里用（单任务访问，不需要额外保护）。 */
	static float rampRecoverBuf[GYRO_ZUPT_RAMP_RECOVER_SAMPLES];
	uint8_t rampRecoverIndex = 0U;
	uint8_t rampRecoverCount = 0U;
#endif
#if (GYRO_ZUPT_JUDGE_MEDIAN3 != 0U)
	float judgeSamples[3] = {0.0f, 0.0f, 0.0f};
	uint8_t judgeIndex = 0U;
	uint8_t judgeFilled = 0U;              /* 已攒了几拍，不足 3 拍先按原值走         */
#endif
#if (GYRO_RETRO_CORRECT_ENABLE != 0U)
	/* 运动段状态：段起点零偏 + 段内实际积分时长 */
	uint8_t retroSegActive = 0U;           /* 1 = 正在一段运动里                      */
	uint32_t retroSegDtMs = 0U;            /* 段内真正做积分的累计时长（ms）          */
	float retroSegStartBiasDps = 0.0f;     /* 段起点用的零偏                          */
	/* 段结束后的静止块平均：用原始未补偿角速度，避免和偏置慢跟踪互相污染 */
	float retroStaticSumDps = 0.0f;
	uint32_t retroStaticCount = 0U;
	uint8_t retroPending = 0U;             /* 1 = 段已结束，等静止样本攒够再算修正    */
	/* 修正量的平滑应用 */
	uint8_t retroApplyActive = 0U;
	uint32_t retroApplySamples = 0U;
	float retroApplyRemainDeg = 0.0f;      /* 还没补完的角度（°/s 限速地靠近 0）      */
	float retroApplyTotalDeg = 0.0f;       /* 已累计补进去的角度，仅用于 Watch        */
#endif
#endif
#if (GYRO_ZUPT_DEBUG_ACTIVE != 0U)
	uint32_t dbgWindowStartMs = 0U;
	uint32_t dbgWindowSamples = 0U;
	uint32_t dbgRateFail = 0U;
	uint32_t dbgStepFail = 0U;
	uint32_t dbgJudgeFail = 0U;
	float dbgRateSum = 0.0f;
	float dbgRateAbsSum = 0.0f;
	float dbgRatePeak = 0.0f;
	float dbgStepPeak = 0.0f;
	float dbgFilteredPeak = 0.0f;
	float dbgJudgePeak = 0.0f;
	float dbgPreviousRaw = 0.0f;           /* 上一拍未中值的原始值，用于跳变统计      */
	uint8_t dbgHavePreviousRaw = 0U;
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
			gyroAngleIntegrityLost = 0U;
			havePreviousRate = 0U;
			/* 角度清零后漂移统计窗口一起重来，避免把清零前的变化算进去 */
			driftWindowStartMs = 0U;
			angleDriftDps = 0.0f;
#if (GYRO_RETRO_CORRECT_ENABLE != 0U)
			/* 清零等于"把此刻定义为 0°"。上一段的回溯修正量如果还没滑完，
			 * 它的基准（零偏假设）属于被清零掉的那段历史，继续补会把
			 * "别的段的误差"加到新基准上，必须一起作废。 */
			retroSegActive = 0U;
			retroSegDtMs = 0U;
			retroPending = 0U;
			retroStaticSumDps = 0.0f;
			retroStaticCount = 0U;
			retroApplyActive = 0U;
			retroApplyRemainDeg = 0.0f;
			retroApplySamples = 0U;
#endif
#if (GYRO_DRIFT_SUPPRESS_ENABLE != 0U)
			/* ★ 清零 = “把此刻的物理位置定义为 0°”，必须同时把静止判据补满。
			 *   否则清零前刚转动过（计数器为 0），之后 0.5s 的静止确认窗口里
			 *   残余零偏会继续被积分，完成确认时角度已经停在 0.00x° 并被永久
			 *   冻结：实测残余零偏只有 1 个传感器 LSB（1/70 ≈ 0.0143 °/s）
			 *   就会留下 0.0069°，上位机 5 位小数把它显示得一清二楚。
			 *   补满后角度精确保持 0.00000°；一旦真的转动（角速度超阈值或
			 *   相邻样本跳变），下面的判据会立刻把计数器清零，积分照常恢复
			 *   （最多晚 1 个采样周期 ≈ 10ms，代价远小于留下假角度）。 */
			zuptCounter = GYRO_ZUPT_CONFIRM_SAMPLES;
			zuptActive = 1U;
			havePreviousJudge = 0U;
#if (GYRO_ZUPT_JUDGE_HYSTERESIS != 0U)
			zuptExitCounter = 0U;
#endif
#if (GYRO_ZUPT_RAMP_RECOVER_ENABLE != 0U)
			/* 刚归零/刚校准，缓冲里的旧增量没有意义，必须丢弃 */
			rampRecoverCount = 0U;
			rampRecoverIndex = 0U;
#endif
#if (GYRO_ZUPT_JUDGE_MEDIAN3 != 0U)
			judgeFilled = 0U;
#endif
			biasTrackBaseDps = biasDps;
			biasTrackPrimed = 0U;
#endif
		}

#if (GYRO_DRIFT_SUPPRESS_ENABLE != 0U)
		/* 已在运动时拒绝重校准，避免整段运动在校准窗口内被漏积。 */
		if ((requestRecalibration != 0U) && (calibrationState == 2U) && (zuptActive == 0U))
		{
			requestRecalibration = 0U;
			gyroRejectedCalibrationCount++;
		}
#endif
		if (requestRecalibration != 0U)
		{
			calibrationCount = 0U;
			calibrationSum = 0.0f;
			calibrationValidationSum = 0.0;
			calibrationValidationBlockSum = 0.0;
			calibrationValidationPeak = 0.0f;
			calibrationValidationResidual = 0.0f;
			calibrationValidationPassed = 0U;
			calibrationDataFault = 0U;
			calibrationMin = 0.0f;
			calibrationMax = 0.0f;
			calibrationState = 1U;
			calibrationDegraded = 0U;
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
		if (XV7001bb_ReadAngle(&rawCode) == HAL_OK)
		{
			measuredRateDps = XV7001bb_AngleToDps(rawCode);
			/* 实际采样时刻取"读完 SPI 那一刻"：sampleStartMs 是任务唤醒时刻，
			 * 与真正拿到数据相差一个 SPI 事务。0x324/0x325 记录这个值，
			 * 离线用相邻样本的差算 dt，才能看到任务抖动和读取耗时。 */
			sampleTickMs = HAL_GetTick();

			/* 只挡物理上不可能的读数（见 XV7001BB_VALID_RATE_DPS 的说明）。
			 * 量程内饱和照样算有效：丢一拍会连带丢掉下一拍的 dt，代价是 2° 级的。 */
			dataValid = ((measuredRateDps <= XV7001BB_VALID_RATE_DPS) &&
						 (measuredRateDps >= -XV7001BB_VALID_RATE_DPS)) ? 1U : 0U;
			if ((measuredRateDps > XV7001BB_RANGE_DPS) ||
				(measuredRateDps < -XV7001BB_RANGE_DPS))
			{
				gyroSaturatedCount++;
			}
		}
		else
		{
			dataValid = 0U;
		}
		if (dataValid != 0U)
		{
			if (lastSuccessfulReadMs != 0U)
			{
				sampleIntervalMs = sampleTickMs - lastSuccessfulReadMs;
			}
			lastSuccessfulReadMs = sampleTickMs;
			gyroReadCount++;
			sampleSequence++;
			/* --- 掉拍跨接的空洞上限 ----------------------------------------
			 * 读失败不再销毁积分状态（见下面的 else 分支），所以恢复后的那一拍
			 * 可能面对一个很大的 elapsedMs。空洞超过 GYRO_MAX_BRIDGE_MS 说明采样
			 * 已经不可信，这时才把积分/ZUPT/滤波状态整体作废重来，避免用一个
			 * 陈旧角速度乘一个很大的 dt。 */
			if ((havePreviousRate != 0U) &&
				((sampleTickMs - previousValidMs) > GYRO_MAX_BRIDGE_MS))
			{
				gyroIntegrationGapCount++;
				gyroAngleIntegrityLost = 1U;
				havePreviousRate = 0U;
				filterPrimed = 0U;
				driftWindowStartMs = 0U;
#if (GYRO_DRIFT_SUPPRESS_ENABLE != 0U)
				zuptCounter = 0U;
				zuptActive = 0U;
				havePreviousJudge = 0U;
				biasTrackPrimed = 0U;
#if (GYRO_ZUPT_JUDGE_HYSTERESIS != 0U)
				zuptExitCounter = 0U;
#endif
#if (GYRO_ZUPT_RAMP_RECOVER_ENABLE != 0U)
				/* 空洞跨越了无效区间，缓冲里的增量没有意义 */
				rampRecoverCount = 0U;
				rampRecoverIndex = 0U;
#endif
#if (GYRO_ZUPT_JUDGE_MEDIAN3 != 0U)
				judgeFilled = 0U;
#endif
#endif
			}
			correctedRateDps = measuredRateDps - biasDps;

			if (calibrationState == 1U)
			{
				if ((calibrationCount > 0U) && (sampleIntervalMs > GYRO_MAX_BRIDGE_MS))
				{
					calibrationDataFault = 1U;
				}
				/* --- 3 拍中值前置滤波 ----------------------------------------
				 * 零偏采用静止窗口算术平均，对孤立尖峰毫无抵抗力：实测出现过单拍
				 * 2865 码 ≈ 40.9 °/s 的 SPI 损坏样本，它一个人就把均值拉偏
				 * 40.9/200 ≈ 0.20 °/s —— 折成角度就是每 2s 行程少算 0.4°，而且
				 * 还会把峰峰值判据顶穿，让一次干净的校准被误判成"降级"。
				 * 中值滤波对孤立尖峰完全免疫，代价只有 2 拍（20ms）延迟，
				 * 相对 20s 的校准窗口很短。 */
				if (calibrationCount == 0U)
				{
					calibrationFilterSamples[0] = measuredRateDps;
					calibrationFilterSamples[1] = measuredRateDps;
					calibrationFilterSamples[2] = measuredRateDps;
					calibrationFilterIndex = 0U;
				}
				else
				{
					calibrationFilterSamples[calibrationFilterIndex] = measuredRateDps;
					calibrationFilterIndex = (uint8_t)
						((calibrationFilterIndex + 1U) % 3U);
				}
				calibrationSample = GyroMedianOfThree(
					calibrationFilterSamples[0],
					calibrationFilterSamples[1],
					calibrationFilterSamples[2]);

				if (calibrationCount == 0U)
				{
					calibrationMin = calibrationSample;
					calibrationMax = calibrationSample;
				}
				else
				{
					if (calibrationSample < calibrationMin)
					{
						calibrationMin = calibrationSample;
					}
					if (calibrationSample > calibrationMax)
					{
						calibrationMax = calibrationSample;
					}
				}
				if (calibrationCount < GYRO_CALIBRATION_SAMPLES)
				{
					calibrationSum += (double)calibrationSample;
				}
				else
				{
					/* 验证使用线性积分实际接收的原始信号，而不是中值。 */
					calibrationValidationSum += (double)measuredRateDps;
					calibrationValidationBlockSum += (double)measuredRateDps;
				}
				calibrationCount++;

				if (calibrationCount == GYRO_CALIBRATION_SAMPLES)
				{
					calibrationMean = (float)(calibrationSum / GYRO_CALIBRATION_SAMPLES);
				}
				if (calibrationCount > GYRO_CALIBRATION_SAMPLES)
				{
					uint32_t validationCount = calibrationCount - GYRO_CALIBRATION_SAMPLES;
					calibrationValidationResidual =
						(float)(calibrationValidationSum / validationCount) - calibrationMean;
					if ((validationCount % GYRO_CALIBRATION_VALIDATION_BLOCK_SAMPLES) == 0U)
					{
						float blockResidual =
							(float)(calibrationValidationBlockSum /
								GYRO_CALIBRATION_VALIDATION_BLOCK_SAMPLES) - calibrationMean;
						float blockAbs = (blockResidual < 0.0f) ? -blockResidual : blockResidual;
						if (blockAbs > calibrationValidationPeak)
						{
							calibrationValidationPeak = blockAbs;
						}
						calibrationValidationBlockSum = 0.0;
					}
				}
				if (calibrationCount >=
					(GYRO_CALIBRATION_SAMPLES + GYRO_CALIBRATION_VALIDATION_SAMPLES))
				{
					stableEnough =
						(((calibrationMax - calibrationMin) <= GYRO_CALIBRATION_MAX_RANGE_DPS) &&
						 (calibrationDataFault == 0U)) ? 1U : 0U;
					meanUsable =
						((calibrationMean <= GYRO_CALIBRATION_MAX_BIAS_DPS) &&
						 (calibrationMean >= -GYRO_CALIBRATION_MAX_BIAS_DPS)) ? 1U : 0U;
					calibrationValidationPassed =
						((stableEnough != 0U) && (meanUsable != 0U) &&
						 (calibrationValidationPeak <= GYRO_CALIBRATION_MAX_RESIDUAL_DPS) &&
						 (calibrationValidationResidual <= GYRO_CALIBRATION_MAX_RESIDUAL_DPS) &&
						 (calibrationValidationResidual >= -GYRO_CALIBRATION_MAX_RESIDUAL_DPS)) ? 1U : 0U;

					/* 判据分三档，避免“一次抖动就把角度永久判死”：
					 *   1) 完整性、峰峰值和独立验证达标 -> 正常校准；
					 *   2) 质量不足但均值可用 -> 降级校准，bit5 标记，不判精度合格；
					 *   3) 均值也不可用（板子明显在动）-> 本轮失败，等待 RETRY_DELAY 自动重试。 */
					if (calibrationValidationPassed != 0U)
					{
						calibrationDegraded = 0U;
					}
					else if (meanUsable != 0U)
					{
						calibrationDegraded = 1U;
						gyroCalibrationDegradedCount++;
					}
					else
					{
						calibrationDegraded = 0U;
						calibrationState = 3U;
						calibrationFailMs = sampleStartMs;
						gyroCalibrationFailCount++;
					}

					if (calibrationState != 3U)
					{
						biasDps = calibrationMean;
						calibrationState = 2U;
						/* 校准只更新零偏；保留累计角度和既有积分质量状态。 */
						correctedRateDps = measuredRateDps - biasDps;
						filterSamples[0] = correctedRateDps;
						filterSamples[1] = correctedRateDps;
						filterSamples[2] = correctedRateDps;
						filteredRateDps = correctedRateDps;
						filterIndex = 0U;
						filterPrimed = 1U;
						/* 校准刚结束：角度保持，漂移统计窗口重新开始 */
						driftWindowStartMs = 0U;
						angleDriftDps = 0.0f;
#if (GYRO_DRIFT_SUPPRESS_ENABLE != 0U)
						/* 以刚校准好的零偏作为跟踪统计基准；校准要求静止，
						 * 直接按“已确认静止”起步，避免残余零偏在 0.5s 确认窗口里
						 * 被积分成 0.00x°（校准本来就要求板子静止）。 */
						biasTrackBaseDps = biasDps;
						biasTrackUpdates = 0U;
						zuptCounter = GYRO_ZUPT_CONFIRM_SAMPLES;
						zuptActive = 1U;
						havePreviousJudge = 0U;
#if (GYRO_ZUPT_JUDGE_HYSTERESIS != 0U)
						zuptExitCounter = 0U;
#endif
#if (GYRO_ZUPT_RAMP_RECOVER_ENABLE != 0U)
						rampRecoverCount = 0U;
						rampRecoverIndex = 0U;
#endif
#if (GYRO_ZUPT_JUDGE_MEDIAN3 != 0U)
						judgeFilled = 0U;
#endif
						biasTrackPrimed = 0U;
#endif
					}
					havePreviousRate = 0U;
				}
			}
			else if (calibrationState == 2U)
			{
				elapsedMs = (havePreviousRate != 0U) ?
					(sampleTickMs - previousValidMs) : 0U;

#if (GYRO_DRIFT_SUPPRESS_ENABLE != 0U)
#if (GYRO_ZUPT_DEBUG_ACTIVE != 0U)
				/* --- 调参统计：只读不改，不参与任何判定 --------------------- */
				{
					/* 分档阶梯：第 4 档（index 3）正好是 GYRO_ZUPT_MAX_RATE_DPS = 40，
					 * 第 6 档（index 5）正好是 GYRO_ZUPT_EXIT_FAST_DPS = 80，
					 * 所以 gyroDbgHistOver[3] / gyroDbgHistSamples 就是入场门限的
					 * "单拍超阈值率"，gyroDbgHistOver[5] 监视 tier-1 的余量。 */
					static const float histThresholds[GYRO_DBG_HIST_BINS] =
						{4.0f, 8.0f, 20.0f, 40.0f, 60.0f, 80.0f};
					float dbgAbsRate = GyroAbsF(correctedRateDps);
					uint8_t i;

					/* 先写入再判触发，触发样本本身留在数组里；冻结后不再覆盖，
					 * 这样超过门槛那一刻之前 1.28s 的波形能被完整保留下来。 */
					if (gyroDbgCaptureFrozen == 0U)
					{
						gyroDbgCapture[gyroDbgCaptureIndex] = correctedRateDps;
						gyroDbgCaptureIndex = (uint16_t)
							((gyroDbgCaptureIndex + 1U) % GYRO_DBG_CAPTURE_LEN);
						if (dbgAbsRate > gyroDbgCaptureTriggerDps)
						{
							gyroDbgCaptureFrozen = 1U;
							/* 环形缓冲按时间顺序摊平：Ordered[0] 最旧，
							 * Ordered[127] 最新（触发点）。只拷一次，不占周期。 */
							{
								uint16_t k;

								for (k = 0U; k < GYRO_DBG_CAPTURE_LEN; k++)
								{
									gyroDbgCaptureOrdered[k] = gyroDbgCapture
										[(gyroDbgCaptureIndex + k) % GYRO_DBG_CAPTURE_LEN];
								}
								gyroDbgCaptureOrderedValid = 1U;
							}
						}
					}

					dbgWindowSamples++;
					dbgRateSum += correctedRateDps;
					dbgRateAbsSum += dbgAbsRate;
					if (dbgAbsRate > dbgRatePeak)
					{
						dbgRatePeak = dbgAbsRate;
					}
					if (dbgAbsRate > gyroDbgRatePeakHoldDps)
					{
						gyroDbgRatePeakHoldDps = dbgAbsRate;
					}
					if (dbgAbsRate > GYRO_ZUPT_MAX_RATE_DPS)
					{
						dbgRateFail++;
					}

					if (dbgHavePreviousRaw != 0U)
					{
						float dbgStep = GyroAbsF(correctedRateDps - dbgPreviousRaw);

						if (dbgStep > dbgStepPeak)
						{
							dbgStepPeak = dbgStep;
						}
						if (dbgStep > gyroDbgStepPeakHoldDps)
						{
							gyroDbgStepPeakHoldDps = dbgStep;
						}
						if (dbgStep > GYRO_ZUPT_MAX_STEP_DPS)
						{
							dbgStepFail++;
						}
					}
					dbgPreviousRaw = correctedRateDps;
					dbgHavePreviousRaw = 1U;

					for (i = 0U; i < GYRO_DBG_HIST_BINS; i++)
					{
						if (dbgAbsRate > histThresholds[i])
						{
							gyroDbgHistOver[i]++;
						}
					}
					gyroDbgHistSamples++;
				}
#endif
				/* --- 零漂抑制第 1 步：静止判定（ZUPT 判据）--------------------
				 * 判据 = 判据信号足够小，且相邻样本不跳变。
				 * 必须连续满足 GYRO_ZUPT_CONFIRM_SAMPLES 个样本（默认 0.5s）才算
				 * 静止：这样正常转动、以及转动中刚好路过 0 的那一瞬间都不会被误判。
				 * 一旦出现真实转动，计数立刻清零，重新计时。
				 * ★ GYRO_ZUPT_JUDGE_MEDIAN3 = 1 时，判据吃的是 3 拍中值：
				 *   孤立单拍尖峰（SPI 损坏 / EMI）不再能把 0.5s 计数清零。 */
				{
#if ((GYRO_ZUPT_RAMP_RECOVER_ENABLE != 0U) || (GYRO_RETRO_CORRECT_ENABLE != 0U))
					zuptWasActive = zuptActive;
#endif
#if (GYRO_ZUPT_JUDGE_MEDIAN3 != 0U)
					judgeSamples[judgeIndex] = correctedRateDps;
					judgeIndex = (uint8_t)((judgeIndex + 1U) % 3U);
					if (judgeFilled < 3U)
					{
						judgeFilled++;
					}
					/* 刚恢复采样（不足 3 拍）时中值没有意义，先按原值走，
					 * 同时不做跳变判据，避免用不连续的数据下结论。 */
					if (judgeFilled < 3U)
					{
						judgeRateDps = correctedRateDps;
						havePreviousJudge = 0U;
					}
					else
					{
						judgeRateDps = GyroMedianOfThree(
							judgeSamples[0], judgeSamples[1], judgeSamples[2]);
					}
#else
					judgeRateDps = correctedRateDps;
#endif
#if (GYRO_ZUPT_JUDGE_HYSTERESIS != 0U)
					if (zuptActive != 0U)
					{
						/* --- 已锁定：走"出场"判据（本拍原值，不用中值）-------
						 * 真实转动零延迟退出；孤立单拍尖峰只能让
						 * zuptExitCounter 到 1，下一拍就复位。
						 * zuptCounter 在锁定期间保持满值，所以这里不清它。 */
						if (GyroAbsF(correctedRateDps) > GYRO_ZUPT_EXIT_FAST_DPS)
						{
							/* tier-1：明显是运动，立刻解锁，一拍角度都不丢 */
#if (GYRO_ZUPT_DEBUG_ACTIVE != 0U)
							gyroDbgHoldDropCount++;
							gyroDbgRateDropCount++;
							gyroDbgFastExitCount++;
#endif
							zuptCounter = 0U;
							zuptExitCounter = 0U;
						}
						else if (GyroAbsF(correctedRateDps) > GYRO_ZUPT_MAX_RATE_DPS)
						{
							/* tier-2：40~80 °/s，可能是噪声也可能是慢转，
							 * 要求连续 GYRO_ZUPT_EXIT_CONFIRM_SAMPLES 拍 */
							if (zuptExitCounter < GYRO_ZUPT_EXIT_CONFIRM_SAMPLES)
							{
								zuptExitCounter++;
							}
							if (zuptExitCounter >= GYRO_ZUPT_EXIT_CONFIRM_SAMPLES)
							{
#if (GYRO_ZUPT_DEBUG_ACTIVE != 0U)
								gyroDbgHoldDropCount++;
								gyroDbgRateDropCount++;
								gyroDbgSlowExitCount++;
#endif
								zuptCounter = 0U;
								zuptExitCounter = 0U;
							}
						}
						else
						{
							zuptExitCounter = 0U;
						}
					}
					else
					{
						/* --- 未锁定：走"入场"判据（判据信号 + 连击计时）----- */
						if ((GyroAbsF(judgeRateDps) <= GYRO_ZUPT_MAX_RATE_DPS) &&
							((havePreviousJudge == 0U) ||
							 (GyroAbsF(judgeRateDps - previousJudgeRateDps) <=
							  GYRO_ZUPT_MAX_STEP_DPS)))
						{
							if (zuptCounter < GYRO_ZUPT_CONFIRM_SAMPLES)
							{
								zuptCounter++;
							}
						}
						else
						{
#if (GYRO_ZUPT_DEBUG_ACTIVE != 0U)
							/* 只在"本来在计时"时记一次：一直失败时 counter 已经是 0，
							 * 不会每 10ms 累加一次，所以这个数就是真实的打断次数。 */
							if (zuptCounter > 0U)
							{
								gyroDbgHoldDropCount++;
								if (GyroAbsF(judgeRateDps) > GYRO_ZUPT_MAX_RATE_DPS)
								{
									gyroDbgRateDropCount++;
								}
								else
								{
									gyroDbgStepDropCount++;
								}
							}
#endif
							zuptCounter = 0U;
						}
						zuptExitCounter = 0U;
					}
#else
					if ((GyroAbsF(judgeRateDps) <= GYRO_ZUPT_MAX_RATE_DPS) &&
						((havePreviousJudge == 0U) ||
						 (GyroAbsF(judgeRateDps - previousJudgeRateDps) <=
						  GYRO_ZUPT_MAX_STEP_DPS)))
					{
						if (zuptCounter < GYRO_ZUPT_CONFIRM_SAMPLES)
						{
							zuptCounter++;
						}
					}
					else
					{
#if (GYRO_ZUPT_DEBUG_ACTIVE != 0U)
						/* 只在"本来在计时"时记一次掉线：一直失败时 counter 已经是 0，
						 * 不会每 10ms 累加一次，所以这个数就是真实的打断次数。 */
						if (zuptCounter > 0U)
						{
							gyroDbgHoldDropCount++;
							if (GyroAbsF(judgeRateDps) > GYRO_ZUPT_MAX_RATE_DPS)
							{
								gyroDbgRateDropCount++;
							}
							else
							{
								gyroDbgStepDropCount++;
							}
						}
#endif
						zuptCounter = 0U;
					}
#endif
					previousJudgeRateDps = judgeRateDps;
					havePreviousJudge = 1U;
#if (GYRO_ZUPT_DEBUG_ACTIVE != 0U)
					/* 判据信号本身的峰值：调 GYRO_ZUPT_MAX_RATE_DPS 就看这组数 */
					if (GyroAbsF(judgeRateDps) > dbgJudgePeak)
					{
						dbgJudgePeak = GyroAbsF(judgeRateDps);
					}
					if (GyroAbsF(judgeRateDps) > gyroDbgJudgePeakHoldDps)
					{
						gyroDbgJudgePeakHoldDps = GyroAbsF(judgeRateDps);
					}
					if (GyroAbsF(judgeRateDps) > GYRO_ZUPT_MAX_RATE_DPS)
					{
						dbgJudgeFail++;
					}
#endif
				}
				zuptActive = (zuptCounter >= GYRO_ZUPT_CONFIRM_SAMPLES) ? 1U : 0U;
#if (GYRO_ZUPT_RAMP_RECOVER_ENABLE != 0U)
				/* --- 起步坡道补偿：锁定刚刚结束，把冻结期间最后 0.25s 的角度补回 ---
				 * 冻结期间角度不动，但平台是从静止加速上来的。解锁时角速度已经
				 * 越过判据门限（tier-1 是 80 °/s），0→门限 这段坡道整段被丢掉了。
				 * 这里把"不冻结时本来会积进去"的增量原样加回去，等价于这段时间
				 * 从未冻结，所以不会引入新的标度误差。
				 * ★ 只在 1→0 的跳变上补一次；静止时这点增量只有零偏×0.25s≈0.002°，
				 *   被 MIN_DEG 门限挡掉，不会污染 0.00000° 的静止显示。 */
				if ((zuptWasActive != 0U) && (zuptActive == 0U))
				{
					float recoverDeg = 0.0f;
					uint8_t recoverIndex;

					for (recoverIndex = 0U; recoverIndex < rampRecoverCount; recoverIndex++)
					{
						recoverDeg += rampRecoverBuf[recoverIndex];
					}
					if (GyroAbsF(recoverDeg) >= GYRO_ZUPT_RAMP_RECOVER_MIN_DEG)
					{
						accumulatedAngleDeg += recoverDeg;
#if (GYRO_ZUPT_DEBUG_ACTIVE != 0U)
						gyroDbgRampRecoverCount++;
						gyroDbgRampRecoverDegSum += recoverDeg;
#endif
					}
					rampRecoverCount = 0U;
					rampRecoverIndex = 0U;
				}
#endif
#if (GYRO_RETRO_CORRECT_ENABLE != 0U)
				/* --- 运动段零偏回溯修正：段边界处理 -------------------------
				 * 见 GYRO_RETRO_CORRECT_ENABLE 的说明。这里只做"记账"和
				 * "结算"，真正的平滑修正放在积分之后（只在静止拍上加）。 */
				if ((zuptWasActive != 0U) && (zuptActive == 0U))
				{
					/* 1→0：一段运动开始。上一段如果还有没滑完的修正量，
					 * 此刻平台已经在动，一次补完看不出来（README 第 7 条）。 */
					if (retroApplyActive != 0U)
					{
						accumulatedAngleDeg += retroApplyRemainDeg;
						retroApplyTotalDeg += retroApplyRemainDeg;
						gyroRetroCorrectionDeg += retroApplyRemainDeg;
						retroApplyRemainDeg = 0.0f;
						retroApplyActive = 0U;
					}
					retroSegActive = 1U;
					retroSegDtMs = 0U;
					retroSegStartBiasDps = biasDps;
					/* 上一段没攒够静止样本就又要动了：本次估计作废 */
					retroPending = 0U;
					retroStaticSumDps = 0.0f;
					retroStaticCount = 0U;
				}
				else if ((zuptWasActive == 0U) && (zuptActive != 0U) &&
						 (retroSegActive != 0U))
				{
					/* 0→1：一段运动结束（减速尾巴已积进去，锁定拍不算）。
					 * 后面开始用静止样本重新估计零偏。 */
					retroSegActive = 0U;
					if (retroSegDtMs >= GYRO_RETRO_MIN_SEGMENT_MS)
					{
						retroPending = 1U;
					}
					retroStaticSumDps = 0.0f;
					retroStaticCount = 0U;
				}
#endif
#if (GYRO_ZUPT_DEBUG_ACTIVE != 0U)
				if (zuptCounter > gyroDbgHoldRunMax)
				{
					gyroDbgHoldRunMax = zuptCounter;
				}
#endif

				if (zuptActive != 0U)
				{
					/* --- 零漂抑制第 2 步：残余零偏慢跟踪 -----------------------
					 * 确认静止后，让 bias 朝残余零偏走一小步（外层 τ≈4s），把
					 * 1~2 个 LSB 的残余零偏慢慢吃掉；内层先用 τ≈0.5s 的滑动平均
					 * 把噪声压掉，否则 bias 会因为噪声而"跳"（见 GYRO_BIAS_TRACK_AVG_GAIN）。
					 * 步长足够小，所以不会在角度曲线上留下台阶。
					 * 跟踪后立刻用新 bias 重算 corrected，保证下面滤波和积分拿到
					 * 的是同一时刻的一致值。 */
#if (GYRO_BIAS_TRACK_ENABLE != 0U)
					if (biasTrackPrimed == 0U)
					{
						/* 刚进入静止：拿当前残余直接起步，避免内层从 0 慢慢爬
						 * （否则前 0.5s 的跟踪量会偏小，收敛更慢） */
						biasTrackAvgDps = correctedRateDps;
						biasTrackPrimed = 1U;
					}
					biasTrackAvgDps +=
						GYRO_BIAS_TRACK_AVG_GAIN * (correctedRateDps - biasTrackAvgDps);
					biasDps += GYRO_BIAS_TRACK_GAIN * biasTrackAvgDps;
					correctedRateDps = measuredRateDps - biasDps;
					biasTrackUpdates++;
#endif

#if (GYRO_RETRO_CORRECT_ENABLE != 0U)
					/* --- 运动段零偏回溯修正：静止块平均 + 结算 ---------------
					 * 用**原始未补偿**的 measuredRateDps 累加，得到的均值就是
					 * "此刻实测零偏的绝对值"；不能累加 correctedRateDps，因为
					 * 上面的慢跟踪每拍都在改 bias，累加它会得到一个越走越小的
					 * 伪均值。
					 * 只收 |补偿后角速度| 很小的样本，挡住减速尾巴和轻微振动。 */
					if ((retroPending != 0U) &&
						(retroStaticCount < GYRO_RETRO_BIAS_MAX_SAMPLES) &&
						(GyroAbsF(correctedRateDps) <= GYRO_RETRO_BIAS_MAX_RATE_DPS))
					{
						retroStaticSumDps += measuredRateDps;
						retroStaticCount++;
					}

					if ((retroPending != 0U) &&
						(retroStaticCount >= GYRO_RETRO_MIN_STATIC_SAMPLES))
					{
						/* 攒够样本，结算本段：
						 *   新零偏 b1 = (b0 × W + Σ实测) / (W + N)
						 *   Δb = b1 − b0
						 *   段内实际积分时长 t（只在真正积分的拍上累加）
						 *   修正量 = −Δb × t × (1 − RAMP/2) × 标度
						 * 负号：bias 变小 ⇒ 原来少算了 ⇒ 角度要加回来。 */
						float newBiasDps;
						float deltaBiasDps;
						float correctionDeg;

						newBiasDps = (retroSegStartBiasDps *
										  GYRO_RETRO_BIAS_PRIOR_WEIGHT +
									  retroStaticSumDps) /
									 (GYRO_RETRO_BIAS_PRIOR_WEIGHT +
									  (float)retroStaticCount);
						deltaBiasDps = newBiasDps - retroSegStartBiasDps;
						correctionDeg = -deltaBiasDps *
										((float)retroSegDtMs / 1000.0f) *
										(1.0f - GYRO_RETRO_BIAS_RAMP * 0.5f) *
										GYRO_ANGLE_SCALE_CALIBRATION;

						gyroRetroEstimateDps = newBiasDps;
						gyroRetroLastCorrectionDeg = correctionDeg;
						gyroRetroSettleCount++;

						if (GyroAbsF(correctionDeg) >= GYRO_RETRO_MIN_CORRECTION_DEG)
						{
							if (GyroAbsF(correctionDeg) <= GYRO_RETRO_MAX_CORRECTION_DEG)
							{
								retroApplyRemainDeg += correctionDeg;
								retroApplyActive = 1U;
								retroApplySamples = 0U;
							}
							else
							{
								/* 超过安全阀：多半是静止样本里混了坏数据，
								 * 这 3° 以上的修正一旦打进去就再也收不回来。 */
								gyroRetroRejectCount++;
							}
						}
						retroPending = 0U;
					}
#endif
				}
				else
				{
					/* 一旦离开静止，内层平均作废：转动中的数据不能混进零偏 */
					biasTrackPrimed = 0U;
				}
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
					integrationInputDps = correctedRateDps;
#if (GYRO_INTEGRATION_MEDIAN3_ENABLE != 0U)
					filterSamples[filterIndex] = correctedRateDps;
					filterIndex = (uint8_t)((filterIndex + 1U) % 3U);
					integrationInputDps = GyroMedianOfThree(
						filterSamples[0], filterSamples[1], filterSamples[2]);
#endif
					if (elapsedMs > 0U)
					{
						dtSeconds = (float)elapsedMs / 1000.0f;
						filterAlpha = dtSeconds / (GYRO_LOWPASS_TAU_SECONDS + dtSeconds);
						filteredRateDps += filterAlpha * (integrationInputDps - filteredRateDps);
					}
#if (GYRO_ZUPT_DEBUG_ACTIVE != 0U)
					/* 同一拍的滤波后峰值：和 raw 峰值差得多，说明是高频振动/尖峰。
					 * 滤波后峰值远小于 raw 峰值时，用滤波值做"入场"判据可以把
					 * 阈值压得很低而不被尖峰打断。 */
					if (GyroAbsF(filteredRateDps) > dbgFilteredPeak)
					{
						dbgFilteredPeak = GyroAbsF(filteredRateDps);
					}
					if (GyroAbsF(filteredRateDps) > gyroDbgFilteredPeakHoldDps)
					{
						gyroDbgFilteredPeakHoldDps = GyroAbsF(filteredRateDps);
					}
#endif
				}

				if (havePreviousRate != 0U)
				{
#if (GYRO_DRIFT_SUPPRESS_ENABLE != 0U)
					if ((GYRO_ZUPT_FREEZE_ENABLE != 0U) && (zuptActive != 0U))
					{
						/* --- 零漂抑制第 3 步：静止锁定 -----------------------------
						 * 已判定静止，本周期不积分：角度保持不变，残余零偏就
						 * 不会继续被累积成角度漂移。滤波状态照常更新，所以
						 * 一旦真的开始转动，角度能立刻平滑接上，不会跳变。 */
						zuptFrozenSamples++;
#if (GYRO_ZUPT_RAMP_RECOVER_ENABLE != 0U)
						/* 把"本来该积进去、被 ZUPT 挡掉"的增量存进环形缓冲，
						 * 解锁时整体补回（公式与下面的积分分支逐字相同）。 */
						rampRecoverBuf[rampRecoverIndex] =
							((previousFilteredRateDps + filteredRateDps) * 0.5f) *
							((float)elapsedMs / 1000.0f) * GYRO_ANGLE_SCALE_CALIBRATION;
						rampRecoverIndex = (uint8_t)
							((rampRecoverIndex + 1U) % GYRO_ZUPT_RAMP_RECOVER_SAMPLES);
						if (rampRecoverCount < GYRO_ZUPT_RAMP_RECOVER_SAMPLES)
						{
							rampRecoverCount++;
						}
#endif
#if (GYRO_ZUPT_DEBUG_ACTIVE != 0U)
						/* 顺手把"本来该积进去、被 ZUPT 挡掉"的角度累下来。
						 * 公式和下面的积分分支完全一致，所以它与真实积分严格可比：
						 * accumulatedAngleDeg + gyroDbgZuptLostDeg = 纯积分角度。 */
						gyroDbgZuptLostDeg +=
							((previousFilteredRateDps + filteredRateDps) * 0.5f) *
							((float)elapsedMs / 1000.0f) * GYRO_ANGLE_SCALE_CALIBRATION;
#endif
					}
					else
#endif
					{
						dtSeconds = (float)elapsedMs / 1000.0f;
						accumulatedAngleDeg +=
							((previousFilteredRateDps + filteredRateDps) * 0.5f) *
							dtSeconds * GYRO_ANGLE_SCALE_CALIBRATION;
#if (GYRO_RETRO_CORRECT_ENABLE != 0U)
						/* 只在真正做积分的拍上累加，所以这个时长与"这段运动
						 * 到底积了多少秒"严格同口径：ZUPT 冻结拍、被跨接的掉拍
						 * 都不算。回溯修正量 = Δ零偏 × 这个时长。 */
						if (retroSegActive != 0U)
						{
							retroSegDtMs += elapsedMs;
						}
#endif
					}
				}
				previousFilteredRateDps = filteredRateDps;
				previousValidMs = sampleTickMs;
				havePreviousRate = 1U;

#if (GYRO_RETRO_CORRECT_ENABLE != 0U)
				/* --- 运动段零偏回溯修正：平滑应用 ---------------------------
				 * 修正量不能阶跃加到角度上（会在静止曲线上留一个 1~2° 的台阶），
				 * 这里按 GYRO_RETRO_SLEW_DPS 限速滑动，且**只在 ZUPT 锁定的拍**
				 * 上加，保证运动段的角度积分不被污染。
				 * 到 GYRO_RETRO_CREEP_MAX_SAMPLES 拍或残量归零就冻结，避免长时间
				 * 静止时角度一直跟着零偏估计游走；冻结后剩下的量在下次起转时
				 * 一次补完（见上面的 1→0 处理）。 */
				if ((retroApplyActive != 0U) && (zuptActive != 0U))
				{
					float creepLimitDeg =
						GYRO_RETRO_SLEW_DPS * ((float)elapsedMs / 1000.0f);
					float creepStepDeg;

					if (creepLimitDeg < 1.0e-4f)
					{
						creepLimitDeg = 1.0e-4f;
					}
					if (retroApplyRemainDeg > creepLimitDeg)
					{
						creepStepDeg = creepLimitDeg;
					}
					else if (retroApplyRemainDeg < -creepLimitDeg)
					{
						creepStepDeg = -creepLimitDeg;
					}
					else
					{
						creepStepDeg = retroApplyRemainDeg;
					}
					accumulatedAngleDeg += creepStepDeg;
					retroApplyTotalDeg += creepStepDeg;
					gyroRetroCorrectionDeg += creepStepDeg;
					retroApplyRemainDeg -= creepStepDeg;
					retroApplySamples++;
					if ((retroApplySamples >= GYRO_RETRO_CREEP_MAX_SAMPLES) ||
						(GyroAbsF(retroApplyRemainDeg) <= 1.0e-4f))
					{
						retroApplyActive = 0U;
					}
				}
#endif

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

#if (GYRO_ZUPT_DEBUG_ACTIVE != 0U)
				/* 每秒把滚动窗口的统计量发布到全局：Watch 里看到的就是"最近一秒"。
				 * "*Hold"、hist 和 capture 不在这里复位，必须手动写 0。 */
				if (dbgWindowStartMs == 0U)
				{
					dbgWindowStartMs = sampleStartMs;
				}
				else if ((sampleStartMs - dbgWindowStartMs) >= 1000U)
				{
					if (dbgWindowSamples > 0U)
					{
						gyroDbgRatePeakDps = dbgRatePeak;
						gyroDbgRateMeanDps = dbgRateSum / (float)dbgWindowSamples;
						gyroDbgRateAbsMeanDps = dbgRateAbsSum / (float)dbgWindowSamples;
						gyroDbgStepPeakDps = dbgStepPeak;
						gyroDbgFilteredPeakDps = dbgFilteredPeak;
						gyroDbgWindowSamples = dbgWindowSamples;
						gyroDbgRateFailCount = dbgRateFail;
						gyroDbgStepFailCount = dbgStepFail;
						gyroDbgJudgePeakDps = dbgJudgePeak;
						gyroDbgJudgeFailCount = dbgJudgeFail;
					}
					dbgWindowStartMs = sampleStartMs;
					dbgWindowSamples = 0U;
					dbgRateSum = 0.0f;
					dbgRateAbsSum = 0.0f;
					dbgRatePeak = 0.0f;
					dbgStepPeak = 0.0f;
					dbgFilteredPeak = 0.0f;
					dbgRateFail = 0U;
					dbgStepFail = 0U;
					dbgJudgePeak = 0.0f;
					dbgJudgeFail = 0U;
				}
#endif
			}
			else if (calibrationState == 3U)
			{
				/* 自动重试：失败后等 RETRY_DELAY 再采一轮，避免一次震动把角度永久锁死。
				 * 重试期间照常采样、照常发 0x321（bit0=0、bit3=1），上位机会显示
				 * “本轮校准失败（自动重试中）”，不会误报成有效角度。 */
				if ((sampleStartMs - calibrationFailMs) >= GYRO_CALIBRATION_RETRY_DELAY_MS)
				{
					calibrationCount = 0U;
					calibrationSum = 0.0f;
					calibrationValidationSum = 0.0;
					calibrationValidationBlockSum = 0.0;
					calibrationValidationPeak = 0.0f;
					calibrationValidationResidual = 0.0f;
					calibrationValidationPassed = 0U;
					calibrationDataFault = 0U;
					calibrationMin = 0.0f;
					calibrationMax = 0.0f;
					calibrationState = 1U;
					calibrationDegraded = 0U;
					gyroCalibrationRetryCount++;
				}
				havePreviousRate = 0U;
			}
			else
			{
				havePreviousRate = 0U;
			}
		}
		else
		{
			/* 本拍读取失败（SPI 出错或码值离谱）：只记账，不销毁积分状态。
			 * 旧实现把 havePreviousRate 清 0，于是恢复后的那一拍 elapsedMs = 0，
			 * 读失败前后整整两个周期（20ms）的角度被整段丢掉 —— 114.6 °/s 时
			 * 一次掉拍就少算 2.3°，几次就足够凑出"180° 只积到 178"的假象。
			 * 现在由下一个有效样本按真实 elapsedMs 把这段跨接积起来；只有空洞
			 * 超过 GYRO_MAX_BRIDGE_MS 才作废重来（见上面的 gap 检查）。
			 * 静止判定因此也不会被一次孤立掉拍打断。 */
			gyroErrCount++;
			dataValid = 0U;
			if (calibrationState == 1U)
			{
				calibrationDataFault = 1U;
			}
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
		gyroRawCode = rawCode;
		gyroSampleTickMs = sampleTickMs;
		gyroSampleSequence = sampleSequence;
		gyroRateCorrectedDps = correctedRateDps;
		gyroRateFilteredDps = filteredRateDps;
		gyroBiasDps = biasDps;
		gyroAccumulatedAngleDeg = accumulatedAngleDeg;
		gyroCalibrationState = calibrationState;
		gyroCalibrationDegraded = calibrationDegraded;
		gyroCalibrationSampleCount = calibrationCount;
		gyroCalibrationValidationSamples = (calibrationCount > GYRO_CALIBRATION_SAMPLES) ?
			(calibrationCount - GYRO_CALIBRATION_SAMPLES) : 0U;
		gyroCalibrationValidationResidualDps = calibrationValidationResidual;
		gyroCalibrationValidationPeakDps = calibrationValidationPeak;
		gyroCalibrationValidationPassed = calibrationValidationPassed;
		gyroCalibrationRangeDps =
			(calibrationCount > 0U) ? (calibrationMax - calibrationMin) : 0.0f;
		/* 长缺口丢掉的角度不能靠后续成功样本恢复，禁止静默重新有效。 */
		gyroDataValid = (dataValid != 0U && gyroAngleIntegrityLost == 0U) ? 1U : 0U;
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

/* 10ms 周期的 CAN 上报。所有帧共用同一次临界区快照，保证角度、温度、
 * 角速度、原始码、零偏严格来自同一拍：
 *   0x321 / DLC=8  角度 + 温度 + 状态位 + 采样间隔
 *   0x323 / DLC=4  滤波后角速度，单位 0.00001 °/s
 *   0x324 / DLC=8  原始 SPI 码 + 实际采样时刻 + 当前零偏（仅新样本时发）
 *   0x325 / DLC=8  滤波后角速度 + 实际采样时刻（仅新样本时发，供配对）
 * 0x324/0x325 用 gyroSampleSequence 判断"有没有新样本"：读取失败或 SPI
 * 超时的周期不重复上报上一拍的旧值。每帧的发送返回值分别判断、分别计数。 */
static void task_can_tx(void const *argument)
{
	const TickType_t periodTicks = pdMS_TO_TICKS(CAN_TX_PERIOD_MS);
	TickType_t lastWakeTick = xTaskGetTickCount();
	uint8_t payload[8];
	uint8_t ratePayload[4];
	uint8_t rawPayload[8];
	uint8_t detailPayload[8];
	uint8_t statusFlags;
	uint8_t sampleInterval;
	uint32_t angleBits;
	uint32_t rateBits;
	uint32_t rawBits;
	uint32_t tickBits;
	uint32_t biasBits;
	uint16_t tempBits;
	int32_t angleRawValue;
	int32_t rateRawValue;
	int16_t tempCentiDegrees;
	int16_t rawCodeSnapshot;
	float angleDegrees;
	float temperatureCelsius;
	float filteredRateDps;
	float biasDpsSnapshot;
	uint8_t dataValid;
	uint8_t tempValid;
	uint8_t calibrationState;
	uint8_t calibrationDegraded;
	uint8_t zuptState;
	uint32_t intervalMs;
	uint32_t sampleTickMs;
	uint32_t sampleSequence;
	uint32_t lastRawSequence = 0U;
	uint8_t rawSnapshotSent = 0U;
	HAL_StatusTypeDef rawSendStatus;
	HAL_StatusTypeDef detailSendStatus;

	(void)argument;

	for (;;)
	{
		/* 一次临界区取走同一拍的全部字段：角度、温度、滤波后角速度、
		 * 原始码、采样时刻和零偏。task_gyro 在同一个临界区里发布这些量，
		 * 所以任何一帧都不会拿到半新半旧的组合。 */
		taskENTER_CRITICAL();
		angleDegrees = gyroAccumulatedAngleDeg;
		temperatureCelsius = gyroTempCelsius;
		filteredRateDps = gyroRateFilteredDps;
		biasDpsSnapshot = gyroBiasDps;
		rawCodeSnapshot = gyroRawCode;
		sampleTickMs = gyroSampleTickMs;
		sampleSequence = gyroSampleSequence;
		dataValid = gyroDataValid;
		tempValid = gyroTempValid;
		calibrationState = gyroCalibrationState;
		calibrationDegraded = gyroCalibrationDegraded;
		zuptState = gyroZuptState;
		intervalMs = gyroSampleIntervalMs;
		taskEXIT_CRITICAL();

		/* 角度按 CAN_GYRO_ANGLE_SCALE 上报，分辨率 0.00001°，与上位机 5 位小数显示对应。
		 * GyroScaleToInt32 内部已按"四舍五入、远离零"处理，超量程会饱和；
		 * 该比例下 int32 上限对应 ±21474.83°，累计角度远达不到。 */
		angleRawValue = GyroScaleToInt32(angleDegrees, CAN_GYRO_ANGLE_SCALE);
		/* 温度保持 0.01℃ 分辨率（int16 上限对应 ±327.67℃） */
		tempCentiDegrees = GyroScaleToInt16(temperatureCelsius, 100.0f);
		/* 角速度按 CAN_GYRO_RATE_SCALE 上报，分辨率 0.00001 °/s，与上位机 5 位小数显示对应。
		 * GyroScaleToInt32 内部按"四舍五入、远离零"处理（正数 +0.5、负数 -0.5）。
		 * 饱和检查：陀螺仪满量程 ±100 °/s 对应 ±10000000，int32 上限对应 ±21474.83 °/s，
		 * 余量 200 倍以上，正常测量不会触及；万一传感器异常给出大值，也是饱和而非回绕。 */
		rateRawValue = GyroScaleToInt32(filteredRateDps, CAN_GYRO_RATE_SCALE);
		angleBits = (uint32_t)angleRawValue;
		rateBits = (uint32_t)rateRawValue;
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
			/* bit4 仅表示判定静止；bit6 指示此时是否仍连续积分。 */
			statusFlags |= 0x10U;
		}
		if ((calibrationState == 2U) && (calibrationDegraded != 0U))
		{
			/* bit5：校准为降级模式（峰峰值超限，但均值可用，零偏取自均值）。
			 * 角度照样有效，只是首次零偏精度略低于正常校准，静止后由慢跟踪继续收敛。 */
			statusFlags |= 0x20U;
		}
#if (GYRO_ZUPT_FREEZE_ENABLE == 0U)
		statusFlags |= 0x40U;
#endif
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
		if (CAN_SendFrameWait(CAN_GYRO_DATA_STD_ID, payload, CAN_GYRO_DATA_DLC,
							 CAN_TX_MAILBOX_TIMEOUT_MS) == HAL_OK)
		{
			gyroCanTxOkCount++;
		}
		else
		{
			gyroCanTxErrCount++;
		}

		/* 帧 2：0x323 滤波后角速度帧，DLC=4，小端 int32，单位 0.00001 °/s */
		ratePayload[0] = (uint8_t)(rateBits & 0xFFU);
		ratePayload[1] = (uint8_t)((rateBits >> 8) & 0xFFU);
		ratePayload[2] = (uint8_t)((rateBits >> 16) & 0xFFU);
		ratePayload[3] = (uint8_t)((rateBits >> 24) & 0xFFU);

		if (CAN_SendFrameWait(CAN_GYRO_RATE_STD_ID, ratePayload, CAN_GYRO_RATE_DLC,
							 CAN_TX_MAILBOX_TIMEOUT_MS) == HAL_OK)
		{
			gyroCanTxOkCount++;
			gyroCanTxRateOkCount++;
		}
		else
		{
			gyroCanTxErrCount++;
			gyroCanTxRateErrCount++;
		}

		/* 帧 3/4：原始采样帧 + 滤波明细帧，只在 task_gyro 产生了新样本时发送。
		 * 读取失败的周期 sampleSequence 不变，这里就跳过，避免把上一拍的
		 * 原始码/零偏/滤波值重复记成一条新记录（上位机按时间戳配对也依赖这一点）。
		 * sampleSequence == 0 表示还没采到第一个样本（上电初期），此时不发。 */
		if ((sampleSequence != 0U) && ((rawSnapshotSent == 0U) || (sampleSequence != lastRawSequence)))
		{
			if (rawSnapshotSent != 0U)
			{
				gyroCanSkippedSampleCount += (uint32_t)(sampleSequence - lastRawSequence) - 1U;
			}
			rawBits = (uint32_t)(uint16_t)rawCodeSnapshot;
			tickBits = sampleTickMs;
			/* 零偏按 0.001 °/s 编码（int16，可表示 ±32.767 °/s）。
			 * 校准判据的均值上限是 ±40 °/s，可能超出日志编码范围；万一超限，
			 * GyroScaleToInt16 会钳到 INT16_MIN/MAX，离线分析时以饱和值处理即可。 */
			biasBits = (uint32_t)(uint16_t)GyroScaleToInt16(biasDpsSnapshot, CAN_GYRO_BIAS_SCALE);

			/* 0x324：D0–D1 原始码 int16；D2–D5 采样时刻 uint32；D6–D7 零偏 int16 */
			rawPayload[0] = (uint8_t)(rawBits & 0xFFU);
			rawPayload[1] = (uint8_t)((rawBits >> 8) & 0xFFU);
			rawPayload[2] = (uint8_t)(tickBits & 0xFFU);
			rawPayload[3] = (uint8_t)((tickBits >> 8) & 0xFFU);
			rawPayload[4] = (uint8_t)((tickBits >> 16) & 0xFFU);
			rawPayload[5] = (uint8_t)((tickBits >> 24) & 0xFFU);
			rawPayload[6] = (uint8_t)(biasBits & 0xFFU);
			rawPayload[7] = (uint8_t)((biasBits >> 8) & 0xFFU);

			rawSendStatus = CAN_SendFrameWait(CAN_GYRO_RAW_STD_ID, rawPayload,
											 CAN_GYRO_RAW_DLC, CAN_TX_MAILBOX_TIMEOUT_MS);
			if (rawSendStatus == HAL_OK)
			{
				gyroCanTxOkCount++;
				gyroCanTxRawOkCount++;
			}
			else
			{
				gyroCanTxErrCount++;
				gyroCanTxRawErrCount++;
			}

			/* 0x325：D0–D3 滤波后角速度 int32；D4–D7 采样时刻 uint32。
			 * 时间戳与 0x324 相同，上位机据此把两帧配成一条完整记录。 */
			detailPayload[0] = (uint8_t)(rateBits & 0xFFU);
			detailPayload[1] = (uint8_t)((rateBits >> 8) & 0xFFU);
			detailPayload[2] = (uint8_t)((rateBits >> 16) & 0xFFU);
			detailPayload[3] = (uint8_t)((rateBits >> 24) & 0xFFU);
			detailPayload[4] = (uint8_t)(tickBits & 0xFFU);
			detailPayload[5] = (uint8_t)((tickBits >> 8) & 0xFFU);
			detailPayload[6] = (uint8_t)((tickBits >> 16) & 0xFFU);
			detailPayload[7] = (uint8_t)((tickBits >> 24) & 0xFFU);

			detailSendStatus = CAN_SendFrameWait(CAN_GYRO_RAWDETAIL_STD_ID, detailPayload,
												CAN_GYRO_RAWDETAIL_DLC, CAN_TX_MAILBOX_TIMEOUT_MS);
			if (detailSendStatus == HAL_OK)
			{
				gyroCanTxOkCount++;
				gyroCanTxRawOkCount++;
			}
			else
			{
				gyroCanTxErrCount++;
				gyroCanTxRawErrCount++;
			}

			if ((rawSendStatus != HAL_OK) || (detailSendStatus != HAL_OK))
			{
				gyroCanRawPairDropCount++;
			}
			lastRawSequence = sampleSequence;
			rawSnapshotSent = 1U;
		}

		vTaskDelayUntil(&lastWakeTick, periodTicks);
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
