#include "spi.h"
#include "cmsis_os.h"

/* SPI2 引脚分配（与原理图网表一致） */
#define GYRO_SPI                        SPI2
#define GYRO_SPI_GPIO_PORT              GPIOB
#define GYRO_SPI_SCK_PIN                GPIO_PIN_13
#define GYRO_SPI_MISO_PIN               GPIO_PIN_14
#define GYRO_SPI_MOSI_PIN               GPIO_PIN_15

static SPI_HandleTypeDef hspi2;
static uint8_t spiInitialized = 0U;

/* ---------------------------------------------------------------------------
 * SPI2 总线锁（互斥量）
 * @note  ★ 为什么必须加锁：SPI2 上挂着陀螺仪，而读它的不只一个任务 ——
 *          task_gyro 每 100ms 读一次温度/角速度，
 *          task_main 每 500ms 巡检一遍接口，
 *        两个任务优先级相同、随时可能被时间片切换打断。如果不加保护，
 *        A 任务正在"地址 + 数据"的传输中途，B 任务插进来把片选拉高，
 *        A 的这次通信就被器件作废了，读回来的数据是错的。
 *        这里把锁加在片选上（SPI_CS(0) 取锁、SPI_CS(1) 放锁），
 *        正好等于"一次完整通信"的边界：谁拿到锁，谁独占总线到通信结束。
 * @note  ★ 锁对象由 SPI_BusMutexInit() 单独创建，不在 SPI_Init() 里创建，
 *          原因（调度器启动前不能碰 FreeRTOS 对象）见 SPI_BusMutexInit() 的说明
 * --------------------------------------------------------------------------- */
osMutexDef(SPI2_BUS_MUTEX);
static osMutexId spiBusMutex = NULL;

/* ---------------------------------------------------------------------------
 * 初始化 SPI2：开时钟 -> 配 4 个引脚 -> 配外设参数 -> 使能外设
 * @retval HAL_OK 成功；HAL_ERROR 失败（主程序会记录失败码并停在那里）
 * --------------------------------------------------------------------------- */
HAL_StatusTypeDef SPI_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* 第 1 步：打开 GPIOB 和 SPI2 的时钟（不开时钟，后面的寄存器写不进去） */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_SPI2_CLK_ENABLE();

	/* 第 2 步：配置 4 个引脚 */

	/* 片选 PB12：普通推挽输出。先拉高（不选中器件），避免初始化期间误选中 */
	GPIO_InitStruct.Pin = SPI_CS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SPI_CS_GPIO_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(SPI_CS_GPIO_PORT, SPI_CS_PIN, GPIO_PIN_SET);

	/* 时钟 PB13、发送 PB15：复用推挽，交给 SPI 外设去驱动 */
	GPIO_InitStruct.Pin = GYRO_SPI_SCK_PIN | GYRO_SPI_MOSI_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GYRO_SPI_GPIO_PORT, &GPIO_InitStruct);

	/* 接收 PB14：普通输入。数据方向是"从机 -> 主机"，主机只读、不驱动这根线，
	   配成浮空输入即可 —— 从机的推挽输出会把电平拉到位 */
	GPIO_InitStruct.Pin = GYRO_SPI_MISO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GYRO_SPI_GPIO_PORT, &GPIO_InitStruct);

	/* 第 3 步：配置 SPI2 外设本身 */
	hspi2.Instance = GYRO_SPI;

	/* 主机模式：时钟由 STM32 产生、由 STM32 发起传输 */
	hspi2.Init.Mode = SPI_MODE_MASTER;

	/* 全双工 2 线：MOSI、MISO 各占一根。
	   SPI 天生是全双工的 —— 主机每发 1 个字节，必然同时收回 1 个字节，
	   XV7001BB 正是利用这一点：地址发出去的同时，数据就顺着收回来了 */
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;

	/* 一次收发 8 位：地址和数据都是 8 位 */
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;

	/* 极性和相位决定"什么时候采样"，必须和器件手册一致，错了数据就是乱的。
	   手册 Fig.5.1.2：XV7001BB 在 SCLK 下降沿把数据推到 MISO 上，主机在
	   上升沿采样最稳。空闲低电平 + 第 1 个边沿（上升沿）采样 = SPI Mode 0 */
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;

	/* 软件片选：NSS 不交给 SPI 硬件自动管理，而是当普通 GPIO 由代码控制，
	   原因见下面 SPI_CS() 的注释 */
	hspi2.Init.NSS = SPI_NSS_SOFT;

	/* SCLK 分频：具体取值与推导放在 spi.h 的 SPI_SCLK_PRESCALER 宏里 */
	hspi2.Init.BaudRatePrescaler = SPI_SCLK_PRESCALER;

	/* 高位先出：地址字节的最高位就是读/写控制位，器件要先收到它 */
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;

	/* 下面三项本工程用不到，用不到就显式关掉 */
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;                  /* TI 的 SSI 协议，不用 */
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;  /* XV7001BB 不带 CRC */
	hspi2.Init.CRCPolynomial = 7;                            /* CRC 关闭时不参与运算 */

	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		return HAL_ERROR;
	}

	/* ★ 这里刻意不创建 SPI2 总线锁：osMutexCreate() 会进 FreeRTOS 临界区，
	   调度器启动前调用会把中断屏蔽位锁死，导致后面 XV7001bb_Init() 里的
	   HAL_Delay() 死等 —— 详细原理见本文件 SPI_BusMutexInit() 的注释 */

	spiInitialized = 1U;
	return HAL_OK;
}

/* ---------------------------------------------------------------------------
 * 创建 SPI2 总线锁（互斥量）
 * @retval HAL_OK 成功；HAL_ERROR 调用顺序不对（SPI 还没初始化）或内存不足
 * @note  ★ 调用位置有严格要求，必须满足两条：
 *           1) 在 XV7001bb_Init() 等所有 HAL_Delay() 之后 —— 因为本函数
 *              一旦执行，SysTick 中断就会被屏蔽（原理见下），任何靠
 *              HAL_GetTick() 计时的延时都会死等；
 *           2) 在创建任务 / osKernelStart() 之前 —— 锁要在任务开始用 SPI 之前建好。
 *           main() 里就是按这两条把它放在"陀螺仪初始化之后、创建任务之前"。
 *
 * @note  ★ 为什么原来放在 SPI_Init() 里会卡死（这个坑务必保留说明）：
 *          osMutexCreate() -> xSemaphoreCreateMutex() -> 队列初始化，
 *          内部会走 FreeRTOS 临界区 vPortEnterCritical() / vPortExitCritical()。
 *          而调度器启动前，port.c 里的临界区嵌套计数还是初值：
 *              static UBaseType_t uxCriticalNesting = 0xaaaaaaaa;
 *          vPortEnterCritical() 让它加 1、vPortExitCritical() 让它减 1，
 *          减完永远不等于 0，于是 vPortExitCritical() 末尾那句
 *              if( uxCriticalNesting == 0 ) { portENABLE_INTERRUPTS(); }
 *          一次都不会执行 —— 中断屏蔽寄存器 BASEPRI 停在
 *          configMAX_SYSCALL_INTERRUPT_PRIORITY（本工程 = 5 << 4 = 0x50）
 *          不再恢复，SysTick（优先级 15，低于该屏蔽值）再也进不了中断，
 *          HAL_GetTick() 的计数停止增长。
 *          紧接着调用 XV7001bb_Init()，它第 1 步就是 HAL_Delay(10)，
 *          而 HAL_Delay() 正是靠 HAL_GetTick() 累加来计时的 -> 死等。
 *          板上现象就是：程序不再往下走，failCode 和 gyroInitOk 都停在 0
 *          （failCode 为 0 说明连 FatalError 都没进，是"卡住"而不是"报错"）。
 *
 * @note  ★ 为什么放到现在这个位置就安全：
 *          本函数之后，main() 里只剩创建任务和 osKernelStart()，没有任何延时；
 *          而 osKernelStart() -> xPortStartScheduler() 里会复位临界区计数
 *          （uxCriticalNesting = 0），随后正常开中断、启动第一个任务，
 *          被屏蔽的 SysTick 自然恢复。
 * --------------------------------------------------------------------------- */
HAL_StatusTypeDef SPI_BusMutexInit(void)
{
	/* 没初始化 SPI 就建锁，说明调用顺序错了，直接报错 */
	if (spiInitialized == 0U)
	{
		return HAL_ERROR;
	}

	/* 重复调用直接当成功返回，避免把上一次的锁对象丢掉（内存泄漏） */
	if (spiBusMutex != NULL)
	{
		return HAL_OK;
	}

	spiBusMutex = osMutexCreate(osMutex(SPI2_BUS_MUTEX));
	if (spiBusMutex == NULL)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

/* ---------------------------------------------------------------------------
 * 片选控制：把"一次通信"的起止交给调用方，而不是在收发函数里自动翻转
 * @param level 0 = 拉低选中器件，1 = 拉高释放器件
 * @note  为什么必须手动控制：XV7001BB 读/写寄存器时，"地址字节 + 数据字节"
 *        必须在同一个片选低电平期间一口气走完；如果每发一个字节就自动拉高
 *        再拉低，器件会把本次通信直接作废，读回来的永远是错的（手册 5-1）
 * @note  ★ 本函数同时兼管 SPI2 总线锁：
 *            level = 0（拉低片选）时先取锁，别的任务想用 SPI2 就得排队；
 *            level = 1（拉高片选）时最后放锁。
 *        这样"片选低电平期间"和"持有总线锁期间"完全重合，
 *        多任务共用一条 SPI 总线也不会互相打断（原因见文件开头的总线锁说明）
 * --------------------------------------------------------------------------- */
void SPI_CS(uint8_t level)
{
	/* ★ 总线锁只在"锁已创建 且 调度器已启动"时才真正使用：
	   1) 初始化阶段（SPI_BusMutexInit() 之前）只有 main() 主流程在跑，
	      单线程、不存在被别的任务打断的可能，无锁反而更简单；
	   2) 锁还没创建时对空句柄取放锁也是不允许的；
	   3) 调度器启动前 FreeRTOS 对象还不能正常使用，一碰就会像
	      SPI_BusMutexInit() 注释里说的那样把中断屏蔽位锁死。
	   所以这里加了双重判断，任务一跑起来（osKernelStart() 之后）就开始正常互斥 */
	uint8_t lockEnabled = 0U;

	if ((spiBusMutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
	{
		lockEnabled = 1U;
	}

	if ((lockEnabled != 0U) && (level == 0U))
	{
		/* 取锁：拿不到就一直等。等锁时本任务被挂起，不占 CPU */
		(void)osMutexWait(spiBusMutex, osWaitForever);
	}

	HAL_GPIO_WritePin(SPI_CS_GPIO_PORT, SPI_CS_PIN,
					  (level != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	if ((lockEnabled != 0U) && (level != 0U))
	{
		/* 放锁：片选已经拉高，本次通信彻底结束，可以让别的任务用了 */
		(void)osMutexRelease(spiBusMutex);
	}
}

/* ---------------------------------------------------------------------------
 * 收发一串字节（全双工：发出去多少，就同时收回来多少）
 * @param tx  要发送的数据；传 NULL 表示只接收
 * @param rx  接收缓冲区；传 NULL 表示只发送（收回来的字节直接丢掉）
 * @param len 字节个数
 * @retval HAL_OK 收发完成；HAL_TIMEOUT 外设卡住超时；HAL_ERROR 参数非法
 * @note  本函数不碰片选，片选必须由调用方用 SPI_CS() 包在外面。
 *        三种用法在这里分流，调用方不必去记 HAL 那几个函数的区别
 * --------------------------------------------------------------------------- */
HAL_StatusTypeDef SPI_Transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
	if ((tx == NULL) && (rx == NULL))
	{
		return HAL_ERROR;   /* 既不发也不收，参数非法 */
	}

	if (len == 0U)
	{
		return HAL_ERROR;
	}

	if (tx != NULL)
	{
		if (rx != NULL)
		{
			/* 边发边收 —— 读寄存器走的就是这一条。
			   HAL 内部是"发 1 个字节 -> 等它对应的接收完成 -> 再发下一个"，
			   所以 rx[0] 一定对应 tx[0]、rx[1] 一定对应 tx[1]，序号不会错位。
			   （强转去掉 const 是因为 HAL 的形参没加 const，它只读发送缓冲区） */
			return HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)tx, rx, len, SPI_TIMEOUT_MS);
		}

		/* 只发不收 —— 写寄存器、发命令走这一条。
		   MISO 上移进来的字节会被 HAL 丢掉，残留的 RXNE/OVR 标志 HAL 返回前清掉 */
		return HAL_SPI_Transmit(&hspi2, (uint8_t *)tx, len, SPI_TIMEOUT_MS);
	}

	/* 只收不发（tx == NULL）。
	   ★ 必须知道这个限制：SPI 的时钟是主机产生的，主机不往外发字节就没有时钟，
		 从机无法把数据移出来；HAL_SPI_Receive() 在主模式下只会干等 RXNE，
		 最后返回 HAL_TIMEOUT。所以本工程从不单独收 —— 读操作都是
		 "发 1 字节地址 + 顺便收回数据"，走上面第一条分支。
		 将来真要在主模式下只收，正确做法是给 tx 填一串 0xFF 走第一条分支 */
	return HAL_SPI_Receive(&hspi2, rx, len, SPI_TIMEOUT_MS);
}

/* ---------------------------------------------------------------------------
 * 发送单个字节，自带一次片选（选中 -> 发 8 位 -> 释放）
 * @param data 要发送的字节
 * @retval HAL_OK 发送完成；HAL_ERROR 未初始化
 * @note  这是给课题四.2 的示波器测试用的简化接口：每 0.5s 发一个 0xAA，
 *        在 PB12(片选) / PB13(SCLK) / PB15(MOSI) 上能看到完整的一组波形
 * --------------------------------------------------------------------------- */
HAL_StatusTypeDef SPI_SendByte(uint8_t data)
{
	HAL_StatusTypeDef status;

	if (spiInitialized == 0U)
	{
		return HAL_ERROR;
	}

	SPI_CS(0);
	status = HAL_SPI_Transmit(&hspi2, &data, 1U, SPI_TIMEOUT_MS);
	SPI_CS(1);

	return status;
}