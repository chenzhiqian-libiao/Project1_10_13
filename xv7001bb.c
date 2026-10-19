#include "xv7001bb.h"
#include "spi.h"

/* ===========================================================================
 * XV7001BB 陀螺仪驱动实现（器件层）
 * ---------------------------------------------------------------------------
 * 底层依赖 spi.c 的 SPI_Init / SPI_CS / SPI_Transfer。
 * 读、写过程中片选由本文件手动控制，保证"地址字节 + 数据字节"在同一个
 * 低电平期间连续走完 —— 这是 XV7001BB 能正确读写的前提（手册 5-1）
 * =========================================================================== */

/* 地址字节的读/写位：手册 5-1 规定 MSB = 1 读、MSB = 0 写 */
#define XV7001BB_ADDR_READ      0x80U
/* 寄存器地址只占低 5 位，高位在有从机地址时固定为 0（本板多从机功能关闭） */
#define XV7001BB_ADDR_MASK      0x1FU
/* 一次写寄存器：1 字节地址 + 1 字节数据 */
#define XV7001BB_RW_LEN         2U
/* 一次读操作最多能读回的数据字节数：当前用 2（温度 / 角速度都是 2 字节），
   留到 3 是为了兼容手册里的 24 bit 角速度输出格式 */
#define XV7001BB_RW_LEN_MAX     3U

static HAL_StatusTypeDef XV7001bb_ReadBytes(uint8_t addr, uint8_t *buf, uint16_t len);

/* ---------------------------------------------------------------------------
 * 私有函数：连续读取 len 个字节（地址字节之后紧跟 len 个数据字节）
 * @param addr 寄存器地址
 * @param buf  数据存放处
 * @param len  要读的字节数（1 ~ XV7001BB_RW_LEN_MAX）
 * @note  手册 Fig.5.1.2：地址发完之后，数据从第 2 个字节开始在 SCLK 下降沿输出，
 *        所以这里一次收发 len+1 个字节，第 1 个字节（地址期间）丢弃
 * --------------------------------------------------------------------------- */
static HAL_StatusTypeDef XV7001bb_ReadBytes(uint8_t addr, uint8_t *buf, uint16_t len)
{
	uint8_t tx[1U + XV7001BB_RW_LEN_MAX];
	uint8_t rx[1U + XV7001BB_RW_LEN_MAX];
	HAL_StatusTypeDef status;
	uint16_t i;

	if ((buf == NULL) || (len == 0U) || (len > XV7001BB_RW_LEN_MAX))
	{
		return HAL_ERROR;
	}

	/* 第 1 个字节是地址（bit7 置 1 表示读），后面填 0x00 占位
	   —— 这些占位字节不表达任何含义，只是为了产生时钟把数据移出来 */
	tx[0] = (uint8_t)(XV7001BB_ADDR_READ | (addr & XV7001BB_ADDR_MASK));
	for (i = 1U; i <= len; i++)
	{
		tx[i] = 0x00U;
	}

	SPI_CS(0);                                          /* 选中器件 */
	status = SPI_Transfer(tx, rx, (uint16_t)(len + 1U)); /* 边发地址边收数据 */
	SPI_CS(1);                                          /* 释放器件 */

	if (status == HAL_OK)
	{
		for (i = 0U; i < len; i++)
		{
			buf[i] = rx[i + 1U];    /* 跳过地址字节，取真正的数据 */
		}
	}

	return status;
}

/* ---------------------------------------------------------------------------
 * a) 写入数据：向寄存器 addr 写 1 个字节
 * @note  地址字节的 bit7 = 0 表示写；数据紧跟其后，同一个片选低电平内完成
 * --------------------------------------------------------------------------- */
HAL_StatusTypeDef XV7001bb_WriteData(uint8_t addr, uint8_t data)
{
	uint8_t tx[XV7001BB_RW_LEN];
	HAL_StatusTypeDef status;

	tx[0] = (uint8_t)(addr & XV7001BB_ADDR_MASK);   /* bit7 = 0：写 */
	tx[1] = data;

	SPI_CS(0);
	status = SPI_Transfer(tx, NULL, XV7001BB_RW_LEN);
	SPI_CS(1);

	return status;
}

/* ---------------------------------------------------------------------------
 * c) 读取寄存器：从寄存器 addr 读 1 个字节
 * --------------------------------------------------------------------------- */
HAL_StatusTypeDef XV7001bb_ReadReg(uint8_t addr, uint8_t *data)
{
	if (data == NULL)
	{
		return HAL_ERROR;
	}

	return XV7001bb_ReadBytes(addr, data, 1U);
}

/* ---------------------------------------------------------------------------
 * 只发送命令地址（C 类命令：SLPIN / SLPOUT / STBY / SWRST / AUTOC / DSPRES / MEMLOAD）
 * @param cmd 命令寄存器地址
 * @note  手册规定这些命令只要收到地址就生效，后面不需要跟数据字节
 * --------------------------------------------------------------------------- */
HAL_StatusTypeDef XV7001bb_SendCmd(uint8_t cmd)
{
	uint8_t tx;
	HAL_StatusTypeDef status;

	tx = (uint8_t)(cmd & XV7001BB_ADDR_MASK);       /* bit7 = 0：写（只有地址） */

	SPI_CS(0);
	status = SPI_Transfer(&tx, NULL, 1U);
	SPI_CS(1);

	return status;
}

/* ---------------------------------------------------------------------------
 * 读状态寄存器 StsRd(0x04)：bit3 = 温度可读，bit2:0 = 器件状态
 * --------------------------------------------------------------------------- */
HAL_StatusTypeDef XV7001bb_ReadStatus(uint8_t *status)
{
	return XV7001bb_ReadReg(XV7001BB_REG_STSRD, status);
}

/* ---------------------------------------------------------------------------
 * d) 读取温度：返回温度传感器原始码（默认 12 bit 输出）
 * --------------------------------------------------------------------------- */
HAL_StatusTypeDef XV7001bb_ReadTmp(int16_t *raw)
{
	uint8_t buf[2];
	uint16_t value;
	HAL_StatusTypeDef status;

	if (raw == NULL)
	{
		return HAL_ERROR;
	}

	status = XV7001bb_ReadBytes(XV7001BB_REG_TEMPRD, buf, 2U);
	if (status != HAL_OK)
	{
		return status;
	}

	/* 12 bit 格式（手册 Table 5.6）：第 1 字节 = D[11:4]，第 2 字节 = D[3:0]，
	   而且第 2 字节低 4 位固定输出 L。拼成 16 位左对齐值后右移 4 位还原成
	   12 bit 原始码 */
	value = (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
	value = (uint16_t)(value >> 4U);

	if ((value & 0x0800U) != 0U)    /* 12 bit 补码的符号位 */
	{
		value |= 0xF000U;           /* 负数：符号扩展到 16 bit */
	}

	*raw = (int16_t)value;
	return HAL_OK;
}

/* ---------------------------------------------------------------------------
 * 温度原始码 -> 摄氏度
 * @note  手册 Table 3.6：12 bit 输出时 +25°C 对应 400 LSB，温度系数 16 LSB/°C
 * --------------------------------------------------------------------------- */
float XV7001bb_TmpToCelsius(int16_t raw)
{
	return 25.0f + (((float)raw - XV7001BB_TEMP_CODE_25C) / XV7001BB_TEMP_LSB_PER_C);
}

/* ---------------------------------------------------------------------------
 * e) 读取角速度：返回角速度原始码（默认 16 bit 输出）
 * --------------------------------------------------------------------------- */
HAL_StatusTypeDef XV7001bb_ReadAngle(int16_t *raw)
{
	uint8_t buf[2];
	HAL_StatusTypeDef status;

	if (raw == NULL)
	{
		return HAL_ERROR;
	}

	status = XV7001bb_ReadBytes(XV7001BB_REG_DATACCON, buf, 2U);
	if (status != HAL_OK)
	{
		return status;
	}

	/* 16 bit 格式（手册 Table 5.5.2.2）：第 1 字节 = D[15:8]，第 2 字节 = D[7:0]，
	   2 的补码，直接拼成 int16_t 就是有符号原始码 */
	*raw = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
	return HAL_OK;
}

/* ---------------------------------------------------------------------------
 * 角速度原始码 -> °/s
 * @note  手册 Table 3.5 标称 16 bit 输出 280 LSB/(°/s)，但本器件实测为
 *        70 LSB/(°/s)（转台 90° 实测，见 xv7001bb.h 里 XV7001BB_SCALE_16BIT
 *        的说明）。这里按实测值除以 70，保证 °/s 是真实物理量。
 * --------------------------------------------------------------------------- */
float XV7001bb_AngleToDps(int16_t raw)
{
	return (float)raw / XV7001BB_SCALE_16BIT;
}

/* 读取温度并换算成摄氏度（两步合并成一个调用，方便任务里直接用） */
HAL_StatusTypeDef XV7001bb_ReadTmpCelsius(float *celsius)
{
	int16_t raw;
	HAL_StatusTypeDef status;

	if (celsius == NULL)
	{
		return HAL_ERROR;
	}

	status = XV7001bb_ReadTmp(&raw);
	if (status != HAL_OK)
	{
		return status;
	}

	*celsius = XV7001bb_TmpToCelsius(raw);
	return HAL_OK;
}

/* 读取角速度并换算成 °/s（两步合并成一个调用，方便任务里直接用） */
HAL_StatusTypeDef XV7001bb_ReadAngleDps(float *dps)
{
	int16_t raw;
	HAL_StatusTypeDef status;

	if (dps == NULL)
	{
		return HAL_ERROR;
	}

	status = XV7001bb_ReadAngle(&raw);
	if (status != HAL_OK)
	{
		return status;
	}

	*dps = XV7001bb_AngleToDps(raw);
	return HAL_OK;
}

/* ---- 器件控制命令的薄封装：一行一个命令，调用处不用记寄存器地址 -------- */

HAL_StatusTypeDef XV7001bb_SoftReset(void)
{
	return XV7001bb_SendCmd(XV7001BB_REG_SWRST);
}

HAL_StatusTypeDef XV7001bb_SleepIn(void)
{
	return XV7001bb_SendCmd(XV7001BB_REG_SLPIN);
}

HAL_StatusTypeDef XV7001bb_SleepOut(void)
{
	return XV7001bb_SendCmd(XV7001BB_REG_SLPOUT);
}

HAL_StatusTypeDef XV7001bb_Standby(void)
{
	return XV7001bb_SendCmd(XV7001BB_REG_STBY);
}

HAL_StatusTypeDef XV7001bb_AutoCalib(void)
{
	return XV7001bb_SendCmd(XV7001BB_REG_AUTOC);
}

HAL_StatusTypeDef XV7001bb_ResetDsp(void)
{
	return XV7001bb_SendCmd(XV7001BB_REG_DSPRES);
}

HAL_StatusTypeDef XV7001bb_MemLoad(void)
{
	return XV7001bb_SendCmd(XV7001BB_REG_MEMLOAD);
}

/* ---------------------------------------------------------------------------
 * 使能 / 禁止零速率校准命令（读-改-写 DspCtl3 的 bit6 EnCalibCmd）
 * @note  手册 6-12：不先把这个位置 1，AutoC 命令发出去也不会有任何效果。
 *        这里用读-改-写，只动 EnCalibCmd 一位，DspCtl3 里其它位保持原值
 * --------------------------------------------------------------------------- */
HAL_StatusTypeDef XV7001bb_SetCalibCmd(uint8_t enable)
{
	uint8_t value;
	HAL_StatusTypeDef status;

	status = XV7001bb_ReadReg(XV7001BB_REG_DSPCTL3, &value);
	if (status != HAL_OK)
	{
		return status;
	}

	if (enable != 0U)
	{
		value |= XV7001BB_DSPCTL3_ENCALIBCMD;
	}
	else
	{
		value &= (uint8_t)(~XV7001BB_DSPCTL3_ENCALIBCMD);
	}

	return XV7001bb_WriteData(XV7001BB_REG_DSPCTL3, value);
}

/* ---------------------------------------------------------------------------
 * b) 初始化：整个上电流程按手册 Table 3.4 的时序走一遍
 *     第 1 步：等 tIF，刚上电时串口通信还没生效（手册要求 ≥1ms，这里等 10ms）
 *     第 2 步：软件复位，把寄存器恢复成出厂默认值，清掉上次运行的残留设置
 *     第 3 步：退出上电态（POR）。刚上电时器件停在"100 = 上电后"，这个状态
 *             既不是睡眠也不是待机，此时发命令器件是不理的，必须先用
 *             Sleep-out 把它拉到"001 = 正常工作态"
 *     第 4 步：关闭 I²C，只保留 4 线 SPI（IFCtl 的 bit0 必须写 0）
 *     第 5 步：设置角速度输出格式（16 bit + 使能角速度输出）
 *     第 6 步：等 tSTA（≤200ms），让角速度数据稳定下来
 * @retval HAL_OK 表示每一步的 SPI 通信都正常
 * --------------------------------------------------------------------------- */
HAL_StatusTypeDef XV7001bb_Init(void)
{
	HAL_StatusTypeDef status;

	/* 第 1 步：等串口通信生效 */
	HAL_Delay(XV7001BB_T_IF_MS);

	/* 第 2 步：软件复位 */
	status = XV7001bb_SoftReset();
	if (status != HAL_OK)
	{
		return status;
	}

	/* 第 3 步：退出上电态，进入正常工作态 */
	status = XV7001bb_SleepOut();
	if (status != HAL_OK)
	{
		return status;
	}

	/* 第 4 步：关 I²C，只用 4 线 SPI */
	status = XV7001bb_WriteData(XV7001BB_REG_IFCTL, XV7001BB_IFCTL_SPI_4WIRE);
	if (status != HAL_OK)
	{
		return status;
	}

	/* 第 5 步：角速度数据格式 = 16 bit 输出并送出去（复位后默认就是这个值，
	   这里显式写一遍，让配置在代码里看得见） */
	status = XV7001bb_WriteData(XV7001BB_REG_OUTCTL1, XV7001BB_OUTCTL1_DEFAULT);
	if (status != HAL_OK)
	{
		return status;
	}

	/* 第 6 步：等传感器输出稳定 */
	HAL_Delay(XV7001BB_T_STA_MS);

	return HAL_OK;
}