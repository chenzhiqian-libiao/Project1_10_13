# 本段数据导出格式（CSV v3）

上位机「数据保存（CSV）」→ **保存本段数据（角度 + 角速度 + 原始采样 + 电机角度）** 导出的文件格式说明。
实现：`host_app/src/GyroHost.Core/Services/MeasurementCsv.cs`
（格式化与归并由单元测试 `MeasurementCsvTests` 覆盖；文件写入在 `MainViewModel`）。

> v2 → v3：新增 `kind=raw` 行与 `raw_code` / `sample_tick_ms` / `bias_dps` 三列（追加在最后），
> 原有 10 列的名称、顺序、格式**完全不变**，旧脚本按列名读不受影响。
>
> v1 → v2：新增 `kind=motor` 行与 `motor_angle_deg` 列（追加在最后一列）。

## 1. 一次导出包含什么

- 数据来源：上位机**内存**中的四份历史 —— `_dataHistory`（0x321）、`_rateHistory`（0x323）、
  `_rawHistory`（0x324+0x325 配对后的原始采样）、`_motorHistory`（电机位置轮询）。
- 范围：自上一次「重置显示数据」以来的**全部**采样；**不是**当前时间窗口内的那一段。
- 上限：每路 `200000` 点（`MaxHistorySamples`）。100 Hz 下约 33 分钟；超出后最早的点被丢弃，
  界面「可保存：…（每路上限 200000 点）」会如实显示实际点数与时间跨度。
- 文件名默认 `gyro_segment_yyyyMMdd_HHmmss.csv`，默认目录 `D:\GyroDatamotor`
  （不存在时回退到「文档」；仍可另选目录）。
- 编码 **UTF-8 with BOM**；行尾 **CRLF**。

## 2. 为什么是「长表」而不是「宽表」

0x321（角度/温度）、0x323（角速度）、0x324+0x325（原始采样）与电机位置是**独立**的消息，
各有自己的时间戳，板端不保证同刻到达。若强行拼成一行（把最近的角速度配到角度行上），
就必须引入“配对窗口/插值”假设，而这些假设无法从协议中证伪。

因此导出采用长表 + `kind` 列：

- 每个**原始采样**恰好一行，行数 = 角度点数 + 角速度点数 + 原始采样点数 + 电机点数（不新增、不丢弃）；
- 时间戳逐条保留，不插值、不配对；
- 缺的列留空。

> **唯一的例外是 `kind=raw` 行**：0x324 与 0x325 是板端用**同一个采样时间戳**背靠背发出的
> 一对帧（D4–D7 与 D2–D5 相同），已经由板端保证同源，不是上位机猜的配对。
> 两帧时间戳不一致时（0x324 丢失）整行不写，只计入「原始帧未配对」计数。

需要宽表时可自行按 `relative_time_s` 对齐，但那是使用者侧的后处理。

## 3. 列定义

| 列 | 类型 | angle 行（0x321） | rate 行（0x323） | raw 行（0x324+0x325） | motor 行（电机位置） |
| --- | --- | --- | --- | --- | --- |
| `relative_time_s` | 秒，`F6` | 有 | 有 | 有 | 有 |
| `kind` | 枚举 | `angle` | `rate` | `raw` | `motor` |
| `angle_deg` | 度，`F5` | 累计角度（板端已完成 4 倍修正，上位机不再乘） | 空 | 空 | 空 |
| `rate_dps` | °/s，`F5` | 空 | 滤波后角速度 | 滤波后角速度（与 0x323 同值） | 空 |
| `temperature_c` | ℃，`F2` | 温度 | 空 | 空 | 空 |
| `status_hex` | `0xNN` | 0x321 的 D6 状态字节 | 空 | 空 | 空 |
| `status_flags` | 名字，`\|` 分隔 | 见 §4 | 空 | 空 | 空 |
| `interval_ms` | 整数 | 0x321 的 D7 采样间隔 | 空 | 空 | 空 |
| `break_before` | `0`/`1` | 1 = 该点之前曲线断开 | 空 | 空 | 空 |
| `motor_angle_deg` | 度，`F5` | 空 | 空 | 空 | 电机相对本段起点 P0 的累计角度（不取模） |
| `raw_code` | 整数 | 空 | 空 | 陀螺仪 16 bit 原始 SPI 码（补码，÷70 得 °/s） | 空 |
| `sample_tick_ms` | 整数（ms） | 空 | 空 | 板端读完 SPI 那一刻的 HAL tick | 空 |
| `bias_dps` | °/s，`F3` | 空 | 空 | 板端当前零偏（含校准均值与静止慢跟踪） | 空 |

格式约束（全部 `InvariantCulture`，小数点固定 `.`，与系统区域设置无关）：

- `relative_time_s`：`F6`，例如 `0.100000`；
- `angle_deg` / `rate_dps` / `motor_angle_deg`：`F5`，对应协议 0.00001° / 0.00001°/s 的分辨率；
  ⚠️ 这是**协议分辨率**，不是传感器分辨率（XV7001BB 实测 70 LSB/(°/s)，约 0.0143°/s 一个原始码）；
- `temperature_c`：`F2`，对应协议 0.01℃；
- `bias_dps`：`F3`，对应协议 0.001°/s；
- 时间基准：`0` = 上一次「重置显示数据」；**断线续接不清零**，时间轴继续累加。
- `sample_tick_ms` 是**板端自己的** ms 计数（上电为 0，约 49.7 天回绕），与 `relative_time_s`
  不是同一个零点；用它算相邻样本的 `dt` 才能避开上位机接收抖动。

### 为什么电机角度单独占一列

两种角度来自不同设备（陀螺仪 vs 电机编码器），含义不同、量程也不同。
塞进同一列会让任何后处理都必须先按 `kind` 分流，很容易读错。
单独一列 + `kind=motor` 读起来是自明的。

电机行的 `relative_time_s` 取 `MotorPositionSample.ChartTimeSeconds`
——**与陀螺仪共用**的全局时间轴，所以三路可以按同一个时间列直接对齐。

⚠️ 不要误用 `MotorPositionSample.TimeSeconds`：那是**电机自己的**时间轴
（「电机显示归零」后从 0 重新计时），与陀螺仪的 0 点不是同一个。
实测混用会导致电机行落在 0.11–225.7 s、陀螺仪在 0–234.6 s，
运动时刻最多错开 6.9 s，逐点对照完全失效。

电机行**不写** `break_before`：电机曲线的断点由「电机显示归零」触发，
而「显示归零」只改电机私有时间轴，全局轴继续走；把断点塞进这列反而会让
「同一个 `break_before=1`」在三种 `kind` 下含义不同。故电机行只给时间戳与角度。

## 4. `status_flags` 全部取值

| 位 | 名称 | 含义 |
| --- | --- | --- |
| `0x01` | `AngleValid` | 角度/采样有效（固件要求 dataValid 且校准完成） |
| `0x02` | `TemperatureValid` | 温度有效 |
| `0x04` | `CalibrationDone` | 零偏校准完成 |
| `0x08` | `CalibrationFailed` | 零偏校准失败 |
| `0x10` | `StillLocked` | 判定静止、角度已锁定（零漂抑制生效） |
| `0x20` | `DegradedCalibration` | 降级校准：角度仍有效，但零偏精度略低 |
| 其余 | `Unknown_0xNN` | 固件新增而文档未覆盖的位，原样保留，不静默丢弃 |

- 无任何位时该列为空（不是 `None`）。
- 多个位用 `|` 连接（**不能用逗号**，否则会被拆成两列）。
- `status_hex` 始终给出原始字节，便于与 `status_flags` 互校。

## 5. 示例

```csv
relative_time_s,kind,angle_deg,rate_dps,temperature_c,status_hex,status_flags,interval_ms,break_before,motor_angle_deg,raw_code,sample_tick_ms,bias_dps
0.100000,angle,1.23456,,25.31,0x13,AngleValid|TemperatureValid|StillLocked,100,0,,,,
0.104000,motor,,,,,,,,-1.23450,,,
0.106000,rate,,0.01234,,,,,,,,,,
0.110000,raw,,0.01234,,,,,,,-82,1234567,0.012
0.200000,angle,1.23500,,25.31,0x13,AngleValid|TemperatureValid|StillLocked,100,0,,,,
```

第 2 行是电机位置：`angle_deg`…`break_before` 共 7 列为空，值在 `motor_angle_deg`。
第 3 行是角速度：`angle_deg`、`temperature_c`、`status_hex`、`status_flags`、`interval_ms`、
`break_before`、`motor_angle_deg`、`raw_code`、`sample_tick_ms`、`bias_dps` 全部为空。
第 4 行是原始采样：`raw_code` / `sample_tick_ms` / `bias_dps` 与 `rate_dps`（滤波后值）写在同一行。

> **离线复算滤波器**：`corrected = raw_code / 70 - bias_dps` 就是进入中值 + 低通之前的
> 去零偏角速度；用 `sample_tick_ms` 相邻差算 `dt`，再按固件的 `GYRO_FILTER_ALPHA`
> 复算，应当逐点还原 `rate_dps`。若对不上，先检查固件的系数/中值窗口是否改动。

## 6. 已知限制

1. **不是全程记录**：只有内存里还在的 200000 点/路（100 Hz 约 33 分钟）。要长时间记录需提高上限或改成流式落盘，本次未做。
2. **原始采样依赖新固件**：只有实现了 0x324/0x325 的固件才会产生 `kind=raw` 行；
   旧固件下该路为空，其余行与列不受影响。0x324 丢失时该拍整行丢弃（计入「原始帧未配对」）。
3. **电机位置是轮询的**（默认 100 ms 一次，可在界面调，下限 20 ms），
   比陀螺仪（采样 + 上报都是 10 ms）更粗；只在**静止读数**上可比，不要用它算动态。
4. **不做时间对齐**：三路时间戳各自独立，见 §2。
5. **GUI 手测状态**：格式化、归并与 `WriteFile`（含 UTF-8 BOM）由单元测试覆盖；
   视图模型侧（按钮可用性、范围文字、状态文字）由 `--selftest` 覆盖；
   **「另存为」对话框本身与真实点击落盘尚未人工验证**。
6. **写入失败**（磁盘满、目录只读、文件被占用）：状态栏显示失败原因并写日志，不产生半截文件的清理动作由
   `FileMode.Create` 的截断语义保证（失败时可能留下 0 字节或部分文件，需人工删除）。
