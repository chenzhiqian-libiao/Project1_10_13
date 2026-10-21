using System.Buffers.Binary;

namespace GyroHost.Core.Protocol;

/// <summary>
/// 电机 CAN 协议（《电机 CAN 通信协议 V11》）中本上位机用到的部分。
/// <para>
/// 已确认可用的部分：
/// <list type="bullet">
/// <item>位置查询：ID <c>0x0408</c>，DLC=2，<c>Data[0]=设备号</c>，<c>Data[1]=0x08</c>；</item>
/// <item>位置位移单位：<c>0.1 mm</c>（回复 ID 为 <c>0x0409</c>）；</item>
/// <item>剩余位移反馈：ID <c>0x0420 + 设备号</c>，DLC=8，<c>Data[0..3]</c>=主通道 s32、<c>Data[4..7]</c>=从通道 s32；</item>
/// <item>力矩反馈：ID <c>0x0440 + 设备号</c>，<c>Data[2..3]</c>=力矩 s16；</item>
/// <item>心跳反馈：ID <c>0x0380 + 设备号</c>，数据内容自定义；</item>
/// <item>错误上报：ID <c>0x0100..0x017F</c>，DLC=2，错误码如 0x0104 堵转、0x0108 断线。</item>
/// </list>
/// </para>
/// <para>
/// 运动控制指令（来自协议表，“电机运动和配置指令”页，本上位机只发送前四条）：
/// <list type="bullet">
/// <item>设置目标位置（主）：ID <c>0x0416</c>，DLC=6，<c>Data[0..3]</c>=目标位置 s32 小端（0.1 mm），
/// <c>Data[4]</c>=设备号，<c>Data[5]</c>=目标类型（0x00 相对 / 0x05 绝对 / 0x0A 归零 / 0x0B 设当前位置）；</item>
/// <item>锁定/释放：ID <c>0x0413</c>，DLC=5，<c>Data[0]</c>=0x5A 使能 / 0x4A 释放，<c>Data[4]</c>=设备号；</item>
/// <item>设置主速度：ID <c>0x0415</c>，DLC=5，<c>Data[0..3]</c>=u32 小端（0.1 mm/s），<c>Data[4]</c>=设备号；</item>
/// <item>设置主加速度：ID <c>0x0414</c>，DLC=5，<c>Data[0..3]</c>=u32 小端（0.1 mm，按协议表原文），<c>Data[4]</c>=设备号。</item>
/// </list>
/// ⚠️ 以上写指令的**字段布局**来自协议表，但本工程尚未在实机上逐条验证（0x041A 参数设置 ACK
/// 的表内 DLC 与“Data[2]=设备号”的描述互相矛盾，也未实测），因此界面把它标为“待实机确认”。
/// </para>
/// <para>
/// 初始化指令（来自补充资料，**不在 V11 协议表内**，均无应答）：
/// <list type="bullet">
/// <item>参数初始化：ID <c>0x0316</c>，DLC=8，<c>Data[0..6]=B4 BF CF AA 00 5F 01</c> 固定、
/// <c>Data[7]</c>=设备号；</item>
/// <item>配置轮径：ID <c>0x0408</c>，DLC=4，<c>Data[0]</c>=设备号、<c>Data[1]=0x24</c>、
/// <c>Data[2..3]</c>=u16 小端轮径（单位 0.01 mm，即 100.00 mm → <c>10 27</c>），需连发 3 次；</item>
/// <item>上电使能 / 下电释放：ID <c>0x0413</c>，DLC=5，<c>Data[0]=0x5A / 0x4A</c>、<c>Data[4]</c>=设备号。</item>
/// </list>
/// </para>
/// <para>
/// ⚠️ 协议表**没有**定义 <c>0x0409</c> 回复的数据长度、设备号/查询码位置、位置字段的
/// 起始字节、符号和字节序。本工程按一帧实机报文
/// <c>0x0409 [01 08 BC 18 00 00 FC FF]</c>（2026-10-17，设备 01，见
/// <c>docs/motor-0x0409-real-frames.md</c>）接入了一份**待进一步实机验证**的布局：
/// <c>Data[0]=设备号</c>、<c>Data[1]=查询码 0x08</c>、<c>Data[2..5]=int32 小端位置</c>、
/// <c>Data[6..7]</c> 含义未知不解析。单帧样本无法区分位置字段的长度（2/4 字节）与有符号性，
/// 因此该布局仍需用“已知角度运动前后 / 反向运动 / 超过 0x7FFF”的回复继续核对。
/// 若后续发现字段不符，只需修改 <see cref="CreateDefaultPositionDecoder"/> 返回的布局，
/// 界面其余部分无需改动。
/// </para>
/// </summary>
public static class MotorProtocol
{
    /// <summary>查询类指令 ID（主控 → 电机）。</summary>
    public const uint QueryId = 0x0408U;

    /// <summary><c>0x0409</c> 同时用于温度、当前位置、状态、版本等查询的回复，必须结合请求上下文区分。</summary>
    public const uint ReplyId = 0x0409U;

    /// <summary>剩余位移反馈 ID 基址：实际 ID = <c>0x0420 + 设备号</c>（设备 01 → 0x0421）。</summary>
    public const uint RemainingDisplacementIdBase = 0x0420U;

    /// <summary>力矩反馈 ID 基址：实际 ID = <c>0x0440 + 设备号</c>（设备 01 → 0x0441）。</summary>
    public const uint TorqueFeedbackIdBase = 0x0440U;

    /// <summary>心跳反馈 ID 基址：实际 ID = <c>0x0380 + 设备号</c>（设备 01 → 0x0381）。</summary>
    public const uint HeartbeatIdBase = 0x0380U;

    /// <summary>参数设置 ACK。</summary>
    public const uint ParameterAckId = 0x041AU;

    /// <summary>设置目标位置（主通道）。</summary>
    public const uint TargetPositionId = 0x0416U;

    /// <summary>控制电机锁定/释放（上电使能 / 下电释放）。</summary>
    public const uint EnableId = 0x0413U;

    /// <summary>
    /// 电机参数初始化 / 建立 PID 基准。数据为固定 7 字节前缀 + 设备号（末字节）。
    /// <para>
    /// ⚠️ 这条帧来自补充资料，《电机 CAN 通信协议 V11》表里没有它，也**没有应答**；
    /// 前缀 <c>B4 BF CF AA 00 5F 01</c> 的各字节含义未定义，本工程原样发送不做解释。
    /// </para>
    /// </summary>
    public const uint ParameterInitId = 0x0316U;

    /// <summary>设置主通道速度。</summary>
    public const uint SetSpeedId = 0x0415U;

    /// <summary>设置主通道加速度。</summary>
    public const uint SetAccelerationId = 0x0414U;

    /// <summary>目标位置帧的 DLC：4 字节位置 + 设备号 + 目标类型。</summary>
    public const int TargetPositionDlc = 6;

    /// <summary>参数初始化帧（<c>0x0316</c>）的 DLC。</summary>
    public const int ParameterInitDlc = 8;

    /// <summary>
    /// 参数初始化帧的前 7 字节（固定）。第 8 字节为设备号，由 <see cref="BuildParameterInit"/> 填入。
    /// </summary>
    private static ReadOnlySpan<byte> ParameterInitPrefix => new byte[] { 0xB4, 0xBF, 0xCF, 0xAA, 0x00, 0x5F, 0x01 };

    /// <summary>配置轮径帧（<c>0x0408</c> 的另一种用途）的 DLC：设备号 + 0x24 + u16 轮径。</summary>
    public const int WheelConfigDlc = 4;

    /// <summary>配置轮径查询码（<c>0x0408</c> 的 <c>Data[1]</c>）。</summary>
    public const byte WheelConfigCode = 0x24;

    /// <summary>配置轮径需要连发三次（协议表要求重复写入）。</summary>
    public const int WheelConfigRepeatCount = 3;

    /// <summary>配置轮径帧里轮径的单位：1 LSB = 0.01 mm（对应位置单位的 0.1 mm）。</summary>
    public const double WheelDiameterMillimetersPerLsb = 0.01;

    /// <summary>可写入的轮径上限（u16 满量程）。</summary>
    public const double MaxWritableWheelDiameterMm = ushort.MaxValue * WheelDiameterMillimetersPerLsb;

    /// <summary>使能帧的 DLC：1 字节命令 + 3 字节保留 + 设备号。</summary>
    public const int EnableDlc = 5;

    /// <summary>速度/加速度帧的 DLC：4 字节 u32 + 设备号。</summary>
    public const int SpeedAccelerationDlc = 5;

    /// <summary>使能命令字节（<c>0x0413</c> 的 <c>Data[0]</c>）。</summary>
    public const byte EnableCommandEnable = 0x5A;

    /// <summary>释放命令字节（<c>0x0413</c> 的 <c>Data[0]</c>）。</summary>
    public const byte EnableCommandRelease = 0x4A;

    /// <summary>错误信息上报段下限（2 字节错误码）。</summary>
    public const uint ErrorIdMin = 0x0100U;

    /// <summary>错误信息上报段上限。</summary>
    public const uint ErrorIdMax = 0x017FU;

    /// <summary>位置查询帧的 DLC。</summary>
    public const int QueryDlc = 2;

    /// <summary>位置查询码（<c>0x0408</c> 的 <c>Data[1]</c>）。</summary>
    public const byte QueryPositionCode = 0x08;

    /// <summary>温度查询码（同一 <c>0x0408</c> 下的另一种查询）。</summary>
    public const byte QueryTemperatureCode = 0x06;

    /// <summary>状态查询码。</summary>
    public const byte QueryStatusCode = 0x25;

    /// <summary>固件版本查询码。</summary>
    public const byte QueryVersionCode = 0xFF;

    /// <summary>默认设备号（协议示例为 01）。</summary>
    public const byte DefaultDeviceId = 0x01;

    /// <summary>
    /// 可用设备号上限。反馈 ID 采用 <c>基址 + 设备号</c> 且落在低 4 位段
    /// （0x0380/0x0420/0x0440），超过 15 会与相邻反馈段重叠，因此这里限制为 1..15。
    /// </summary>
    public const byte MaxDeviceId = 0x0F;

    /// <summary>默认轮径（mm）。只用于上位机角度换算，不写回电机。</summary>
    public const double DefaultWheelDiameterMm = 100.0;

    /// <summary>位置原始值单位：1 LSB = 0.1 mm。</summary>
    public const double PositionMillimetersPerLsb = 0.1;

    /// <summary>
    /// 协议表里缺失、必须实机/完整协议确认后才能安全解码 <c>0x0409</c> 的字段。
    /// 界面会把这些字段直接列出来，避免把“未确认”误当成“没数据”。
    /// </summary>
    public static IReadOnlyList<string> MissingReplyLayoutFields { get; } = Array.AsReadOnly(new[]
    {
        "位置字段的长度（2 字节还是 4 字节；真实报文 Data[4..5]=00 00，两种解释结果相同）",
        "位置字段的符号（有符号 s16/s32 还是无符号；6332 在四种解释下都成立）",
        "0x0409 位置回复的 DLC 是否恒为 8（目前只有 1 帧）",
        "设备号/查询码是否恒为 Data[0]/Data[1]（目前只验证设备 01 + 查询码 0x08）",
        "位置字段的字节序（仅 1 帧小端旁证，协议表未定义）",
        "Data[6..7] 的含义（真实报文为 FC FF，用途未知，不做猜测）",
        "多通道电机的主/从通道字段布局（若回复包含两路）",
        "回复中是否还有校验、状态或保留字节",
    });

    /// <summary>构造 <c>0x0408</c> 位置查询的 2 字节负载：<c>[设备号, 0x08]</c>。</summary>
    public static byte[] BuildPositionQuery(byte deviceId)
        => new[] { deviceId, QueryPositionCode };

    /// <summary>目标类型的中文说明（日志/界面用）。</summary>
    public static string DescribeTargetType(MotorTargetType targetType) => targetType switch
    {
        MotorTargetType.Relative => "相对当前位置",
        MotorTargetType.Absolute => "绝对位置",
        MotorTargetType.Zero => "归零",
        MotorTargetType.SetCurrent => "设为当前位置",
        _ => $"未知（0x{(byte)targetType:X2}）",
    };

    /// <summary>
    /// 构造 <c>0x0416</c> 设置目标位置（主通道）的 6 字节负载：
    /// <c>Data[0..3]=目标位置 s32 小端（0.1 mm）</c>、<c>Data[4]=设备号</c>、<c>Data[5]=目标类型</c>。
    /// </summary>
    public static byte[] BuildTargetPosition(int rawPosition, byte deviceId, MotorTargetType targetType)
    {
        byte[] payload = new byte[TargetPositionDlc];
        BinaryPrimitives.WriteInt32LittleEndian(payload.AsSpan(0, 4), rawPosition);
        payload[4] = deviceId;
        payload[5] = (byte)targetType;
        return payload;
    }

    /// <summary>
    /// 构造 <c>0x0413</c> 锁定/释放的 5 字节负载：
    /// <c>Data[0]=0x5A 使能 / 0x4A 释放</c>、<c>Data[1..3]=0</c>、<c>Data[4]=设备号</c>。
    /// </summary>
    public static byte[] BuildEnable(byte deviceId, bool enable)
    {
        byte[] payload = new byte[EnableDlc];
        payload[0] = enable ? EnableCommandEnable : EnableCommandRelease;
        payload[4] = deviceId;
        return payload;
    }

    /// <summary>
    /// 构造 <c>0x0316</c> 参数初始化（8 字节）：
    /// 固定前缀 <c>B4 BF CF AA 00 5F 01</c> + <c>Data[7]=设备号</c>。
    /// <para>⚠️ 补充资料帧，不在 V11 协议表内，无应答，未实机验证。</para>
    /// </summary>
    public static byte[] BuildParameterInit(byte deviceId)
    {
        byte[] payload = new byte[ParameterInitDlc];
        ParameterInitPrefix.CopyTo(payload);
        payload[ParameterInitDlc - 1] = deviceId;
        return payload;
    }

    /// <summary>
    /// 构造 <c>0x0408</c> 配置轮径（4 字节）：
    /// <c>Data[0]=设备号</c>、<c>Data[1]=0x24</c>、<c>Data[2..3]=u16 小端轮径（0.01 mm）</c>。
    /// <para>协议表要求同一条帧连发 3 次，见 <see cref="WheelConfigRepeatCount"/>。</para>
    /// </summary>
    public static byte[] BuildWheelDiameterConfig(byte deviceId, double wheelDiameterMm)
    {
        ushort raw = MillimetersToWheelDiameterRaw(wheelDiameterMm);
        byte[] payload = new byte[WheelConfigDlc];
        payload[0] = deviceId;
        payload[1] = WheelConfigCode;
        BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(2, 2), raw);
        return payload;
    }

    /// <summary>
    /// 轮径（mm）→ 配置帧原始值（0.01 mm/LSB）：<c>round(D / 0.01)</c>。
    /// <para>超出 <c>0.01 .. 655.35 mm</c> 或非有限值时抛异常，不做静默截断。</para>
    /// </summary>
    public static ushort MillimetersToWheelDiameterRaw(double wheelDiameterMm)
    {
        if (!double.IsFinite(wheelDiameterMm) || wheelDiameterMm <= 0.0)
        {
            throw new ArgumentOutOfRangeException(
                nameof(wheelDiameterMm),
                wheelDiameterMm,
                "轮径必须是大于 0 的有限值（mm）。");
        }

        double raw = Math.Round(wheelDiameterMm / WheelDiameterMillimetersPerLsb, MidpointRounding.AwayFromZero);
        if (raw < 1.0 || raw > ushort.MaxValue)
        {
            throw new ArgumentOutOfRangeException(
                nameof(wheelDiameterMm),
                wheelDiameterMm,
                $"轮径必须落在 0.01 .. {MaxWritableWheelDiameterMm:0.##} mm 之间（u16，单位 0.01 mm）。");
        }

        return (ushort)raw;
    }

    /// <summary>配置帧原始值 → 轮径（mm）：<c>raw × 0.01</c>。</summary>
    public static double WheelDiameterRawToMillimeters(ushort raw)
        => raw * WheelDiameterMillimetersPerLsb;

    /// <summary>
    /// 构造 <c>0x0415</c> 设置主速度的 5 字节负载：
    /// <c>Data[0..3]=u32 小端（0.1 mm/s）</c>、<c>Data[4]=设备号</c>。
    /// </summary>
    public static byte[] BuildSpeed(byte deviceId, uint rawSpeed)
    {
        byte[] payload = new byte[SpeedAccelerationDlc];
        BinaryPrimitives.WriteUInt32LittleEndian(payload.AsSpan(0, 4), rawSpeed);
        payload[4] = deviceId;
        return payload;
    }

    /// <summary>
    /// 构造 <c>0x0414</c> 设置主加速度的 5 字节负载：
    /// <c>Data[0..3]=u32 小端（0.1 mm）</c>、<c>Data[4]=设备号</c>。
    /// </summary>
    public static byte[] BuildAcceleration(byte deviceId, uint rawAcceleration)
    {
        byte[] payload = new byte[SpeedAccelerationDlc];
        BinaryPrimitives.WriteUInt32LittleEndian(payload.AsSpan(0, 4), rawAcceleration);
        payload[4] = deviceId;
        return payload;
    }

    /// <summary>
    /// 角度增量 → 位置增量（LSB，0.1 mm）：<c>round(Δ角度 × π × D / 36)</c>。
    /// <para>与 <see cref="PositionDeltaLsbToDegrees"/> 互逆；D=100 mm 时 90° → 785、180° → 1571、360° → 3142、720° → 6283。</para>
    /// </summary>
    public static int DegreesToPositionDelta(double angleDeg, double wheelDiameterMm)
    {
        if (!double.IsFinite(angleDeg))
        {
            throw new ArgumentOutOfRangeException(nameof(angleDeg), angleDeg, "角度必须是有限值（°）。");
        }

        if (!double.IsFinite(wheelDiameterMm) || wheelDiameterMm <= 0.0)
        {
            throw new ArgumentOutOfRangeException(nameof(wheelDiameterMm), wheelDiameterMm, "轮径必须是大于 0 的有限值（mm）。");
        }

        double raw = angleDeg * Math.PI * wheelDiameterMm / (PositionMillimetersPerLsb * 360.0);
        double rounded = Math.Round(raw, MidpointRounding.AwayFromZero);
        if (rounded > int.MaxValue)
        {
            return int.MaxValue;
        }

        if (rounded < int.MinValue)
        {
            return int.MinValue;
        }

        return (int)rounded;
    }

    /// <summary>
    /// 角度容差 → 位置容差（LSB），至少 1 个 LSB，避免轮径很小时容差变成 0 而永远判不到位。
    /// <para>非有限容差（NaN/∞）按 1 个 LSB 处理，避免状态机因为一个坏参数直接抛异常。</para>
    /// </summary>
    public static int AngleToleranceToPositionLsb(double toleranceDeg, double wheelDiameterMm)
    {
        if (!double.IsFinite(toleranceDeg))
        {
            return 1;
        }

        return Math.Max(1, Math.Abs(DegreesToPositionDelta(toleranceDeg, wheelDiameterMm)));
    }

    /// <summary>
    /// 毫米/秒 → 协议速度原始值（单位 0.1 mm/s），负数按 0 处理（协议只有 u32）。
    /// </summary>
    public static uint MillimetersPerSecondToRawSpeed(double millimetersPerSecond)
    {
        if (!double.IsFinite(millimetersPerSecond) || millimetersPerSecond <= 0.0)
        {
            return 0U;
        }

        double raw = Math.Round(millimetersPerSecond * 10.0, MidpointRounding.AwayFromZero);
        return raw >= uint.MaxValue ? uint.MaxValue : (uint)raw;
    }

    /// <summary>
    /// 毫米/秒² → 协议加速度原始值（单位 0.1 mm），负数按 0 处理（协议只有 u32）。
    /// </summary>
    public static uint MillimetersPerSecondSquaredToRawAcceleration(double millimetersPerSecondSquared)
    {
        if (!double.IsFinite(millimetersPerSecondSquared) || millimetersPerSecondSquared <= 0.0)
        {
            return 0U;
        }

        double raw = Math.Round(millimetersPerSecondSquared * 10.0, MidpointRounding.AwayFromZero);
        return raw >= uint.MaxValue ? uint.MaxValue : (uint)raw;
    }

    /// <summary>位置原始值 P → 绝对位移（mm）：<c>P × 0.1</c>。</summary>
    public static double PositionLsbToMillimeters(int rawPosition)
        => rawPosition * PositionMillimetersPerLsb;

    /// <summary>
    /// 位置增量 → 相对起点角度（°）：
    /// <c>(P - P0) × 0.1 × 360 / (π × D)</c>。
    /// <para>电机角度不使用陀螺仪的 4 倍修正系数。</para>
    /// </summary>
    public static double PositionDeltaLsbToDegrees(int rawDelta, double wheelDiameterMm)
    {
        if (!double.IsFinite(wheelDiameterMm) || wheelDiameterMm <= 0.0)
        {
            throw new ArgumentOutOfRangeException(nameof(wheelDiameterMm), wheelDiameterMm, "轮径必须是大于 0 的有限值（mm）。");
        }

        return rawDelta * PositionMillimetersPerLsb * 360.0 / (Math.PI * wheelDiameterMm);
    }

    /// <summary>一个位置单位对应的角度（°）：<c>0.1 × 360 / (π × D)</c>。</summary>
    public static double DegreesPerPositionLsb(double wheelDiameterMm)
        => PositionDeltaLsbToDegrees(1, wheelDiameterMm);

    /// <summary>
    /// 解析剩余位移反馈（<c>0x0420 + 设备号</c>，DLC=8）。
    /// <para>⚠️ 这是“剩余位移”，不是当前位置，不能用于角度换算。</para>
    /// </summary>
    public static bool TryDecodeRemainingDisplacement(ReadOnlySpan<byte> data, out int mainChannelRaw, out int slaveChannelRaw)
    {
        mainChannelRaw = 0;
        slaveChannelRaw = 0;
        if (data.Length < 8)
        {
            return false;
        }

        mainChannelRaw = BinaryPrimitives.ReadInt32LittleEndian(data[..4]);
        slaveChannelRaw = BinaryPrimitives.ReadInt32LittleEndian(data.Slice(4, 4));
        return true;
    }

    /// <summary>
    /// 解析力矩反馈（<c>0x0440 + 设备号</c>）：协议表规定 <c>Data[2..3]</c> 为 s16 力矩值。
    /// <para>实测该帧 DLC 为 8（协议表写的是 4），因此这里接受 4..8 字节，只读取 Data[2..3]。</para>
    /// </summary>
    public static bool TryDecodeTorque(ReadOnlySpan<byte> data, out short torqueRaw)
    {
        torqueRaw = 0;
        if (data.Length < 4)
        {
            return false;
        }

        torqueRaw = BinaryPrimitives.ReadInt16LittleEndian(data.Slice(2, 2));
        return true;
    }

    /// <summary>判断 ID 是否落在“剩余位移”反馈段（0x0420..0x042F）。</summary>
    public static bool IsRemainingDisplacementId(uint id)
        => id is >= 0x0420U and <= 0x042FU;

    /// <summary>判断 ID 是否落在“力矩”反馈段（0x0440..0x044F）。</summary>
    public static bool IsTorqueFeedbackId(uint id)
        => id is >= 0x0440U and <= 0x044FU;

    /// <summary>判断 ID 是否落在“心跳”反馈段（0x0380..0x038F）。</summary>
    public static bool IsHeartbeatId(uint id)
        => id is >= 0x0380U and <= 0x038FU;

    /// <summary>判断 ID 是否落在错误信息上报段（0x0100..0x017F）。</summary>
    public static bool IsErrorId(uint id)
        => id is >= ErrorIdMin and <= ErrorIdMax;

    /// <summary>把错误码翻译成协议表里的中文含义；未知码显示为“未知错误码”。</summary>
    public static string DescribeErrorCode(ushort code) => code switch
    {
        0x0102 => "任务未完成，未达到目标点",
        0x0104 => "堵转",
        0x0105 => "超温",
        0x0107 => "欠压",
        0x0108 => "断线",
        _ => "未知错误码",
    };

    /// <summary>
    /// 本工程默认的位置回复解码器。
    /// <para>
    /// ⚠️ **待进一步实机验证的布局**：依据是 2026-10-17 抓到的一帧真实回复
    /// <c>0x0408 [01 08]</c> → <c>0x0409 / DLC=8 / 01 08 BC 18 00 00 FC FF</c>
    /// （解析值 6332，即 633.2 mm）。
    /// <list type="bullet">
    /// <item><c>Data[0]</c> = 设备号：该帧与请求设备号 01 一致，但只验证过设备 01；</item>
    /// <item><c>Data[1]</c> = 查询码：该帧为 0x08，与位置查询一致，用于区分共享 0x0409 的
    /// 温度/状态/版本回复；</item>
    /// <item><c>Data[2..5]</c> = int32 小端位置，单位 0.1 mm；按大端解释会得到 −113.9 km，
    /// 因此小端有强旁证；</item>
    /// <item><c>Data[6..7]</c> = 含义未知（本帧 <c>FC FF</c>），**不参与计算**。</item>
    /// </list>
    /// </para>
    /// <para>
    /// ⚠️ 单帧样本**不能**完全确认位置字段的长度与符号：<c>Data[4..5] = 00 00</c>，
    /// 2 字节与 4 字节解释都得 6332；6332 在 s16/u16/s32/u32 下都成立。
    /// 因此后续仍需用“已知角度运动前后、反向运动、位置超过 0x7FFF”的真实回复核对，
    /// 详见 <c>docs/motor-0x0409-real-frames.md</c>。
    /// </para>
    /// <para>
    /// 普通真实连接（以及模拟测试）都走这一个入口，不做“只在测试里生效”的分支。
    /// 若回复**不含设备号**，把 <c>deviceIdOffset</c> 改为 <c>-1</c> 按单设备处理。
    /// </para>
    /// </summary>
    public static IMotorPositionDecoder CreateDefaultPositionDecoder()
        => new MotorPositionReplyLayout(
            expectedDlc: 8,
            deviceIdOffset: 0,
            queryCodeOffset: 1,
            positionOffset: 2,
            positionLength: 4,
            positionIsLittleEndian: true,
            positionIsSigned: true,
            description: "实机 0x0409 [01 08 BC 18 00 00 FC FF]：Data[0]=设备号、Data[1]=查询码 0x08、" +
                         "Data[2..5]=int32 小端位置（0.1 mm）；Data[6..7] 含义未知不解析。" +
                         "⚠️ 仅 1 帧样本，位置字段长度/有符号性仍待更多实机报文验证");
}

/// <summary>
/// <c>0x0416</c> 的 <c>Data[5]</c> 目标类型（《电机 CAN 通信协议 V11》原表）。
/// </summary>
public enum MotorTargetType : byte
{
    /// <summary>相对当前位置（<c>0x00</c>）。往复运动使用这一种。</summary>
    Relative = 0x00,

    /// <summary>绝对目标位置（<c>0x05</c>）。</summary>
    Absolute = 0x05,

    /// <summary>归零（<c>0x0A</c>）。本工程不使用：清零只在上位机显示层完成。</summary>
    Zero = 0x0A,

    /// <summary>把当前位置设为指定值（<c>0x0B</c>）。</summary>
    SetCurrent = 0x0B,
}
