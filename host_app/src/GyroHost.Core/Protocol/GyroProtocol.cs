using System.Buffers.Binary;

namespace GyroHost.Core.Protocol;

/// <summary>
/// 固件实际使用的协议常量与编解码。
/// 与 Project1_10_13.c 的 task_can_tx() / task_can_rx() 一一对应，
/// 服务端（板端）为唯一权威，本文件不得自行增删字段含义。
/// <para>
/// 全部多字节数值：小端序、有符号整数，不是 IEEE 浮点。
/// </para>
/// </summary>
public static class GyroProtocol
{
    /// <summary>0x321 / DLC=8：角度 + 温度 + 状态位 + 采样间隔。</summary>
    public const uint DataId = 0x321U;

    /// <summary>0x322 / DLC=1：命令帧。</summary>
    public const uint CommandId = 0x322U;

    /// <summary>0x323 / DLC=4：滤波后角速度。</summary>
    public const uint RateId = 0x323U;

    /// <summary>0x324 / DLC=8：原始 SPI 码 + 实际采样时刻 + 当前零偏。</summary>
    public const uint RawId = 0x324U;

    /// <summary>0x325 / DLC=8：滤波后角速度 + 实际采样时刻（用于与 0x324 配对）。</summary>
    public const uint RawDetailId = 0x325U;

    public const int DataDlc = 8;
    public const int RateDlc = 4;
    public const int CommandDlc = 1;
    public const int RawDlc = 8;
    public const int RawDetailDlc = 8;

    /// <summary>0x322 数据 = 0x01：重新零偏校准（要求保持静止）。</summary>
    public const byte CommandRecalibrate = 0x01;

    /// <summary>0x322 数据 = 0x02：板端角度清零。</summary>
    public const byte CommandZeroAngle = 0x02;

    /// <summary>固件校准：60 秒估计 + 20 秒独立静止验证，完成状态为准。</summary>
    public const double CalibrationDurationSeconds = 80.0;

    /// <summary>
    /// 角度比例：原始值 × 0.00001°（固件 <c>GyroScaleToInt32(angle, CAN_GYRO_ANGLE_SCALE)</c>）。
    /// <para>
    /// 固件 2026-10-16 起把比例从 100（0.01°）提高到 100000（0.00001°），
    /// 让上位机的 5 位小数显示有真实分辨率支撑。若固件改回旧比例，
    /// 这里必须同步改回，否则角度会整体差 1000 倍。
    /// </para>
    /// </summary>
    public const double AngleLsbPerDegree = 100000.0;

    /// <summary>温度比例：原始值 × 0.01℃（固件 GyroScaleToInt16(temp, 100)）。</summary>
    public const double TemperatureLsbPerDegree = 100.0;

    /// <summary>
    /// 角速度比例：原始值 × 0.00001°/s（固件 <c>CAN_GYRO_RATE_SCALE = 100000</c>）。
    /// <para>
    /// 固件 2026-10-16 起把比例从 1000（0.001°/s）提高到 100000（0.00001°/s），
    /// 与角度一样对齐 5 位小数显示。
    /// </para>
    /// <para>
    /// ⚠️ 注意这个分辨率<b>远细于传感器本身</b>：XV7001BB 实测灵敏度 70 LSB/(°/s)，
    /// 原始码 1 个 LSB ≈ 0.0143°/s，比协议 1 个 LSB 粗约 1429 倍。
    /// 所以 5 位小数里后面几位反映的是零偏补偿量与滤波输出（连续量），
    /// 不是传感器能独立分辨的量级——看绝对值请以 0.001°/s 量级为准。
    /// </para>
    /// </summary>
    public const double RateLsbPerDps = 100000.0;

    /// <summary>
    /// 零偏比例：原始值 × 0.001°/s（固件 <c>CAN_GYRO_BIAS_SCALE = 1000</c>）。
    /// <para>0x324 的 D6–D7 是"当前零偏"，已经包含上电校准均值和静止时的慢跟踪修正。</para>
    /// </summary>
    public const double BiasLsbPerDps = 1000.0;

    /// <summary>
    /// 原始码到角速度的换算：70 LSB/(°/s)（固件 <c>XV7001BB_SCALE_16BIT</c>）。
    /// <para>
    /// 仅用于把 <c>raw_code</c> 还原成物理角速度做离线核对；板端上报的
    /// <c>rate_dps</c> 已经是换算并滤波后的值，不要重复乘除。
    /// </para>
    /// </summary>
    public const double RawLsbPerDps = 70.0;

    /// <summary>
    /// 板端发送周期（固件 <c>CAN_TX_PERIOD_MS</c>）。
    /// <para>
    /// 2026-10-20 由 100 ms 改为 10 ms：积分本来就在 100 Hz 跑，上报却只有 10 Hz，
    /// 一次 180° 往返的加/减速斜坡只剩 2 个采样点，起动与停止丢掉的那 0.2~0.5°
    /// 在曲线上分辨不出来。这里只影响模拟器节奏与「一帧多长时间算超时」的量级判断，
    /// 真实时间轴始终来自字节到达时间。
    /// </para>
    /// </summary>
    public const int NominalFramePeriodMs = 10;

    /// <summary>
    /// 解析 0x321：D0..D3 角度 int32、D4..D5 温度 int16、D6 状态、D7 采样间隔。
    /// </summary>
    public static bool TryDecodeData(ReadOnlySpan<byte> data, out DataPayload payload)
    {
        payload = default;
        if (data.Length < DataDlc)
        {
            return false;
        }

        int angleRaw = BinaryPrimitives.ReadInt32LittleEndian(data[..4]);
        short temperatureRaw = BinaryPrimitives.ReadInt16LittleEndian(data.Slice(4, 2));

        payload = new DataPayload(
            AngleDeg: angleRaw / AngleLsbPerDegree,
            TemperatureC: temperatureRaw / TemperatureLsbPerDegree,
            Status: (GyroStatus)data[6],
            IntervalMs: data[7]);
        return true;
    }

    /// <summary>解析 0x323：D0..D3 滤波后角速度 int32，单位 0.00001°/s。</summary>
    public static bool TryDecodeRate(ReadOnlySpan<byte> data, out double rateDps)
    {
        rateDps = 0.0;
        if (data.Length < RateDlc)
        {
            return false;
        }

        int rateRaw = BinaryPrimitives.ReadInt32LittleEndian(data[..4]);
        rateDps = rateRaw / RateLsbPerDps;
        return true;
    }

    /// <summary>
    /// 解析 0x324：D0..D1 原始码 int16、D2..D5 采样时刻 uint32（ms）、
    /// D6..D7 零偏 int16，单位 0.001°/s。
    /// </summary>
    public static bool TryDecodeRaw(ReadOnlySpan<byte> data, out RawPayload payload)
    {
        payload = default;
        if (data.Length < RawDlc)
        {
            return false;
        }

        short rawCode = BinaryPrimitives.ReadInt16LittleEndian(data[..2]);
        uint sampleTickMs = BinaryPrimitives.ReadUInt32LittleEndian(data.Slice(2, 4));
        short biasRaw = BinaryPrimitives.ReadInt16LittleEndian(data.Slice(6, 2));

        payload = new RawPayload(rawCode, sampleTickMs, biasRaw / BiasLsbPerDps);
        return true;
    }

    /// <summary>
    /// 解析 0x325：D0..D3 滤波后角速度 int32，单位 0.00001°/s；
    /// D4..D7 采样时刻 uint32（ms），必须与 0x324 的 D2..D5 相同才算同一条记录。
    /// </summary>
    public static bool TryDecodeRawDetail(ReadOnlySpan<byte> data, out RawDetailPayload payload)
    {
        payload = default;
        if (data.Length < RawDetailDlc)
        {
            return false;
        }

        int rateRaw = BinaryPrimitives.ReadInt32LittleEndian(data[..4]);
        uint sampleTickMs = BinaryPrimitives.ReadUInt32LittleEndian(data.Slice(4, 4));

        payload = new RawDetailPayload(rateRaw / RateLsbPerDps, sampleTickMs);
        return true;
    }

    /// <summary>把原始码值换算回协议整数，仅用于测试与模拟器。</summary>
    public static int EncodeAngleRaw(double angleDeg) => (int)Math.Round(angleDeg * AngleLsbPerDegree, MidpointRounding.AwayFromZero);

    /// <summary>把温度换算回协议整数，仅用于测试与模拟器。</summary>
    public static short EncodeTemperatureRaw(double temperatureC) => (short)Math.Round(temperatureC * TemperatureLsbPerDegree, MidpointRounding.AwayFromZero);

    /// <summary>把角速度换算回协议整数，仅用于测试与模拟器。</summary>
    public static int EncodeRateRaw(double rateDps) => (int)Math.Round(rateDps * RateLsbPerDps, MidpointRounding.AwayFromZero);

    /// <summary>按协议打包一条 0x321 帧（模拟器与测试用）。</summary>
    public static byte[] BuildDataFrame(double angleDeg, double temperatureC, GyroStatus status, int intervalMs)
    {
        byte[] data = new byte[DataDlc];
        BinaryPrimitives.WriteInt32LittleEndian(data.AsSpan(0, 4), EncodeAngleRaw(angleDeg));
        BinaryPrimitives.WriteInt16LittleEndian(data.AsSpan(4, 2), EncodeTemperatureRaw(temperatureC));
        data[6] = (byte)status;
        data[7] = (byte)Math.Clamp(intervalMs, 0, 255);
        return data;
    }

    /// <summary>按协议打包一条 0x323 帧（模拟器与测试用）。</summary>
    public static byte[] BuildRateFrame(double rateDps)
    {
        byte[] data = new byte[RateDlc];
        BinaryPrimitives.WriteInt32LittleEndian(data.AsSpan(0, 4), EncodeRateRaw(rateDps));
        return data;
    }

    /// <summary>按协议打包一条 0x324 帧（模拟器与测试用）。</summary>
    public static byte[] BuildRawFrame(short rawCode, uint sampleTickMs, double biasDps)
    {
        byte[] data = new byte[RawDlc];
        double biasScaled = Math.Round(biasDps * BiasLsbPerDps, MidpointRounding.AwayFromZero);
        short biasRaw = (short)Math.Clamp(biasScaled, short.MinValue, short.MaxValue);
        BinaryPrimitives.WriteInt16LittleEndian(data.AsSpan(0, 2), rawCode);
        BinaryPrimitives.WriteUInt32LittleEndian(data.AsSpan(2, 4), sampleTickMs);
        BinaryPrimitives.WriteInt16LittleEndian(data.AsSpan(6, 2), biasRaw);
        return data;
    }

    /// <summary>按协议打包一条 0x325 帧（模拟器与测试用）。</summary>
    public static byte[] BuildRawDetailFrame(double filteredRateDps, uint sampleTickMs)
    {
        byte[] data = new byte[RawDetailDlc];
        BinaryPrimitives.WriteInt32LittleEndian(data.AsSpan(0, 4), EncodeRateRaw(filteredRateDps));
        BinaryPrimitives.WriteUInt32LittleEndian(data.AsSpan(4, 4), sampleTickMs);
        return data;
    }

    /// <summary>0x322 命令帧载荷。</summary>
    public static byte[] BuildCommandFrame(byte command) => new[] { command };
}

/// <summary>0x321 解码结果。</summary>
public readonly record struct DataPayload(
    double AngleDeg,
    double TemperatureC,
    GyroStatus Status,
    int IntervalMs);

/// <summary>0x324 解码结果（原始 SPI 码、实际采样时刻、当前零偏）。</summary>
/// <param name="RawCode">陀螺仪 16 bit 原始码（补码），除以 <see cref="GyroProtocol.RawLsbPerDps"/> 得 °/s。</param>
/// <param name="SampleTickMs">板端读完 SPI 那一刻的 HAL tick（ms）。</param>
/// <param name="BiasDps">当前零偏（°/s），已包含校准均值与静止慢跟踪。</param>
public readonly record struct RawPayload(
    short RawCode,
    uint SampleTickMs,
    double BiasDps);

/// <summary>0x325 解码结果（滤波后角速度 + 用于配对的采样时刻）。</summary>
/// <param name="FilteredRateDps">滤波后角速度（°/s），与 0x323 同值。</param>
/// <param name="SampleTickMs">采样时刻（ms），应与同一次上报的 0x324 相同。</param>
public readonly record struct RawDetailPayload(
    double FilteredRateDps,
    uint SampleTickMs);

/// <summary>
/// 0x321 的 D6 状态位，含义取自固件 task_can_tx()。
/// </summary>
[Flags]
public enum GyroStatus : byte
{
    /// <summary>没有任何标志。</summary>
    None = 0x00,

    /// <summary>bit0：角度/采样有效（固件要求 dataValid 且校准完成）。</summary>
    AngleValid = 0x01,

    /// <summary>bit1：温度有效。</summary>
    TemperatureValid = 0x02,

    /// <summary>bit2：零偏校准完成。</summary>
    CalibrationDone = 0x04,

    /// <summary>bit3：零偏校准失败。</summary>
    CalibrationFailed = 0x08,

    /// <summary>bit4：判定静止；只有未设置 ContinuousIntegration 时角度才被冻结。</summary>
    StillLocked = 0x10,

    /// <summary>
    /// bit5：零偏校准为降级模式——校准期间峰峰值超限（多半是桌面震动/碰到线），
    /// 但均值仍在板端允许范围内，于是零偏取均值，角度<b>照常有效</b>，
    /// 只是首次零偏精度略低，静止后由板端偏置慢跟踪继续收敛。
    /// </summary>
    DegradedCalibration = 0x20,

    /// <summary>bit6：连续积分模式，静止判定不冻结角度，保留真实漂移。</summary>
    ContinuousIntegration = 0x40,
}
