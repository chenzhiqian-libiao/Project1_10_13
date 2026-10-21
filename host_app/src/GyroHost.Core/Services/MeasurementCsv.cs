using System.Globalization;
using System.Text;
using GyroHost.Core.Protocol;

namespace GyroHost.Core.Services;

/// <summary>一次 CSV 导出的统计结果。</summary>
/// <param name="AngleRows">写出的角度（0x321）行数。</param>
/// <param name="RateRows">写出的角速度（0x323）行数。</param>
/// <param name="RawRows">写出的原始采样（0x324+0x325）行数。</param>
/// <param name="MotorRows">写出的电机位置行数。</param>
/// <param name="FirstTimeSeconds">四路中最小的相对时间（秒）；无数据时为 <see cref="double.NaN"/>。</param>
/// <param name="LastTimeSeconds">四路中最大的相对时间（秒）；无数据时为 <see cref="double.NaN"/>。</param>
public readonly record struct CsvExportSummary(
    int AngleRows,
    int RateRows,
    int RawRows,
    int MotorRows,
    double FirstTimeSeconds,
    double LastTimeSeconds);

/// <summary>
/// 把内存中的角度（0x321）、角速度（0x323）、原始采样（0x324+0x325）与电机位置历史写成长表 CSV。
/// <para>
/// 刻意只做格式化、不碰文件系统：调用方负责打开文件/流，这里只写 <see cref="TextWriter"/>，
/// 这样格式与归并顺序可以被单元测试完整覆盖。
/// </para>
/// <para>
/// 各路数据来自互相独立的消息（陀螺仪 0x321/0x323/0x324+0x325 与电机位置轮询），时间戳各不相同，因此
/// <b>不做插值、不补旧值</b>：每个原始采样写一行，用 <c>kind</c> 列区分来源，缺的列留空。
/// </para>
/// <para>
/// 原始采样行（<c>kind=raw</c>）是 0x324 与 0x325 配对后的结果：<c>raw_code</c>、
/// <c>sample_tick_ms</c>、<c>bias_dps</c> 与 <c>rate_dps</c>（滤波后值）写在同一行，
/// 便于离线复算滤波器。配对失败时整行不写（<c>0x323</c> 仍会记录滤波值）。
/// </para>
/// <para>
/// 电机角度单独占 <c>motor_angle_deg</c> 一列，而不是复用 <c>angle_deg</c>：
/// 两种角度写在同列会让后处理必须按 <c>kind</c> 分流，容易读错。
/// </para>
/// </summary>
public static class MeasurementCsv
{
    /// <summary>表头。列顺序即数据行的字段顺序。</summary>
    public const string Header =
        "relative_time_s,kind,angle_deg,rate_dps,temperature_c,status_hex,status_flags,interval_ms,break_before,motor_angle_deg,raw_code,sample_tick_ms,bias_dps";

    /// <summary>行尾固定用 CRLF，与 Windows Excel 一致，不受运行平台影响。</summary>
    private const string LineEnding = "\r\n";

    /// <summary>导出的文件编码：UTF-8 with BOM（Excel 打开中文环境才不乱码）。</summary>
    private static readonly UTF8Encoding Utf8WithBom = new(encoderShouldEmitUTF8Identifier: true);

    private static readonly (GyroStatus Flag, string Name)[] StatusFlagNames =
    {
        (GyroStatus.AngleValid, "AngleValid"),
        (GyroStatus.TemperatureValid, "TemperatureValid"),
        (GyroStatus.CalibrationDone, "CalibrationDone"),
        (GyroStatus.CalibrationFailed, "CalibrationFailed"),
        (GyroStatus.StillLocked, "StillLocked"),
        (GyroStatus.DegradedCalibration, "DegradedCalibration"),
        (GyroStatus.ContinuousIntegration, "ContinuousIntegration"),
    };

    private const GyroStatus KnownStatusMask =
        GyroStatus.AngleValid |
        GyroStatus.TemperatureValid |
        GyroStatus.CalibrationDone |
        GyroStatus.CalibrationFailed |
        GyroStatus.StillLocked |
        GyroStatus.DegradedCalibration |
        GyroStatus.ContinuousIntegration;

    /// <summary>
    /// 按时间升序归并各路数据并写出 CSV（含表头）。
    /// <para>同一时刻的写出顺序固定为 角度 → 角速度 → 原始采样 → 电机，保证行序可复现；顺序只影响行序，不增删行。</para>
    /// </summary>
    public static CsvExportSummary Write(
        TextWriter writer,
        IReadOnlyList<DataSample>? data,
        IReadOnlyList<RateSample>? rate,
        IReadOnlyList<MotorPositionSample>? motor,
        IReadOnlyList<RawSample>? raw = null)
    {
        ArgumentNullException.ThrowIfNull(writer);

        int angleCount = data?.Count ?? 0;
        int rateCount = rate?.Count ?? 0;
        int rawCount = raw?.Count ?? 0;
        int motorCount = motor?.Count ?? 0;

        writer.Write(Header);
        writer.Write(LineEnding);

        int angleRows = 0;
        int rateRows = 0;
        int rawRows = 0;
        int motorRows = 0;
        double firstTime = double.NaN;
        double lastTime = double.NaN;

        int i = 0;
        int j = 0;
        int l = 0;
        int k = 0;
        while (i < angleCount || j < rateCount || l < rawCount || k < motorCount)
        {
            // 电机必须按它「真正写出去的那个时间轴」比较，否则会出现
            // 「按电机私有时间排序、却写出全局时间」的自相矛盾行序。
            double motorTime = k < motorCount ? motor![k].ChartTimeSeconds : double.NaN;
            double rawTime = l < rawCount ? raw![l].TimeSeconds : double.NaN;

            bool takeAngle =
                i < angleCount &&
                (j >= rateCount || data![i].TimeSeconds <= rate![j].TimeSeconds) &&
                (l >= rawCount || data![i].TimeSeconds <= rawTime) &&
                (k >= motorCount || data![i].TimeSeconds <= motorTime);

            if (takeAngle)
            {
                DataSample sample = data![i];
                WriteAngleRow(writer, sample);
                TrackTime(sample.TimeSeconds, ref firstTime, ref lastTime);
                i++;
                angleRows++;
                continue;
            }

            bool takeRate =
                j < rateCount &&
                (l >= rawCount || rate![j].TimeSeconds <= rawTime) &&
                (k >= motorCount || rate![j].TimeSeconds <= motorTime);

            if (takeRate)
            {
                RateSample sample = rate![j];
                WriteRateRow(writer, sample);
                TrackTime(sample.TimeSeconds, ref firstTime, ref lastTime);
                j++;
                rateRows++;
                continue;
            }

            bool takeRaw =
                l < rawCount &&
                (k >= motorCount || raw![l].TimeSeconds <= motorTime);

            if (takeRaw)
            {
                RawSample sample = raw![l];
                WriteRawRow(writer, sample);
                TrackTime(sample.TimeSeconds, ref firstTime, ref lastTime);
                l++;
                rawRows++;
                continue;
            }

            MotorPositionSample motorSample = motor![k];
            WriteMotorRow(writer, motorSample);
            TrackTime(motorSample.ChartTimeSeconds, ref firstTime, ref lastTime);
            k++;
            motorRows++;
        }

        return new CsvExportSummary(angleRows, rateRows, rawRows, motorRows, firstTime, lastTime);
    }

    /// <summary>
    /// 直接写入 CSV 文件（覆盖同名文件，UTF-8 with BOM）。
    /// <para>
    /// 界面的「保存本段数据」走这里，测试也走这里，保证 BOM 与编码不是未被覆盖的代码。
    /// </para>
    /// </summary>
    public static CsvExportSummary WriteFile(
        string path,
        IReadOnlyList<DataSample>? data,
        IReadOnlyList<RateSample>? rate,
        IReadOnlyList<MotorPositionSample>? motor,
        IReadOnlyList<RawSample>? raw = null)
    {
        ArgumentException.ThrowIfNullOrEmpty(path);

        using FileStream stream = new(path, FileMode.Create, FileAccess.Write, FileShare.Read);
        using StreamWriter writer = new(stream, Utf8WithBom);
        return Write(writer, data, rate, motor, raw);
    }

    private static void TrackTime(double seconds, ref double first, ref double last)
    {
        if (double.IsNaN(first) || seconds < first)
        {
            first = seconds;
        }

        if (double.IsNaN(last) || seconds > last)
        {
            last = seconds;
        }
    }

    private static void WriteAngleRow(TextWriter writer, DataSample sample)
    {
        writer.Write(FormatTime(sample.TimeSeconds));
        writer.Write(",angle,");
        writer.Write(FormatFixed(sample.AngleDeg, "F5"));
        writer.Write(',');
        // rate_dps 留空
        writer.Write(',');
        writer.Write(FormatFixed(sample.TemperatureC, "F2"));
        writer.Write(',');
        writer.Write(FormatStatusHex(sample.Status));
        writer.Write(',');
        writer.Write(FormatStatusFlags(sample.Status));
        writer.Write(',');
        writer.Write(sample.IntervalMs.ToString(CultureInfo.InvariantCulture));
        writer.Write(',');
        writer.Write(sample.BreakBefore ? '1' : '0');
        writer.Write(',');
        // motor_angle_deg / raw_code / sample_tick_ms / bias_dps 留空
        writer.Write(",,,");
        writer.Write(LineEnding);
    }

    private static void WriteRateRow(TextWriter writer, RateSample sample)
    {
        writer.Write(FormatTime(sample.TimeSeconds));
        writer.Write(",rate,");
        // angle_deg 留空
        writer.Write(',');
        writer.Write(FormatFixed(sample.RateDps, "F5"));
        // temperature_c / status_hex / status_flags / interval_ms / break_before / motor_angle_deg
        // raw_code / sample_tick_ms / bias_dps 全部留空
        writer.Write(",,,,,,,,,");
        writer.Write(LineEnding);
    }

    /// <summary>
    /// 原始采样行：<c>kind=raw</c>，把 0x324+0x325 配对的四项写在同一行 ——
    /// <c>raw_code</c>（原始 SPI 码）、<c>sample_tick_ms</c>（板端实际采样时刻）、
    /// <c>bias_dps</c>（当前零偏）、<c>rate_dps</c>（滤波后值，与 0x323 同源）。
    /// </summary>
    private static void WriteRawRow(TextWriter writer, RawSample sample)
    {
        writer.Write(FormatTime(sample.TimeSeconds));
        writer.Write(",raw,");
        // angle_deg 留空
        writer.Write(',');
        writer.Write(FormatFixed(sample.FilteredDps, "F5"));
        // temperature_c / status_hex / status_flags / interval_ms / break_before / motor_angle_deg 留空
        writer.Write(",,,,,,,");
        writer.Write(sample.RawCode.ToString(CultureInfo.InvariantCulture));
        writer.Write(',');
        writer.Write(sample.SampleTickMs.ToString(CultureInfo.InvariantCulture));
        writer.Write(',');
        writer.Write(FormatFixed(sample.BiasDps, "F3"));
        writer.Write(LineEnding);
    }

    /// <summary>
    /// 电机位置行：<c>kind=motor</c>，值写在最后一列 <c>motor_angle_deg</c>。
    /// <para>
    /// 时间戳取 <see cref="MotorPositionSample.ChartTimeSeconds"/>：那是与陀螺仪共用的全局时间轴，
    /// 电机行因此能和 angle / rate 行按同一个 <c>relative_time_s</c> 对齐。
    /// </para>
    /// <para>
    /// ⚠️ 不能用 <see cref="MotorPositionSample.TimeSeconds"/>：那是<b>电机自己的时间轴</b>
    /// （「电机显示归零」后从 0 重新计时），和陀螺仪的 0 点不是同一个。
    /// 实测用它导出时电机行落在 0.11–225.7 s，而陀螺仪是 0–234.6 s，
    /// 两边的运动时刻错开最多 6.9 s，完全无法逐点对照。
    /// </para>
    /// </summary>
    private static void WriteMotorRow(TextWriter writer, MotorPositionSample sample)
    {
        writer.Write(FormatTime(sample.ChartTimeSeconds));
        writer.Write(",motor");
        // angle_deg / rate_dps / temperature_c / status_hex / status_flags / interval_ms / break_before 共 7 列留空
        writer.Write(",,,,,,,,");
        writer.Write(FormatFixed(sample.RelativeAngleDeg, "F5"));
        // raw_code / sample_tick_ms / bias_dps 留空
        writer.Write(",,,");
        writer.Write(LineEnding);
    }

    private static string FormatTime(double seconds) => FormatFixed(seconds, "F6");

    private static string FormatFixed(double value, string format)
        => value.ToString(format, CultureInfo.InvariantCulture);

    private static string FormatStatusHex(GyroStatus status)
        => "0x" + ((byte)status).ToString("X2", CultureInfo.InvariantCulture);

    /// <summary>
    /// 状态位转成可读名字，用 <c>|</c> 分隔（不能用逗号，否则 CSV 会被拆列）。
    /// 未知位保留成 <c>Unknown_0x40</c> 形式，避免静默丢信息。
    /// </summary>
    private static string FormatStatusFlags(GyroStatus status)
    {
        if (status == GyroStatus.None)
        {
            return string.Empty;
        }

        StringBuilder builder = new();
        foreach ((GyroStatus flag, string name) in StatusFlagNames)
        {
            if ((status & flag) == 0)
            {
                continue;
            }

            if (builder.Length > 0)
            {
                builder.Append('|');
            }

            builder.Append(name);
        }

        GyroStatus unknown = (GyroStatus)((byte)status & (byte)~KnownStatusMask);
        if (unknown != GyroStatus.None)
        {
            if (builder.Length > 0)
            {
                builder.Append('|');
            }

            builder.Append("Unknown_0x");
            builder.Append(((byte)unknown).ToString("X2", CultureInfo.InvariantCulture));
        }

        return builder.ToString();
    }
}
