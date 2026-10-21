using System.Globalization;
using System.Text;
using GyroHost.Core.Protocol;
using GyroHost.Core.Services;

namespace GyroHost.Tests;

/// <summary>
/// <see cref="MeasurementCsv"/> 的格式契约测试。
/// <para>
/// 这些断言直接对应 README「5.5 保存本段数据（CSV）」里的列定义：
/// 改格式必须先改文档与这些用例，避免导出文件悄悄变形。
/// </para>
/// </summary>
public sealed class MeasurementCsvTests
{
    private static DataSample Angle(
        double timeSeconds,
        double angleDeg = 1.0,
        double temperatureC = 25.0,
        GyroStatus status = GyroStatus.AngleValid | GyroStatus.TemperatureValid,
        int intervalMs = 100,
        bool breakBefore = false)
        => new(TimeSpan.FromSeconds(timeSeconds), timeSeconds, angleDeg, temperatureC, status, intervalMs, breakBefore);

    private static RateSample Rate(double timeSeconds, double rateDps = 0.5)
        => new(TimeSpan.FromSeconds(timeSeconds), timeSeconds, rateDps);

    private static RawSample Raw(
        double timeSeconds,
        short rawCode = 1234,
        uint sampleTickMs = 123456U,
        double biasDps = 0.012,
        double filteredDps = 1.5)
        => new(TimeSpan.FromSeconds(timeSeconds), timeSeconds, rawCode, sampleTickMs, biasDps, filteredDps);

    /// <summary>
    /// 构造电机采样。
    /// <para>
    /// 默认让 <c>TimeSeconds</c>（电机<b>私有</b>时间轴，「电机显示归零」后从 0 重算）
    /// 比 <c>ChartTimeSeconds</c>（与陀螺仪<b>共用</b>的全局时间轴）大 5 s：
    /// 万一导出误用了私有轴，断言会立刻失败，而不是"看起来都合理"。
    /// </para>
    /// </summary>
    private static MotorPositionSample Motor(
        double chartTimeSeconds,
        double relativeAngleDeg = 0.0,
        double privateTimeOffsetSeconds = 5.0)
        => new(
            TimeSpan.FromSeconds(chartTimeSeconds),
            chartTimeSeconds + privateTimeOffsetSeconds,
            chartTimeSeconds,
            (int)Math.Round(relativeAngleDeg, MidpointRounding.AwayFromZero),
            relativeAngleDeg * 0.1,
            relativeAngleDeg,
            false);

    private static (string Text, CsvExportSummary Summary) Export(
        IReadOnlyList<DataSample>? data,
        IReadOnlyList<RateSample>? rate,
        IReadOnlyList<MotorPositionSample>? motor = null,
        IReadOnlyList<RawSample>? raw = null)
    {
        using StringWriter writer = new(CultureInfo.InvariantCulture);
        CsvExportSummary summary = MeasurementCsv.Write(writer, data, rate, motor, raw);
        return (writer.ToString(), summary);
    }

    private static string[] Lines(string text)
        => text.Split("\r\n", StringSplitOptions.RemoveEmptyEntries);

    private static string[] Fields(string line)
        => line.Split(',');

    [Fact]
    public void Header_MatchesDocumentedColumnOrder()
    {
        (string text, _) = Export(null, null);

        Assert.Equal(
            "relative_time_s,kind,angle_deg,rate_dps,temperature_c,status_hex,status_flags,interval_ms,break_before,motor_angle_deg,raw_code,sample_tick_ms,bias_dps",
            Lines(text)[0]);
        Assert.Equal(MeasurementCsv.Header, Lines(text)[0]);
    }

    [Fact]
    public void EmptyInput_WritesHeaderOnly()
    {
        (string text, CsvExportSummary summary) = Export(
            Array.Empty<DataSample>(),
            Array.Empty<RateSample>(),
            Array.Empty<MotorPositionSample>());

        Assert.Single(Lines(text));
        Assert.Equal(0, summary.AngleRows);
        Assert.Equal(0, summary.RateRows);
        Assert.Equal(0, summary.RawRows);
        Assert.Equal(0, summary.MotorRows);
        Assert.True(double.IsNaN(summary.FirstTimeSeconds));
        Assert.True(double.IsNaN(summary.LastTimeSeconds));
    }

    [Fact]
    public void AngleRow_UsesProtocolPrecisionAndFlags()
    {
        (string text, CsvExportSummary summary) = Export(
            new[] { Angle(0.1, 1.23456, 25.31, GyroStatus.AngleValid | GyroStatus.TemperatureValid | GyroStatus.StillLocked) },
            null);

        string[] fields = Fields(Lines(text)[1]);

        Assert.Equal(13, fields.Length);
        Assert.Equal("0.100000", fields[0]);
        Assert.Equal("angle", fields[1]);
        Assert.Equal("1.23456", fields[2]);
        Assert.Equal(string.Empty, fields[3]);
        Assert.Equal("25.31", fields[4]);
        Assert.Equal("0x13", fields[5]);
        Assert.Equal("AngleValid|TemperatureValid|StillLocked", fields[6]);
        Assert.Equal("100", fields[7]);
        Assert.Equal("0", fields[8]);
        Assert.Equal(string.Empty, fields[9]);
        for (int i = 10; i < fields.Length; i++)
        {
            Assert.Equal(string.Empty, fields[i]);
        }

        Assert.Equal(1, summary.AngleRows);
        Assert.Equal(0, summary.RateRows);
        Assert.Equal(0, summary.MotorRows);
        Assert.Equal(0.1, summary.FirstTimeSeconds, 9);
        Assert.Equal(0.1, summary.LastTimeSeconds, 9);
    }

    [Fact]
    public void RateRow_LeavesAngleOnlyColumnsEmpty()
    {
        (string text, CsvExportSummary summary) = Export(null, new[] { Rate(0.104, 0.01234) });

        string[] fields = Fields(Lines(text)[1]);

        Assert.Equal(13, fields.Length);
        Assert.Equal("0.104000", fields[0]);
        Assert.Equal("rate", fields[1]);
        Assert.Equal(string.Empty, fields[2]);
        Assert.Equal("0.01234", fields[3]);
        for (int i = 4; i < fields.Length; i++)
        {
            Assert.Equal(string.Empty, fields[i]);
        }

        Assert.Equal(0, summary.AngleRows);
        Assert.Equal(1, summary.RateRows);
        Assert.Equal(0, summary.RawRows);
        Assert.Equal(0, summary.MotorRows);
    }

    /// <summary>
    /// 原始采样行必须把四项写全：<c>raw_code</c>、<c>sample_tick_ms</c>、<c>bias_dps</c>
    /// 与 <c>rate_dps</c>（滤波后值）。这四列就是离线复算滤波器所需的全部输入/输出。
    /// </summary>
    [Fact]
    public void RawRow_ContainsRawCodeTickBiasAndFilteredValue()
    {
        (string text, CsvExportSummary summary) = Export(
            null,
            null,
            null,
            new[] { Raw(0.56, rawCode: -1234, sampleTickMs: 4294967000U, biasDps: -0.0123, filteredDps: -1.23456) });

        string[] fields = Fields(Lines(text)[1]);

        Assert.Equal(13, fields.Length);
        Assert.Equal("0.560000", fields[0]);
        Assert.Equal("raw", fields[1]);
        Assert.Equal(string.Empty, fields[2]);
        Assert.Equal("-1.23456", fields[3]);
        for (int i = 4; i < 10; i++)
        {
            Assert.Equal(string.Empty, fields[i]);
        }

        Assert.Equal("-1234", fields[10]);
        Assert.Equal("4294967000", fields[11]);
        Assert.Equal("-0.012", fields[12]);
        Assert.Equal(0, summary.AngleRows);
        Assert.Equal(0, summary.RateRows);
        Assert.Equal(1, summary.RawRows);
        Assert.Equal(0, summary.MotorRows);
        Assert.Equal(0.56, summary.FirstTimeSeconds, 9);
        Assert.Equal(0.56, summary.LastTimeSeconds, 9);
    }

    [Fact]
    public void MotorRow_PutsAngleInItsOwnColumn()
    {
        (string text, CsvExportSummary summary) = Export(null, null, new[] { Motor(0.3, 12.34567) });

        string[] fields = Fields(Lines(text)[1]);

        Assert.Equal(13, fields.Length);
        Assert.Equal("0.300000", fields[0]);
        Assert.Equal("motor", fields[1]);
        for (int i = 2; i < 9; i++)
        {
            Assert.Equal(string.Empty, fields[i]);
        }

        Assert.Equal("12.34567", fields[9]);
        for (int i = 10; i < fields.Length; i++)
        {
            Assert.Equal(string.Empty, fields[i]);
        }

        Assert.Equal(0, summary.AngleRows);
        Assert.Equal(0, summary.RateRows);
        Assert.Equal(0, summary.RawRows);
        Assert.Equal(1, summary.MotorRows);
    }

    /// <summary>
    /// 电机行必须用与陀螺仪共用的全局时间轴（<c>ChartTimeSeconds</c>）。
    /// 用私有轴导出的效果是电机行整体偏移，画在同一张图上就和角度/角速度对不上，
    /// 而且 0 点不同——实测偏差最大到 6.9 s。
    /// </summary>
    [Fact]
    public void MotorRow_UsesSharedGlobalTimeAxis_NotPrivateMotorAxis()
    {
        // 私有轴 20.5 s，全局轴 7.5 s；两者差 13 s，不可能同时通过下面的断言。
        MotorPositionSample sample = Motor(7.5, 42.0, privateTimeOffsetSeconds: 13.0);

        (string text, CsvExportSummary summary) = Export(null, null, new[] { sample });

        string[] fields = Fields(Lines(text)[1]);

        Assert.Equal("7.500000", fields[0]);
        Assert.NotEqual("20.500000", fields[0]);
        Assert.Equal(7.5, summary.FirstTimeSeconds, 9);
        Assert.Equal(7.5, summary.LastTimeSeconds, 9);
    }

    /// <summary>
    /// 归并顺序也必须按全局轴：电机私有轴若与全局轴错位，按私有轴排会让
    /// 「行序」和「写出的时间戳」自相矛盾（时间列出现非单调回退）。
    /// </summary>
    [Fact]
    public void MotorRow_IsMergedByGlobalAxis_OutputStaysTimeOrdered()
    {
        // 电机全局轴 1.5 s / 2.5 s，但私有轴是 11.5 s / 12.5 s（都被推到最后）。
        // 若归并误用私有轴，这两行会跑到角度行之后。
        (string text, _) = Export(
            new[] { Angle(1.0), Angle(2.0), Angle(3.0) },
            null,
            new[] { Motor(1.5), Motor(2.5) });

        string[] lines = Lines(text).Skip(1).ToArray();
        Assert.Equal(new[] { "angle", "motor", "angle", "motor", "angle" }, lines.Select(l => Fields(l)[1]).ToArray());

        double[] times = lines.Select(l => double.Parse(Fields(l)[0], CultureInfo.InvariantCulture)).ToArray();
        for (int i = 1; i < times.Length; i++)
        {
            Assert.True(times[i] >= times[i - 1], $"第 {i} 行时间回退：{times[i - 1]} → {times[i]}");
        }
    }

    [Fact]
    public void BreakBefore_IsWrittenAsOne()
    {
        (string text, _) = Export(new[] { Angle(0.2, breakBefore: true) }, null);

        Assert.Equal("1", Fields(Lines(text)[1])[8]);
    }

    [Fact]
    public void RowsAreMergedByTime_WithAngleFirstOnTie()
    {
        (string text, _) = Export(
            new[] { Angle(1.0), Angle(2.0) },
            new[] { Rate(0.5), Rate(2.0) },
            new[] { Motor(2.0), Motor(3.0) },
            new[] { Raw(2.0), Raw(2.5) });

        string[] kinds = Lines(text).Skip(1).Select(line => Fields(line)[1]).ToArray();

        Assert.Equal(new[] { "rate", "angle", "angle", "rate", "raw", "motor", "raw", "motor" }, kinds);
    }

    [Fact]
    public void RowCountEqualsInputCount_NoInterpolationNoDrop()
    {
        DataSample[] data = { Angle(1.0), Angle(1.1), Angle(1.2) };
        RateSample[] rate = { Rate(1.05), Rate(1.15) };
        MotorPositionSample[] motor = { Motor(1.02), Motor(1.12), Motor(1.22) };
        RawSample[] raw = { Raw(1.07), Raw(1.17) };

        (string text, CsvExportSummary summary) = Export(data, rate, motor, raw);

        Assert.Equal(data.Length + rate.Length + raw.Length + motor.Length, Lines(text).Length - 1);
        Assert.Equal(data.Length, summary.AngleRows);
        Assert.Equal(rate.Length, summary.RateRows);
        Assert.Equal(raw.Length, summary.RawRows);
        Assert.Equal(motor.Length, summary.MotorRows);
    }

    [Fact]
    public void TimestampsArePreservedExactly()
    {
        DataSample[] data = { Angle(0.7), Angle(2.4) };
        RateSample[] rate = { Rate(1.3) };

        (string text, _) = Export(data, rate);

        string[] times = Lines(text).Skip(1).Select(line => Fields(line)[0]).ToArray();
        string[] expected = data.Select(sample => sample.TimeSeconds)
            .Concat(rate.Select(sample => sample.TimeSeconds))
            .Select(seconds => seconds.ToString("F6", CultureInfo.InvariantCulture))
            .ToArray();

        Assert.Equal(expected.OrderBy(value => value, StringComparer.Ordinal), times.OrderBy(value => value, StringComparer.Ordinal));
        Assert.Equal(expected.Length, times.Length);
    }

    [Fact]
    public void SummaryReportsOverallTimeRange()
    {
        (_, CsvExportSummary summary) = Export(
            new[] { Angle(3.5), Angle(4.0) },
            new[] { Rate(1.25) });

        Assert.Equal(1.25, summary.FirstTimeSeconds, 9);
        Assert.Equal(4.0, summary.LastTimeSeconds, 9);
    }

    [Fact]
    public void UsesInvariantDecimalPoint_RegardlessOfCurrentCulture()
    {
        CultureInfo original = CultureInfo.CurrentCulture;
        try
        {
            CultureInfo.CurrentCulture = new CultureInfo("de-DE");
            (string text, _) = Export(
                new[] { Angle(1.5, 12.34567, 26.5) },
                new[] { Rate(1.5, 3.5) },
                new[] { Motor(1.5, -7.89012) },
                new[] { Raw(1.5, rawCode: 1234, sampleTickMs: 987654U, biasDps: -0.0123, filteredDps: 1.23456) });

            Assert.Contains("1.500000,angle,12.34567,,26.50,", text, StringComparison.Ordinal);
            Assert.Contains("1.500000,rate,,3.50000,,,,,,,,,", text, StringComparison.Ordinal);
            Assert.Contains("1.500000,motor,,,,,,,,-7.89012,,,", text, StringComparison.Ordinal);
            Assert.Contains("1.500000,raw,,1.23456,,,,,,,1234,987654,-0.012", text, StringComparison.Ordinal);
            Assert.DoesNotContain("12,34567", text, StringComparison.Ordinal);
            Assert.DoesNotContain("-7,89012", text, StringComparison.Ordinal);
            Assert.DoesNotContain("-0,012", text, StringComparison.Ordinal);
        }
        finally
        {
            CultureInfo.CurrentCulture = original;
        }
    }

    [Fact]
    public void EveryLineEndsWithCrLf()
    {
        (string text, _) = Export(new[] { Angle(0.1) }, new[] { Rate(0.2) });

        Assert.EndsWith("\r\n", text, StringComparison.Ordinal);
        Assert.DoesNotContain("\n", text.Replace("\r\n", string.Empty, StringComparison.Ordinal), StringComparison.Ordinal);
    }

    [Fact]
    public void StatusFlags_MapKnownBitsAndKeepUnknownBitsVisible()
    {
        (string noneText, _) = Export(new[] { Angle(0.1, status: GyroStatus.None) }, null);
        Assert.Equal(string.Empty, Fields(Lines(noneText)[1])[6]);
        Assert.Equal("0x00", Fields(Lines(noneText)[1])[5]);

        (string degradedText, _) = Export(
            new[] { Angle(0.1, status: GyroStatus.AngleValid | GyroStatus.DegradedCalibration) },
            null);
        Assert.Equal("AngleValid|DegradedCalibration", Fields(Lines(degradedText)[1])[6]);

        (string continuousText, _) = Export(
            new[] { Angle(0.1, status: GyroStatus.StillLocked | GyroStatus.ContinuousIntegration) }, null);
        Assert.Equal("0x50", Fields(Lines(continuousText)[1])[5]);
        Assert.Equal("StillLocked|ContinuousIntegration", Fields(Lines(continuousText)[1])[6]);

        (string unknownText, _) = Export(new[] { Angle(0.1, status: (GyroStatus)0x80) }, null);
        Assert.Equal("0x80", Fields(Lines(unknownText)[1])[5]);
        Assert.Equal("Unknown_0x80", Fields(Lines(unknownText)[1])[6]);
    }

    [Fact]
    public void OutputIsUtf8SafeAscii_NoCommaInsideFields()
    {
        (string text, _) = Export(
            new[] { Angle(0.1, status: GyroStatus.AngleValid | GyroStatus.TemperatureValid) },
            new[] { Rate(0.2) },
            new[] { Motor(0.3) },
            new[] { Raw(0.25) });

        foreach (string line in Lines(text))
        {
            Assert.Equal(13, Fields(line).Length);
            Assert.All(line, ch => Assert.True(ch < 128, $"非 ASCII 字符：{ch}"));
        }
    }

    [Fact]
    public void WriteFile_WritesUtf8BomAndReadsBackAsDocumentedCsv()
    {
        string directory = Path.Combine(Path.GetTempPath(), "gyrohost-csv-tests");
        Directory.CreateDirectory(directory);
        string path = Path.Combine(directory, "segment-" + Guid.NewGuid().ToString("N") + ".csv");

        try
        {
            CsvExportSummary summary = MeasurementCsv.WriteFile(
                path,
                new[] { Angle(0.1, 1.23456, 25.31), Angle(0.2, 1.23500, 25.31) },
                new[] { Rate(0.15, 0.01234) },
                new[] { Motor(0.12, 1.20000) },
                new[] { Raw(0.11) });

            byte[] bytes = File.ReadAllBytes(path);
            Assert.Equal(new byte[] { 0xEF, 0xBB, 0xBF }, bytes.Take(3).ToArray());

            string text = File.ReadAllText(path, Encoding.UTF8);
            string[] lines = Lines(text);
            Assert.Equal(6, lines.Length);
            Assert.Equal(MeasurementCsv.Header, lines[0]);
            Assert.Equal(2, summary.AngleRows);
            Assert.Equal(1, summary.RateRows);
            Assert.Equal(1, summary.RawRows);
            Assert.Equal(1, summary.MotorRows);
            Assert.Equal(0.1, summary.FirstTimeSeconds, 9);
            Assert.Equal(0.2, summary.LastTimeSeconds, 9);
            Assert.Equal("angle", Fields(lines[1])[1]);
            Assert.Equal("raw", Fields(lines[2])[1]);
            Assert.Equal("motor", Fields(lines[3])[1]);
            Assert.Equal("rate", Fields(lines[4])[1]);
        }
        finally
        {
            if (File.Exists(path))
            {
                File.Delete(path);
            }
        }
    }
}
