using GyroHost.Core.Communication;
using GyroHost.Core.Protocol;
using GyroHost.Core.Services;

namespace GyroHost.Tests;

/// <summary>
/// 模拟模式端到端测试：模拟器产生的是真实格式的 SLCAN 文本（并刻意分包），
/// 走的是与真实串口完全相同的解析、会话与数据链路。
/// </summary>
public sealed class SimulationTests
{
    [Fact]
    public void CalibrationDefaults_ShouldCoverEstimateAndValidation()
    {
        Assert.Equal(80.0, GyroProtocol.CalibrationDurationSeconds);
        Assert.Equal(80.0, new SimulationConnectionOptions().CalibrationSeconds);
        Assert.Equal(80.5, MotorMotionController.GyroRecalibrationSeconds);
        using SlcanSession session = CreateSession(new SimulationConnectionOptions());
        Assert.Equal(100.0, session.CalibrationTimeoutSeconds);
    }

    private static SlcanSession CreateSession(SimulationConnectionOptions options)
        => new(new SimulationConnection(options));

    /// <summary>模拟器应同时产生两种测量帧，且刻意分包不会产生异常帧。</summary>
    [Fact]
    public async Task Simulation_ShouldFeedBothFrameTypesWithoutParseErrors()
    {
        SimulationConnectionOptions options = new()
        {
            FramePeriodMs = 20,
            ChunkSplitProbabilityPercent = 70,
        };

        using SlcanSession session = CreateSession(options);
        await session.ConnectAsync();
        SessionCollector collector = new(session);

        await SessionCollector.WaitUntilAsync(
            () =>
            {
                collector.Poll(session);
                return collector.Last.Counters.DataFrames >= 10 && collector.Last.Counters.RateFrames >= 10;
            },
            4000);

        await session.DisconnectAsync();

        Assert.True(collector.Last.Counters.DataFrames >= 10, $"0x321 帧数不足：{collector.Last.Counters.DataFrames}");
        Assert.True(collector.Last.Counters.RateFrames >= 10, $"0x323 帧数不足：{collector.Last.Counters.RateFrames}");
        Assert.Equal(0, collector.Last.Counters.MalformedLines);
        Assert.Equal(0, collector.Last.Counters.BadDlcFrames);
        Assert.Equal(0, collector.Last.Counters.UnsupportedFrames);
        Assert.True(collector.Last.HasData);
        Assert.True(collector.Last.HasRate);
    }

    /// <summary>
    /// 模拟器也应产生 0x324/0x325，并让会话层配对成原始采样记录：
    /// 覆盖「模拟 → 解析 → 配对 → 数据仓库」整条链路。
    /// </summary>
    [Fact]
    public async Task Simulation_ShouldProducePairedRawSamples()
    {
        SimulationConnectionOptions options = new()
        {
            FramePeriodMs = 20,
            ChunkSplitProbabilityPercent = 70,
        };

        using SlcanSession session = CreateSession(options);
        await session.ConnectAsync();
        SessionCollector collector = new(session);

        await SessionCollector.WaitUntilAsync(
            () =>
            {
                collector.Poll(session);
                return collector.Raw.Count >= 5;
            },
            4000);

        await session.DisconnectAsync();

        Assert.True(collector.Raw.Count >= 5, $"原始采样记录不足：{collector.Raw.Count}");
        Assert.Equal(0, collector.Last.Counters.RawPairMismatches);
        Assert.True(collector.Last.Counters.RawFrames >= collector.Raw.Count * 2);

        // 模拟器的零偏固定为 0，原始码必须能按 70 LSB/(°/s) 还原成滤波值
        // （模拟器用 double 反推 int16 原始码，误差最多半个原始码）。
        RawSample sample = collector.Raw[0];
        double reconstructed = (sample.RawCode / GyroProtocol.RawLsbPerDps) - sample.BiasDps;
        Assert.InRange(
            Math.Abs(sample.FilteredDps - reconstructed),
            0.0,
            (0.5 / GyroProtocol.RawLsbPerDps) + 1e-9);
        Assert.Equal(0.0, sample.BiasDps, 6);
    }

    /// <summary>模拟器按真实节奏发送：曲线时间轴应来自真实到达时间，间隔接近帧周期。</summary>
    [Fact]
    public async Task Simulation_ShouldKeepSampleInterval()
    {
        SimulationConnectionOptions options = new() { FramePeriodMs = 100, ChunkSplitProbabilityPercent = 0 };
        using SlcanSession session = CreateSession(options);
        await session.ConnectAsync();
        SessionCollector collector = new(session);

        await SessionCollector.WaitUntilAsync(
            () =>
            {
                collector.Poll(session);
                return collector.Data.Count >= 5;
            },
            4000);

        double elapsed = collector.Data.Count >= 5
            ? collector.Data[^1].TimeSeconds - collector.Data[0].TimeSeconds
            : 0.0;

        await session.DisconnectAsync();

        // 4 个间隔 × 100ms ≈ 0.4s，允许调度抖动。
        Assert.InRange(elapsed, 0.25, 0.75);
    }

    /// <summary>模拟器响应角度清零命令，上位机以回传数据确认结果。</summary>
    [Fact]
    public async Task Simulation_ShouldHonorZeroAngleCommand()
    {
        SimulationConnectionOptions options = new()
        {
            FramePeriodMs = 20,
            ChunkSplitProbabilityPercent = 0,
            MotionPeriodSeconds = 8,
            CalibrationSeconds = 0.2,
        };

        using SlcanSession session = CreateSession(options);
        await session.ConnectAsync();
        SessionCollector collector = new(session);

        // 等到角度明显偏离 0，清零才有意义。
        await SessionCollector.WaitUntilAsync(
            () =>
            {
                collector.Poll(session);
                return collector.Last.LastData.AngleDeg > 20.0;
            },
            6000);

        await session.SendZeroAngleAsync();

        bool verified = await SessionCollector.WaitUntilAsync(
            () =>
            {
                collector.Poll(session);
                return collector.Last.ZeroAngle == ZeroAnglePhase.Verified;
            },
            3000);

        await session.DisconnectAsync();

        Assert.True(verified, "模拟器应清零角度并由回传数据确认");
    }

    /// <summary>模拟器响应重新校准命令，并走完 请求 → 进行中 → 完成 的完整状态变化。</summary>
    [Fact]
    public async Task Simulation_ShouldHonorRecalibrationCommand()
    {
        SimulationConnectionOptions options = new()
        {
            FramePeriodMs = 20,
            ChunkSplitProbabilityPercent = 50,
            CalibrationSeconds = 0.5,
        };

        using SlcanSession session = CreateSession(options);
        await session.ConnectAsync();
        SessionCollector collector = new(session);

        // 等首次上电校准结束，"校准完成"位出现之后再进行本次实验。
        await SessionCollector.WaitUntilAsync(
            () =>
            {
                collector.Poll(session);
                return (collector.Last.LastData.Status & GyroStatus.CalibrationDone) != 0;
            },
            4000);

        await session.SendRecalibrateAsync();

        bool sawInProgress = await SessionCollector.WaitUntilAsync(
            () =>
            {
                collector.Poll(session);
                return collector.Last.Calibration == CalibrationPhase.InProgress;
            },
            3000);

        bool completed = await SessionCollector.WaitUntilAsync(
            () =>
            {
                collector.Poll(session);
                return collector.Last.Calibration == CalibrationPhase.Completed;
            },
            3000);

        await session.DisconnectAsync();

        Assert.True(sawInProgress, "应观察到板端进入校准采样");
        Assert.True(completed, "应观察到本次校准完成");
    }

    /// <summary>持续接收过程中重置显示：旧数据不重现，新数据从 0 重新计时，接收不中断。</summary>
    [Fact]
    public async Task Simulation_ResetDuringReceiving_ShouldRestartTimeline()
    {
        SimulationConnectionOptions options = new() { FramePeriodMs = 20, ChunkSplitProbabilityPercent = 40 };
        using SlcanSession session = CreateSession(options);
        await session.ConnectAsync();
        SessionCollector collector = new(session);

        await SessionCollector.WaitUntilAsync(
            () =>
            {
                collector.Poll(session);
                return collector.Last.Counters.DataFrames >= 10;
            },
            4000);

        session.ResetDisplay();

        // 清空累加器，只观察重置之后采集到的点：它们必须全部属于新时间轴。
        collector.Data.Clear();
        await SessionCollector.WaitUntilAsync(
            () =>
            {
                collector.Poll(session);
                return collector.Data.Count >= 3;
            },
            4000);

        double firstRelativeTime = collector.Data[0].TimeSeconds;
        double span = collector.Data[^1].TimeSeconds - collector.Data[0].TimeSeconds;
        long framesAfterReset = collector.Last.Counters.DataFrames;
        await session.DisconnectAsync();

        Assert.True(firstRelativeTime < 0.5, $"重置后时间轴应从 0 重新开始，实际 {firstRelativeTime:F3}s");
        Assert.True(framesAfterReset > 10, "重置不应中断接收");
        Assert.True(span < 1.0, "重置后的点不应包含旧时间轴数据");
    }
}
