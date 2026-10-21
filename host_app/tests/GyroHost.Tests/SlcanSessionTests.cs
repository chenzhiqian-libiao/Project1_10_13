using GyroHost.Core.Communication;
using GyroHost.Core.Protocol;
using GyroHost.Core.Services;

namespace GyroHost.Tests;

/// <summary>
/// 会话层测试：帧分发、异常统计、两种超时分别判定、控制命令的状态机与重置并发。
/// </summary>
public sealed class SlcanSessionTests
{
    private static readonly GyroStatus ReadyStatus =
        GyroStatus.AngleValid | GyroStatus.TemperatureValid | GyroStatus.CalibrationDone;

    private static (SlcanSession Session, FakeCanConnection Connection) CreateSession()
    {
        FakeCanConnection connection = new();
        SlcanSession session = new(connection);
        return (session, connection);
    }

    /// <summary>两种测量帧各自计数，并保留最近一次有效值。</summary>
    [Fact]
    public void Frames_ShouldBeDispatchedByProtocolId()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();

        connection.EmitText("t321800000000540C170A\r");
        connection.EmitRateFrame(1.25);

        SessionSnapshot snapshot = session.TakeSnapshot();

        Assert.Equal(1, snapshot.Counters.DataFrames);
        Assert.Equal(1, snapshot.Counters.RateFrames);
        Assert.True(snapshot.HasData);
        Assert.True(snapshot.HasRate);
        Assert.Equal(31.56, snapshot.LastData.TemperatureC, 6);
        Assert.Equal(10, snapshot.LastData.IntervalMs);
        Assert.Equal(1.25, snapshot.LastRateDps, 6);
        Assert.Single(snapshot.Data);
        Assert.Single(snapshot.Rate);
    }

    /// <summary>
    /// 0x324 与 0x325 必须按采样时间戳配对成一条记录：原始码、零偏、滤波值来自同一拍。
    /// </summary>
    [Fact]
    public void RawFrames_ShouldBePairedBySampleTick()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();

        connection.EmitRawFrame(rawCode: -1234, sampleTickMs: 5000U, biasDps: 0.012);
        connection.EmitRawDetailFrame(filteredRateDps: 1.25, sampleTickMs: 5000U);

        SessionSnapshot snapshot = session.TakeSnapshot();

        RawSample sample = Assert.Single(snapshot.Raw);
        Assert.Equal(-1234, sample.RawCode);
        Assert.Equal(5000U, sample.SampleTickMs);
        Assert.Equal(0.012, sample.BiasDps, 6);
        Assert.Equal(1.25, sample.FilteredDps, 6);
        Assert.Equal(2, snapshot.Counters.RawFrames);
        Assert.Equal(0, snapshot.Counters.RawPairMismatches);
    }

    /// <summary>
    /// 0x324 丢失时，0x325 不能和上一拍的原始值拼在一起：整条记录丢弃并计数。
    /// 0x323 的滤波值仍然独立记录，不受影响。
    /// </summary>
    [Fact]
    public void RawDetail_WithoutMatchingRaw_ShouldBeDroppedAndCounted()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();

        connection.EmitRawFrame(rawCode: -1, sampleTickMs: 1000U, biasDps: 0.0);
        connection.EmitRawDetailFrame(filteredRateDps: 1.0, sampleTickMs: 2000U);
        connection.EmitRateFrame(1.0);

        SessionSnapshot snapshot = session.TakeSnapshot();

        Assert.Empty(snapshot.Raw);
        Assert.Single(snapshot.Rate);
        Assert.Equal(2, snapshot.Counters.RawFrames);
        Assert.Equal(1, snapshot.Counters.RawPairMismatches);
    }

    [Fact]
    public void RawFrame_ReplacingUnpairedRaw_ShouldCountLossAndPairOnlyLatest()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();

        connection.EmitRawFrame(rawCode: 1, sampleTickMs: 1000U, biasDps: 0.0);
        connection.EmitRawFrame(rawCode: 2, sampleTickMs: 1010U, biasDps: 0.1);
        connection.EmitRawDetailFrame(filteredRateDps: 1.5, sampleTickMs: 1010U);

        SessionSnapshot snapshot = session.TakeSnapshot();
        RawSample sample = Assert.Single(snapshot.Raw);
        Assert.Equal(2, sample.RawCode);
        Assert.Equal(1010U, sample.SampleTickMs);
        Assert.Equal(1, snapshot.Counters.RawPairMismatches);
        Assert.Contains(snapshot.LogLines, line => line.Contains("未收到配对的 0x325"));
    }

    [Fact]
    public void RawFrames_WithoutAnyDetails_ShouldCountEveryLossWithoutFloodingLog()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();
        for (uint i = 0; i < 102; i++)
        {
            connection.EmitRawFrame(rawCode: 1, sampleTickMs: i * 10, biasDps: 0.0);
        }

        SessionSnapshot snapshot = session.TakeSnapshot();
        Assert.Empty(snapshot.Raw);
        Assert.Equal(101, snapshot.Counters.RawPairMismatches);
        Assert.Equal(2, snapshot.LogLines.Count(line => line.Contains("未收到配对的 0x325")));
    }

    /// <summary>两种帧的到达时刻要分别记录，便于界面分别判定超时。</summary>
    [Fact]
    public void Arrivals_ShouldBeTrackedPerFrameType()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();

        connection.EmitText("t321800000000540C170A\r");

        SessionSnapshot snapshot = session.TakeSnapshot();

        Assert.NotNull(snapshot.LastDataArrival);
        Assert.Null(snapshot.LastRateArrival);
    }

    /// <summary>非法 DLC、未知 ID、扩展帧、异常行都要分别计数，不能当有效数据。</summary>
    [Fact]
    public void InvalidInputs_ShouldBeCountedSeparately()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();

        connection.EmitText("t3214DEADBEEF\r");           // 0x321 但 DLC=4，与协议要求不符
        connection.EmitText("t3232FFFF\r");               // 0x323 但 DLC=2，与协议要求不符
        connection.EmitText("t321700000000540C170A\r");   // 行级错误：声明长度与数据不符
        connection.EmitText("t555400000000\r");           // 未知 ID（避开 0x0100..0x017F 的电机错误段）
        connection.EmitText("T00000321800000000540C170A\r"); // 扩展帧
        connection.EmitText("garbage\r");                 // 非法行

        SessionSnapshot snapshot = session.TakeSnapshot();

        Assert.Equal(2, snapshot.Counters.BadDlcFrames);
        Assert.Equal(2, snapshot.Counters.UnsupportedFrames);
        Assert.Equal(2, snapshot.Counters.MalformedLines);
        Assert.Equal(0, snapshot.Counters.DataFrames);
        Assert.Equal(0, snapshot.Counters.RateFrames);
        Assert.False(snapshot.HasData);
    }

    /// <summary>0x322 是上位机发送方向；被回显时只计数，不计为异常。</summary>
    [Fact]
    public void CommandEcho_ShouldNotBeAnError()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();

        connection.EmitText("t322102\r");

        SessionSnapshot snapshot = session.TakeSnapshot();

        Assert.Equal(1, snapshot.Counters.CommandEchoFrames);
        Assert.Equal(0, snapshot.Counters.UnsupportedFrames);
        Assert.Equal(0, snapshot.Counters.MalformedLines);
    }

    /// <summary>发送角度清零：命令内容必须是 0x322 / DLC=1 / 数据 02，且只发一次。</summary>
    [Fact]
    public async Task SendZeroAngle_ShouldEmitSingleCommandFrame()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();
        await session.ConnectAsync();

        await session.SendZeroAngleAsync();

        Assert.Equal(new[] { "t322102\r" }, connection.SentLines);
        Assert.Equal(ZeroAnglePhase.Sent, session.TakeSnapshot().ZeroAngle);
    }

    /// <summary>发送重新校准：0x322 / DLC=1 / 数据 01。</summary>
    [Fact]
    public async Task SendRecalibrate_ShouldEmitSingleCommandFrame()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();
        await session.ConnectAsync();

        await session.SendRecalibrateAsync();

        Assert.Equal(new[] { "t322101\r" }, connection.SentLines);
        Assert.Equal(CalibrationPhase.Requested, session.TakeSnapshot().Calibration);
    }

    /// <summary>未连接时发送命令必须报错，而不是假装成功。</summary>
    [Fact]
    public async Task SendCommand_WithoutConnection_ShouldThrow()
    {
        (SlcanSession session, _) = CreateSession();

        await Assert.ThrowsAsync<InvalidOperationException>(() => session.SendZeroAngleAsync());
    }

    /// <summary>
    /// 校准状态机：仅凭发送前遗留的"校准完成"标志不能判定成功，
    /// 必须观察到 完成位消失 → 再次出现。
    /// </summary>
    [Fact]
    public async Task Recalibration_ShouldRequireObservedStateTransition()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();
        await session.ConnectAsync();
        await session.SendRecalibrateAsync();

        // 板端还没反应时，仍在上报"校准完成"：不能算成功。
        connection.EmitDataFrame(0, 30.0, ReadyStatus);
        Assert.Equal(CalibrationPhase.Requested, session.TakeSnapshot().Calibration);

        // 板端进入校准采样：完成位消失。
        connection.EmitDataFrame(0, 30.0, GyroStatus.AngleValid | GyroStatus.TemperatureValid);
        Assert.Equal(CalibrationPhase.InProgress, session.TakeSnapshot().Calibration);

        // 校准完成：完成位重新出现，此时才算这次校准成功。
        connection.EmitDataFrame(0, 30.0, ReadyStatus);
        SessionSnapshot completed = session.TakeSnapshot();
        Assert.Equal(CalibrationPhase.Completed, completed.Calibration);
        Assert.Contains(completed.LogLines, line => line.Contains("校准完成", StringComparison.Ordinal));
    }

    [Fact]
    public async Task Recalibration_WithDegradedBit_ShouldReportInsufficientQuality()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();
        using (session)
        {
            await session.ConnectAsync();
            await session.SendRecalibrateAsync();
            connection.EmitDataFrame(12.5, 30.0, GyroStatus.TemperatureValid);
            connection.EmitDataFrame(12.5, 30.0, ReadyStatus | GyroStatus.DegradedCalibration);

            SessionSnapshot completed = session.TakeSnapshot();
            Assert.Equal(CalibrationPhase.Completed, completed.Calibration);
            Assert.Contains(completed.LogLines, line => line.Contains("不能判精度合格", StringComparison.Ordinal));
            Assert.DoesNotContain(completed.LogLines, line => line.Contains("本次零偏校准完成", StringComparison.Ordinal));
        }
    }

    /// <summary>板端上报校准失败位时，本次校准判定为失败。</summary>
    [Fact]
    public async Task Recalibration_WithFailBit_ShouldBeFailed()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();
        await session.ConnectAsync();
        await session.SendRecalibrateAsync();

        connection.EmitDataFrame(0, 30.0, GyroStatus.AngleValid | GyroStatus.CalibrationFailed);

        Assert.Equal(CalibrationPhase.Failed, session.TakeSnapshot().Calibration);
    }

    /// <summary>板端没有响应时，超时后必须明确报告"无法确认"，不能默认成功。</summary>
    [Fact]
    public async Task Recalibration_WithoutStateChange_ShouldTimeOut()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();
        session.CalibrationTimeoutSeconds = 0.05;
        await session.ConnectAsync();
        await session.SendRecalibrateAsync();

        await Task.Delay(80);
        connection.EmitDataFrame(0, 30.0, ReadyStatus);

        Assert.Equal(CalibrationPhase.NoResponse, session.TakeSnapshot().Calibration);
    }

    /// <summary>角度清零只有在回传数据确认归零后才算成功，并给曲线打上断开标记。</summary>
    [Fact]
    public async Task ZeroAngle_ShouldBeVerifiedByReturnedData()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();
        await session.ConnectAsync();
        connection.EmitDataFrame(90.0, 30.0, ReadyStatus);
        session.TakeSnapshot();

        await session.SendZeroAngleAsync();

        // 板端尚未执行：角度仍是 90°，不能算成功。
        connection.EmitDataFrame(90.0, 30.0, ReadyStatus);
        Assert.Equal(ZeroAnglePhase.Sent, session.TakeSnapshot().ZeroAngle);

        // 板端执行清零。
        connection.EmitDataFrame(0.0, 30.0, ReadyStatus);

        SessionSnapshot snapshot = session.TakeSnapshot();
        Assert.Equal(ZeroAnglePhase.Verified, snapshot.ZeroAngle);
        Assert.True(snapshot.Data[^1].BreakBefore);
        // 日志要带上回传值，才能区分"真的停在 0.00000°"和旧固件残留的 0.00x°。
        Assert.Contains(snapshot.LogLines, line => line.Contains("回传 0.00000°", StringComparison.Ordinal));
    }

    /// <summary>清零命令发出去但板端没执行：超时后报告未观察到归零。</summary>
    [Fact]
    public async Task ZeroAngle_WithoutAngleChange_ShouldReportNoChange()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();
        session.ZeroVerifyTimeoutSeconds = 0.05;
        await session.ConnectAsync();

        await session.SendZeroAngleAsync();
        await Task.Delay(80);
        connection.EmitDataFrame(90.0, 30.0, ReadyStatus);

        Assert.Equal(ZeroAnglePhase.NoChange, session.TakeSnapshot().ZeroAngle);
    }

    /// <summary>连续接收中重置显示：旧数据不重现、相对时间归零、连接与计数保持。</summary>
    [Fact]
    public async Task ResetDuringReceiving_ShouldDropOldDataAndKeepConnection()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();
        await session.ConnectAsync();
        connection.EmitText("t321800000000540C170A\r");
        connection.EmitRateFrame(1.0);
        session.TakeSnapshot();

        // 重置前到达、尚未被取走的帧（模拟仍在串口/解析缓冲里的旧数据）。
        TimeSpan oldArrival = MonotonicClock.Now - TimeSpan.FromMilliseconds(5);

        session.ResetDisplay();

        byte[] stale = System.Text.Encoding.ASCII.GetBytes("t321800000000540C170A\r");
        connection.Emit(stale, oldArrival);
        connection.EmitDataFrame(12.5, 30.0, ReadyStatus);
        connection.EmitRateFrame(2.0);

        SessionSnapshot snapshot = session.TakeSnapshot();

        Assert.Single(snapshot.Data);
        Assert.Single(snapshot.Rate);
        Assert.Equal(12.5, snapshot.Data[0].AngleDeg, 6);
        Assert.Equal(2.0, snapshot.Rate[0].RateDps, 6);
        // 被丢弃的旧帧仍然计入"已解析帧数"（它确实到达了），只是不上屏；接收没有中断。
        Assert.Equal(1, snapshot.Counters.DroppedBeforeReset);
        Assert.Equal(3, snapshot.Counters.DataFrames);
        Assert.True(connection.IsConnected);
    }

    /// <summary>连接状态变化要反映到快照，供界面显示。</summary>
    [Fact]
    public async Task LinkState_ShouldBeExposedInSnapshot()
    {
        (SlcanSession session, _) = CreateSession();

        Assert.Equal(LinkState.Disconnected, session.TakeSnapshot().LinkState);

        await session.ConnectAsync();
        Assert.Equal(LinkState.Connected, session.TakeSnapshot().LinkState);

        await session.DisconnectAsync();
        Assert.Equal(LinkState.Disconnected, session.TakeSnapshot().LinkState);
    }

    /// <summary>取走快照后日志被清空，不会重复堆积。</summary>
    [Fact]
    public void Snapshot_ShouldDrainLogLines()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();

        connection.EmitText("bad line\r");
        Assert.NotEmpty(session.TakeSnapshot().LogLines);
        Assert.Empty(session.TakeSnapshot().LogLines);
    }
}
