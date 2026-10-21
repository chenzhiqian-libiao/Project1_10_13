using GyroHost.Core.Protocol;
using GyroHost.Core.Services;

namespace GyroHost.Tests;

/// <summary>
/// 数据仓库测试：相对时间基准、重置语义、缓存上限。
/// </summary>
public sealed class MeasurementStoreTests
{
    private static readonly GyroStatus Valid = GyroStatus.AngleValid | GyroStatus.CalibrationDone;

    /// <summary>首帧作为时间 0，后续点以真实到达时刻计算相对时间。</summary>
    [Fact]
    public void FirstSample_ShouldDefineTimeOrigin()
    {
        MeasurementStore store = new();

        store.AddData(TimeSpan.FromSeconds(100), 1.0, 25.0, Valid, 10);
        store.AddData(TimeSpan.FromSeconds(100.5), 2.0, 25.0, Valid, 10);

        TransferBatch batch = store.Drain();

        Assert.Equal(0.0, batch.Data[0].TimeSeconds, 6);
        Assert.Equal(0.5, batch.Data[1].TimeSeconds, 6);
    }

    /// <summary>
    /// 原始采样共用同一条相对时间轴：AddRaw 也要走 EnsureEpoch，
    /// 且 ResetDisplay 必须把原始采样队列一起清掉（否则导出会混入重置前的旧值）。
    /// </summary>
    [Fact]
    public void RawSamples_ShouldShareTimelineAndBeClearedByReset()
    {
        MeasurementStore store = new();

        store.AddData(TimeSpan.FromSeconds(10), 1.0, 25.0, Valid, 10);
        store.AddRaw(TimeSpan.FromSeconds(10.5), -1234, 500U, 0.012, 1.25);

        TransferBatch batch = store.Drain();
        RawSample sample = Assert.Single(batch.Raw);
        Assert.Equal(0.5, sample.TimeSeconds, 6);
        Assert.Equal(-1234, sample.RawCode);
        Assert.Equal(500U, sample.SampleTickMs);
        Assert.Equal(0.012, sample.BiasDps, 6);
        Assert.Equal(1.25, sample.FilteredDps, 6);
        Assert.Equal(1, store.RawAcceptedCount);

        store.AddRaw(TimeSpan.FromSeconds(11), 1, 600U, 0.0, 0.5);
        store.ResetDisplay(TimeSpan.FromSeconds(12));

        Assert.Empty(store.Drain().Raw);
    }

    /// <summary>重置显示：批次号 +1、清空待绘制数据、相对时间从 0 重新开始。</summary>
    [Fact]
    public void ResetDisplay_ShouldClearPendingAndRestartTime()
    {
        MeasurementStore store = new();
        store.AddData(TimeSpan.FromSeconds(1), 5.0, 25.0, Valid, 10);
        store.AddRate(TimeSpan.FromSeconds(1), 1.0);
        Assert.Equal(0, store.Generation);

        store.ResetDisplay(TimeSpan.FromSeconds(2));

        TransferBatch cleared = store.Drain();
        Assert.Empty(cleared.Data);
        Assert.Empty(cleared.Rate);
        Assert.Equal(1, cleared.Generation);

        store.AddData(TimeSpan.FromSeconds(2.4), 6.0, 25.0, Valid, 10);
        TransferBatch afterReset = store.Drain();
        Assert.Equal(0.4, afterReset.Data[0].TimeSeconds, 6);
        Assert.Equal(0, afterReset.Generation - cleared.Generation);
    }

    /// <summary>重置前已经到达但还没被取走的数据，必须被丢弃，不能重新出现在曲线上。</summary>
    [Fact]
    public void DataQueuedBeforeReset_ShouldNotReappear()
    {
        MeasurementStore store = new();

        // 这些点在重置之前就已经到达（模拟界面还没取走 + 串口缓冲里排队的旧数据）。
        store.AddData(TimeSpan.FromSeconds(2.0), 10.0, 25.0, Valid, 10);
        store.AddRate(TimeSpan.FromSeconds(2.0), 3.0);

        store.ResetDisplay(TimeSpan.FromSeconds(2.5));

        // 解析线程在重置之后才处理到"重置前到达"的帧：必须拒绝。
        Assert.False(store.AddData(TimeSpan.FromSeconds(2.2), 11.0, 25.0, Valid, 10));
        Assert.False(store.AddRate(TimeSpan.FromSeconds(2.2), 4.0));

        TransferBatch batch = store.Drain();
        Assert.Empty(batch.Data);
        Assert.Empty(batch.Rate);
        Assert.Equal(2, store.DroppedBeforeResetCount);
    }

    /// <summary>重置后到达的新数据正常写入，相对时间从 0 起算。</summary>
    [Fact]
    public void DataAfterReset_ShouldBeAcceptedWithFreshTimeline()
    {
        MeasurementStore store = new();
        store.ResetDisplay(TimeSpan.FromSeconds(10));

        Assert.True(store.AddData(TimeSpan.FromSeconds(10.1), 20.0, 30.0, Valid, 10));
        Assert.True(store.AddRate(TimeSpan.FromSeconds(10.1), -0.5));

        TransferBatch batch = store.Drain();
        Assert.Equal(0.1, batch.Data[0].TimeSeconds, 6);
        Assert.Equal(0.1, batch.Rate[0].TimeSeconds, 6);
        Assert.Equal(1, batch.Generation);
    }

    /// <summary>清零/校准标记会挂到下一个 0x321 点上，供界面断开曲线。</summary>
    [Fact]
    public void MarkAngleBreak_ShouldAttachToNextDataPoint()
    {
        MeasurementStore store = new();
        store.AddData(TimeSpan.FromSeconds(1), 90.0, 25.0, Valid, 10);
        store.MarkAngleBreak();

        store.AddData(TimeSpan.FromSeconds(1.1), 0.0, 25.0, Valid, 10);

        TransferBatch batch = store.Drain();
        Assert.False(batch.Data[0].BreakBefore);
        Assert.True(batch.Data[1].BreakBefore);
    }

    /// <summary>待绘制队列必须有上限，长时间运行不会无限增长。</summary>
    [Fact]
    public void PendingQueue_ShouldBeBounded()
    {
        MeasurementStore store = new();
        int total = MeasurementStore.MaxPendingSamples + 250;

        for (int i = 0; i < total; i++)
        {
            store.AddData(TimeSpan.FromMilliseconds(i), i, 25.0, Valid, 10);
        }

        TransferBatch batch = store.Drain();

        Assert.Equal(MeasurementStore.MaxPendingSamples, batch.Data.Count);
        Assert.Equal(250, store.PendingOverflowCount);
        // 丢弃的是最旧的点，保留的最新点仍然正确。
        Assert.Equal(TimeSpan.FromMilliseconds(total - 1), batch.Data[^1].Arrival);
    }

    /// <summary>取走后队列清空，第二次取应为空（避免重复上屏）。</summary>
    [Fact]
    public void Drain_ShouldBeIdempotent()
    {
        MeasurementStore store = new();
        store.AddData(TimeSpan.FromSeconds(1), 1.0, 25.0, Valid, 10);

        Assert.Single(store.Drain().Data);
        Assert.Empty(store.Drain().Data);
    }

    /// <summary>电机曲线有独立时间轴：显示归零只清电机数据，不碰陀螺仪曲线。</summary>
    [Fact]
    public void ResetMotorDisplay_ShouldOnlyRestartMotorTimeline()
    {
        MeasurementStore store = new();
        store.AddData(TimeSpan.FromSeconds(10), 1.0, 25.0, Valid, 10);
        store.AddMotor(TimeSpan.FromSeconds(10), 100, 10.0, 1.0);
        store.AddMotor(TimeSpan.FromSeconds(10.1), 101, 10.1, 1.1);
        store.Drain(); // 先取走旧数据，单独验证“电机显示归零”不会清陀螺仪队列

        store.ResetMotorDisplay(TimeSpan.FromSeconds(20));

        TransferBatch cleared = store.Drain();
        Assert.Empty(cleared.Motor);
        Assert.Equal(1, cleared.MotorGeneration);

        store.AddData(TimeSpan.FromSeconds(20.2), 2.0, 25.0, Valid, 10);
        store.AddMotor(TimeSpan.FromSeconds(20.5), 102, 10.2, 0.2);

        TransferBatch after = store.Drain();
        Assert.Single(after.Motor);
        Assert.Equal(0.5, after.Motor[0].TimeSeconds, 6);
        Assert.Equal(10.2, after.Data[0].TimeSeconds, 6); // 陀螺仪时间轴仍以原来的 10 s 为基准
    }

    /// <summary>全局重置会同时清空电机曲线并递增电机批次号，P0 由会话层保留。</summary>
    [Fact]
    public void ResetDisplay_ShouldAlsoClearMotorQueue()
    {
        MeasurementStore store = new();
        store.AddMotor(TimeSpan.FromSeconds(1), 100, 10.0, 0.0);

        store.ResetDisplay(TimeSpan.FromSeconds(2));

        TransferBatch cleared = store.Drain();
        Assert.Empty(cleared.Motor);
        Assert.Equal(1, cleared.MotorGeneration);
        Assert.Equal(1, cleared.Generation);
    }

    /// <summary>位置反馈超时恢复时，断开标记必须挂在下一个电机点上。</summary>
    [Fact]
    public void MarkMotorBreak_ShouldAttachToNextMotorPoint()
    {
        MeasurementStore store = new();
        store.AddMotor(TimeSpan.FromSeconds(1), 100, 10.0, 0.0);
        store.MarkMotorBreak();

        store.AddMotor(TimeSpan.FromSeconds(2), 101, 10.1, 0.1);

        TransferBatch batch = store.Drain();
        Assert.False(batch.Motor[0].BreakBefore);
        Assert.True(batch.Motor[1].BreakBefore);
    }

    /// <summary>
    /// 电机采样同时带两个时间：<c>TimeSeconds</c> 用电机自己的时间轴，
    /// <c>ChartTimeSeconds</c> 与陀螺仪共用同一时间基准（画在同一张图上用）。
    /// </summary>
    [Fact]
    public void MotorSample_ShouldCarrySharedChartTimeline()
    {
        MeasurementStore store = new();
        store.AddData(TimeSpan.FromSeconds(10), 1.0, 25.0, Valid, 10);
        store.AddMotor(TimeSpan.FromSeconds(10.25), 100, 10.0, 0.0);

        TransferBatch batch = store.Drain();
        Assert.Equal(0.0, batch.Data[0].TimeSeconds, 6);
        Assert.Equal(0.25, batch.Motor[0].ChartTimeSeconds, 6);
        Assert.Equal(0.0, batch.Motor[0].TimeSeconds, 6);
    }

    /// <summary>电机帧先到时，共用时间轴从这一帧起算，陀螺仪点不会倒退成负数。</summary>
    [Fact]
    public void MotorFirst_ShouldInitializeSharedTimeline()
    {
        MeasurementStore store = new();
        store.AddMotor(TimeSpan.FromSeconds(5), 100, 10.0, 0.0);
        store.AddData(TimeSpan.FromSeconds(5.5), 1.0, 25.0, Valid, 10);

        TransferBatch batch = store.Drain();
        Assert.Equal(0.0, batch.Motor[0].ChartTimeSeconds, 6);
        Assert.Equal(0.5, batch.Data[0].TimeSeconds, 6);
    }

    /// <summary>“电机显示归零”只重置电机自己的时间轴，共用时间基准继续走。</summary>
    [Fact]
    public void ResetMotorDisplay_ShouldKeepSharedTimeline()
    {
        MeasurementStore store = new();
        store.AddData(TimeSpan.FromSeconds(10), 1.0, 25.0, Valid, 10);
        store.ResetMotorDisplay(TimeSpan.FromSeconds(20));

        store.AddMotor(TimeSpan.FromSeconds(20.5), 100, 10.0, 0.0);
        store.AddData(TimeSpan.FromSeconds(20.6), 2.0, 25.0, Valid, 10);

        TransferBatch batch = store.Drain();
        Assert.Equal(10.5, batch.Motor[0].ChartTimeSeconds, 6);
        Assert.Equal(0.5, batch.Motor[0].TimeSeconds, 6);
        Assert.Equal(10.6, batch.Data[^1].TimeSeconds, 6);
    }

    /// <summary>
    /// 断线续接：时间轴从上一段最后一点接着走（不留空档），第一个点之前先断开，
    /// 电机批次号沿用上一段，界面不会因为换了新会话而清空旧曲线。
    /// </summary>
    [Fact]
    public void Resume_ShouldContinueTimelineWithBreakAndSameGeneration()
    {
        MeasurementStore store = new(startTimeOffsetSeconds: 120.0, motorGenerationSeed: 5, breakAtStart: true);

        store.AddData(TimeSpan.FromSeconds(500), 1.0, 25.0, Valid, 10);
        store.AddData(TimeSpan.FromSeconds(500.5), 2.0, 25.0, Valid, 10);
        store.AddMotor(TimeSpan.FromSeconds(500.2), 100, 10.0, 0.0);

        TransferBatch batch = store.Drain();

        Assert.Equal(120.0, batch.Data[0].TimeSeconds, 6);
        Assert.True(batch.Data[0].BreakBefore);
        Assert.False(batch.Data[1].BreakBefore);
        Assert.Equal(120.5, batch.Data[1].TimeSeconds, 6);

        // 电机曲线与陀螺仪共用同一条时间轴，也保留批次号。
        Assert.Equal(120.2, batch.Motor[0].ChartTimeSeconds, 6);
        Assert.True(batch.Motor[0].BreakBefore);
        Assert.Equal(5, batch.MotorGeneration);
    }

    /// <summary>续接后点“清零”：批次号 +1（界面据此清曲线），时间轴回到 0。</summary>
    [Fact]
    public void ResetAfterResume_ShouldBumpGenerationAndRestartFromZero()
    {
        MeasurementStore store = new(startTimeOffsetSeconds: 60.0, motorGenerationSeed: 3, breakAtStart: true);
        store.AddData(TimeSpan.FromSeconds(200), 1.0, 25.0, Valid, 10);
        store.Drain();

        store.ResetDisplay(TimeSpan.FromSeconds(210));
        store.AddData(TimeSpan.FromSeconds(210.4), 3.0, 25.0, Valid, 10);

        TransferBatch batch = store.Drain();
        Assert.Equal(1, batch.Generation);
        Assert.Equal(4, batch.MotorGeneration);
        Assert.Equal(0.4, batch.Data[0].TimeSeconds, 6);
        Assert.False(batch.Data[0].BreakBefore);
    }

    /// <summary>起始时间偏移必须是有限非负值，避免时间轴出现 NaN。</summary>
    [Theory]
    [InlineData(-1.0)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    public void Resume_WithInvalidOffset_ShouldThrow(double offset)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new MeasurementStore(offset, 0, true));
    }
}
