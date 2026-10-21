using GyroHost.Core.Protocol;

namespace GyroHost.Core.Services;

/// <summary>一条 0x321 采样点。时间轴为相对时间（秒），基准由重置决定。</summary>
/// <param name="Arrival">字节到达时刻（单调时钟），用于判定重置前旧数据。</param>
/// <param name="TimeSeconds">相对时间（秒）。</param>
/// <param name="AngleDeg">累计角度（°），板端已完成 4 倍修正，上位机不得再乘。</param>
/// <param name="TemperatureC">温度（℃）。</param>
/// <param name="Status">状态位。</param>
/// <param name="IntervalMs">板端采样间隔（ms）。</param>
/// <param name="BreakBefore">true = 本点之前需要断开曲线（例如板端清零造成的跳变）。</param>
public readonly record struct DataSample(
    TimeSpan Arrival,
    double TimeSeconds,
    double AngleDeg,
    double TemperatureC,
    GyroStatus Status,
    int IntervalMs,
    bool BreakBefore);

/// <summary>一条 0x323 采样点。</summary>
public readonly record struct RateSample(
    TimeSpan Arrival,
    double TimeSeconds,
    double RateDps);

/// <summary>
/// 一条 0x324 + 0x325 配对后的原始采样记录。
/// <para>
/// 时间轴与 <see cref="DataSample"/> 相同（相对时间，秒），所以原始记录可以
/// 和角度、角速度行按 <c>relative_time_s</c> 直接对齐。
/// </para>
/// </summary>
/// <param name="Arrival">0x325 到达时刻（单调时钟），用于判定重置前旧数据。</param>
/// <param name="TimeSeconds">相对时间（秒）。</param>
/// <param name="RawCode">陀螺仪 16 bit 原始码（补码）。</param>
/// <param name="SampleTickMs">板端实际采样时刻（HAL tick，ms）。</param>
/// <param name="BiasDps">板端当前零偏（°/s）。</param>
/// <param name="FilteredDps">滤波后角速度（°/s），与 0x323 同值。</param>
public readonly record struct RawSample(
    TimeSpan Arrival,
    double TimeSeconds,
    short RawCode,
    uint SampleTickMs,
    double BiasDps,
    double FilteredDps);

/// <summary>一次批量取走的待绘制数据。</summary>
/// <param name="Generation">陀螺仪批次号。全局重置会使其递增，界面据此丢弃过期批次。</param>
/// <param name="MotorGeneration">电机曲线批次号。全局重置或“电机显示归零”会使其递增。</param>
public sealed record TransferBatch(
    long Generation,
    IReadOnlyList<DataSample> Data,
    IReadOnlyList<RateSample> Rate,
    IReadOnlyList<RawSample> Raw,
    IReadOnlyList<MotorPositionSample> Motor,
    long MotorGeneration);

/// <summary>
/// 采样数据仓库：连接线程写入，界面线程批量取走。
/// <para>
/// 职责有三个：
/// 1) 提供统一的相对时间基准（重置后从 0 重新计时）；
/// 2) 丢弃"重置之前已到达"的数据，保证重置前排队的旧数据不会重新出现在曲线上；
/// 3) 限制待处理队列长度，长时间运行不会无限占用内存。
/// </para>
/// <para>所有方法都是线程安全的。</para>
/// </summary>
public sealed class MeasurementStore
{
    /// <summary>待绘制队列上限。正常刷新间隔下远不会触及，只是兜底防止界面卡顿时堆积。</summary>
    public const int MaxPendingSamples = 20000;

    private readonly object _gate = new();
    private readonly List<DataSample> _pendingData = new();
    private readonly List<RateSample> _pendingRate = new();
    private readonly List<RawSample> _pendingRaw = new();
    private readonly List<MotorPositionSample> _pendingMotor = new();

    /// <summary>断线续接时的时间轴起点（秒）。首次连接为 0。</summary>
    private double _startTimeOffsetSeconds;

    private TimeSpan _epoch;
    private bool _epochInitialized;
    private long _generation;
    private bool _breakPending;

    private TimeSpan _motorEpoch;
    private bool _motorEpochInitialized;
    private long _motorGeneration;
    private bool _motorBreakPending;

    /// <summary>首次连接：时间轴从 0 开始，第一段没有断点。</summary>
    public MeasurementStore()
        : this(0.0, 0, false)
    {
    }

    /// <summary>
    /// 断线续接用。时间轴从 <paramref name="startTimeOffsetSeconds"/> 继续（紧接上一段最后一点，不留空档），
    /// <paramref name="motorGenerationSeed"/> 沿用上一段的电机批次号，使界面不会因为换了新会话就把旧曲线清掉；
    /// <paramref name="breakAtStart"/> = true 时第一个点之前先断开曲线，避免跨越断线画出一条“假线”。
    /// </summary>
    public MeasurementStore(double startTimeOffsetSeconds, long motorGenerationSeed, bool breakAtStart)
    {
        if (!double.IsFinite(startTimeOffsetSeconds) || startTimeOffsetSeconds < 0.0)
        {
            throw new ArgumentOutOfRangeException(
                nameof(startTimeOffsetSeconds),
                startTimeOffsetSeconds,
                "时间偏移必须是非负有限值（秒）。");
        }

        _startTimeOffsetSeconds = startTimeOffsetSeconds;
        _motorGeneration = motorGenerationSeed;
        if (breakAtStart)
        {
            _breakPending = true;
            _motorBreakPending = true;
        }
    }

    /// <summary>当前批次号。每次"重置显示"递增。</summary>
    public long Generation
    {
        get
        {
            lock (_gate)
            {
                return _generation;
            }
        }
    }

    /// <summary>已接受的 0x321 采样点数。</summary>
    public long DataAcceptedCount { get; private set; }

    /// <summary>电机曲线当前批次号。“电机显示归零”和全局重置都会使其递增。</summary>
    public long MotorGeneration
    {
        get
        {
            lock (_gate)
            {
                return _motorGeneration;
            }
        }
    }

    /// <summary>已接受的 0x323 采样点数。</summary>
    public long RateAcceptedCount { get; private set; }

    /// <summary>已接受的 0x324+0x325 原始采样记录数。</summary>
    public long RawAcceptedCount { get; private set; }

    /// <summary>已接受的电机位置采样点数。</summary>
    public long MotorAcceptedCount { get; private set; }

    /// <summary>因到达时刻早于重置时刻而被丢弃的点数（重置前排队的旧数据）。</summary>
    public long DroppedBeforeResetCount { get; private set; }

    /// <summary>因队列已满而被丢弃的点数。</summary>
    public long PendingOverflowCount { get; private set; }

    /// <summary>
    /// 写入一个 0x321 采样点。返回 false 表示被丢弃（早于重置时刻或队列已满）。
    /// </summary>
    public bool AddData(TimeSpan arrival, double angleDeg, double temperatureC, GyroStatus status, int intervalMs)
    {
        lock (_gate)
        {
            EnsureEpoch(arrival);

            if (arrival < _epoch)
            {
                DroppedBeforeResetCount++;
                return false;
            }

            bool breakBefore = _breakPending;
            _breakPending = false;

            _pendingData.Add(new DataSample(
                arrival,
                _startTimeOffsetSeconds + (arrival - _epoch).TotalSeconds,
                angleDeg,
                temperatureC,
                status,
                intervalMs,
                breakBefore));

            if (_pendingData.Count > MaxPendingSamples)
            {
                _pendingData.RemoveAt(0);
                PendingOverflowCount++;
            }

            DataAcceptedCount++;
            return true;
        }
    }

    /// <summary>写入一个 0x323 采样点。返回 false 表示被丢弃。</summary>
    public bool AddRate(TimeSpan arrival, double rateDps)
    {
        lock (_gate)
        {
            EnsureEpoch(arrival);

            if (arrival < _epoch)
            {
                DroppedBeforeResetCount++;
                return false;
            }

            _pendingRate.Add(new RateSample(arrival, _startTimeOffsetSeconds + (arrival - _epoch).TotalSeconds, rateDps));

            if (_pendingRate.Count > MaxPendingSamples)
            {
                _pendingRate.RemoveAt(0);
                PendingOverflowCount++;
            }

            RateAcceptedCount++;
            return true;
        }
    }

    /// <summary>
    /// 写入一条 0x324+0x325 配对后的原始采样记录。返回 false 表示被丢弃。
    /// </summary>
    public bool AddRaw(
        TimeSpan arrival,
        short rawCode,
        uint sampleTickMs,
        double biasDps,
        double filteredDps)
    {
        lock (_gate)
        {
            EnsureEpoch(arrival);

            if (arrival < _epoch)
            {
                DroppedBeforeResetCount++;
                return false;
            }

            _pendingRaw.Add(new RawSample(
                arrival,
                _startTimeOffsetSeconds + (arrival - _epoch).TotalSeconds,
                rawCode,
                sampleTickMs,
                biasDps,
                filteredDps));

            if (_pendingRaw.Count > MaxPendingSamples)
            {
                _pendingRaw.RemoveAt(0);
                PendingOverflowCount++;
            }

            RawAcceptedCount++;
            return true;
        }
    }

    /// <summary>
    /// 写入一个电机位置采样点。返回 false 表示被丢弃（早于电机时间轴重置时刻或队列已满）。
    /// </summary>
    public bool AddMotor(TimeSpan arrival, int rawPosition, double displacementMm, double relativeAngleDeg)
    {
        lock (_gate)
        {
            // 电机曲线与陀螺仪曲线共用同一个时间基准：如果电机先到，就把全局时间 0 定在这里。
            EnsureEpoch(arrival);
            EnsureMotorEpoch(arrival);

            if (arrival < _motorEpoch)
            {
                DroppedBeforeResetCount++;
                return false;
            }

            bool breakBefore = _motorBreakPending;
            _motorBreakPending = false;

            _pendingMotor.Add(new MotorPositionSample(
                arrival,
                _startTimeOffsetSeconds + (arrival - _motorEpoch).TotalSeconds,
                _startTimeOffsetSeconds + (arrival - _epoch).TotalSeconds,
                rawPosition,
                displacementMm,
                relativeAngleDeg,
                breakBefore));

            if (_pendingMotor.Count > MaxPendingSamples)
            {
                _pendingMotor.RemoveAt(0);
                PendingOverflowCount++;
            }

            MotorAcceptedCount++;
            return true;
        }
    }

    /// <summary>
    /// 标记下一条 0x321 采样点需要断开曲线（板端清零造成的跳变、重新校准造成的采样中断）。
    /// </summary>
    public void MarkAngleBreak()
    {
        lock (_gate)
        {
            _breakPending = true;
        }
    }

    /// <summary>
    /// 标记下一条电机位置采样点需要断开曲线（位置反馈超时后恢复）。
    /// </summary>
    public void MarkMotorBreak()
    {
        lock (_gate)
        {
            _motorBreakPending = true;
        }
    }

    /// <summary>
    /// 只重置电机曲线：清空待绘制电机数据、批次号 +1、电机时间轴从 <paramref name="now"/> 重新计时。
    /// 陀螺仪数据、时间轴和连接都不受影响。
    /// </summary>
    public void ResetMotorDisplay(TimeSpan now)
    {
        lock (_gate)
        {
            _pendingMotor.Clear();
            _motorBreakPending = false;
            _motorEpoch = now;
            _motorEpochInitialized = true;
            _motorGeneration++;
        }
    }

    /// <summary>
    /// 重置显示：清空待绘制数据、批次号 +1、相对时间基准设为 <paramref name="now"/>。
    /// 到达时刻早于该时刻的数据（重置前已经在队列或串口缓冲里的）会被全部丢弃。
    /// <para>电机曲线也一起清空并重新计时，但电机起点 P0 不变（P0 只由“电机显示归零”改变）。</para>
    /// </summary>
    public void ResetDisplay(TimeSpan now)
    {
        lock (_gate)
        {
            _pendingData.Clear();
            _pendingRate.Clear();
            _pendingRaw.Clear();
            _pendingMotor.Clear();
            _breakPending = false;
            _motorBreakPending = false;
            _epoch = now;
            _epochInitialized = true;
            _motorEpoch = now;
            _motorEpochInitialized = true;

            // 清零后时间轴真的回到 0：断线续接时留下的起点偏移必须一起清掉。
            _startTimeOffsetSeconds = 0.0;
            _generation++;
            _motorGeneration++;
        }
    }

    /// <summary>取走并清空待绘制数据。</summary>
    public TransferBatch Drain()
    {
        lock (_gate)
        {
            DataSample[] data = _pendingData.ToArray();
            RateSample[] rate = _pendingRate.ToArray();
            RawSample[] raw = _pendingRaw.ToArray();
            MotorPositionSample[] motor = _pendingMotor.ToArray();
            _pendingData.Clear();
            _pendingRate.Clear();
            _pendingRaw.Clear();
            _pendingMotor.Clear();
            return new TransferBatch(_generation, data, rate, raw, motor, _motorGeneration);
        }
    }

    private void EnsureEpoch(TimeSpan arrival)
    {
        if (!_epochInitialized)
        {
            // 首个采样点作为时间 0，避免曲线从"进程启动到第一帧"的空白开始。
            _epoch = arrival;
            _epochInitialized = true;
        }
    }

    private void EnsureMotorEpoch(TimeSpan arrival)
    {
        if (!_motorEpochInitialized)
        {
            _motorEpoch = arrival;
            _motorEpochInitialized = true;
        }
    }
}
