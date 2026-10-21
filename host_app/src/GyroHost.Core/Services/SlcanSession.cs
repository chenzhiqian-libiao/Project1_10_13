using GyroHost.Core.Communication;
using GyroHost.Core.Protocol;

namespace GyroHost.Core.Services;

/// <summary>重新零偏校准的过程状态。板端没有命令应答，只能靠回传状态位推断。</summary>
public enum CalibrationPhase
{
    /// <summary>没有进行中的校准。</summary>
    Idle,

    /// <summary>命令已发出，等待板端状态发生变化。</summary>
    Requested,

    /// <summary>已观察到"校准完成"标志消失，说明板端确实进入了采样。</summary>
    InProgress,

    /// <summary>本次校准成功（观察到 未完成 → 完成 的完整变化）。</summary>
    Completed,

    /// <summary>本次校准失败（板端置起校准失败位）。</summary>
    Failed,

    /// <summary>命令已发送，但超时未观察到状态变化（可能是板端没收到）。</summary>
    NoResponse,
}

/// <summary>板端角度清零命令的状态。</summary>
public enum ZeroAnglePhase
{
    /// <summary>没有进行中的清零。</summary>
    Idle,

    /// <summary>命令已发出，等待回传数据验证。</summary>
    Sent,

    /// <summary>已由后续数据确认角度归零。</summary>
    Verified,

    /// <summary>命令已发送，但超时未观察到角度归零。</summary>
    NoChange,
}

/// <summary>会话计数器快照。</summary>
public readonly record struct SessionCounters(
    long DataFrames,
    long RateFrames,
    long RawFrames,
    long RawPairMismatches,
    long CommandEchoFrames,
    long UnsupportedFrames,
    long BadDlcFrames,
    long MalformedLines,
    long TxAckAccepted,
    long TxAckRejected,
    long AdapterStatusEvents,
    long DroppedBeforeReset,
    long PendingOverflow);

/// <summary>一次取走的会话状态 + 待绘制数据。</summary>
public sealed record SessionSnapshot(
    long Generation,
    IReadOnlyList<DataSample> Data,
    IReadOnlyList<RateSample> Rate,
    IReadOnlyList<RawSample> Raw,
    SessionCounters Counters,
    TimeSpan? LastDataArrival,
    TimeSpan? LastRateArrival,
    DataPayload LastData,
    double LastRateDps,
    bool HasData,
    bool HasRate,
    CalibrationPhase Calibration,
    ZeroAnglePhase ZeroAngle,
    LinkState LinkState,
    string LinkMessage,
    IReadOnlyList<string> LogLines,
    MotorSnapshot Motor);

/// <summary>
/// 一次完整会话：连接 → SLCAN 解析 → 协议解码 → 数据仓库，并管理控制命令。
/// <para>
/// 界面只通过 <see cref="TakeSnapshot"/> 批量取数据与状态，避免逐帧刷新 UI，
/// 也避免把后台线程的对象直接交给绑定层。
/// </para>
/// </summary>
public sealed class SlcanSession : IDisposable
{
    /// <summary>清零后判定"已归零"的角度阈值（°）。固件清零是精确归零，留余量即可。</summary>
    public const double ZeroVerifiedThresholdDeg = 5.0;

    private const int MaxLogLines = 400;
    private const int MaxMotorRawLogLines = 2000;
    private const int MinMotorPollIntervalMs = 20;
    private const int MaxMotorPollIntervalMs = 5000;
    private const int MinMotorReplyTimeoutMs = 20;
    private const int MaxMotorReplyTimeoutMs = 5000;
    private const int MinMotorDataTimeoutMs = 100;
    private const int MaxMotorDataTimeoutMs = 10000;

    private readonly ICanConnection _connection;
    private readonly SlcanFrameParser _parser = new();
    private readonly MeasurementStore _store;
    private readonly IMotorPositionDecoder _motorDecoder;
    private readonly MotorResumeState? _resume;

    /// <summary>串行化解析，保证即使字节来自多个线程也不会交叉解析。</summary>
    private readonly object _parseGate = new();

    /// <summary>保护计数器与状态机的锁。</summary>
    private readonly object _gate = new();

    private readonly List<string> _log = new();
    private readonly List<string> _motorRawLog = new();

    private CancellationTokenSource? _motorPollCts;
    private Task? _motorPollTask;

    private long _dataFrames;
    private long _rateFrames;
    private long _rawFrames;
    private long _rawPairMismatches;
    private long _commandEchoFrames;
    private long _unsupportedFrames;
    private long _badDlcFrames;
    private long _malformedLines;
    private long _txAckAccepted;
    private long _txAckRejected;
    private long _adapterStatusEvents;

    private TimeSpan? _lastDataArrival;
    private TimeSpan? _lastRateArrival;
    private DataPayload _lastData;
    private double _lastRateDps;
    private bool _hasData;
    private bool _hasRate;

    /// <summary>等待 0x325 配对的 0x324 原始采样（两帧由固件背靠背发出）。</summary>
    private RawPayload _pendingRaw;
    private TimeSpan _pendingRawArrival;
    private bool _hasPendingRaw;

    private CalibrationPhase _calibration = CalibrationPhase.Idle;
    private TimeSpan _calibrationSentAt;
    private ZeroAnglePhase _zeroAngle = ZeroAnglePhase.Idle;
    private TimeSpan _zeroAngleSentAt;

    private LinkState _linkState = LinkState.Disconnected;
    private string _linkMessage = "未连接。";
    private bool _disposed;

    // ------------------------------------------------------------ 电机位置监视
    // 默认关闭轮询：会话层测试/模拟器不会凭空向总线发查询。
    // 界面在连接时按用户设置（默认 100 ms）打开。
    private bool _motorPollingEnabled;
    private byte _motorDeviceId = MotorProtocol.DefaultDeviceId;
    private double _motorWheelDiameterMm = MotorProtocol.DefaultWheelDiameterMm;
    private int _motorPollIntervalMs = 100;
    private int _motorReplyTimeoutMs = 250;
    private int _motorDataTimeoutMs = 600;

    private long _motorQueriesSent;
    private long _motorPositionReplies;
    private long _motorAllReplies;
    private long _motorTimeouts;
    private long _motorRejectedReplies;
    private long _motorUnconfirmedReplies;
    private long _motorRemainingDisplacementFrames;
    private long _motorTorqueFrames;
    private long _motorHeartbeatFrames;
    private long _motorOtherQueryFrames;
    private long _motorErrorFrames;
    private ushort? _motorLastErrorCode;
    private TimeSpan? _motorLastErrorArrival;

    /// <summary>设备号/连接配置代数。应答处理期间配置变了，旧应答不能更新实时值。</summary>
    private long _motorConfigGeneration;

    private bool _motorHasReference;
    private int _motorReferenceRaw;
    private bool _motorHasPosition;
    private int _motorLastRaw;
    private double _motorLastDisplacementMm;
    private double _motorLastRelativeAngleDeg;
    private TimeSpan? _motorLastPositionArrival;
    private TimeSpan? _motorLastReplyArrival;
    private int _motorLastReplyDlc;
    private string? _motorLastReplyHex;
    private TimeSpan? _motorLastQueryAt;
    private TimeSpan? _motorPendingSentAt;
    private bool _motorPending;
    private MotorRequestState _motorRequestState = MotorRequestState.Idle;
    private bool _motorFresh;
    private bool _motorTimeoutLogged;
    private bool _motorUnconfirmedReplyLogged;

    private bool _motorHasRemainingDisplacement;
    private int _motorRemainingRaw;
    private TimeSpan? _motorRemainingArrival;

    private long _motorTargetCommandsSent;
    private long _motorParameterAcks;
    private uint? _motorLastAckEchoId;
    private TimeSpan? _motorLastAckArrival;

    /// <summary>
    /// 构造会话。
    /// </summary>
    /// <param name="connection">底层 CAN 连接（SLCAN 串口或测试替身）。</param>
    /// <param name="motorPositionDecoder">0x0409 位置解码器；null = 用默认（待实机验证）布局。</param>
    /// <param name="resume">
    /// 断线重连时沿用上一段的时间轴 / 电机批次号 / P0。null = 全新开始（时间轴从 0、清电机曲线）。
    /// </param>
    public SlcanSession(
        ICanConnection connection,
        IMotorPositionDecoder? motorPositionDecoder = null,
        MotorResumeState? resume = null)
    {
        _connection = connection ?? throw new ArgumentNullException(nameof(connection));
        _motorDecoder = motorPositionDecoder ?? MotorProtocol.CreateDefaultPositionDecoder();
        _resume = resume;
        _store = resume is null
            ? new MeasurementStore()
            : new MeasurementStore(
                resume.TimeOffsetSeconds,
                resume.MotorGeneration,
                breakAtStart: resume.TimeOffsetSeconds > 0.0);
        _connection.BytesReceived += OnBytesReceived;
        _connection.StateChanged += OnConnectionStateChanged;
    }

    /// <summary>校准状态变化的观察超时（秒）。超过则认为板端未响应或状态未变。</summary>
    public double CalibrationTimeoutSeconds { get; set; } = GyroProtocol.CalibrationDurationSeconds + 20.0;

    /// <summary>角度清零的验证超时（秒）。</summary>
    public double ZeroVerifyTimeoutSeconds { get; set; } = 2.0;

    /// <summary>底层连接（界面用来读取描述信息）。</summary>
    public ICanConnection Connection => _connection;

    /// <summary>当前位置回复解码器（协议未确认时为 <see cref="UnconfirmedMotorPositionDecoder"/>）。</summary>
    public IMotorPositionDecoder MotorPositionDecoder => _motorDecoder;

    /// <summary>是否启用电机位置轮询。默认关闭，界面连接时按用户设置打开。</summary>
    public bool MotorPollingEnabled
    {
        get
        {
            lock (_gate)
            {
                return _motorPollingEnabled;
            }
        }

        set
        {
            lock (_gate)
            {
                _motorPollingEnabled = value;
            }
        }
    }

    /// <summary>轮询的设备号（1..255）。修改后 P0、统计与电机曲线会重置，但不向电机发送配置命令。</summary>
    public byte MotorDeviceId
    {
        get
        {
            lock (_gate)
            {
                return _motorDeviceId;
            }
        }

        set
        {
            if (value == 0 || value > MotorProtocol.MaxDeviceId)
            {
                throw new ArgumentOutOfRangeException(nameof(value), value, $"设备号必须为 1..{MotorProtocol.MaxDeviceId}。");
            }

            bool changed;
            lock (_gate)
            {
                changed = _motorDeviceId != value;
                if (changed)
                {
                    _motorDeviceId = value;
                    ResetMotorStateLocked();
                }
            }

            if (changed)
            {
                _store.ResetMotorDisplay(MonotonicClock.Now);
                AppendLog($"电机设备号已改为 {value:00}：P0、位置统计与电机曲线已重置（未向电机发送配置命令）。");
            }
        }
    }

    /// <summary>上位机角度换算用的轮径（mm）。只影响换算，不写回电机。</summary>
    public double MotorWheelDiameterMm
    {
        get
        {
            lock (_gate)
            {
                return _motorWheelDiameterMm;
            }
        }

        set
        {
            if (!double.IsFinite(value) || value <= 0.0 || value > 100000.0)
            {
                throw new ArgumentOutOfRangeException(nameof(value), value, "轮径必须是 0.1..100000 mm 的有限值。");
            }

            lock (_gate)
            {
                _motorWheelDiameterMm = value;
            }
        }
    }

    /// <summary>位置查询周期（ms）。默认 100 ms。</summary>
    public int MotorPollIntervalMs
    {
        get
        {
            lock (_gate)
            {
                return _motorPollIntervalMs;
            }
        }

        set
        {
            lock (_gate)
            {
                _motorPollIntervalMs = Math.Clamp(value, MinMotorPollIntervalMs, MaxMotorPollIntervalMs);
            }
        }
    }

    /// <summary>单次位置查询的应答超时（ms）。超时前不会发下一条查询，避免请求堆积。</summary>
    public int MotorReplyTimeoutMs
    {
        get
        {
            lock (_gate)
            {
                return _motorReplyTimeoutMs;
            }
        }

        set
        {
            lock (_gate)
            {
                _motorReplyTimeoutMs = Math.Clamp(value, MinMotorReplyTimeoutMs, MaxMotorReplyTimeoutMs);
            }
        }
    }

    /// <summary>位置数据失效超时（ms）。超过后界面必须显示“--”，不能沿用旧值。</summary>
    public int MotorDataTimeoutMs
    {
        get
        {
            lock (_gate)
            {
                return _motorDataTimeoutMs;
            }
        }

        set
        {
            lock (_gate)
            {
                _motorDataTimeoutMs = Math.Clamp(value, MinMotorDataTimeoutMs, MaxMotorDataTimeoutMs);
            }
        }
    }

    /// <summary>连接 + SLCAN 初始化。</summary>
    public async Task ConnectAsync(CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        lock (_parseGate)
        {
            _parser.Reset();
        }

        lock (_gate)
        {
            ResetMotorStateLocked();

            // 断线重连：沿用上一段的 P0，曲线才画在同一根零点上。
            if (_resume?.ReferenceRawPosition is { } seed)
            {
                _motorHasReference = true;
                _motorReferenceRaw = seed;
            }
        }

        if (_resume is null)
        {
            // 全新开始才重置电机曲线；重连时旧图线必须保留（批次号不变，界面不会清空）。
            _store.ResetMotorDisplay(MonotonicClock.Now);
        }

        await _connection.ConnectAsync(cancellationToken).ConfigureAwait(false);
        StartMotorPoller();
    }

    /// <summary>断开连接并释放接收资源。可重复调用。</summary>
    public async Task DisconnectAsync()
    {
        if (_disposed)
        {
            return;
        }

        await StopMotorPollerAsync().ConfigureAwait(false);
        await _connection.DisconnectAsync().ConfigureAwait(false);
    }

    /// <summary>
    /// “电机显示归零”：把当前有效位置记为新的起点 P0，清空电机曲线并让电机时间轴从 0 重新计时。
    /// <para>只改上位机显示，不发送任何电机归零/配置命令。没有有效位置时返回 false。</para>
    /// </summary>
    public bool ZeroMotorDisplay()
    {
        ObjectDisposedException.ThrowIf(_disposed, this);

        UpdateMotorTimeouts();
        TimeSpan now = MonotonicClock.Now;
        int referenceRaw;
        lock (_gate)
        {
            if (!_motorHasPosition || !_motorFresh)
            {
                return false;
            }

            _motorHasReference = true;
            _motorReferenceRaw = _motorLastRaw;
            referenceRaw = _motorReferenceRaw;
        }

        _store.ResetMotorDisplay(now);
        AppendLog($"电机显示已归零：P0={referenceRaw}（原始值）。只更新上位机换算与曲线，未向电机发送归零命令。");
        return true;
    }

    /// <summary>
    /// 重置显示：清空曲线数据、历史与待绘制数据，相对时间从 0 重新开始。
    /// 只影响上位机显示，不发送任何命令，连接保持。
    /// </summary>
    public void ResetDisplay()
    {
        _store.ResetDisplay(MonotonicClock.Now);
        AppendLog("已重置显示数据：曲线与缓存清空，相对时间从 0 重新计时（板端数据未改动；电机 P0 保持不变）。");
    }

    /// <summary>
    /// 清掉电机起点 P0：下一个有效位置反馈会重新作为 P0，电机曲线从 0 起画。
    /// 只影响上位机换算，不发送任何电机命令。
    /// </summary>
    public void ResetMotorReference()
    {
        lock (_gate)
        {
            _motorHasReference = false;
            _motorReferenceRaw = 0;
        }
    }

    /// <summary>发送 0x322 / DLC=1 / 数据 02：板端角度清零（单次发送）。</summary>
    public async Task SendZeroAngleAsync(CancellationToken cancellationToken = default)
    {
        await SendCommandAsync(GyroProtocol.CommandZeroAngle, cancellationToken).ConfigureAwait(false);
        lock (_gate)
        {
            _zeroAngle = ZeroAnglePhase.Sent;
            _zeroAngleSentAt = MonotonicClock.Now;
        }

        AppendLog("已发送板端角度清零命令（0x322 = 02）。等待回传数据确认实际结果。");
    }

    /// <summary>发送 0x322 / DLC=1 / 数据 01：重新零偏校准（单次发送）。</summary>
    public async Task SendRecalibrateAsync(CancellationToken cancellationToken = default)
    {
        await SendCommandAsync(GyroProtocol.CommandRecalibrate, cancellationToken).ConfigureAwait(false);
        lock (_gate)
        {
            _calibration = CalibrationPhase.Requested;
            _calibrationSentAt = MonotonicClock.Now;
        }

        AppendLog("已发送重新零偏校准命令（0x322 = 01）。请保持板子静止，等待状态变化确认结果。");
    }

    /// <summary>取走待绘制数据与最新状态。</summary>
    public SessionSnapshot TakeSnapshot()
    {
        UpdateMotorTimeouts();
        TransferBatch batch = _store.Drain();

        lock (_gate)
        {
            string[] logs = _log.ToArray();
            _log.Clear();
            string[] motorRawLogs = _motorRawLog.ToArray();
            _motorRawLog.Clear();

            MotorSnapshot motor = new(
                batch.Motor,
                batch.MotorGeneration,
                _motorDecoder.IsConfirmed,
                _motorDecoder.Description,
                _motorDecoder.MissingFields,
                _motorRequestState,
                _motorHasPosition,
                _motorFresh,
                _motorHasReference,
                _motorReferenceRaw,
                _motorLastRaw,
                _motorLastDisplacementMm,
                _motorLastRelativeAngleDeg,
                _motorLastPositionArrival,
                _motorLastQueryAt,
                _motorLastReplyArrival,
                _motorLastReplyDlc,
                _motorLastReplyHex,
                _motorTargetCommandsSent,
                _motorParameterAcks,
                _motorLastAckEchoId,
                _motorLastAckArrival,
                _motorDeviceId,
                _motorWheelDiameterMm,
                _motorPollIntervalMs,
                _motorReplyTimeoutMs,
                _motorDataTimeoutMs,
                _motorQueriesSent,
                _motorPositionReplies,
                _motorAllReplies,
                _motorTimeouts,
                _motorRejectedReplies,
                _motorUnconfirmedReplies,
                _motorRemainingDisplacementFrames,
                _motorHasRemainingDisplacement,
                _motorRemainingRaw,
                MotorProtocol.PositionLsbToMillimeters(_motorRemainingRaw),
                _motorRemainingArrival,
                _motorTorqueFrames,
                _motorHeartbeatFrames,
                _motorOtherQueryFrames,
                _motorErrorFrames,
                _motorLastErrorCode,
                _motorLastErrorArrival,
                motorRawLogs);

            return new SessionSnapshot(
                batch.Generation,
                batch.Data,
                batch.Rate,
                batch.Raw,
                new SessionCounters(
                    _dataFrames,
                    _rateFrames,
                    _rawFrames,
                    _rawPairMismatches,
                    _commandEchoFrames,
                    _unsupportedFrames,
                    _badDlcFrames,
                    _malformedLines,
                    _txAckAccepted,
                    _txAckRejected,
                    _adapterStatusEvents,
                    _store.DroppedBeforeResetCount,
                    _store.PendingOverflowCount),
                _lastDataArrival,
                _lastRateArrival,
                _lastData,
                _lastRateDps,
                _hasData,
                _hasRate,
                _calibration,
                _zeroAngle,
                _linkState,
                _linkMessage,
                logs,
                motor);
        }
    }

    /// <inheritdoc />
    public void Dispose()
    {
        if (_disposed)
        {
            return;
        }

        _disposed = true;
        _connection.BytesReceived -= OnBytesReceived;
        _connection.StateChanged -= OnConnectionStateChanged;

        try
        {
            StopMotorPollerAsync().GetAwaiter().GetResult();
        }
        catch (Exception)
        {
            // 释放路径不抛异常。
        }

        _connection.Dispose();
    }

    private async Task SendCommandAsync(byte command, CancellationToken cancellationToken)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        if (!_connection.IsConnected)
        {
            throw new InvalidOperationException("尚未连接，无法发送控制命令。");
        }

        byte[] payload = GyroProtocol.BuildCommandFrame(command);
        string line = SlcanCodec.EncodeStandardDataFrame(GyroProtocol.CommandId, payload);
        await _connection.SendSlcanLineAsync(line, cancellationToken).ConfigureAwait(false);
    }

    // ------------------------------------------------------------ 电机位置轮询

    /// <summary>
    /// 查询一次当前位置（0x0408 / DLC=2 / [设备号, 0x08]）。
    /// <para>
    /// 同一设备最多保留一个待应答查询：已有请求未完成时返回 false。
    /// 正常轮询由后台任务调用；测试或手动刷新也可以直接调用。
    /// </para>
    /// </summary>
    public async Task<bool> QueryMotorPositionOnceAsync(CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        if (!_connection.IsConnected)
        {
            throw new InvalidOperationException("尚未连接，无法查询电机位置。");
        }

        UpdateMotorTimeouts();

        byte deviceId;
        lock (_gate)
        {
            if (_motorPending)
            {
                return false;
            }

            TimeSpan now = MonotonicClock.Now;
            _motorPending = true;
            _motorPendingSentAt = now;
            _motorLastQueryAt = now;
            _motorRequestState = MotorRequestState.WaitingForReply;
            _motorQueriesSent++;
            deviceId = _motorDeviceId;
        }

        byte[] payload = MotorProtocol.BuildPositionQuery(deviceId);
        string line = SlcanCodec.EncodeStandardDataFrame(MotorProtocol.QueryId, payload);
        AppendMotorRawTx(
            MotorProtocol.QueryId,
            payload,
            $"位置查询（设备 {deviceId:00}，查询码 {MotorProtocol.QueryPositionCode:X2}）");

        try
        {
            await _connection.SendSlcanLineAsync(line, cancellationToken).ConfigureAwait(false);
            return true;
        }
        catch
        {
            lock (_gate)
            {
                _motorPending = false;
                _motorPendingSentAt = null;
                _motorRequestState = MotorRequestState.Idle;
                _motorQueriesSent--;
            }

            throw;
        }
    }

    // ------------------------------------------------------------ 电机运动控制指令

    /// <summary>
    /// 发送 <c>0x0416</c>（DLC=6）：设置目标位置（主通道）。
    /// <para>
    /// 帧格式来自协议表：<c>Data[0..3]</c>=目标位置 s32 小端（0.1 mm）、<c>Data[4]</c>=设备号、
    /// <c>Data[5]</c>=目标类型。设备的确认帧是 <c>0x041A</c>；本机只记录 ACK 与原始日志，不因缺少 ACK 而中断运动。
    /// </para>
    /// </summary>
    public async Task SendMotorTargetPositionAsync(
        int rawPosition,
        MotorTargetType targetType,
        CancellationToken cancellationToken = default)
    {
        byte deviceId = GetMotorDeviceIdOrThrow();
        byte[] payload = MotorProtocol.BuildTargetPosition(rawPosition, deviceId, targetType);
        await SendMotorFrameAsync(
            MotorProtocol.TargetPositionId,
            payload,
            $"设置目标位置 {rawPosition} LSB（{MotorProtocol.PositionLsbToMillimeters(rawPosition):0.0} mm），" +
            $"类型 {MotorProtocol.DescribeTargetType(targetType)}（0x{(byte)targetType:X2}），设备 {deviceId:00}",
            cancellationToken).ConfigureAwait(false);

        lock (_gate)
        {
            _motorTargetCommandsSent++;
        }
    }

    /// <summary>发送 <c>0x0413</c>（DLC=5）：锁定（上电使能 <c>0x5A</c>）或释放（<c>0x4A</c>）。</summary>
    public async Task SendMotorEnableAsync(bool enable, CancellationToken cancellationToken = default)
    {
        byte deviceId = GetMotorDeviceIdOrThrow();
        byte[] payload = MotorProtocol.BuildEnable(deviceId, enable);
        await SendMotorFrameAsync(
            MotorProtocol.EnableId,
            payload,
            $"{(enable ? "锁定（上电使能）" : "释放（下电）")}，设备 {deviceId:00}",
            cancellationToken).ConfigureAwait(false);
    }

    /// <summary>
    /// 发送 <c>0x0316</c>（DLC=8）：参数初始化 / 建立 PID 基准。
    /// <para>⚠️ 补充资料帧，不在 V11 协议表内，**没有应答**；发送成功只表示已交给适配器。</para>
    /// </summary>
    public async Task SendMotorParameterInitAsync(CancellationToken cancellationToken = default)
    {
        byte deviceId = GetMotorDeviceIdOrThrow();
        byte[] payload = MotorProtocol.BuildParameterInit(deviceId);
        await SendMotorFrameAsync(
            MotorProtocol.ParameterInitId,
            payload,
            $"参数初始化（建立 PID 基准），设备 {deviceId:00}（无应答，只表示已发送）",
            cancellationToken).ConfigureAwait(false);
    }

    /// <summary>
    /// 发送 <c>0x0408</c>（DLC=4）配置轮径，按协议要求**连续写 3 次**（不插延时）。
    /// <para><c>Data[2..3]</c> 为 u16 小端，单位 0.01 mm；100.00 mm → <c>10 27</c>。</para>
    /// <para>⚠️ 补充资料帧，不在 V11 协议表内，**没有应答**；发送成功只表示已交给适配器。</para>
    /// </summary>
    public async Task SendMotorWheelDiameterConfigAsync(
        double wheelDiameterMm,
        CancellationToken cancellationToken = default)
    {
        byte deviceId = GetMotorDeviceIdOrThrow();
        ushort raw = MotorProtocol.MillimetersToWheelDiameterRaw(wheelDiameterMm);

        for (int i = 1; i <= MotorProtocol.WheelConfigRepeatCount; i++)
        {
            byte[] payload = MotorProtocol.BuildWheelDiameterConfig(deviceId, wheelDiameterMm);
            await SendMotorFrameAsync(
                MotorProtocol.QueryId,
                payload,
                $"配置轮径 {wheelDiameterMm:0.##} mm（原始值 {raw}，0x{raw:X4}）第 {i}/{MotorProtocol.WheelConfigRepeatCount} 次，" +
                $"设备 {deviceId:00}（无应答，只表示已发送）",
                cancellationToken).ConfigureAwait(false);
        }
    }

    /// <summary>发送 <c>0x0415</c>（DLC=5）：设置主通道速度（原始值，单位 0.1 mm/s）。</summary>
    public async Task SendMotorSpeedAsync(uint rawSpeed, CancellationToken cancellationToken = default)
    {
        byte deviceId = GetMotorDeviceIdOrThrow();
        byte[] payload = MotorProtocol.BuildSpeed(deviceId, rawSpeed);
        await SendMotorFrameAsync(
            MotorProtocol.SetSpeedId,
            payload,
            $"设置主速度 {rawSpeed}（{rawSpeed / 10.0:0.0} mm/s），设备 {deviceId:00}",
            cancellationToken).ConfigureAwait(false);
    }

    /// <summary>发送 <c>0x0414</c>（DLC=5）：设置主通道加速度（原始值，单位 0.1 mm，按协议表原文）。</summary>
    public async Task SendMotorAccelerationAsync(uint rawAcceleration, CancellationToken cancellationToken = default)
    {
        byte deviceId = GetMotorDeviceIdOrThrow();
        byte[] payload = MotorProtocol.BuildAcceleration(deviceId, rawAcceleration);
        await SendMotorFrameAsync(
            MotorProtocol.SetAccelerationId,
            payload,
            $"设置主加速度 {rawAcceleration}（协议表单位 0.1 mm），设备 {deviceId:00}",
            cancellationToken).ConfigureAwait(false);
    }

    private byte GetMotorDeviceIdOrThrow()
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        if (!_connection.IsConnected)
        {
            throw new InvalidOperationException("尚未连接，无法发送电机命令。");
        }

        lock (_gate)
        {
            return _motorDeviceId;
        }
    }

    private async Task SendMotorFrameAsync(
        uint id,
        byte[] payload,
        string note,
        CancellationToken cancellationToken)
    {
        string line = SlcanCodec.EncodeStandardDataFrame(id, payload);
        AppendMotorRawTx(id, payload, note);
        await _connection.SendSlcanLineAsync(line, cancellationToken).ConfigureAwait(false);
    }

    private void StartMotorPoller()
    {
        lock (_gate)
        {
            if (_motorPollCts is not null)
            {
                return;
            }

            CancellationTokenSource cts = new();
            _motorPollCts = cts;
            _motorPollTask = Task.Run(() => RunMotorPollingAsync(cts.Token));
        }
    }

    private async Task StopMotorPollerAsync()
    {
        CancellationTokenSource? cts;
        Task? task;
        lock (_gate)
        {
            cts = _motorPollCts;
            task = _motorPollTask;
            _motorPollCts = null;
            _motorPollTask = null;
        }

        if (cts is null)
        {
            return;
        }

        try
        {
            cts.Cancel();
        }
        catch (ObjectDisposedException)
        {
            // 已释放。
        }

        if (task is not null && task.Id != Task.CurrentId)
        {
            try
            {
                await task.WaitAsync(TimeSpan.FromSeconds(1)).ConfigureAwait(false);
            }
            catch (OperationCanceledException)
            {
                // 正常停止。
            }
            catch (TimeoutException)
            {
                AppendLog("[电机] 停止位置轮询超时，后台任务正在退出。");
            }
            catch (Exception ex)
            {
                AppendLog($"[电机] 停止位置轮询时出现异常：{ex.Message}");
            }
        }

        cts.Dispose();
    }

    private async Task RunMotorPollingAsync(CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            bool failed = false;
            try
            {
                UpdateMotorTimeouts();
                if (ShouldSendMotorQuery())
                {
                    await QueryMotorPositionOnceAsync(token).ConfigureAwait(false);
                }
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception ex)
            {
                failed = true;
                if (!token.IsCancellationRequested)
                {
                    AppendLog($"[电机] 位置轮询失败：{ex.Message}");
                }

                if (!_connection.IsConnected)
                {
                    break;
                }
            }

            int delay = failed ? 500 : GetMotorPollDelayMs();
            try
            {
                await Task.Delay(delay, token).ConfigureAwait(false);
            }
            catch (OperationCanceledException)
            {
                break;
            }
        }
    }

    private bool ShouldSendMotorQuery()
    {
        TimeSpan now = MonotonicClock.Now;
        lock (_gate)
        {
            if (!_motorPollingEnabled || _motorPending || !_connection.IsConnected)
            {
                return false;
            }

            return _motorLastQueryAt is not { } last ||
                   now - last >= TimeSpan.FromMilliseconds(_motorPollIntervalMs);
        }
    }

    private int GetMotorPollDelayMs()
    {
        TimeSpan now = MonotonicClock.Now;
        lock (_gate)
        {
            if (!_motorPollingEnabled)
            {
                return 100;
            }

            TimeSpan until;
            if (_motorPending && _motorPendingSentAt is { } sentAt)
            {
                until = sentAt + TimeSpan.FromMilliseconds(_motorReplyTimeoutMs) - now;
            }
            else if (_motorLastQueryAt is { } lastQuery)
            {
                until = lastQuery + TimeSpan.FromMilliseconds(_motorPollIntervalMs) - now;
            }
            else
            {
                until = TimeSpan.Zero;
            }

            return (int)Math.Clamp(until.TotalMilliseconds, 5.0, 50.0);
        }
    }

    /// <summary>
    /// 更新两类超时：
    /// 1) 查询应答超时：同一设备只保留一个待应答请求，超时后才允许发下一条；
    /// 2) 位置数据超时：标记失效，并让下一个恢复点断开曲线。
    /// </summary>
    private void UpdateMotorTimeouts()
    {
        TimeSpan now = MonotonicClock.Now;
        bool replyTimedOut = false;
        bool dataTimedOut = false;

        lock (_gate)
        {
            if (_motorPending &&
                _motorPendingSentAt is { } sentAt &&
                now - sentAt >= TimeSpan.FromMilliseconds(_motorReplyTimeoutMs))
            {
                _motorPending = false;
                _motorPendingSentAt = null;
                _motorRequestState = MotorRequestState.TimedOut;
                _motorTimeouts++;
                if (!_motorTimeoutLogged)
                {
                    _motorTimeoutLogged = true;
                    replyTimedOut = true;
                }
            }

            if (_motorHasPosition && _motorLastPositionArrival is { } lastArrival)
            {
                bool fresh = now - lastArrival <= TimeSpan.FromMilliseconds(_motorDataTimeoutMs);
                if (!fresh && _motorFresh)
                {
                    _motorFresh = false;
                    dataTimedOut = true;
                }
                else if (fresh)
                {
                    _motorFresh = true;
                }
            }
            else
            {
                _motorFresh = false;
            }
        }

        if (replyTimedOut)
        {
            AppendLog($"电机位置查询超时：{_motorReplyTimeoutMs} ms 内未收到可确认的位置应答（设备 {_motorDeviceId:00}）。");
        }

        if (dataTimedOut)
        {
            _store.MarkMotorBreak();
            AppendLog($"电机位置反馈已超过 {_motorDataTimeoutMs} ms 未更新：数值标记失效，曲线在下一点断开。");
        }
    }

    private void ResetMotorStateLocked()
    {
        _motorConfigGeneration++;
        _motorQueriesSent = 0;
        _motorPositionReplies = 0;
        _motorAllReplies = 0;
        _motorTimeouts = 0;
        _motorRejectedReplies = 0;
        _motorUnconfirmedReplies = 0;
        _motorRemainingDisplacementFrames = 0;
        _motorTorqueFrames = 0;
        _motorHeartbeatFrames = 0;
        _motorOtherQueryFrames = 0;
        _motorErrorFrames = 0;
        _motorLastErrorCode = null;
        _motorLastErrorArrival = null;

        _motorHasReference = false;
        _motorReferenceRaw = 0;
        _motorHasPosition = false;
        _motorLastRaw = 0;
        _motorLastDisplacementMm = 0.0;
        _motorLastRelativeAngleDeg = 0.0;
        _motorLastPositionArrival = null;
        _motorLastReplyArrival = null;
        _motorLastReplyDlc = 0;
        _motorLastReplyHex = null;
        _motorLastQueryAt = null;
        _motorPending = false;
        _motorPendingSentAt = null;
        _motorRequestState = MotorRequestState.Idle;
        _motorFresh = false;
        _motorTimeoutLogged = false;
        _motorUnconfirmedReplyLogged = false;
        _motorHasRemainingDisplacement = false;
        _motorRemainingRaw = 0;
        _motorRemainingArrival = null;

        _motorTargetCommandsSent = 0;
        _motorParameterAcks = 0;
        _motorLastAckEchoId = null;
        _motorLastAckArrival = null;
    }

    // ------------------------------------------------------------ 电机报文处理

    private void HandleMotorReplyFrame(CanFrame frame, TimeSpan arrival)
    {
        byte expectedDeviceId;
        bool pending;
        bool wasFresh;
        bool hadPosition;
        double wheelDiameterMm;

        lock (_gate)
        {
            _motorAllReplies++;
            _motorLastReplyArrival = arrival;
            _motorLastReplyDlc = frame.Dlc;
            _motorLastReplyHex = frame.DataText;
            expectedDeviceId = _motorDeviceId;
            pending = _motorPending;
            wasFresh = _motorFresh;
            hadPosition = _motorHasPosition;
            wheelDiameterMm = _motorWheelDiameterMm;
        }

        bool decoded = _motorDecoder.TryDecode(frame.Data, expectedDeviceId, out MotorPositionReply reply);
        if (!decoded)
        {
            lock (_gate)
            {
                if (_motorDecoder.IsConfirmed)
                {
                    _motorRejectedReplies++;
                }
                else
                {
                    _motorUnconfirmedReplies++;
                }
            }

            bool logOnce = false;
            lock (_gate)
            {
                if (!_motorDecoder.IsConfirmed && !_motorUnconfirmedReplyLogged)
                {
                    _motorUnconfirmedReplyLogged = true;
                    logOnce = true;
                }
            }

            if (logOnce)
            {
                AppendLog("已收到 0x0409 原始回复，但协议表没有定义位置回复布局：只记录原始字节，不把任何 0x0409 当成位置。");
            }

            string reason = _motorDecoder.IsConfirmed
                ? (!pending
                    ? "当前没有待应答的位置查询，不能确认是本次请求的回复（已记录原始字节）"
                    : "设备号或查询码与本次位置查询不匹配，可能是温度/状态/版本回复")
                : "0x0409 回复布局未确认，不能判定为位置数据（已记录原始字节）";
            AppendMotorRaw("RX", frame, reason);
            return;
        }

        if (!pending)
        {
            lock (_gate)
            {
                _motorRejectedReplies++;
            }

            AppendMotorRaw("RX", frame, "没有待应答的位置查询，不能把该帧当成本次位置反馈");
            return;
        }

        double displacementMm = MotorProtocol.PositionLsbToMillimeters(reply.RawPosition);
        int referenceRaw;
        double angleDeg;
        bool firstPosition;
        bool recovered;
        long configGeneration;

        lock (_gate)
        {
            firstPosition = !_motorHasPosition;
            recovered = hadPosition && !wasFresh;
            referenceRaw = _motorHasReference ? _motorReferenceRaw : reply.RawPosition;
            angleDeg = MotorProtocol.PositionDeltaLsbToDegrees(reply.RawPosition - referenceRaw, wheelDiameterMm);
            configGeneration = _motorConfigGeneration;
        }

        // 先让仓库决定这条采样是否属于当前时间轴。全局重置后到达的旧帧会被丢弃，
        // 不能再用它更新实时值或 P0。
        bool stored = _store.AddMotor(arrival, reply.RawPosition, displacementMm, angleDeg);

        bool configChanged;
        lock (_gate)
        {
            configChanged = _motorConfigGeneration != configGeneration;
            if (!configChanged)
            {
                _motorPositionReplies++;
                _motorPending = false;
                _motorPendingSentAt = null;
                _motorRequestState = MotorRequestState.Received;
                _motorTimeoutLogged = false;

                if (stored)
                {
                    _motorHasPosition = true;
                    _motorLastRaw = reply.RawPosition;
                    _motorLastDisplacementMm = displacementMm;
                    _motorLastPositionArrival = arrival;
                    _motorFresh = true;

                    if (!_motorHasReference)
                    {
                        _motorHasReference = true;
                        _motorReferenceRaw = referenceRaw;
                    }

                    _motorLastRelativeAngleDeg = angleDeg;
                }
            }
        }

        if (configChanged)
        {
            AppendMotorRaw("RX", frame, $"设备号/连接在应答处理期间已变化，丢弃旧配置的位置反馈（原始值 {reply.RawPosition}）");
            return;
        }

        if (!stored)
        {
            AppendMotorRaw("RX", frame, $"位置反馈到达时刻早于显示重置，已丢弃，不更新实时值（原始值 {reply.RawPosition}）");
            return;
        }

        AppendMotorRaw(
            "RX",
            frame,
            $"位置反馈（设备 {reply.DeviceId:00}，查询码 {reply.QueryCode:X2}，原始值 {reply.RawPosition}，" +
            $"{displacementMm:0.0} mm，相对 P0={referenceRaw} 为 {angleDeg:F5}°）");

        if (firstPosition)
        {
            AppendLog($"收到首个有效电机位置：原始值 {reply.RawPosition}（{displacementMm:0.0} mm），已作为起点 P0。后续角度按轮径 {wheelDiameterMm:0.###} mm 换算。");
        }
        else if (recovered)
        {
            AppendLog("电机位置反馈已恢复（此前已标记超时）。");
        }
    }

    private void HandleOtherMotorQueryFrame(CanFrame frame)
    {
        lock (_gate)
        {
            _motorOtherQueryFrames++;
        }

        string note = frame.Dlc == MotorProtocol.WheelConfigDlc
            ? "收到 0x0408/DLC=4（配置轮径的回显或其它主控下发）；它是命令方向，不当作位置反馈"
            : "收到 0x0408（可能是其它主控的位置查询或本机回显）；它是查询方向，不当作位置反馈";
        AppendMotorRaw("RX", frame, note);
    }

    /// <summary>
    /// 处理 0x0421（剩余位移）、0x0441（力矩）、0x0381（心跳）等反馈帧。
    /// 这些都不是当前位置，不能参与电机角度换算。
    /// </summary>
    private bool TryHandleMotorFeedbackFrame(CanFrame frame, TimeSpan arrival)
    {
        if (MotorProtocol.IsErrorId(frame.Id))
        {
            if (frame.Dlc != 2)
            {
                lock (_gate)
                {
                    _badDlcFrames++;
                }

                AppendLog($"[电机] 错误上报 0x{frame.Id:X4} 期望 DLC=2，实际 {frame.Dlc}。");
                AppendMotorRaw("RX", frame, "错误上报 DLC 不符（协议表要求 2 字节）");
                return true;
            }

            // 协议表只给出 Data[0..1]=错误码（如 0x0104），未显式说明字节序。
            // 这里按 Data[0]<<8|Data[1] 显示；原始字节同时留在日志里，便于核对。
            ushort errorCode = (ushort)((frame.Data[0] << 8) | frame.Data[1]);
            bool isNewCode;
            lock (_gate)
            {
                _motorErrorFrames++;
                isNewCode = _motorLastErrorCode != errorCode;
                _motorLastErrorCode = errorCode;
                _motorLastErrorArrival = arrival;
            }

            AppendMotorRaw(
                "RX",
                frame,
                $"错误上报：0x{errorCode:X4} {MotorProtocol.DescribeErrorCode(errorCode)}（按 Data[0]<<8|Data[1] 显示）");
            if (isNewCode)
            {
                AppendLog($"[电机] 收到错误上报 0x{errorCode:X4}：{MotorProtocol.DescribeErrorCode(errorCode)}。");
            }

            return true;
        }

        if (frame.Id == MotorProtocol.ParameterAckId)
        {
            HandleParameterAckFrame(frame, arrival);
            return true;
        }

        if (MotorProtocol.IsRemainingDisplacementId(frame.Id) ||
            frame.Id == MotorProtocol.RemainingDisplacementIdBase + _motorDeviceId)
        {
            bool ours = frame.Id == MotorProtocol.RemainingDisplacementIdBase + _motorDeviceId;
            if (ours)
            {
                if (frame.Dlc != 8 || !MotorProtocol.TryDecodeRemainingDisplacement(frame.Data, out int mainRaw, out int slaveRaw))
                {
                    lock (_gate)
                    {
                        _badDlcFrames++;
                    }

                    AppendLog($"[电机] 剩余位移反馈 0x{frame.Id:X4} 期望 DLC=8，实际 {frame.Dlc}。");
                    AppendMotorRaw("RX", frame, "剩余位移反馈格式错误（期望 DLC=8）");
                    return true;
                }

                lock (_gate)
                {
                    _motorRemainingDisplacementFrames++;
                    _motorHasRemainingDisplacement = true;
                    _motorRemainingRaw = mainRaw;
                    _motorRemainingArrival = arrival;
                }

                AppendMotorRaw(
                    "RX",
                    frame,
                    $"剩余位移反馈（主通道 {MotorProtocol.PositionLsbToMillimeters(mainRaw):0.0} mm，" +
                    $"从通道 {MotorProtocol.PositionLsbToMillimeters(slaveRaw):0.0} mm）——不是当前位置，不参与角度换算");
            }
            else
            {
                AppendMotorRaw("RX", frame, "其它设备的剩余位移反馈（设备号不匹配）——不参与本机角度换算");
            }

            return true;
        }

        if (MotorProtocol.IsTorqueFeedbackId(frame.Id) ||
            frame.Id == MotorProtocol.TorqueFeedbackIdBase + _motorDeviceId)
        {
            bool ours = frame.Id == MotorProtocol.TorqueFeedbackIdBase + _motorDeviceId;
            if (ours)
            {
                if (frame.Dlc is < 4 or > 8 || !MotorProtocol.TryDecodeTorque(frame.Data, out short torqueRaw))
                {
                    lock (_gate)
                    {
                        _badDlcFrames++;
                    }

                    AppendLog($"[电机] 力矩反馈 0x{frame.Id:X4} 期望 DLC=4..8（实测为 8），实际 {frame.Dlc}。");
                    AppendMotorRaw("RX", frame, "力矩反馈格式错误（期望至少包含 Data[2..3]）");
                    return true;
                }

                lock (_gate)
                {
                    _motorTorqueFrames++;
                }

                AppendMotorRaw("RX", frame, $"力矩反馈（Data[2..3] s16 = {torqueRaw}，DLC={frame.Dlc}）——不是位置");
            }
            else
            {
                AppendMotorRaw("RX", frame, "其它设备的力矩反馈（设备号不匹配）");
            }

            return true;
        }

        if (MotorProtocol.IsHeartbeatId(frame.Id) ||
            frame.Id == MotorProtocol.HeartbeatIdBase + _motorDeviceId)
        {
            bool ours = frame.Id == MotorProtocol.HeartbeatIdBase + _motorDeviceId;
            if (ours)
            {
                lock (_gate)
                {
                    _motorHeartbeatFrames++;
                }

                AppendMotorRaw("RX", frame, "心跳反馈（内容自定义）——不是位置");
            }
            else
            {
                AppendMotorRaw("RX", frame, "其它设备的心跳反馈（设备号不匹配）");
            }

            return true;
        }

        return false;
    }

    /// <summary>
    /// 处理 <c>0x041A</c> 参数设置 ACK。
    /// <para>
    /// 协议表这一行把 DLC 写成 2，却又说 <c>Data[2]</c> 是设备号，自相矛盾，实机也尚未核对。
    /// 所以这里只做计数 + 原始日志：只有当 <c>Data[0..1]</c> 按小端恰好等于本机发送过的
    /// 四个命令 ID 之一时，才记一句“可能回显了 0x0416”，**不据此判定命令成功**。
    /// </para>
    /// </summary>
    private void HandleParameterAckFrame(CanFrame frame, TimeSpan arrival)
    {
        uint? echoId = null;
        if (frame.Dlc >= 2)
        {
            uint littleEndian = (uint)(frame.Data[0] | (frame.Data[1] << 8));
            if (IsKnownMotorCommandId(littleEndian))
            {
                echoId = littleEndian;
            }
        }

        lock (_gate)
        {
            _motorParameterAcks++;
            _motorLastAckArrival = arrival;
            _motorLastAckEchoId = echoId;
        }

        string echoText = echoId is { } id
            ? $"可能回显 0x{id:X4}（按小端解释 Data[0..1]，待实机确认）"
            : "回显的命令 ID 无法确认（只记录原始字节）";
        AppendMotorRaw(
            "RX",
            frame,
            $"参数设置 ACK：{echoText}；DLC={frame.Dlc}（协议表 DLC 与 Data[2]=设备号 的描述互相矛盾）");
    }

    private static bool IsKnownMotorCommandId(uint id)
        => id is MotorProtocol.TargetPositionId
            or MotorProtocol.EnableId
            or MotorProtocol.SetSpeedId
            or MotorProtocol.SetAccelerationId;

    private void AppendMotorRaw(string direction, CanFrame frame, string note)
    {
        // 协议表用 4 位十六进制书写（0x0408/0x0409），日志也保持同样的宽度，便于逐字核对。
        string line = $"{DateTimeOffset.Now:HH:mm:ss.fff} {direction} 0x{frame.Id:X4} DLC={frame.Dlc} {frame.DataText}  {note}";
        lock (_gate)
        {
            _motorRawLog.Add(line);
            if (_motorRawLog.Count > MaxMotorRawLogLines)
            {
                _motorRawLog.RemoveRange(0, _motorRawLog.Count - MaxMotorRawLogLines);
            }
        }
    }

    private void AppendMotorRawTx(uint id, ReadOnlySpan<byte> data, string note)
    {
        string dataText = data.Length == 0 ? "-" : Convert.ToHexString(data);
        string line = $"{DateTimeOffset.Now:HH:mm:ss.fff} TX 0x{id:X4} DLC={data.Length} {dataText}  {note}";
        lock (_gate)
        {
            _motorRawLog.Add(line);
            if (_motorRawLog.Count > MaxMotorRawLogLines)
            {
                _motorRawLog.RemoveRange(0, _motorRawLog.Count - MaxMotorRawLogLines);
            }
        }
    }

    private void OnConnectionStateChanged(LinkState state, string message)
    {
        lock (_gate)
        {
            _linkState = state;
            _linkMessage = message;
        }

        AppendLog($"[连接] {message}");
    }

    private void OnBytesReceived(byte[] chunk, TimeSpan arrival)
    {
        IReadOnlyList<SlcanEvent> events;
        lock (_parseGate)
        {
            events = _parser.Feed(chunk, arrival);
        }

        foreach (SlcanEvent evt in events)
        {
            switch (evt)
            {
                case SlcanCanFrame frame:
                    HandleFrame(frame.Frame, frame.Arrival);
                    break;

                case SlcanTxAck ack:
                    lock (_gate)
                    {
                        if (ack.Accepted)
                        {
                            _txAckAccepted++;
                        }
                        else
                        {
                            _txAckRejected++;
                        }
                    }

                    if (!ack.Accepted)
                    {
                        AppendLog("适配器拒绝了上一帧（SLCAN 返回 Z）：CAN 发送失败。");
                    }

                    break;

                case SlcanStatusFlags flags:
                    lock (_gate)
                    {
                        _adapterStatusEvents++;
                    }

                    AppendLog($"[适配器状态] 0x{flags.Flags:X2}");
                    break;

                case SlcanParseError error:
                    lock (_gate)
                    {
                        _malformedLines++;
                    }

                    AppendLog($"[异常帧] {error.Reason}{(string.IsNullOrEmpty(error.Line) ? string.Empty : $"：{error.Line}")}");
                    break;

                case SlcanText text:
                    AppendLog($"[适配器] {text.Text}");
                    break;
            }
        }
    }

    private void HandleFrame(CanFrame frame, TimeSpan arrival)
    {
        if (frame.IsExtended || frame.IsRemote)
        {
            // 本课题只用标准数据帧，出现扩展帧/远程帧说明总线上有其它设备或适配器配置不对。
            lock (_gate)
            {
                _unsupportedFrames++;
            }

            AppendLog($"[不支持] 收到{(frame.IsExtended ? "扩展" : "标准")}{(frame.IsRemote ? "远程" : "数据")}帧 {frame.IdText}");
            return;
        }

        switch (frame.Id)
        {
            case GyroProtocol.DataId:
                HandleDataFrame(frame, arrival);
                break;

            case GyroProtocol.RateId:
                HandleRateFrame(frame, arrival);
                break;

            case GyroProtocol.RawId:
                HandleRawFrame(frame, arrival);
                break;

            case GyroProtocol.RawDetailId:
                HandleRawDetailFrame(frame, arrival);
                break;

            case GyroProtocol.CommandId:
                // 0x322 是上位机发往板端的方向。收到它说明适配器开了回显或总线上有其它节点转发。
                lock (_gate)
                {
                    _commandEchoFrames++;
                }

                break;

            case MotorProtocol.ReplyId:
                // 0x0409 同时用于温度/当前位置/状态/版本回复，必须由解码器结合请求上下文区分。
                HandleMotorReplyFrame(frame, arrival);
                break;

            case MotorProtocol.QueryId:
                HandleOtherMotorQueryFrame(frame);
                break;

            case MotorProtocol.ParameterInitId:
            case MotorProtocol.EnableId:
            case MotorProtocol.SetAccelerationId:
            case MotorProtocol.SetSpeedId:
            case MotorProtocol.TargetPositionId:
                // 这些是上位机 → 电机的命令方向。收到它们说明适配器开了回显或总线上有其它主控转发，
                // 不能当作反馈解析，也不该刷“未知 ID”。
                AppendMotorRaw("RX", frame, "收到电机命令回显（上位机发送方向），不当作反馈");
                break;

            default:
                if (TryHandleMotorFeedbackFrame(frame, arrival))
                {
                    break;
                }

                lock (_gate)
                {
                    _unsupportedFrames++;
                }

                AppendLog($"[未知 ID] {frame.IdText}，DLC={frame.Dlc}");
                break;
        }
    }

    private void HandleDataFrame(CanFrame frame, TimeSpan arrival)
    {
        if (frame.Dlc != GyroProtocol.DataDlc || !GyroProtocol.TryDecodeData(frame.Data, out DataPayload payload))
        {
            lock (_gate)
            {
                _badDlcFrames++;
            }

            AppendLog($"[格式错误] 0x321 期望 DLC={GyroProtocol.DataDlc}，实际 {frame.Dlc}。");
            return;
        }

        lock (_gate)
        {
            _dataFrames++;
            _lastData = payload;
            _lastDataArrival = arrival;
            _hasData = true;
        }

        // 顺序很重要：先更新状态机（必要时标记曲线断开），再写入数据点，
        // 这样断开标记会挂在清零后的第一个点上。
        UpdateCalibration(payload.Status);
        UpdateZeroAngle(payload.AngleDeg);

        _store.AddData(arrival, payload.AngleDeg, payload.TemperatureC, payload.Status, payload.IntervalMs);
    }

    private void HandleRateFrame(CanFrame frame, TimeSpan arrival)
    {
        if (frame.Dlc != GyroProtocol.RateDlc || !GyroProtocol.TryDecodeRate(frame.Data, out double rateDps))
        {
            lock (_gate)
            {
                _badDlcFrames++;
            }

            AppendLog($"[格式错误] 0x323 期望 DLC={GyroProtocol.RateDlc}，实际 {frame.Dlc}。");
            return;
        }

        lock (_gate)
        {
            _rateFrames++;
            _lastRateDps = rateDps;
            _lastRateArrival = arrival;
            _hasRate = true;
        }

        _store.AddRate(arrival, rateDps);
    }

    /// <summary>
    /// 0x324：原始码 + 实际采样时刻 + 零偏。先暂存，等同一拍的 0x325 到达后再配对写入，
    /// 这样一条记录里的原始码/零偏/滤波值必然来自同一拍。
    /// </summary>
    private void HandleRawFrame(CanFrame frame, TimeSpan arrival)
    {
        if (frame.Dlc != GyroProtocol.RawDlc || !GyroProtocol.TryDecodeRaw(frame.Data, out RawPayload payload))
        {
            lock (_gate)
            {
                _badDlcFrames++;
            }

            AppendLog($"[格式错误] 0x324 期望 DLC={GyroProtocol.RawDlc}，实际 {frame.Dlc}。");
            return;
        }

        uint? missingDetailTick = null;
        long mismatchCount = 0;
        lock (_gate)
        {
            _rawFrames++;
            if (_hasPendingRaw)
            {
                missingDetailTick = _pendingRaw.SampleTickMs;
                mismatchCount = ++_rawPairMismatches;
            }
            _pendingRaw = payload;
            _pendingRawArrival = arrival;
            _hasPendingRaw = true;
        }

        if (missingDetailTick.HasValue && (mismatchCount == 1 || mismatchCount % 100 == 0))
        {
            AppendLog($"[原始采样] {missingDetailTick.Value} ms 的 0x324 未收到配对的 0x325，已被新样本替换；累计未配对 {mismatchCount}。请检查固件 CAN 发送邮箱与链路，当前 CSV 原始采样不完整。");
        }
    }

    /// <summary>
    /// 0x325：滤波后角速度 + 采样时刻。只有时间戳与暂存的 0x324 一致才配对成功；
    /// 0x324 丢失时丢弃本条原始记录（0x323 仍会正常记录滤波值），避免把两拍的数据拼在一起。
    /// </summary>
    private void HandleRawDetailFrame(CanFrame frame, TimeSpan arrival)
    {
        if (frame.Dlc != GyroProtocol.RawDetailDlc ||
            !GyroProtocol.TryDecodeRawDetail(frame.Data, out RawDetailPayload detail))
        {
            lock (_gate)
            {
                _badDlcFrames++;
            }

            AppendLog($"[格式错误] 0x325 期望 DLC={GyroProtocol.RawDetailDlc}，实际 {frame.Dlc}。");
            return;
        }

        RawPayload raw;
        TimeSpan sampleArrival;
        bool paired;

        lock (_gate)
        {
            _rawFrames++;
            paired = _hasPendingRaw && _pendingRaw.SampleTickMs == detail.SampleTickMs;
            raw = _pendingRaw;
            sampleArrival = _pendingRawArrival;
            _hasPendingRaw = false;
            if (!paired)
            {
                _rawPairMismatches++;
            }
        }

        if (!paired)
        {
            AppendLog($"[原始采样] 0x325 时间戳 {detail.SampleTickMs} ms 找不到配对的 0x324，本拍不写记录。");
            return;
        }

        _store.AddRaw(
            sampleArrival,
            raw.RawCode,
            raw.SampleTickMs,
            raw.BiasDps,
            detail.FilteredRateDps);
    }

    /// <summary>
    /// 校准状态机。板端没有命令应答，所以只能靠"先看到校准完成位消失、再看到它出现"
    /// 来确认这一次校准确实执行并成功；仅凭发送前遗留的完成标志不能算成功。
    /// </summary>
    private void UpdateCalibration(GyroStatus status)
    {
        CalibrationPhase? transition = null;
        TimeSpan now = MonotonicClock.Now;

        lock (_gate)
        {
            switch (_calibration)
            {
                case CalibrationPhase.Requested:
                    if ((status & GyroStatus.CalibrationFailed) != 0)
                    {
                        _calibration = CalibrationPhase.Failed;
                        transition = CalibrationPhase.Failed;
                    }
                    else if ((status & GyroStatus.CalibrationDone) == 0)
                    {
                        _calibration = CalibrationPhase.InProgress;
                        transition = CalibrationPhase.InProgress;
                    }
                    else if (now - _calibrationSentAt > TimeSpan.FromSeconds(CalibrationTimeoutSeconds))
                    {
                        _calibration = CalibrationPhase.NoResponse;
                        transition = CalibrationPhase.NoResponse;
                    }

                    break;

                case CalibrationPhase.InProgress:
                    if ((status & GyroStatus.CalibrationFailed) != 0)
                    {
                        _calibration = CalibrationPhase.Failed;
                        transition = CalibrationPhase.Failed;
                    }
                    else if ((status & GyroStatus.CalibrationDone) != 0)
                    {
                        _calibration = CalibrationPhase.Completed;
                        transition = CalibrationPhase.Completed;
                    }
                    else if (now - _calibrationSentAt > TimeSpan.FromSeconds(CalibrationTimeoutSeconds))
                    {
                        _calibration = CalibrationPhase.NoResponse;
                        transition = CalibrationPhase.NoResponse;
                    }

                    break;
            }
        }

        switch (transition)
        {
            case CalibrationPhase.InProgress:
                AppendLog($"检测到板端已进入校准采样，请持续保持静止约{GyroProtocol.CalibrationDurationSeconds:0}秒（60秒估计＋20秒独立验证），以完成状态为准。");
                break;

            case CalibrationPhase.Completed:
                AppendLog((status & GyroStatus.DegradedCalibration) != 0
                    ? "本次校准完成但已降级：静止验证、采样完整性或波动判据未通过，不能判精度合格。请保持实际使能状态、确认机械静止后重新校准；累计角度保留。"
                    : "本次零偏校准完成（已观察到完整状态变化），板端保留累计角度（需配套新版固件）。");
                // 校准期间角度无效，不跨越该区间连接曲线。
                _store.MarkAngleBreak();
                break;

            case CalibrationPhase.Failed:
                AppendLog("本次零偏校准未达严格判据（板端置起校准失败位），板端会在 2 秒后自动重试；若持续失败请保持板子静止。");
                break;

            case CalibrationPhase.NoResponse:
                AppendLog($"已发送重新校准命令，但 {CalibrationTimeoutSeconds:0.#} 秒内未观察到状态变化：无法确认板端是否执行。");
                break;
        }
    }

    /// <summary>角度清零验证：只有回传数据出现归零才认为成功。</summary>
    private void UpdateZeroAngle(double angleDeg)
    {
        bool verified = false;
        bool noChange = false;

        lock (_gate)
        {
            if (_zeroAngle == ZeroAnglePhase.Sent)
            {
                if (Math.Abs(angleDeg) <= ZeroVerifiedThresholdDeg)
                {
                    _zeroAngle = ZeroAnglePhase.Verified;
                    verified = true;
                }
                else if (MonotonicClock.Now - _zeroAngleSentAt > TimeSpan.FromSeconds(ZeroVerifyTimeoutSeconds))
                {
                    _zeroAngle = ZeroAnglePhase.NoChange;
                    noChange = true;
                }
            }
        }

        if (verified)
        {
            // 把回传值记进日志：判断"是否真的停在 0.00000°"要靠这 5 位小数，
            // 旧固件会停在 0.00x°（清零后 0.5s 静止确认窗口里积了残余零偏）。
            AppendLog(
                $"板端角度已清零，并已由后续回传数据确认（回传 {angleDeg.ToString("F5", System.Globalization.CultureInfo.InvariantCulture)}°）。");
            _store.MarkAngleBreak();
        }
        else if (noChange)
        {
            AppendLog("角度清零命令已发送，但未观察到角度归零，请检查板端状态。");
        }
    }

    private void AppendLog(string message)
    {
        string line = $"{DateTimeOffset.Now:HH:mm:ss.fff} {message}";
        lock (_gate)
        {
            _log.Add(line);
            if (_log.Count > MaxLogLines)
            {
                _log.RemoveRange(0, _log.Count - MaxLogLines);
            }
        }
    }
}
