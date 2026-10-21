using GyroHost.Core.Protocol;

namespace GyroHost.Core.Services;

/// <summary>电机往复运动的整体状态。</summary>
public enum MotorMotionState
{
    /// <summary>未运行。</summary>
    Idle,

    /// <summary>正在执行（配置 → 正转/返回往复）。</summary>
    Running,

    /// <summary>全部轮次完成并回到起点。</summary>
    Completed,

    /// <summary>失败（到位超时、位置反馈中断、发送异常等）。</summary>
    Failed,

    /// <summary>用户取消。</summary>
    Cancelled,
}

/// <summary>状态机内部的阶段，用于生成说明文字。</summary>
public enum MotorMotionPhase
{
    /// <summary>未运行。</summary>
    Idle,

    /// <summary>准备发送使能/速度/加速度。</summary>
    Configure,

    /// <summary>配置命令已发出，等待其生效。</summary>
    ConfigureDelay,

    /// <summary>已发送正转目标，等待到位。</summary>
    WaitForwardArrival,

    /// <summary>正转已到位，停在目标角度等待 <see cref="MotorMotionController.DwellSeconds"/> 秒。</summary>
    DwellAtTarget,

    /// <summary>
    /// 一轮（正转 + 返回）结束、已回到起点，停顿等待板端重新零偏校准，再开始下一轮。
    /// 停顿时长同 <see cref="MotorMotionController.DwellSeconds"/>。
    /// </summary>
    DwellAfterRound,

    /// <summary>已发送返回目标，等待到位。</summary>
    WaitBackwardArrival,

    /// <summary>校验终点是否回到起点。</summary>
    VerifyEnd,

    /// <summary>已发送终点偏差修正，等待到位。</summary>
    WaitCorrection,

    /// <summary>完成。</summary>
    Completed,

    /// <summary>失败。</summary>
    Failed,

    /// <summary>已取消。</summary>
    Cancelled,
}

/// <summary>需要由上层真正发到 CAN 上的一条命令。</summary>
public enum MotorCommandKind
{
    /// <summary><c>0x0413</c> 锁定/释放。</summary>
    Enable,

    /// <summary><c>0x0415</c> 设置主速度。</summary>
    SetSpeed,

    /// <summary><c>0x0414</c> 设置主加速度。</summary>
    SetAcceleration,

    /// <summary><c>0x0416</c> 设置目标位置。</summary>
    TargetPosition,

    /// <summary><c>0x322</c> 数据 <c>01</c>：要求板端重新零偏校准（2 s 静止采样后把零偏换成新均值并归零角度）。</summary>
    RecalibrateGyro,
}

/// <summary>状态机产出的一条待发送命令。</summary>
/// <param name="Kind">命令类型。</param>
/// <param name="RawPosition">目标位置原始值（<see cref="MotorCommandKind.TargetPosition"/> 时有效，单位 0.1 mm）。</param>
/// <param name="RawValue">速度/加速度原始值（单位 0.1 mm/s、0.1 mm）。</param>
/// <param name="Enable">锁定（true）或释放（false），仅 <see cref="MotorCommandKind.Enable"/> 有效。</param>
/// <param name="TargetType"><c>0x0416</c> 的目标类型，本状态机只用相对。</param>
/// <param name="Description">写进原始 CAN 日志的说明。</param>
public sealed record MotorCommand(
    MotorCommandKind Kind,
    int RawPosition = 0,
    uint RawValue = 0,
    bool Enable = true,
    MotorTargetType TargetType = MotorTargetType.Relative,
    string Description = "");

/// <summary>
/// 电机往复运动状态机：以目标角度“正转 → 返回”往复若干轮，最后回到起点。
/// <para>
/// 不依赖定时器、不做 I/O：调用方每 tick 用最新的位置反馈喂一次 <see cref="Tick"/>，
/// 把返回的命令真正发到 CAN 上。这样到位判定、超时、取消都能在单元测试里确定性地验证。
/// </para>
/// <para>
/// 运动前先发 <c>0x0413</c> 锁定（上电使能）+ <c>0x0415</c> 速度 + <c>0x0414</c> 加速度，
/// 再逐段发 <c>0x0416</c> 相对目标；每个停靠点（正转到位、每轮回到起点）按
/// <see cref="DwellSeconds"/> 停顿，并在 <see cref="RecalibrateOnStop"/> 开启时先发
/// <c>0x322 = 01</c> 让板端重新校准零偏，保留角度；**不释放电机**。
/// </para>
/// </summary>
public sealed class MotorMotionController
{
    /// <summary>驱动反馈到位判据：默认 ±0.25°；不等同于独立测量验收。</summary>
    public const double DefaultToleranceDeg = 0.25;

    /// <summary>单段（正转或返回）超时：默认 30 s。</summary>
    public const double DefaultSegmentTimeoutSeconds = 30.0;

    /// <summary>位置反馈中断多久判失败：默认 0.6 s（与界面“数据超时”一致）。</summary>
    public const double DefaultPositionTimeoutSeconds = 0.6;

    /// <summary>配置命令发出后等待生效的时间。</summary>
    public const double ConfigureDelaySeconds = 0.2;

    /// <summary>正转到位后停顿多久再返程：默认 3 s（常规往复不重新校准）。</summary>
    public const double DefaultDwellSeconds = 3.0;

    /// <summary>
    /// 板端 60 秒估计和 20 秒独立验证所需静止时长，再留 0.5 秒余量。
    /// <para>启用 <see cref="RecalibrateOnStop"/> 时，每个停靠点的实际停顿时长取
    /// <c>max(DwellSeconds, GyroRecalibrationSeconds)</c>，避免校准被返程打断。</para>
    /// </summary>
    public const double GyroRecalibrationSeconds = GyroProtocol.CalibrationDurationSeconds + 0.5;

    /// <summary>停顿时长上限（秒），仅供界面校验。</summary>
    public const double MaxDwellSeconds = 60.0;

    /// <summary>需要连续多少个“新到达”的位置样本落在容差内才判到位。</summary>
    public const int RequiredSettleSamples = 3;

    private static readonly MotorCommand[] NoCommands = Array.Empty<MotorCommand>();

    private int _startRaw;
    private int _deltaRaw;
    private int _commandedOffsetRaw;
    private int _expectedRaw;
    private int _currentRaw;
    private bool _hasCurrent;
    private int _settleCount;
    private TimeSpan _segmentStartedAt;
    private TimeSpan _dwellStartedAt;
    private TimeSpan? _lastFreshAt;
    private TimeSpan? _lastCountedArrival;

    /// <summary>整体状态。</summary>
    public MotorMotionState State { get; private set; } = MotorMotionState.Idle;

    /// <summary>当前阶段。</summary>
    public MotorMotionPhase Phase { get; private set; } = MotorMotionPhase.Idle;

    /// <summary>本次目标角度（°，正数表示“正转方向”）。</summary>
    public int TargetAngleDeg { get; private set; }

    /// <summary>本次往复轮数。</summary>
    public int TotalRounds { get; private set; }

    /// <summary>已完成的轮数（正转 + 返回都到位才算一轮）。</summary>
    public int CompletedRounds { get; private set; }

    /// <summary>当前轮次（1 起；未运行为 0）。</summary>
    public int CurrentRound { get; private set; }

    /// <summary>运动开始时用的起点位置 P（原始值，0.1 mm）。</summary>
    public int StartRawPosition => _startRaw;

    /// <summary>每段的角度增量对应的位置增量（LSB）。</summary>
    public int SegmentDeltaLsb => _deltaRaw;

    /// <summary>尚未走完的相对位移累计（相对起点，LSB），仅用于说明文字。</summary>
    public int CommandedOffsetLsb => _commandedOffsetRaw;

    /// <summary>失败原因；未失败时为 null。</summary>
    public string? FailureReason { get; private set; }

    /// <summary>最近一次观察到的位置（原始值）。</summary>
    public int? LastObservedRawPosition => _hasCurrent ? _currentRaw : null;

    /// <summary>状态说明文字（界面直接显示）。</summary>
    public string StatusText { get; private set; } = "未运行";

    /// <summary>驱动反馈到位容差（°），默认 ±0.25°。</summary>
    public double ToleranceDeg { get; set; } = DefaultToleranceDeg;

    /// <summary>单段超时（秒），默认 30 s。</summary>
    public double SegmentTimeoutSeconds { get; set; } = DefaultSegmentTimeoutSeconds;

    /// <summary>位置反馈中断判失败的时长（秒），默认 0.6 s。</summary>
    public double PositionTimeoutSeconds { get; set; } = DefaultPositionTimeoutSeconds;

    /// <summary>运动前设置的主速度（mm/s），默认 100。</summary>
    public double SpeedMmPerSecond { get; set; } = 100.0;

    /// <summary>运动前设置的主加速度（mm/s²），默认 500。</summary>
    public double AccelerationMmPerSecondSquared { get; set; } = 500.0;

    /// <summary>
    /// 每轮正转到位后停顿多久再返程（秒），默认 3 s；0 表示不停顿、立即返程。
    /// <para>每轮返回起点后同样停顿这么久，然后再开始下一轮正转。</para>
    /// <para>启用 <see cref="RecalibrateOnStop"/> 时，实际停顿取
    /// <c>max(本值, GyroRecalibrationSeconds)</c>。</para>
    /// </summary>
    public double DwellSeconds { get; set; } = DefaultDwellSeconds;

    /// <summary>
    /// 每个停靠点（正转到位、每轮返回起点）是否先让板端重新零偏校准（<c>0x322 = 01</c>）。
    /// <para>默认关闭：整批往复保持同一角度原点和校准零偏，保留真实累计残差。
    /// 显式开启仅供独立校准实验，不用于往复验收。</para>
    /// </summary>
    public bool RecalibrateOnStop { get; set; } = false;

    /// <summary>本次运动使用的轮径（mm），由 <see cref="Start"/> 时的设置决定。</summary>
    public double WheelDiameterMm { get; private set; } = MotorProtocol.DefaultWheelDiameterMm;

    /// <summary>运动是否处于进行中。</summary>
    public bool IsRunning => State == MotorMotionState.Running;

    /// <summary>
    /// 开始一次往复运动。
    /// </summary>
    /// <param name="now">当前单调时刻。</param>
    /// <param name="angleDeg">目标角度（°），必须 &gt; 0。</param>
    /// <param name="rounds">往复轮数，1..50。</param>
    /// <param name="wheelDiameterMm">轮径（mm），只用于角度 ↔ 位移换算。</param>
    /// <param name="hasFreshPosition">当前是否有新鲜有效的位置反馈。</param>
    /// <param name="rawPosition">当前位置原始值（0.1 mm）。</param>
    /// <param name="error">失败原因；成功时为 null。</param>
    /// <returns>true = 已启动。</returns>
    public bool Start(
        TimeSpan now,
        int angleDeg,
        int rounds,
        double wheelDiameterMm,
        bool hasFreshPosition,
        int rawPosition,
        out string? error)
    {
        if (State == MotorMotionState.Running)
        {
            error = "上一次运动尚未结束，请先等待完成或取消。";
            return false;
        }

        if (angleDeg <= 0)
        {
            error = "目标角度必须大于 0°。";
            return false;
        }

        if (rounds < 1 || rounds > 50)
        {
            error = "往复轮数必须是 1..50。";
            return false;
        }

        if (!hasFreshPosition)
        {
            error = "当前没有有效的电机位置反馈，无法开始运动（不能用旧值或假零点代替）。";
            return false;
        }

        int delta;
        try
        {
            delta = MotorProtocol.DegreesToPositionDelta(angleDeg, wheelDiameterMm);
        }
        catch (ArgumentOutOfRangeException ex)
        {
            error = ex.Message;
            return false;
        }

        if (delta == 0)
        {
            error = "目标角度换算出的位置增量是 0，轮径或角度设置不合理。";
            return false;
        }

        WheelDiameterMm = wheelDiameterMm;
        TargetAngleDeg = angleDeg;
        TotalRounds = rounds;
        CompletedRounds = 0;
        CurrentRound = 1;
        _deltaRaw = delta;
        _startRaw = rawPosition;
        _commandedOffsetRaw = 0;
        _expectedRaw = rawPosition;
        _currentRaw = rawPosition;
        _hasCurrent = true;
        _settleCount = 0;
        _segmentStartedAt = now;
        _dwellStartedAt = now;
        _lastFreshAt = now;
        _lastCountedArrival = null;
        FailureReason = null;
        State = MotorMotionState.Running;
        Phase = MotorMotionPhase.Configure;
        StatusText = StopSeconds > 0
            ? $"准备运动：{angleDeg}° × {rounds} 轮（每段 {delta} LSB，" +
              (RecalibrateOnStop
                  ? $"每次停靠重新校准零偏 {StopSeconds:0.##} s）"
                  : $"到位停顿 {StopSeconds:0.##} s）")
            : $"准备运动：{angleDeg}° × {rounds} 轮（每段 {delta} LSB，不停顿）";
        error = null;
        return true;
    }

    /// <summary>取消运动。已经发出的命令不会撤回，只是不再产生新命令。</summary>
    public void Cancel()
    {
        if (State != MotorMotionState.Running)
        {
            return;
        }

        State = MotorMotionState.Cancelled;
        Phase = MotorMotionPhase.Cancelled;
        StatusText = $"已取消（完成 {CompletedRounds}/{TotalRounds} 轮；电机可能还在执行最后一条指令）";
    }

    /// <summary>外部原因（例如发送失败、连接断开）导致的中止。</summary>
    public void Abort(string reason)
    {
        if (State != MotorMotionState.Running)
        {
            return;
        }

        Fail(string.IsNullOrWhiteSpace(reason) ? "外部中止" : reason);
    }

    /// <summary>回到 Idle，允许重新开始。</summary>
    public void Reset()
    {
        if (State == MotorMotionState.Running)
        {
            return;
        }

        State = MotorMotionState.Idle;
        Phase = MotorMotionPhase.Idle;
        CompletedRounds = 0;
        CurrentRound = 0;
        _settleCount = 0;
        _commandedOffsetRaw = 0;
        FailureReason = null;
        StatusText = "未运行";
    }

    /// <summary>
    /// 推进状态机。每 tick 调用一次；返回本次需要发送的命令（通常 0 或 1 条）。
    /// </summary>
    /// <param name="now">当前单调时刻。</param>
    /// <param name="hasFreshPosition">是否有新鲜有效的位置反馈（数据未超时）。</param>
    /// <param name="rawPosition">最新位置原始值（0.1 mm）。</param>
    /// <param name="lastPositionArrival">最新位置样本的到达时刻；用于避免同一个样本被重复计数。</param>
    public IReadOnlyList<MotorCommand> Tick(
        TimeSpan now,
        bool hasFreshPosition,
        int rawPosition,
        TimeSpan? lastPositionArrival)
    {
        if (State != MotorMotionState.Running)
        {
            return NoCommands;
        }

        if (hasFreshPosition)
        {
            _lastFreshAt = now;
            _currentRaw = rawPosition;
            _hasCurrent = true;

            // 只有“新到达”的样本才计入到位判据，避免同一个样本被连续 tick 重复计数。
            if (lastPositionArrival != _lastCountedArrival)
            {
                _lastCountedArrival = lastPositionArrival;
                _settleCount = IsSettled(rawPosition) ? _settleCount + 1 : 0;
            }
        }

        switch (Phase)
        {
            case MotorMotionPhase.Configure:
                _segmentStartedAt = now;
                Phase = MotorMotionPhase.ConfigureDelay;
                return new[]
                {
                    new MotorCommand(
                        MotorCommandKind.Enable,
                        Enable: true,
                        Description: "锁定（上电使能）"),
                    new MotorCommand(
                        MotorCommandKind.SetSpeed,
                        RawValue: MotorProtocol.MillimetersPerSecondToRawSpeed(SpeedMmPerSecond),
                        Description: $"设置主速度 {SpeedMmPerSecond:0.#} mm/s"),
                    new MotorCommand(
                        MotorCommandKind.SetAcceleration,
                        RawValue: MotorProtocol.MillimetersPerSecondSquaredToRawAcceleration(AccelerationMmPerSecondSquared),
                        Description: $"设置主加速度 {AccelerationMmPerSecondSquared:0.#} mm/s²"),
                };

            case MotorMotionPhase.ConfigureDelay:
                if (ElapsedSeconds(now) >= ConfigureDelaySeconds)
                {
                    return BeginSegment(now, _deltaRaw, MotorMotionPhase.WaitForwardArrival, Prefixed("正转"));
                }

                return NoCommands;

            case MotorMotionPhase.WaitForwardArrival:
                if (CheckTimeout(now, out string? forwardTimeout))
                {
                    return Fail(forwardTimeout!);
                }

                if (!IsSettledEnough())
                {
                    StatusText = Prefixed($"正转中：当前位置 {_currentRaw}，目标 {_expectedRaw}（±{ToleranceLsb}）");
                    return NoCommands;
                }

                // 到位后先停在目标角度，停顿结束才返程；停顿时长见 StopSeconds。
                if (StopSeconds > 0)
                {
                    _dwellStartedAt = now;
                    Phase = MotorMotionPhase.DwellAtTarget;
                    StatusText = Prefixed(StopStatusText("正转到位"));
                    return StopEntryCommands();
                }

                StatusText = Prefixed("正转到位");
                return BeginSegment(now, -_deltaRaw, MotorMotionPhase.WaitBackwardArrival, Prefixed("返回起点"));

            case MotorMotionPhase.DwellAtTarget:
                // 停顿不计入单段超时，但位置反馈中断仍然要判失败（否则会盲目返程）。
                if (IsFeedbackLost(now, out string? dwellFeedbackLost))
                {
                    return Fail(dwellFeedbackLost!);
                }

                double remainingSeconds = StopSeconds - ElapsedSince(now, _dwellStartedAt);
                if (remainingSeconds > 0)
                {
                    StatusText = Prefixed(StopRemainingText("已到目标角度", remainingSeconds));
                    return NoCommands;
                }

                return BeginSegment(now, -_deltaRaw, MotorMotionPhase.WaitBackwardArrival, Prefixed("返回起点"));

            case MotorMotionPhase.WaitBackwardArrival:
                if (CheckTimeout(now, out string? backwardTimeout))
                {
                    return Fail(backwardTimeout!);
                }

                if (!IsSettledEnough())
                {
                    StatusText = Prefixed($"返回中：当前位置 {_currentRaw}，目标 {_expectedRaw}（±{ToleranceLsb}）");
                    return NoCommands;
                }

                CompletedRounds++;
                if (CompletedRounds < TotalRounds)
                {
                    CurrentRound = CompletedRounds + 1;
                }

                // 每轮结束先停稳。默认不重新校准，保留整批往复的累计残差。
                _dwellStartedAt = now;
                Phase = MotorMotionPhase.DwellAfterRound;
                StatusText = CompletedRounds < TotalRounds
                    ? Prefixed(StopStatusText($"第 {CompletedRounds} 轮完成、已回到起点"))
                    : $"第 {CompletedRounds}/{TotalRounds} 轮完成：{StopStatusText("已回到起点")}";
                return StopEntryCommands();

            case MotorMotionPhase.DwellAfterRound:
                if (IsFeedbackLost(now, out string? roundFeedbackLost))
                {
                    return Fail(roundFeedbackLost!);
                }

                double roundRemaining = StopSeconds - ElapsedSince(now, _dwellStartedAt);
                if (roundRemaining > 0)
                {
                    StatusText = CompletedRounds < TotalRounds
                        ? Prefixed(StopRemainingText("返回后等待停稳", roundRemaining))
                        : $"第 {CompletedRounds}/{TotalRounds} 轮完成：{StopRemainingText("返回后等待停稳", roundRemaining)}";
                    return NoCommands;
                }

                if (CompletedRounds < TotalRounds)
                {
                    return BeginSegment(now, _deltaRaw, MotorMotionPhase.WaitForwardArrival, Prefixed("正转"));
                }

                StatusText = $"第 {CompletedRounds}/{TotalRounds} 轮完成：校验终点位置";
                Phase = MotorMotionPhase.VerifyEnd;
                _segmentStartedAt = now;
                return NoCommands;

            case MotorMotionPhase.VerifyEnd:
                return VerifyEnd(now);

            case MotorMotionPhase.WaitCorrection:
                if (CheckTimeout(now, out string? correctionTimeout))
                {
                    return Fail(correctionTimeout!);
                }

                if (!IsSettledEnough())
                {
                    StatusText = "终点修正中";
                    return NoCommands;
                }

                return VerifyEnd(now);

            default:
                return NoCommands;
        }
    }

    private IReadOnlyList<MotorCommand> VerifyEnd(TimeSpan now)
    {
        int residual = _currentRaw - _startRaw;
        if (Math.Abs(residual) <= ToleranceLsb)
        {
            State = MotorMotionState.Completed;
            Phase = MotorMotionPhase.Completed;
            StatusText = $"完成：{CompletedRounds} 轮往复，已回到起点（终点 {_currentRaw}，起点 {_startRaw}，" +
                         $"残余 {residual} LSB ≈ {MotorProtocol.PositionDeltaLsbToDegrees(residual, WheelDiameterMm):0.000}°）";
            return NoCommands;
        }

        if (Phase == MotorMotionPhase.WaitCorrection)
        {
            return Fail($"终点修正后仍偏离起点 {residual} LSB（>{ToleranceLsb}）");
        }

        // 只修正一次，避免反复来回找点。
        int correction = _startRaw - _currentRaw;
        State = MotorMotionState.Running;
        StatusText = $"终点偏离 {residual} LSB，发送一次修正 {correction} LSB";
        return BeginSegment(now, correction, MotorMotionPhase.WaitCorrection, StatusText);
    }

    /// <summary>
    /// 每个停靠点的实际停顿时长：启用重新校准时至少 <see cref="GyroRecalibrationSeconds"/>，
    /// 否则固件 20 s 的校准窗口会被返程打断，会漏掉校准期间的运动。
    /// </summary>
    private double StopSeconds => RecalibrateOnStop
        ? Math.Max(DwellSeconds, GyroRecalibrationSeconds)
        : DwellSeconds;

    /// <summary>进入停靠点时要发的命令：启用重新校准时发一条 <c>0x322 = 01</c>。</summary>
    private IReadOnlyList<MotorCommand> StopEntryCommands()
        => RecalibrateOnStop
            ? new[]
            {
                new MotorCommand(
                    MotorCommandKind.RecalibrateGyro,
                    Description: "板端重新零偏校准（0x322 = 01，要求静止至少 20 s）"),
            }
            : NoCommands;

    private string StopStatusText(string prefix)
        => RecalibrateOnStop
            ? $"{prefix}，等板端重新校准零偏 {StopSeconds:0.##} s"
            : $"{prefix}，停顿 {StopSeconds:0.##} s";

    private string StopRemainingText(string prefix, double remainingSeconds)
        => RecalibrateOnStop
            ? $"{prefix}，等待板端重新校准零偏（剩余 {remainingSeconds:0.0} s）"
            : $"{prefix}，停顿中（剩余 {remainingSeconds:0.0} s）";

    private IReadOnlyList<MotorCommand> BeginSegment(
        TimeSpan now,
        int deltaLsb,
        MotorMotionPhase waitPhase,
        string text)
    {
        _commandedOffsetRaw += deltaLsb;

        // 相对目标是以“下发命令那一刻的位置”为基准的，所以期望位置 = 当前观测位置 + Δ。
        // 这样即使上一段有残余偏差，也不会把误差一层层累加到“永远不到位”。
        _expectedRaw = _currentRaw + deltaLsb;
        _segmentStartedAt = now;
        _settleCount = 0;
        Phase = waitPhase;
        StatusText = $"{text}：目标 {_expectedRaw}（相对 {deltaLsb:+0;-0} LSB ≈ " +
                     $"{MotorProtocol.PositionDeltaLsbToDegrees(_commandedOffsetRaw, WheelDiameterMm):0.000}°）";
        return new[]
        {
            new MotorCommand(
                MotorCommandKind.TargetPosition,
                RawPosition: deltaLsb,
                TargetType: MotorTargetType.Relative,
                Description: text),
        };
    }

    private IReadOnlyList<MotorCommand> Fail(string reason)
    {
        State = MotorMotionState.Failed;
        Phase = MotorMotionPhase.Failed;
        FailureReason = reason;
        StatusText = $"失败：{reason}";
        return NoCommands;
    }

    private bool CheckTimeout(TimeSpan now, out string? reason)
    {
        if (IsFeedbackLost(now, out reason))
        {
            return true;
        }

        if (ElapsedSeconds(now) > SegmentTimeoutSeconds)
        {
            reason = $"{SegmentTimeoutSeconds:0.#} s 内未到位（当前 {_currentRaw}，目标 {_expectedRaw}，" +
                     $"容差 ±{ToleranceLsb} LSB）";
            return true;
        }

        reason = null;
        return false;
    }

    private bool IsFeedbackLost(TimeSpan now, out string? reason)
    {
        if (_lastFreshAt is not { } freshAt ||
            (now - freshAt).TotalSeconds > PositionTimeoutSeconds)
        {
            reason = $"位置反馈中断超过 {PositionTimeoutSeconds * 1000:0} ms，无法判断是否到位";
            return true;
        }

        reason = null;
        return false;
    }

    private bool IsSettledEnough() => _settleCount >= RequiredSettleSamples;

    private bool IsSettled(int rawPosition)
    {
        // 还没发出目标时（配置阶段）不做到位判定。
        if (Phase is MotorMotionPhase.Configure or MotorMotionPhase.ConfigureDelay)
        {
            return false;
        }

        return Math.Abs(rawPosition - _expectedRaw) <= ToleranceLsb;
    }

    private int ToleranceLsb
        => MotorProtocol.AngleToleranceToPositionLsb(ToleranceDeg, WheelDiameterMm);

    private double ElapsedSeconds(TimeSpan now) => ElapsedSince(now, _segmentStartedAt);

    private static double ElapsedSince(TimeSpan now, TimeSpan start) => (now - start).TotalSeconds;

    private string Prefixed(string text)
        => $"第 {Math.Min(CurrentRound, TotalRounds)}/{TotalRounds} 轮：{text}";
}
