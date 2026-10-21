namespace GyroHost.Core.Services;

/// <summary>位置查询的请求状态。</summary>
public enum MotorRequestState
{
    /// <summary>空闲（未启用轮询或尚未发送查询）。</summary>
    Idle,

    /// <summary>已发送查询，等待可确认的 <c>0x0409</c> 应答。</summary>
    WaitingForReply,

    /// <summary>最近一次查询已收到可确认的位置应答。</summary>
    Received,

    /// <summary>最近一次查询在超时时间内没有收到可确认的位置应答。</summary>
    TimedOut,
}

/// <summary>
/// 一个已经解码的电机位置采样点。
/// <para>
/// <see cref="TimeSeconds"/> 使用电机自己的时间轴（“电机显示归零”后从 0 重新计时），
/// 保留给需要电机相对计时的场景。
/// <see cref="ChartTimeSeconds"/> 与陀螺仪曲线共用同一时间基准（同一个
/// <see cref="MeasurementStore"/> 的全局时间轴），因此电机曲线可以和陀螺仪曲线画在同一张图上，
/// 不会各自从不同起点开始。
/// </para>
/// <para>
/// <see cref="DisplacementMm"/> 是绝对位移 <c>P × 0.1</c>；
/// <see cref="RelativeAngleDeg"/> 是相对起点 <c>P0</c> 的累计角度，不取模。
/// </para>
/// </summary>
public readonly record struct MotorPositionSample(
    TimeSpan Arrival,
    double TimeSeconds,
    double ChartTimeSeconds,
    int RawPosition,
    double DisplacementMm,
    double RelativeAngleDeg,
    bool BreakBefore);

/// <summary>
/// 电机位置监视的一次快照。数值型字段在协议未确认或数据超时时仍保留最后一次诊断值，
/// 界面必须结合 <see cref="DecoderConfirmed"/>、<see cref="HasPosition"/> 和
/// <see cref="IsFresh"/> 判断能不能显示，不能用旧值冒充实时值。
/// </summary>
public sealed record MotorSnapshot(
    IReadOnlyList<MotorPositionSample> Samples,
    long Generation,
    bool DecoderConfirmed,
    string DecoderDescription,
    IReadOnlyList<string> MissingReplyFields,
    MotorRequestState RequestState,
    bool HasPosition,
    bool IsFresh,
    bool HasReference,
    int ReferenceRawPosition,
    int RawPosition,
    double DisplacementMm,
    double RelativeAngleDeg,
    TimeSpan? LastPositionArrival,
    TimeSpan? LastQuerySentAt,
    TimeSpan? LastReplyArrival,
    int LastReplyDlc,
    string? LastReplyHex,
    long TargetCommandsSent,
    long ParameterAcks,
    uint? LastAckEchoId,
    TimeSpan? LastAckArrival,
    byte DeviceId,
    double WheelDiameterMm,
    int PollIntervalMs,
    int ReplyTimeoutMs,
    int DataTimeoutMs,
    long QueriesSent,
    long PositionReplies,
    long AllRepliesReceived,
    long Timeouts,
    long RejectedReplies,
    long UnconfirmedReplies,
    long RemainingDisplacementFrames,
    bool HasRemainingDisplacement,
    int RemainingDisplacementRaw,
    double RemainingDisplacementMm,
    TimeSpan? RemainingDisplacementArrival,
    long TorqueFrames,
    long HeartbeatFrames,
    long OtherQueryFrames,
    long ErrorFrames,
    ushort? LastErrorCode,
    TimeSpan? LastErrorArrival,
    IReadOnlyList<string> RawLogLines);

/// <summary>
/// 断线重连时沿用的状态，让“没点清零”的下一次连接接着上一段继续：
/// 时间轴紧接上次最后一点、电机批次号不变（旧图线保留）、P0 沿用。
/// </summary>
/// <param name="TimeOffsetSeconds">新一段的时间轴起点（= 上一段最后一点的时间，秒）。</param>
/// <param name="MotorGeneration">上一段的电机曲线批次号。</param>
/// <param name="ReferenceRawPosition">上一段的电机起点 P0（原始位置）；null = 没有建立过。</param>
public sealed record MotorResumeState(
    double TimeOffsetSeconds,
    long MotorGeneration,
    int? ReferenceRawPosition);
