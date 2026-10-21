namespace GyroHost.Core.Protocol;

/// <summary>
/// SLCAN 解析产物。所有事件都带到达时刻，便于界面按真实接收时间绘图。
/// </summary>
/// <param name="Arrival">该字节块从串口读到的时刻（单调时钟）。</param>
public abstract record SlcanEvent(TimeSpan Arrival);

/// <summary>收到一条完整 CAN 帧（t / r / T / R 行）。</summary>
public sealed record SlcanCanFrame(CanFrame Frame, TimeSpan Arrival) : SlcanEvent(Arrival);

/// <summary>适配器对发送请求的应答：z = 已写入总线，Z = 被拒绝。</summary>
public sealed record SlcanTxAck(bool Accepted, TimeSpan Arrival) : SlcanEvent(Arrival);

/// <summary>适配器主动上报的状态标志（F 行）。</summary>
public sealed record SlcanStatusFlags(byte Flags, TimeSpan Arrival) : SlcanEvent(Arrival);

/// <summary>适配器的文本信息（V 版本号、N 序列号等）。</summary>
public sealed record SlcanText(string Text, TimeSpan Arrival) : SlcanEvent(Arrival);

/// <summary>无法解析的行，或适配器回送了 BEL。计入异常帧统计。</summary>
public sealed record SlcanParseError(string Reason, string Line, TimeSpan Arrival) : SlcanEvent(Arrival);
