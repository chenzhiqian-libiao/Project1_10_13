using GyroHost.Core.Protocol;

namespace GyroHost.Core.Communication;

/// <summary>连接状态。</summary>
public enum LinkState
{
    /// <summary>未连接。</summary>
    Disconnected,

    /// <summary>正在打开串口并初始化 CAN 通道。</summary>
    Connecting,

    /// <summary>已连接，正在接收。</summary>
    Connected,

    /// <summary>出错（串口占用、设备拔出、初始化失败等），消息里带原因。</summary>
    Faulted,
}

/// <summary>
/// CAN 通道抽象。真实实现走 SLCAN 串口，模拟实现产生同样的 SLCAN 文本，
/// 因此两者共用完全相同的下游解析和显示逻辑。
/// </summary>
public interface ICanConnection : IDisposable
{
    /// <summary>用于界面显示的目标描述，例如 "COM3 @115200, 1 Mbps"。</summary>
    string Description { get; }

    /// <summary>当前是否处于已连接状态。</summary>
    bool IsConnected { get; }

    /// <summary>状态变化通知。可能在后台线程触发。</summary>
    event Action<LinkState, string>? StateChanged;

    /// <summary>收到原始串口字节。第二个参数是该字节块的到达时刻（单调时钟）。</summary>
    event Action<byte[], TimeSpan>? BytesReceived;

    /// <summary>打开通道并完成 CAN 初始化。失败抛异常，由上层转成界面状态。</summary>
    Task ConnectAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// 发送一条 SLCAN 命令/帧文本（调用方需保证以 '\r' 结尾）。
    /// 注意这里写入成功只代表"指令已交给适配器"，不代表板端已执行。
    /// </summary>
    Task SendSlcanLineAsync(string line, CancellationToken cancellationToken = default);

    /// <summary>关闭通道。必须可重复调用。</summary>
    Task DisconnectAsync();
}
