using System.Text;
using GyroHost.Core.Communication;
using GyroHost.Core.Protocol;

namespace GyroHost.Tests;

/// <summary>
/// 测试用假连接：让测试直接投喂串口字节，并记录上位机发出的 SLCAN 命令。
/// </summary>
internal sealed class FakeCanConnection : ICanConnection
{
    private readonly List<string> _sentLines = new();

    public string Description => "FAKE";

    public bool IsConnected { get; private set; }

    public event Action<LinkState, string>? StateChanged;

    public event Action<byte[], TimeSpan>? BytesReceived;

    public IReadOnlyList<string> SentLines
    {
        get
        {
            lock (_sentLines)
            {
                return _sentLines.ToArray();
            }
        }
    }

    public Task ConnectAsync(CancellationToken cancellationToken = default)
    {
        IsConnected = true;
        StateChanged?.Invoke(LinkState.Connected, "fake connected");
        return Task.CompletedTask;
    }

    public Task SendSlcanLineAsync(string line, CancellationToken cancellationToken = default)
    {
        lock (_sentLines)
        {
            _sentLines.Add(line);
        }

        return Task.CompletedTask;
    }

    public Task DisconnectAsync()
    {
        IsConnected = false;
        StateChanged?.Invoke(LinkState.Disconnected, "fake disconnected");
        return Task.CompletedTask;
    }

    public void Dispose()
    {
    }

    /// <summary>以当前单调时钟投喂一段 SLCAN 文本。</summary>
    public void EmitText(string text) => Emit(Encoding.ASCII.GetBytes(text), MonotonicClock.Now);

    /// <summary>以指定到达时刻投喂字节，用于验证"重置前已到达的数据"。</summary>
    public void Emit(byte[] bytes, TimeSpan arrival) => BytesReceived?.Invoke(bytes, arrival);

    /// <summary>按 0x321 协议投喂一条测量帧。</summary>
    public void EmitDataFrame(
        double angleDeg,
        double temperatureC,
        GyroHost.Core.Protocol.GyroStatus status,
        int intervalMs = 10)
    {
        byte[] data = GyroHost.Core.Protocol.GyroProtocol.BuildDataFrame(angleDeg, temperatureC, status, intervalMs);
        EmitText(SlcanCodec.EncodeStandardDataFrame(GyroHost.Core.Protocol.GyroProtocol.DataId, data));
    }

    /// <summary>按 0x323 协议投喂一条角速度帧。</summary>
    public void EmitRateFrame(double rateDps)
    {
        byte[] data = GyroHost.Core.Protocol.GyroProtocol.BuildRateFrame(rateDps);
        EmitText(SlcanCodec.EncodeStandardDataFrame(GyroHost.Core.Protocol.GyroProtocol.RateId, data));
    }

    /// <summary>按 0x324 协议投喂一条原始采样帧。</summary>
    public void EmitRawFrame(short rawCode, uint sampleTickMs, double biasDps)
    {
        byte[] data = GyroProtocol.BuildRawFrame(rawCode, sampleTickMs, biasDps);
        EmitText(SlcanCodec.EncodeStandardDataFrame(GyroProtocol.RawId, data));
    }

    /// <summary>按 0x325 协议投喂一条滤波明细帧。</summary>
    public void EmitRawDetailFrame(double filteredRateDps, uint sampleTickMs)
    {
        byte[] data = GyroProtocol.BuildRawDetailFrame(filteredRateDps, sampleTickMs);
        EmitText(SlcanCodec.EncodeStandardDataFrame(GyroProtocol.RawDetailId, data));
    }
}
