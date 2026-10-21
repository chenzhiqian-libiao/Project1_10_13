using System.Buffers.Binary;
using System.Globalization;
using System.Text;
using GyroHost.Core.Communication;
using GyroHost.Core.Protocol;
using GyroHost.Core.Services;

namespace GyroHost.App.ViewModels;

/// <summary>
/// 自检用的脚本化 CAN 通道：只回答 0x0408 位置查询，用**合成报文**驱动真实的
/// 会话、解码、仓库和界面刷新路径。
/// <para>
/// 这里**不注入自定义布局**：解码走 <see cref="MotorProtocol.CreateDefaultPositionDecoder"/>，
/// 与真实串口连接完全同一条路径，因此自检同时验证“默认解码器确实启用”。
/// ⚠️ 但报文本身是合成的，自检通过**不代表**实机 0x0409 布局已验证。
/// </para>
/// </summary>
internal sealed class ScriptedMotorConnection : ICanConnection
{
    private static readonly int[] Positions = { 1000, 1060, 1120, 1180, 1240, 1300, 1360, 1420, 1480, 1540 };

    private readonly object _gate = new();
    private readonly List<string> _sentLines = new();
    private int _positionIndex;
    private bool _disposed;

    /// <inheritdoc />
    public string Description => "脚本化电机测试通道（合成报文）";

    /// <inheritdoc />
    public bool IsConnected { get; private set; }

    /// <summary>设为 true 后不再回复位置查询，用于验证应答/数据超时。</summary>
    public bool StopResponding { get; set; }

    /// <inheritdoc />
    public event Action<LinkState, string>? StateChanged;

    /// <inheritdoc />
    public event Action<byte[], TimeSpan>? BytesReceived;

    /// <inheritdoc />
    public Task ConnectAsync(CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        IsConnected = true;
        StateChanged?.Invoke(LinkState.Connected, Description);
        return Task.CompletedTask;
    }

    /// <inheritdoc />
    public Task SendSlcanLineAsync(string line, CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);

        string trimmed = line.TrimEnd('\r', '\n');
        lock (_gate)
        {
            _sentLines.Add(trimmed);
        }

        if (!TryParsePositionQuery(trimmed, out byte deviceId, out _))
        {
            return Task.CompletedTask;
        }

        int position = NextPosition();
        _ = Task.Run(
            async () =>
            {
                try
                {
                    await Task.Delay(20, cancellationToken).ConfigureAwait(false);
                }
                catch (OperationCanceledException)
                {
                    return;
                }

                if (!IsConnected || StopResponding)
                {
                    return;
                }

                byte[] data = new byte[8];
                data[0] = deviceId;
                data[1] = MotorProtocol.QueryPositionCode;
                BinaryPrimitives.WriteInt32LittleEndian(data.AsSpan(2, 4), position);

                string reply = SlcanCodec.EncodeStandardDataFrame(MotorProtocol.ReplyId, data);
                BytesReceived?.Invoke(Encoding.ASCII.GetBytes(reply), MonotonicClock.Now);
            },
            CancellationToken.None);

        return Task.CompletedTask;
    }

    /// <inheritdoc />
    public Task DisconnectAsync()
    {
        IsConnected = false;
        StateChanged?.Invoke(LinkState.Disconnected, "脚本化通道已断开");
        return Task.CompletedTask;
    }

    /// <inheritdoc />
    public void Dispose()
    {
        _disposed = true;
        IsConnected = false;
    }

    /// <summary>取出发送过的 SLCAN 行（用于确认“显示归零”没有发 CAN 命令）。</summary>
    public IReadOnlyList<string> GetSentLines()
    {
        lock (_gate)
        {
            return _sentLines.ToArray();
        }
    }

    private int NextPosition()
    {
        lock (_gate)
        {
            int position = Positions[Math.Min(_positionIndex, Positions.Length - 1)];
            _positionIndex++;
            return position;
        }
    }

    private static bool TryParsePositionQuery(string line, out byte deviceId, out byte queryCode)
    {
        deviceId = 0;
        queryCode = 0;

        // SLCAN 标准数据帧：t + 3 位 ID + 1 位 DLC + 数据。
        if (line.Length < 9 ||
            line[0] is not ('t' or 'T') ||
            !line.AsSpan(1, 3).Equals("408", StringComparison.OrdinalIgnoreCase) ||
            line[4] != '2')
        {
            return false;
        }

        deviceId = byte.Parse(line.AsSpan(5, 2), NumberStyles.HexNumber, CultureInfo.InvariantCulture);
        queryCode = byte.Parse(line.AsSpan(7, 2), NumberStyles.HexNumber, CultureInfo.InvariantCulture);
        return queryCode == MotorProtocol.QueryPositionCode;
    }
}
