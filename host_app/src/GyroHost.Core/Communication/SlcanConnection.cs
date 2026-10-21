using System.Diagnostics;
using System.IO.Ports;
using System.Text;
using GyroHost.Core.Protocol;

namespace GyroHost.Core.Communication;

/// <summary>串口 + SLCAN 初始化参数。</summary>
public sealed class SlcanConnectionOptions
{
    /// <summary>串口号，例如 "COM3"。</summary>
    public string PortName { get; set; } = "COM3";

    /// <summary>串口波特率（与 CAN 波特率无关）。</summary>
    public int SerialBaudRate { get; set; } = SerialPortCatalog.DefaultBaudRate;

    /// <summary>CAN 总线波特率，单位 kbps。当前课题为 1000。</summary>
    public int CanBitRateKbps { get; set; } = SlcanCodec.DefaultBitRateKbps;

    /// <summary>读超时。用于让接收线程周期性检查取消标志，并让断开及时生效。</summary>
    public int ReadTimeoutMs { get; set; } = 150;

    /// <summary>初始化命令之间的等待时间，给适配器留出处理时间。</summary>
    public int InitCommandDelayMs { get; set; } = 80;

    /// <summary>初始化前先发 C 关闭通道，避免适配器残留上一次的打开状态。</summary>
    public bool SendCloseFirst { get; set; } = true;

    /// <summary>初始化完成后发送 O 以正常模式打开通道。</summary>
    public bool SendOpenLast { get; set; } = true;
}

/// <summary>
/// 通过串口和 SLCAN（LAWICEL/CANABLE 风格 ASCII 协议）访问 CAN 总线。
/// <para>
/// 初始化顺序：C（关闭）→ Sx（设置 CAN 波特率）→ O（正常模式打开）。
/// 之后适配器主动上报 "tIIILDD..\r" 形式的帧文本；本类只负责搬运原始字节，
/// 行切分与帧解析交给 <see cref="SlcanFrameParser"/>。
/// </para>
/// </summary>
public sealed class SlcanConnection : ICanConnection
{
    private readonly SlcanConnectionOptions _options;
    private readonly object _writeGate = new();

    private SerialPort? _port;
    private Thread? _receiveThread;
    private CancellationTokenSource? _cts;

    public SlcanConnection(SlcanConnectionOptions options)
    {
        _options = options ?? throw new ArgumentNullException(nameof(options));
    }

    /// <inheritdoc />
    public string Description =>
        $"{_options.PortName} @{_options.SerialBaudRate} bps, CAN {_options.CanBitRateKbps} kbps";

    /// <inheritdoc />
    public bool IsConnected { get; private set; }

    /// <inheritdoc />
    public event Action<LinkState, string>? StateChanged;

    /// <inheritdoc />
    public event Action<byte[], TimeSpan>? BytesReceived;

    /// <inheritdoc />
    public async Task ConnectAsync(CancellationToken cancellationToken = default)
    {
        if (IsConnected)
        {
            throw new InvalidOperationException("通道已经连接，请先断开。");
        }

        if (!SlcanCodec.TryBuildBitRateCommand(_options.CanBitRateKbps, out string bitRateCommand))
        {
            throw new NotSupportedException($"适配器协议（标准 SLCAN）不支持 {_options.CanBitRateKbps} kbps，请改用支持的标准速率。");
        }

        RaiseState(LinkState.Connecting, $"正在打开 {_options.PortName} ...");

        SerialPort port = new(_options.PortName, _options.SerialBaudRate, Parity.None, 8, StopBits.One)
        {
            ReadTimeout = _options.ReadTimeoutMs,
            WriteTimeout = 1000,
            Handshake = Handshake.None,
            DtrEnable = false,
            RtsEnable = false,
        };

        try
        {
            port.Open();
        }
        catch (Exception ex)
        {
            port.Dispose();
            string message = ex switch
            {
                UnauthorizedAccessException => $"打开 {_options.PortName} 失败：端口被占用或无权限（可能已被 USB-CAN 助手等程序打开）。",
                IOException => $"打开 {_options.PortName} 失败：{ex.Message}（设备可能已被拔出）。",
                ArgumentException => $"串口名 {_options.PortName} 无效。",
                _ => $"打开 {_options.PortName} 失败：{ex.Message}",
            };
            RaiseState(LinkState.Faulted, message);
            throw new InvalidOperationException(message, ex);
        }

        _port = port;
        _cts = new CancellationTokenSource();
        _receiveThread = new Thread(() => ReceiveLoop(port, _cts.Token))
        {
            IsBackground = true,
            Name = "SLCAN-RX",
        };
        _receiveThread.Start();

        try
        {
            if (_options.SendCloseFirst)
            {
                await WriteLineAsync(SlcanCodec.CloseChannel, cancellationToken).ConfigureAwait(false);
                await Task.Delay(_options.InitCommandDelayMs, cancellationToken).ConfigureAwait(false);
            }

            await WriteLineAsync(bitRateCommand, cancellationToken).ConfigureAwait(false);
            await Task.Delay(_options.InitCommandDelayMs, cancellationToken).ConfigureAwait(false);

            if (_options.SendOpenLast)
            {
                await WriteLineAsync(SlcanCodec.OpenChannel, cancellationToken).ConfigureAwait(false);
                await Task.Delay(_options.InitCommandDelayMs, cancellationToken).ConfigureAwait(false);
            }
        }
        catch (Exception ex)
        {
            await DisconnectAsync().ConfigureAwait(false);
            string message = $"SLCAN 初始化失败：{ex.Message}";
            RaiseState(LinkState.Faulted, message);
            throw new InvalidOperationException(message, ex);
        }

        IsConnected = true;
        RaiseState(LinkState.Connected, $"已连接：{Description}（经典 CAN、标准数据帧、正常模式）");
    }

    /// <inheritdoc />
    public async Task SendSlcanLineAsync(string line, CancellationToken cancellationToken = default)
    {
        ArgumentException.ThrowIfNullOrEmpty(line);
        await WriteLineAsync(line, cancellationToken).ConfigureAwait(false);
    }

    /// <inheritdoc />
    public async Task DisconnectAsync()
    {
        CancellationTokenSource? cts = _cts;
        Thread? thread = _receiveThread;
        SerialPort? port = _port;

        _cts = null;
        _receiveThread = null;
        _port = null;
        IsConnected = false;

        if (cts is not null)
        {
            try
            {
                cts.Cancel();
            }
            catch (ObjectDisposedException)
            {
                // 已释放，忽略。
            }
        }

        if (port is not null)
        {
            try
            {
                if (port.IsOpen)
                {
                    // 关闭前尽量让适配器回到关闭状态，避免下次连接时通道仍处于打开状态。
                    try
                    {
                        byte[] close = Encoding.ASCII.GetBytes(SlcanCodec.CloseChannel);
                        port.Write(close, 0, close.Length);
                        await Task.Delay(30).ConfigureAwait(false);
                    }
                    catch (Exception)
                    {
                        // 关闭命令发不出去不影响断开流程。
                    }

                    port.Close();
                }
            }
            catch (Exception)
            {
                // 设备已拔出时 Close 可能抛异常，忽略即可。
            }
            finally
            {
                port.Dispose();
            }
        }

        if (thread is not null && thread.IsAlive)
        {
            // 接收线程靠读超时轮询取消标志，这里给足它退出时间，避免和下一次连接叠加。
            thread.Join(TimeSpan.FromMilliseconds(1000));
        }

        if (cts is not null)
        {
            cts.Dispose();
        }

        RaiseState(LinkState.Disconnected, "已断开。");
    }

    /// <inheritdoc />
    public void Dispose()
    {
        try
        {
            DisconnectAsync().GetAwaiter().GetResult();
        }
        catch (Exception)
        {
            // 释放路径不抛异常。
        }
    }

    private async Task WriteLineAsync(string line, CancellationToken cancellationToken)
    {
        SerialPort port = _port ?? throw new InvalidOperationException("串口尚未打开。");
        if (!port.IsOpen)
        {
            throw new InvalidOperationException("串口已关闭。");
        }

        byte[] bytes = Encoding.ASCII.GetBytes(line.EndsWith('\r') ? line : line + "\r");

        await Task.Run(
            () =>
            {
                // 串口写入必须串行化：命令发送（UI 线程）与接收线程互不干扰。
                lock (_writeGate)
                {
                    port.Write(bytes, 0, bytes.Length);
                }
            },
            cancellationToken).ConfigureAwait(false);
    }

    private void ReceiveLoop(SerialPort port, CancellationToken token)
    {
        byte[] buffer = new byte[4096];

        while (!token.IsCancellationRequested)
        {
            try
            {
                int read = port.Read(buffer, 0, buffer.Length);
                if (read > 0)
                {
                    // 以"读到字节的时刻"作为到达时间，保证绘图时间轴反映真实接收节奏。
                    TimeSpan arrival = MonotonicClock.Now;
                    byte[] chunk = new byte[read];
                    Buffer.BlockCopy(buffer, 0, chunk, 0, read);
                    BytesReceived?.Invoke(chunk, arrival);
                }
            }
            catch (TimeoutException)
            {
                // 正常现象：这段时间没有数据。回到循环顶部检查取消标志。
                continue;
            }
            catch (Exception ex) when (token.IsCancellationRequested || ex is ObjectDisposedException or InvalidOperationException)
            {
                // 断开流程主动关闭了串口，正常退出。
                break;
            }
            catch (Exception ex)
            {
                if (!token.IsCancellationRequested)
                {
                    IsConnected = false;
                    RaiseState(LinkState.Faulted, $"串口读取失败：{ex.Message}（设备可能已拔出）");
                }

                break;
            }
        }
    }

    private void RaiseState(LinkState state, string message)
    {
        Debug.WriteLine($"[SLCAN] {state}: {message}");
        StateChanged?.Invoke(state, message);
    }
}
