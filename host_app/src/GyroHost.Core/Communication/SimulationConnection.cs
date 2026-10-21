using System.Text;
using GyroHost.Core.Protocol;

namespace GyroHost.Core.Communication;

/// <summary>模拟器参数。</summary>
public sealed class SimulationConnectionOptions
{
    /// <summary>测量帧周期，与固件 CAN_TX_PERIOD_MS 一致。</summary>
    public int FramePeriodMs { get; set; } = GyroProtocol.NominalFramePeriodMs;

    /// <summary>温度刷新周期，与固件 GYRO_TEMP_PERIOD_MS 一致。</summary>
    public int TemperaturePeriodMs { get; set; } = 1000;

    /// <summary>把一行文本拆成多次发送的概率（%），用于模拟串口分包。</summary>
    public int ChunkSplitProbabilityPercent { get; set; } = 40;

    /// <summary>起始温度。</summary>
    public double InitialTemperatureC { get; set; } = 31.56;

    /// <summary>运动周期（秒）：静止 → 转 +90° → 静止 → 转回 0°。</summary>
    public double MotionPeriodSeconds { get; set; } = 16.0;

    /// <summary>校准耗时（秒），模拟固件 60 秒估计和 20 秒独立验证的等待。</summary>
    public double CalibrationSeconds { get; set; } = GyroProtocol.CalibrationDurationSeconds;
}

/// <summary>
/// 无硬件模拟器。
/// <para>
/// 它不直接产生"测量对象"，而是生成与真实适配器完全相同格式的 SLCAN 文本字节，
/// 并刻意制造分包/粘包。因此下游走的是同一套 <see cref="SlcanFrameParser"/>
/// 和协议解析代码，模拟模式能真实覆盖解析与显示链路。
/// </para>
/// <para>同时它会响应 0x322 命令：01 = 重新校准（80 秒后完成，保留角度）、02 = 角度清零。</para>
/// </summary>
public sealed class SimulationConnection : ICanConnection
{
    private readonly SimulationConnectionOptions _options;
    private readonly SlcanFrameParser _commandParser = new();
    private readonly Random _random = new(20251016);

    /// <summary>保护模拟内部状态。</summary>
    private readonly object _gate = new();

    /// <summary>保证一条 SLCAN 文本不会与另一个线程发出的字节交叉。</summary>
    private readonly object _emitGate = new();

    private CancellationTokenSource? _cts;
    private Task? _worker;

    private double _angleOffsetDeg;
    private TimeSpan? _calibrationEndsAt;
    private double _emittedTemperatureC;
    private TimeSpan _lastTemperatureAt = TimeSpan.MinValue;

    public SimulationConnection(SimulationConnectionOptions? options = null)
    {
        _options = options ?? new SimulationConnectionOptions();
        _emittedTemperatureC = _options.InitialTemperatureC;
    }

    /// <inheritdoc />
    public string Description => "模拟数据源（与真实接收共用同一套解析与显示逻辑）";

    /// <inheritdoc />
    public bool IsConnected { get; private set; }

    /// <inheritdoc />
    public event Action<LinkState, string>? StateChanged;

    /// <inheritdoc />
    public event Action<byte[], TimeSpan>? BytesReceived;

    /// <inheritdoc />
    public Task ConnectAsync(CancellationToken cancellationToken = default)
    {
        if (IsConnected)
        {
            throw new InvalidOperationException("模拟通道已经连接。");
        }

        RaiseState(LinkState.Connecting, "正在启动模拟数据源 ...");

        lock (_gate)
        {
            _cts = new CancellationTokenSource();
            _angleOffsetDeg = 0.0;
            _calibrationEndsAt = MonotonicClock.Now + TimeSpan.FromSeconds(_options.CalibrationSeconds);
            _lastTemperatureAt = TimeSpan.MinValue;
            _worker = Task.Run(() => RunAsync(_cts.Token), CancellationToken.None);
        }

        IsConnected = true;
        RaiseState(LinkState.Connected, $"已连接：{Description}");
        return Task.CompletedTask;
    }

    /// <inheritdoc />
    public Task SendSlcanLineAsync(string line, CancellationToken cancellationToken = default)
    {
        if (!IsConnected)
        {
            throw new InvalidOperationException("模拟通道未连接。");
        }

        // 用真实解析器解析自己发出的命令，保证模拟行为与协议完全一致。
        byte[] bytes = Encoding.ASCII.GetBytes(line.EndsWith('\r') ? line : line + "\r");
        IReadOnlyList<SlcanEvent> events;
        lock (_commandParser)
        {
            events = _commandParser.Feed(bytes, MonotonicClock.Now);
        }

        foreach (SlcanEvent evt in events)
        {
            if (evt is SlcanCanFrame { Frame: { Id: GyroProtocol.CommandId } frame } && frame.Data.Length >= 1)
            {
                ApplyCommand(frame.Data[0]);
            }
        }

        // 适配器会回 z（已写入总线）。模拟器保持一致，让上位机的发送确认逻辑同样被覆盖。
        EmitText("z\r");
        return Task.CompletedTask;
    }

    /// <inheritdoc />
    public Task DisconnectAsync()
    {
        CancellationTokenSource? cts;
        Task? worker;

        lock (_gate)
        {
            cts = _cts;
            worker = _worker;
            _cts = null;
            _worker = null;
        }

        IsConnected = false;

        if (cts is not null)
        {
            try
            {
                cts.Cancel();
            }
            catch (ObjectDisposedException)
            {
                // 已释放。
            }
        }

        try
        {
            worker?.Wait(TimeSpan.FromMilliseconds(500));
        }
        catch (AggregateException)
        {
            // 取消导致的异常无需上报。
        }

        cts?.Dispose();
        _commandParser.Reset();
        RaiseState(LinkState.Disconnected, "模拟数据源已停止。");
        return Task.CompletedTask;
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

    private async Task RunAsync(CancellationToken token)
    {
        try
        {
            while (!token.IsCancellationRequested)
            {
                TimeSpan now = MonotonicClock.Now;
                EmitFrames(now);
                await Task.Delay(_options.FramePeriodMs, token).ConfigureAwait(false);
            }
        }
        catch (OperationCanceledException)
        {
            // 正常停止。
        }
        catch (Exception ex)
        {
            if (!token.IsCancellationRequested)
            {
                RaiseState(LinkState.Faulted, $"模拟数据源异常：{ex.Message}");
            }
        }
    }

    private void EmitFrames(TimeSpan now)
    {
        double seconds = now.TotalSeconds;
        double angle = CurrentAngle(seconds);
        double rate = CurrentRate(seconds);

        StringBuilder builder = new();
        builder.Append(SlcanCodec.EncodeStandardDataFrame(
            GyroProtocol.DataId,
            GyroProtocol.BuildDataFrame(angle, RefreshTemperature(now), BuildStatus(rate), _options.FramePeriodMs)));
        builder.Append(SlcanCodec.EncodeStandardDataFrame(
            GyroProtocol.RateId,
            GyroProtocol.BuildRateFrame(rate)));

        // 0x324/0x325：用与 0x323 相同的 rate 反推原始码，零偏固定为 0。
        // 模拟模式因此也能覆盖「原始码 → 上位机配对 → CSV 导出」这条链路。
        uint sampleTickMs = unchecked((uint)(long)now.TotalMilliseconds);
        short rawCode = (short)Math.Clamp(
            Math.Round(rate * GyroProtocol.RawLsbPerDps, MidpointRounding.AwayFromZero),
            short.MinValue,
            short.MaxValue);
        builder.Append(SlcanCodec.EncodeStandardDataFrame(
            GyroProtocol.RawId,
            GyroProtocol.BuildRawFrame(rawCode, sampleTickMs, 0.0)));
        builder.Append(SlcanCodec.EncodeStandardDataFrame(
            GyroProtocol.RawDetailId,
            GyroProtocol.BuildRawDetailFrame(rate, sampleTickMs)));

        EmitText(builder.ToString());
    }

    private double RefreshTemperature(TimeSpan now)
    {
        lock (_gate)
        {
            if (_lastTemperatureAt == TimeSpan.MinValue ||
                now - _lastTemperatureAt >= TimeSpan.FromMilliseconds(_options.TemperaturePeriodMs))
            {
                _lastTemperatureAt = now;
                // 温度缓慢漂移，便于观察温度曲线的变化。
                _emittedTemperatureC = _options.InitialTemperatureC + (0.5 * Math.Sin(now.TotalSeconds * 2.0 * Math.PI / 120.0));
            }

            return _emittedTemperatureC;
        }
    }

    private GyroStatus BuildStatus(double rate)
    {
        GyroStatus status = GyroStatus.AngleValid | GyroStatus.TemperatureValid;

        lock (_gate)
        {
            TimeSpan now = MonotonicClock.Now;
            if (_calibrationEndsAt is { } endsAt && now >= endsAt)
            {
                // 校准仅更新零偏；不能将累计运动角度归零。
                _calibrationEndsAt = null;
            }

            if (_calibrationEndsAt is null)
            {
                status |= GyroStatus.CalibrationDone;
            }
        }

        // 固件用 GYRO_ZUPT_MAX_RATE_DPS = 6.5 °/s 判定静止；模拟器没有 10 ms 级
        // 噪声，这里把门限取成 0.3 °/s，让 StillLocked 只在接近换向点（转速≈0）
        // 时出现，锁定/运动切换看起来更干脆。
        if (Math.Abs(rate) < 0.3)
        {
            status |= GyroStatus.StillLocked;
        }

        return status;
    }

    private double CurrentAngle(double seconds)
    {
        lock (_gate)
        {
            return MotionAngle(seconds) - _angleOffsetDeg;
        }
    }

    private double CurrentRate(double seconds)
    {
        const double Delta = 0.02;
        return (MotionAngle(seconds + Delta) - MotionAngle(seconds - Delta)) / (2.0 * Delta);
    }

    /// <summary>运动曲线：静止 → 转 +90° → 静止 → 转回 0°，如此循环。</summary>
    private double MotionAngle(double seconds)
    {
        double period = Math.Max(8.0, _options.MotionPeriodSeconds);
        double phase = seconds % period;
        double quarter = period / 4.0;

        if (phase < quarter)
        {
            return 0.0;
        }

        if (phase < 2 * quarter)
        {
            return 90.0 * Smoothstep((phase - quarter) / quarter);
        }

        if (phase < 3 * quarter)
        {
            return 90.0;
        }

        return 90.0 * (1.0 - Smoothstep((phase - (3 * quarter)) / quarter));
    }

    private static double Smoothstep(double x)
    {
        x = Math.Clamp(x, 0.0, 1.0);
        return x * x * (3.0 - (2.0 * x));
    }

    private void ApplyCommand(byte command)
    {
        lock (_gate)
        {
            switch (command)
            {
                case GyroProtocol.CommandRecalibrate:
                    _calibrationEndsAt = MonotonicClock.Now + TimeSpan.FromSeconds(_options.CalibrationSeconds);
                    break;

                case GyroProtocol.CommandZeroAngle:
                    _angleOffsetDeg = MotionAngle(MonotonicClock.Now.TotalSeconds);
                    break;
            }
        }
    }

    /// <summary>把一行文本按随机粒度切成多个字节块发出，制造分包与粘包。</summary>
    private void EmitText(string text)
    {
        if (text.Length == 0)
        {
            return;
        }

        // 整行在同一把锁内发出，避免命令应答与周期帧在线路中间交叉，产生非法文本。
        lock (_emitGate)
        {
            int offset = 0;
            while (offset < text.Length)
            {
                int remaining = text.Length - offset;
                bool split = remaining > 1 && _random.Next(100) < _options.ChunkSplitProbabilityPercent;
                int size = split ? _random.Next(1, remaining) : remaining;

                byte[] chunk = Encoding.ASCII.GetBytes(text.Substring(offset, size));
                BytesReceived?.Invoke(chunk, MonotonicClock.Now);
                offset += size;
            }
        }
    }

    private void RaiseState(LinkState state, string message) => StateChanged?.Invoke(state, message);
}
