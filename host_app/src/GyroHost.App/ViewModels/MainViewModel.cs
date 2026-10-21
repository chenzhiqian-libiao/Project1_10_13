using System.Collections.ObjectModel;
using System.Globalization;
using System.IO;
using System.Windows;
using System.Windows.Media;
using System.Windows.Threading;
using GyroHost.Core.Communication;
using GyroHost.Core.Protocol;
using GyroHost.Core.Services;
using Microsoft.Win32;
using OxyPlot;

namespace GyroHost.App.ViewModels;

/// <summary>
/// 主界面视图模型：连接管理、实时数值、三张曲线、接收过程中的重置。
/// <para>
/// 与后台的分工：接收线程只负责解析并写入数据仓库；
/// 这里用 100ms 定时器批量取数据再刷新界面，避免每收到一帧就更新 UI。
/// </para>
/// </summary>
public sealed class MainViewModel : ViewModelBase, IDisposable
{
    /// <summary>某一路数据超过该时间没有更新即判定超时（板端 10ms 一帧，留 60 帧余量）。</summary>
    private static readonly TimeSpan FrameTimeout = TimeSpan.FromMilliseconds(600);

    /// <summary>
    /// 每路历史缓存上限。100 Hz 下约 33 分钟（200000 点）；
    /// 内存约 3 × 200000 × 40 B ≈ 24 MB，长测试不会因为缓存先满而丢起点。
    /// </summary>
    private const int MaxHistorySamples = 200000;
    private const int MaxLogLines = 300;
    private const int MaxMotorRawLogLines = 500;
    private const int MotorAngleDecimalPlaces = 5;

    /// <summary>「保存本段数据」第一次使用时的默认目录；不存在时回退到「文档」。</summary>
    private const string PreferredExportDirectory = @"D:\GyroDatamotor";

    /// <summary>
    /// 角度数值显示用的小数位数。
    /// <para>
    /// 固件把角度按 <c>角度 × 100000</c> 四舍五入成 int32 上报
    /// （见 <see cref="GyroProtocol.AngleLsbPerDegree"/>），一个 LSB = 0.00001°，
    /// 所以 5 位小数是协议能表达的完整精度。
    /// 若固件再改比例，需要同时改协议常量与这里的位数。
    /// </para>
    /// </summary>
    private const int AngleDecimalPlaces = 5;

    /// <summary>
    /// 角速度数值显示用的小数位数。
    /// <para>
    /// 固件把角速度按 <c>角速度 × 100000</c> 四舍五入成 int32 上报
    /// （见 <see cref="GyroProtocol.RateLsbPerDps"/>），一个 LSB = 0.00001°/s，
    /// 所以 5 位小数是协议能表达的完整精度。
    /// </para>
    /// <para>
    /// ⚠️ 这是"协议分辨率"，不是"测量分辨率"：传感器 XV7001BB 灵敏度
    /// 280 LSB/(°/s)，1 个原始码 ≈ 0.0036°/s，比协议 1 个 LSB 粗约 357 倍。
    /// 后几位小数显示的是零偏补偿量与滤波输出，看绝对值请以 0.001°/s 量级为准。
    /// </para>
    /// </summary>
    private const int RateDecimalPlaces = 5;

    /// <summary>角度数值格式串，例如 "F5"。</summary>
    private static readonly string AngleFormat = "F" + AngleDecimalPlaces;

    /// <summary>角速度数值格式串，例如 "F5"。</summary>
    private static readonly string RateFormat = "F" + RateDecimalPlaces;

    /// <summary>
    /// 生成曲线悬浮读数格式串。占位符含义（OxyPlot 约定）：
    /// {0}=曲线名、{1}=X 轴名、{2}=X 值（相对时间，3 位小数够用）、{3}=Y 轴名、{4}=Y 值。
    /// <para>
    /// Y 值必须显式给出精度：OxyPlot 默认格式不带小数位数，
    /// 会把 double 的完整精度打印出来（例如 <c>31.560000000000002</c>）。
    /// </para>
    /// </summary>
    private static string BuildTrackerFormat(int decimalPlaces) =>
        "{0}\n{1}: {2:0.###}\n{3}: {4:F" + decimalPlaces + "}";

    /// <summary>角度曲线的悬浮读数格式，精度与角度数值显示一致。</summary>
    private static readonly string AngleTrackerFormat = BuildTrackerFormat(AngleDecimalPlaces);

    /// <summary>角速度曲线的悬浮读数格式，精度与角速度数值显示一致。</summary>
    private static readonly string RateTrackerFormat = BuildTrackerFormat(RateDecimalPlaces);

    /// <summary>电机相对角度曲线的悬浮读数格式。</summary>
    private static readonly string MotorAngleTrackerFormat = BuildTrackerFormat(MotorAngleDecimalPlaces);

    private readonly DispatcherTimer _timer;
    private readonly List<DataSample> _dataHistory = new();
    private readonly List<RateSample> _rateHistory = new();
    private readonly List<RawSample> _rawHistory = new();
    private readonly List<MotorPositionSample> _motorHistory = new();

    private SlcanSession? _session;
    private ICanConnection? _connection;

    private string? _selectedPort;
    private int _selectedBaudRate = SerialPortCatalog.DefaultBaudRate;
    private int _selectedCanBitRateKbps = SlcanCodec.DefaultBitRateKbps;
    private bool _useSimulation;
    private bool _isConnected;
    private bool _isConnecting;
    private string _connectionStateText = "未连接";
    private string _connectionMessageText = "请选择串口后点击“连接”。若暂时没有硬件，可勾选“模拟模式”。";
    private Brush _connectionBrush = Brushes.Gray;

    private string _angleWrappedText = "--";
    private string _angleCumulativeText = "--";
    private string _temperatureText = "--";
    private string _rateText = "--";
    private string _calibrationStateText = "--";
    private string _stillLockText = "--";
    private string _angleValidText = "--";
    private string _sampleIntervalText = "--";
    private string _zeroAngleStateText = "未执行";
    private string _frameCountText = "0x321 0 帧 / 0x323 0 帧";
    private string _errorCountText = "0";
    private string _timeoutText = "0x321 无数据 / 0x323 无数据";
    private string _droppedText = "0";

    // ------------------------------------------------------------ 电机角度（0x0408/0x0409）
    private byte _motorDeviceId = MotorProtocol.DefaultDeviceId;
    private double _motorWheelDiameterMm = MotorProtocol.DefaultWheelDiameterMm;
    private int _motorPollIntervalMs = 100;
    private bool _motorPollingEnabled = true;
    private string _motorDeviceIdText = MotorProtocol.DefaultDeviceId.ToString(CultureInfo.InvariantCulture);
    private string _motorWheelDiameterText = MotorProtocol.DefaultWheelDiameterMm.ToString("0.###", CultureInfo.InvariantCulture);

    private string _motorProtocolStatusText = "位置解码已启用（待实机验证）：等待连接。";
    private string _motorMissingFieldsText = MotorStillUnverifiedText;
    private string _motorRawPositionText = "--";
    private string _motorDisplacementText = "--";
    private string _motorAngleText = "--";
    private string _motorP0Text = "--";
    private string _motorValidText = "--";
    private string _motorQueryStateText = "未连接";
    private string _motorLastFeedbackText = "--";
    private string _motorLastRawReplyText = "--";
    private string _motorRemainingDisplacementText = "--";
    private string _motorErrorText = "--";
    private string _motorCountersText = "查询 0 / 0x0409 0 / 超时 0";
    private string _motorResolutionText = string.Empty;
    private bool _motorCanZero;

    private int? _motorP0Raw;
    private TimeSpan? _motorResetAt;
    private long _motorGeneration;

    // ------------------------------------------------------------ 电机运动控制（0x0413/0x0414/0x0415/0x0416）
    private readonly MotorMotionController _motion = new();
    private bool _motionSendInFlight;
    private string _motorMotionStatusText = "未运行";
    private bool _isMotorCycleRunning;
    private int _motorCycleRounds = 3;
    private string _motorSpeedText = "100";
    private string _motorAccelerationText = "500";
    private string _motorDwellSecondsText = "3";
    private string _motorAckText = "--";

    // ------------------------------------------------------------ 电机初始化（0x0316 / 0x0408 配轮径 / 0x0413）
    private string _motorInitStatusText = "未执行";

    /// <summary>最近一帧的位置反馈是否“新鲜有效”（状态机用它判断能不能开始 / 是否到位）。</summary>
    private bool _lastMotorLive;
    private bool _lastGyroReady;
    private TimeSpan? _lastGyroArrival;
    private double _lastGyroAngle;
    private bool _lastGyroDegraded;
    private double _cycleGyroOrigin;
    private bool _cycleGyroQualityBad;
    private int _residualRound;
    private int _reportedResidualRound;
    private TimeSpan _residualWaitStarted;

    private int _lastMotorRaw;
    private bool _motionWasRunning;

    private int _selectedTimeWindowSeconds = 60;
    private bool _autoFollow = true;

    /// <summary>上一次真正写进曲线的横轴窗口，用于识别用户是否用鼠标改过视角。</summary>
    private double _appliedWindowStart;
    private double _appliedWindowEnd;
    private bool _hasAppliedWindow;

    /// <summary>
    /// 已接入但尚未实机验证的字段。显示在电机卡片的提示框里，
    /// 避免把“按一帧报文接入”误认为“协议已完全确认”。
    /// </summary>
    private const string MotorStillUnverifiedText =
        "待实机核对：位置字段 2/4 字节、有符号性、Data[6..7] 含义；" +
        "用已知角度运动前后的回复验证换算比例（详见 docs/motor-0x0409-real-frames.md）。";

    private double _latestTimeSeconds;
    private TimeSpan? _displayResetAt;
    private bool _disposed;

    // ------------------------------------------------------------ 数据保存（CSV）
    private string _exportRangeText = "可保存：暂无数据（连接并接收后即可保存）";
    private string _exportStatusText = "尚未保存";
    private string? _lastExportDirectory;
    private bool _exportHasData;

    private readonly ICanConnection? _injectedConnection;
    private readonly IMotorPositionDecoder? _injectedMotorDecoder;

    public MainViewModel()
        : this(null, null)
    {
    }

    /// <summary>
    /// 测试/自检用：注入自定义连接和位置解码器，走与真实连接完全相同的会话、解析和刷新路径。
    /// 正常界面始终使用无参构造。
    /// </summary>
    internal MainViewModel(ICanConnection? injectedConnection, IMotorPositionDecoder? motorPositionDecoder)
    {
        _injectedConnection = injectedConnection;
        _injectedMotorDecoder = motorPositionDecoder;

        AngleChart = new LiveChart(
            "角度 — 时间",
            "角度 (°)",
            OxyColor.Parse("#E67E22"),
            jumpThreshold: 180.0,
            trackerFormat: AngleTrackerFormat,
            secondaryTitle: "电机角度",
            secondaryColor: OxyColor.Parse("#8E44AD"),
            secondaryTrackerFormat: MotorAngleTrackerFormat,
            seriesTitle: "陀螺仪角度",
            yMinimum: -180.0,
            yMaximum: 180.0);
        TemperatureChart = new LiveChart("温度 — 时间", "温度 (℃)", OxyColor.Parse("#2980B9"));
        RateChart = new LiveChart(
            "角速度 — 时间",
            "角速度 (°/s)",
            OxyColor.Parse("#27AE60"),
            trackerFormat: RateTrackerFormat);
        GyroCharts = new[] { AngleChart, TemperatureChart, RateChart };

        RefreshPortsCommand = new RelayCommand(RefreshPorts);
        ConnectCommand = new AsyncRelayCommand(ConnectAsync, () => !_isConnected && !_isConnecting, OnCommandError);
        DisconnectCommand = new AsyncRelayCommand(DisconnectAsync, () => _session is not null, OnCommandError);
        ResetDisplayCommand = new RelayCommand(ResetDisplay, () => _session is not null || HasHistory);
        SaveSegmentCommand = new AsyncRelayCommand(SaveSegmentAsync, () => _exportHasData, OnCommandError);
        ZeroAngleCommand = new AsyncRelayCommand(SendZeroAngleAsync, () => _isConnected, OnCommandError);
        RecalibrateCommand = new AsyncRelayCommand(SendRecalibrateAsync, () => _isConnected, OnCommandError);
        MotorZeroCommand = new RelayCommand(ZeroMotorDisplay, () => _motorCanZero);
        RestoreAutoFollowCommand = new RelayCommand(() => RestoreAutoFollow("恢复自动跟随"));
        RestoreDefaultViewCommand = new RelayCommand(RestoreDefaultView);
        RunMotorCycleCommand = new RelayCommand<string>(RunMotorCycle, CanRunMotorCycle);
        CancelMotorCycleCommand = new RelayCommand(CancelMotorCycle, () => _isMotorCycleRunning);
        InitMotorParametersCommand = new AsyncRelayCommand(SendMotorParameterInitAsync, CanSendMotorInitCommand, OnCommandError);
        WriteMotorWheelDiameterCommand = new AsyncRelayCommand(SendMotorWheelDiameterConfigAsync, CanSendMotorInitCommand, OnCommandError);
        MotorEnableCommand = new AsyncRelayCommand(SendMotorEnableAsync, CanSendMotorInitCommand, OnCommandError);
        MotorReleaseCommand = new AsyncRelayCommand(SendMotorReleaseAsync, CanSendMotorInitCommand, OnCommandError);
        ClearLogCommand = new RelayCommand(LogLines.Clear);
        ClearMotorRawLogCommand = new RelayCommand(MotorRawLogLines.Clear);
        CopyMotorRawLogCommand = new RelayCommand(CopyMotorRawLog);

        _timer = new DispatcherTimer(DispatcherPriority.Background)
        {
            Interval = TimeSpan.FromMilliseconds(100),
        };
        _timer.Tick += (_, _) => OnRefreshTick();
        _timer.Start();

        UpdateMotorResolutionText();
        RefreshPorts();
        RefreshExportState();
    }

    // ---------------------------------------------------------------- 图表

    public LiveChart AngleChart { get; }

    public LiveChart TemperatureChart { get; }

    public LiveChart RateChart { get; }

    /// <summary>三张陀螺仪曲线，共用同一时间轴；角度图包含“陀螺仪角度”和“电机角度”两条曲线。</summary>
    private IReadOnlyList<LiveChart> GyroCharts { get; }

    // ------------------------------------------------------------ 连接相关

    public ObservableCollection<string> Ports { get; } = new();

    public ObservableCollection<string> LogLines { get; } = new();

    /// <summary>电机相关 CAN 原始日志（0x0408 发送、0x0409/0x0421/0x0441/0x0381 接收）。</summary>
    public ObservableCollection<string> MotorRawLogLines { get; } = new();

    public IReadOnlyList<int> BaudRates => SerialPortCatalog.StandardBaudRates;

    public IReadOnlyList<int> CanBitRatesKbps => SlcanCodec.SupportedBitRatesKbps;

    public IReadOnlyList<int> TimeWindowOptions { get; } = new[] { 10, 30, 60, 120, 300 };

    public string? SelectedPort
    {
        get => _selectedPort;
        set
        {
            if (SetProperty(ref _selectedPort, value))
            {
                ConnectCommand.RaiseCanExecuteChanged();
            }
        }
    }

    public int SelectedBaudRate
    {
        get => _selectedBaudRate;
        set => SetProperty(ref _selectedBaudRate, value);
    }

    public int SelectedCanBitRateKbps
    {
        get => _selectedCanBitRateKbps;
        set => SetProperty(ref _selectedCanBitRateKbps, value);
    }

    public bool UseSimulation
    {
        get => _useSimulation;
        set
        {
            if (SetProperty(ref _useSimulation, value))
            {
                ConnectCommand.RaiseCanExecuteChanged();
            }
        }
    }

    public bool IsConnected
    {
        get => _isConnected;
        private set
        {
            if (SetProperty(ref _isConnected, value))
            {
                OnPropertyChanged(nameof(CanEditConnectionSettings));
                ConnectCommand.RaiseCanExecuteChanged();
                DisconnectCommand.RaiseCanExecuteChanged();
                ZeroAngleCommand.RaiseCanExecuteChanged();
                RecalibrateCommand.RaiseCanExecuteChanged();
                MotorZeroCommand.RaiseCanExecuteChanged();
            }
        }
    }

    /// <summary>未连接时才允许修改串口/波特率等连接参数。</summary>
    public bool CanEditConnectionSettings => !_isConnected;

    /// <summary>连接状态文字，直接显示在界面上。</summary>
    public string ConnectionStateText
    {
        get => _connectionStateText;
        private set => SetProperty(ref _connectionStateText, value);
    }

    /// <summary>连接状态细节或错误原因。</summary>
    public string ConnectionMessageText
    {
        get => _connectionMessageText;
        private set => SetProperty(ref _connectionMessageText, value);
    }

    public Brush ConnectionBrush
    {
        get => _connectionBrush;
        private set => SetProperty(ref _connectionBrush, value);
    }

    // ------------------------------------------------------------ 实时数值

    public string AngleWrappedText
    {
        get => _angleWrappedText;
        private set => SetProperty(ref _angleWrappedText, value);
    }

    public string AngleCumulativeText
    {
        get => _angleCumulativeText;
        private set => SetProperty(ref _angleCumulativeText, value);
    }

    public string TemperatureText
    {
        get => _temperatureText;
        private set => SetProperty(ref _temperatureText, value);
    }

    public string RateText
    {
        get => _rateText;
        private set => SetProperty(ref _rateText, value);
    }

    public string CalibrationStateText
    {
        get => _calibrationStateText;
        private set => SetProperty(ref _calibrationStateText, value);
    }

    public string StillLockText
    {
        get => _stillLockText;
        private set => SetProperty(ref _stillLockText, value);
    }

    public string AngleValidText
    {
        get => _angleValidText;
        private set => SetProperty(ref _angleValidText, value);
    }

    public string SampleIntervalText
    {
        get => _sampleIntervalText;
        private set => SetProperty(ref _sampleIntervalText, value);
    }

    /// <summary>最近一次"板端角度清零"的实际结果（以回传数据为准）。</summary>
    public string ZeroAngleStateText
    {
        get => _zeroAngleStateText;
        private set => SetProperty(ref _zeroAngleStateText, value);
    }

    public string FrameCountText
    {
        get => _frameCountText;
        private set => SetProperty(ref _frameCountText, value);
    }

    public string ErrorCountText
    {
        get => _errorCountText;
        private set => SetProperty(ref _errorCountText, value);
    }

    public string TimeoutText
    {
        get => _timeoutText;
        private set => SetProperty(ref _timeoutText, value);
    }

    public string DroppedText
    {
        get => _droppedText;
        private set => SetProperty(ref _droppedText, value);
    }

    // ------------------------------------------------------------ 电机角度

    /// <summary>位置查询的设备号（1..15）。</summary>
    public string MotorDeviceIdText
    {
        get => _motorDeviceIdText;
        set
        {
            if (!SetProperty(ref _motorDeviceIdText, value))
            {
                return;
            }

            if (!byte.TryParse(value, NumberStyles.Integer, CultureInfo.InvariantCulture, out byte deviceId) ||
                deviceId == 0 ||
                deviceId > MotorProtocol.MaxDeviceId)
            {
                AppendLogLine($"电机设备号必须是 1..{MotorProtocol.MaxDeviceId} 的整数，修改未生效。");
                _motorDeviceIdText = _motorDeviceId.ToString(CultureInfo.InvariantCulture);
                OnPropertyChanged(nameof(MotorDeviceIdText));
                return;
            }

            if (deviceId == _motorDeviceId)
            {
                return;
            }

            CancelMotorCycle();
            _motorDeviceId = deviceId;
            if (_session is not null)
            {
                _session.MotorDeviceId = deviceId;
            }

            MotorInitStatusText = $"未执行（设备号已改为 {deviceId:00}，初始化需重新发送）";
            ResetMotorChartAndValues();
            AppendLogLine($"电机设备号已改为 {deviceId:00}：P0、位置曲线与统计已重置（未向电机发送配置命令）。");
        }
    }

    /// <summary>角度换算用的轮径（mm）。只影响上位机换算，不写回电机。</summary>
    public string MotorWheelDiameterText
    {
        get => _motorWheelDiameterText;
        set
        {
            if (!SetProperty(ref _motorWheelDiameterText, value))
            {
                return;
            }

            if (!TryParseMotorNumber(value, out double diameter) || diameter <= 0.0 || diameter > 100000.0)
            {
                AppendLogLine("电机轮径必须是大于 0 的数值（mm），修改未生效。");
                _motorWheelDiameterText = _motorWheelDiameterMm.ToString("0.###", CultureInfo.InvariantCulture);
                OnPropertyChanged(nameof(MotorWheelDiameterText));
                return;
            }

            if (Math.Abs(diameter - _motorWheelDiameterMm) < 1e-9)
            {
                return;
            }

            CancelMotorCycle();
            _motorWheelDiameterMm = diameter;
            if (_session is not null)
            {
                _session.MotorWheelDiameterMm = diameter;
            }

            UpdateMotorResolutionText();
            RebuildMotorChart();
            OnPropertyChanged(nameof(MotorCycleSummaryText));
            OnPropertyChanged(nameof(MotorWheelConfigText));
            AppendLogLine($"电机轮径已改为 {diameter:0.###} mm：只影响上位机角度换算，未改写电机轮径配置。");
        }
    }

    public IReadOnlyList<int> MotorPollIntervalOptions { get; } = new[] { 50, 100, 200, 500, 1000 };

    /// <summary>位置查询周期（ms）。默认 100 ms。</summary>
    public int MotorPollIntervalMs
    {
        get => _motorPollIntervalMs;
        set
        {
            if (!SetProperty(ref _motorPollIntervalMs, value))
            {
                return;
            }

            if (_session is not null)
            {
                _session.MotorPollIntervalMs = value;
            }

            AppendLogLine($"电机位置查询周期已改为 {value} ms。");
        }
    }

    /// <summary>是否启用位置轮询（默认开启）。</summary>
    public bool MotorPollingEnabled
    {
        get => _motorPollingEnabled;
        set
        {
            if (!SetProperty(ref _motorPollingEnabled, value))
            {
                return;
            }

            if (_session is not null)
            {
                _session.MotorPollingEnabled = value;
            }

            AppendLogLine(value ? "已启用电机位置轮询。" : "已停用电机位置轮询（不会向总线发送 0x0408）。");
        }
    }

    /// <summary>协议状态横幅：未确认时明确说明缺失字段，确认后显示解码器说明。</summary>
    public string MotorProtocolStatusText
    {
        get => _motorProtocolStatusText;
        private set => SetProperty(ref _motorProtocolStatusText, value);
    }

    public string MotorMissingFieldsText
    {
        get => _motorMissingFieldsText;
        private set => SetProperty(ref _motorMissingFieldsText, value);
    }

    /// <summary>位置原始值。协议未确认时显示“待确认协议”，确认后无有效数据显示“--”。</summary>
    public string MotorRawPositionText
    {
        get => _motorRawPositionText;
        private set => SetProperty(ref _motorRawPositionText, value);
    }

    public string MotorDisplacementText
    {
        get => _motorDisplacementText;
        private set => SetProperty(ref _motorDisplacementText, value);
    }

    /// <summary>相对起点累计角度（°），不取模。</summary>
    public string MotorAngleText
    {
        get => _motorAngleText;
        private set => SetProperty(ref _motorAngleText, value);
    }

    public string MotorP0Text
    {
        get => _motorP0Text;
        private set => SetProperty(ref _motorP0Text, value);
    }

    public string MotorValidText
    {
        get => _motorValidText;
        private set => SetProperty(ref _motorValidText, value);
    }

    public string MotorQueryStateText
    {
        get => _motorQueryStateText;
        private set => SetProperty(ref _motorQueryStateText, value);
    }

    /// <summary>最新一次“有效位置”反馈的墙钟时间；没有有效位置时显示“--”。</summary>
    public string MotorLastFeedbackText
    {
        get => _motorLastFeedbackText;
        private set => SetProperty(ref _motorLastFeedbackText, value);
    }

    /// <summary>最新一次 0x0409 原始回复（不论能否解码），用于协议确认。</summary>
    public string MotorLastRawReplyText
    {
        get => _motorLastRawReplyText;
        private set => SetProperty(ref _motorLastRawReplyText, value);
    }

    public string MotorRemainingDisplacementText
    {
        get => _motorRemainingDisplacementText;
        private set => SetProperty(ref _motorRemainingDisplacementText, value);
    }

    /// <summary>最近一次 0x0100..0x017F 错误上报（协议表错误码）。</summary>
    public string MotorErrorText
    {
        get => _motorErrorText;
        private set => SetProperty(ref _motorErrorText, value);
    }

    public string MotorCountersText
    {
        get => _motorCountersText;
        private set => SetProperty(ref _motorCountersText, value);
    }

    /// <summary>分辨率说明：电机角度受位置单位（0.1 mm）限制，不能照搬陀螺仪的 0.00001°。</summary>
    public string MotorResolutionText
    {
        get => _motorResolutionText;
        private set => SetProperty(ref _motorResolutionText, value);
    }

    // ------------------------------------------------------------ 电机运动控制

    /// <summary>可选的往复目标角度（°）。</summary>
    public IReadOnlyList<int> MotorAngleTargetOptions { get; } = new[] { 90, 180, 360, 720 };

    /// <summary>往复轮数（1..50），默认 3。</summary>
    public int MotorCycleRounds
    {
        get => _motorCycleRounds;
        set
        {
            int clamped = Math.Clamp(value, 1, 50);
            if (SetProperty(ref _motorCycleRounds, clamped))
            {
                OnPropertyChanged(nameof(MotorCycleSummaryText));
            }
        }
    }

    /// <summary>运动前设置的主速度（mm/s）。</summary>
    public string MotorSpeedText
    {
        get => _motorSpeedText;
        set
        {
            if (SetProperty(ref _motorSpeedText, value))
            {
                OnPropertyChanged(nameof(MotorCycleSummaryText));
            }
        }
    }

    /// <summary>运动前设置的主加速度（mm/s²）。</summary>
    public string MotorAccelerationText
    {
        get => _motorAccelerationText;
        set
        {
            if (SetProperty(ref _motorAccelerationText, value))
            {
                OnPropertyChanged(nameof(MotorCycleSummaryText));
            }
        }
    }

    /// <summary>每轮正转到位后停顿多久再返程（s），0 = 不停顿。</summary>
    public string MotorDwellSecondsText
    {
        get => _motorDwellSecondsText;
        set
        {
            if (SetProperty(ref _motorDwellSecondsText, value))
            {
                OnPropertyChanged(nameof(MotorCycleSummaryText));
            }
        }
    }

    /// <summary>运动状态说明（状态机文字）。</summary>
    public string MotorMotionStatusText
    {
        get => _motorMotionStatusText;
        private set => SetProperty(ref _motorMotionStatusText, value);
    }

    /// <summary>是否正在执行往复运动。</summary>
    public bool IsMotorCycleRunning
    {
        get => _isMotorCycleRunning;
        private set
        {
            if (SetProperty(ref _isMotorCycleRunning, value))
            {
                RunMotorCycleCommand.RaiseCanExecuteChanged();
                CancelMotorCycleCommand.RaiseCanExecuteChanged();
                RaiseMotorInitCommandStatesChanged();
            }
        }
    }

    // ------------------------------------------------------------ 电机初始化

    /// <summary>
    /// 最近一次初始化动作（0x0316 / 0x0408 配轮径 / 0x0413）的结果说明。
    /// <para>这些帧都没有应答，所以只显示“已发送”，不会显示“已生效”。</para>
    /// </summary>
    public string MotorInitStatusText
    {
        get => _motorInitStatusText;
        private set => SetProperty(ref _motorInitStatusText, value);
    }

    /// <summary>
    /// 当前轮径与 <c>0x0408 / 0x24</c> 实际会写入的原始值的对照，例如
    /// <c>100 mm → 0x2710（10000 × 0.01 mm），连发 3 次</c>。
    /// </summary>
    public string MotorWheelConfigText
    {
        get
        {
            try
            {
                ushort raw = MotorProtocol.MillimetersToWheelDiameterRaw(_motorWheelDiameterMm);
                return $"{_motorWheelDiameterMm:0.##} mm → 0x{raw:X4}（{raw} × 0.01 mm），连发 " +
                       $"{MotorProtocol.WheelConfigRepeatCount} 次";
            }
            catch (ArgumentOutOfRangeException)
            {
                return $"当前轮径 {_motorWheelDiameterMm:0.##} mm 超出可写入范围 " +
                       $"0.01..{MotorProtocol.MaxWritableWheelDiameterMm:0.##} mm，无法写入电机";
            }
        }
    }

    /// <summary>最近一次 0x041A 参数设置 ACK 的说明（协议未确认，只做诊断）。</summary>
    public string MotorAckText
    {
        get => _motorAckText;
        private set => SetProperty(ref _motorAckText, value);
    }

    /// <summary>运动参数摘要，放在按钮上方提示会发哪些命令。</summary>
    public string MotorCycleSummaryText
    {
        get
        {
            string deltas;
            try
            {
                deltas = string.Join(
                    " / ",
                    MotorAngleTargetOptions.Select(
                        angle => $"{angle}° ≈ {MotorProtocol.DegreesToPositionDelta(angle, _motorWheelDiameterMm)}"));
            }
            catch (ArgumentOutOfRangeException)
            {
                deltas = "轮径无效";
            }

            return $"每段位移（轮径 {_motorWheelDiameterMm:0.###} mm）：{deltas} LSB；" +
                   $"共 {_motorCycleRounds} 轮（正转→{_motorDwellSecondsText} s 停靠→返回→停靠，" +
                   $"停靠不重新校准、不清零，保留返回残差，0 = 不停顿）；" +
                   $"速度 {_motorSpeedText} mm/s、加速度 {_motorAccelerationText} mm/s²。";
        }
    }

    // ------------------------------------------------------------ 显示选项

    public int SelectedTimeWindowSeconds
    {
        get => _selectedTimeWindowSeconds;
        set
        {
            if (SetProperty(ref _selectedTimeWindowSeconds, value))
            {
                // 换窗口是显式操作：即使当前暂停跟随，也要按新窗口重画一次，
                // 同时清掉 OxyPlot 的缩放覆盖（否则设了范围也看不出变化）。
                _hasAppliedWindow = false;
                ApplyFollowWindow();
                FlushCharts();
            }
        }
    }

    /// <summary>自动跟随最新数据。用户缩放/拖动后自动关闭，可点按钮恢复。</summary>
    public bool AutoFollow
    {
        get => _autoFollow;
        private set
        {
            if (SetProperty(ref _autoFollow, value))
            {
                OnPropertyChanged(nameof(AutoFollowText));
            }
        }
    }

    /// <summary>自动跟随状态说明文字。</summary>
    public string AutoFollowText => _autoFollow
        ? "自动跟随：开（视角持续跟随最新数据）"
        : "自动跟随：已暂停（可用鼠标缩放/拖动；点“跟随最新数据”立即回到最新并持续跟随）";

    // -------------------------------------------------------------- 数据保存（CSV）

    /// <summary>
    /// 可保存范围说明（点数 + 时间跨度）。
    /// <para>显式写出上限，避免用户误以为导出了全程。</para>
    /// </summary>
    public string ExportRangeText
    {
        get => _exportRangeText;
        private set => SetProperty(ref _exportRangeText, value);
    }

    /// <summary>上一次保存的结果（或取消、失败原因）。</summary>
    public string ExportStatusText
    {
        get => _exportStatusText;
        private set => SetProperty(ref _exportStatusText, value);
    }

    // -------------------------------------------------------------- 命令

    public RelayCommand RefreshPortsCommand { get; }

    public AsyncRelayCommand ConnectCommand { get; }

    public AsyncRelayCommand DisconnectCommand { get; }

    public RelayCommand ResetDisplayCommand { get; }

    public AsyncRelayCommand ZeroAngleCommand { get; }

    public AsyncRelayCommand RecalibrateCommand { get; }

    /// <summary>“电机显示归零”：把当前有效位置设为 P0。不发送任何电机命令。</summary>
    public RelayCommand MotorZeroCommand { get; }

    public RelayCommand RestoreAutoFollowCommand { get; }

    /// <summary>
    /// 把当前内存中的角度（0x321）与角速度（0x323）历史保存为 CSV。
    /// <para>
    /// 范围是「上一次『重置显示数据』以来的全部缓存」，每路上限 <see cref="MaxHistorySamples"/> 点。
    /// 两路各自保留本帧时间戳，不插值、不配对。
    /// </para>
    /// </summary>
    public AsyncRelayCommand SaveSegmentCommand { get; }

    /// <summary>恢复默认视角：重新跟随 + 纵轴回到默认（角度图回 ±180°）。</summary>
    public RelayCommand RestoreDefaultViewCommand { get; }

    /// <summary>以指定角度（参数为 “90”/“180”/“360”/“720”）执行往复运动。</summary>
    public RelayCommand<string> RunMotorCycleCommand { get; }

    /// <summary>取消正在进行的往复运动（不会撤回已发出的命令）。</summary>
    public RelayCommand CancelMotorCycleCommand { get; }

    /// <summary>
    /// 发送 <c>0x0316</c> 参数初始化（建立 PID 基准）。
    /// <para>⚠️ 补充资料帧，无应答；发送成功只表示已交给适配器。</para>
    /// </summary>
    public AsyncRelayCommand InitMotorParametersCommand { get; }

    /// <summary>
    /// 发送 <c>0x0408 / 0x24</c> 配置轮径（连发 3 次，单位 0.01 mm）。
    /// <para>⚠️ 补充资料帧，无应答；发送成功只表示已交给适配器。</para>
    /// </summary>
    public AsyncRelayCommand WriteMotorWheelDiameterCommand { get; }

    /// <summary>发送 <c>0x0413 = 0x5A</c> 上电使能（锁定）。</summary>
    public AsyncRelayCommand MotorEnableCommand { get; }

    /// <summary>发送 <c>0x0413 = 0x4A</c> 下电释放。</summary>
    public AsyncRelayCommand MotorReleaseCommand { get; }

    public RelayCommand ClearLogCommand { get; }

    public RelayCommand ClearMotorRawLogCommand { get; }

    public RelayCommand CopyMotorRawLogCommand { get; }

    // -------------------------------------------------------------- 操作

    /// <summary>刷新可用串口列表，不写死 COM 号。</summary>
    public void RefreshPorts()
    {
        string? previous = SelectedPort;
        Ports.Clear();
        foreach (string name in SerialPortCatalog.GetPortNames())
        {
            Ports.Add(name);
        }

        if (Ports.Count == 0)
        {
            SelectedPort = null;
            AppendLogLine("未检测到可用串口。请检查 USB-CAN 适配器是否插好，或改用模拟模式。");
            return;
        }

        SelectedPort = previous is not null && Ports.Contains(previous) ? previous : Ports[0];
        AppendLogLine($"检测到 {Ports.Count} 个串口：{string.Join("、", Ports)}");
    }

    /// <summary>
    /// 重置显示（“清零”）：清空三张曲线（角度图含陀螺仪与电机两条）、历史缓存与待绘制数据，
    /// 相对时间从 0 重新开始，纵轴回到默认 ±180°，并自动恢复跟随。
    /// <para>
    /// 只影响上位机显示，不发送任何板端命令，连接与接收保持。
    /// 电机的起点 P0 会重新建立（由清零之后的第一帧位置反馈决定），电机曲线才真正从 0 起画。
    /// </para>
    /// </summary>
    public void ResetDisplay()
    {
        // 正在进行往复运动时先停状态机：否则曲线清空后它还会继续发目标位置命令。
        CancelMotorCycle();

        _session?.ResetDisplay();
        _session?.ResetMotorReference();

        _dataHistory.Clear();
        _rateHistory.Clear();
        _rawHistory.Clear();
        _motorHistory.Clear();
        _latestTimeSeconds = 0.0;
        AngleChart.Clear();
        TemperatureChart.Clear();
        RateChart.Clear();

        // 数值显示等待下一帧：既不复用重置前的旧值，也不把无效数据显示成 0。
        TimeSpan now = MonotonicClock.Now;
        _displayResetAt = now;
        _motorResetAt = now;
        _motorP0Raw = null;

        // 清零后不能再拿重置前的“新鲜位置”开新运动，必须等下一帧反馈。
        _lastMotorLive = false;
        AngleWrappedText = "--";
        AngleCumulativeText = "--";
        TemperatureText = "--";
        RateText = "--";
        SampleIntervalText = "--";
        AngleValidText = "--";
        MotorRawPositionText = "--";
        MotorDisplacementText = "--";
        MotorAngleText = "--";
        MotorP0Text = "--";
        MotorValidText = "--";
        MotorQueryStateText = "等待新反馈";
        MotorLastFeedbackText = "--";
        _motorCanZero = false;
        MotorZeroCommand.RaiseCanExecuteChanged();

        RefreshExportState();
        ExportStatusText = "尚未保存（显示数据已重置，保存范围从 0 重新计时）";

        ApplyDefaultView();
        AppendLogLine(
            "已重置显示数据：曲线已清空、纵轴回到 ±180° 并自动跟随，相对时间从 0 重新开始" +
            "（板端未受影响；电机 P0 将由清零后的第一帧位置反馈重新建立）。");
    }

    /// <summary>恢复自动跟随：立即跳到最新数据并持续跟随（不清数据、不改纵轴）。</summary>
    public void RestoreAutoFollow(string reason)
    {
        bool wasFollowing = AutoFollow;

        // 丢弃旧的“已应用窗口”记录：否则刚才那次缩放会被立刻判成用户操作，又暂停一次。
        _hasAppliedWindow = false;

        AutoFollow = true;
        UpdateChartWindows();
        FlushCharts();

        AppendLogLine(wasFollowing
            ? $"视角已回到最新数据（{reason}）：自动跟随保持开启。"
            : $"已恢复自动跟随（{reason}）：视角跳到最新数据并持续跟随。");
    }

    /// <summary>
    /// 恢复默认视角：重新跟随 + 纵轴回到默认范围（角度图回 ±180°）。
    /// <para>
    /// 纵轴与横轴都会清掉 OxyPlot 的鼠标缩放/拖动覆盖（<c>View*</c>），
    /// 因此这个按钮在任何缩放状态下都一定生效。
    /// </para>
    /// </summary>
    public void RestoreDefaultView()
    {
        _hasAppliedWindow = false;

        foreach (LiveChart chart in GyroCharts)
        {
            chart.ResetYRange();
        }

        AutoFollow = true;
        UpdateChartWindows();
        FlushCharts();
        AppendLogLine("已恢复默认视角：三张曲线纵轴回到默认范围（角度 ±180°）、横轴回到最新窗口并持续跟随。");
    }

    /// <summary>
    /// 用户用鼠标缩放/拖动后暂停跟随。OxyPlot 的缩放是异步的，所以调用方可以延后一点点再调，
    /// 这样不会立刻被下一次刷新覆盖。
    /// </summary>
    public void PauseAutoFollow(string reason)
    {
        if (!AutoFollow)
        {
            return;
        }

        AutoFollow = false;
        AppendLogLine($"已暂停自动跟随（{reason}）：可以自由缩放/拖动；点“跟随最新数据”回到最新并持续跟随。");
    }

    /// <summary>是否有可清空的历史数据（断开后也能清零）。</summary>
    public bool HasHistory
        => _dataHistory.Count > 0 || _rateHistory.Count > 0 || _rawHistory.Count > 0 ||
           _motorHistory.Count > 0 || _latestTimeSeconds > 0.0;

    /// <summary>
    /// 把当前内存中的角度、角速度与电机位置历史导出为 CSV。
    /// <para>
    /// 电机角度必须一起导出：只看陀螺仪分不出「平台真的没转回来」和「陀螺仪积分少算了」，
    /// 两者在同一段时间戳上的最终值一对比就能定性。
    /// </para>
    /// <para>
    /// 只在 UI 线程做 <c>ToArray()</c> 快照（200000 点量级的毫秒级操作），
    /// 写盘放到后台线程，避免长文件写入时卡住 100ms 刷新。
    /// </para>
    /// </summary>
    private async Task SaveSegmentAsync()
    {
        DataSample[] data = _dataHistory.ToArray();
        RateSample[] rate = _rateHistory.ToArray();
        RawSample[] raw = _rawHistory.ToArray();
        MotorPositionSample[] motor = _motorHistory.ToArray();
        if (data.Length == 0 && rate.Length == 0 && raw.Length == 0 && motor.Length == 0)
        {
            ExportStatusText = "没有可保存的数据：请先连接并接收一段时间。";
            return;
        }

        SaveFileDialog dialog = new()
        {
            Title = "保存本段数据（角度 + 角速度 + 原始采样 + 电机角度）",
            Filter = "CSV 文件 (*.csv)|*.csv|所有文件 (*.*)|*.*",
            DefaultExt = ".csv",
            AddExtension = true,
            OverwritePrompt = true,
            FileName = "gyro_segment_" + DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture) + ".csv",
        };

        string? initialDirectory = ResolveExportDirectory();
        if (!string.IsNullOrEmpty(initialDirectory))
        {
            dialog.InitialDirectory = initialDirectory;
        }

        Window? owner = Application.Current?.MainWindow;
        bool? confirmed = owner is null ? dialog.ShowDialog() : dialog.ShowDialog(owner);
        if (confirmed != true)
        {
            ExportStatusText = "已取消保存（数据保持不动）";
            return;
        }

        string path = dialog.FileName;
        string? directory = Path.GetDirectoryName(path);
        if (!string.IsNullOrEmpty(directory) && Directory.Exists(directory))
        {
            _lastExportDirectory = directory;
        }

        ExportStatusText = "正在保存…";

        CsvExportSummary summary;
        try
        {
            summary = await Task.Run(() => MeasurementCsv.WriteFile(path, data, rate, motor, raw)).ConfigureAwait(true);
        }
        catch (Exception ex)
        {
            // 状态文字说清失败位置；异常继续抛给 AsyncRelayCommand -> OnCommandError 写日志。
            ExportStatusText = $"保存失败：{ex.Message}（请检查 {path}）";
            throw;
        }

        string range = summary.AngleRows + summary.RateRows + summary.RawRows + summary.MotorRows == 0
            ? string.Empty
            : string.Format(
                CultureInfo.InvariantCulture,
                "（{0:F3}–{1:F3} s）",
                summary.FirstTimeSeconds,
                summary.LastTimeSeconds);

        ExportStatusText = string.Format(
            CultureInfo.InvariantCulture,
            "已保存：角度 {0} 行 + 角速度 {1} 行 + 原始采样 {2} 行 + 电机 {3} 行{4} → {5}",
            summary.AngleRows,
            summary.RateRows,
            summary.RawRows,
            summary.MotorRows,
            range,
            path);

        AppendLogLine(
            $"已导出本段数据：角度 {summary.AngleRows} 行、角速度 {summary.RateRows} 行、原始采样 {summary.RawRows} 行、电机 {summary.MotorRows} 行{range} → {path}");
    }

    /// <summary>
    /// 解析「另存为」起始目录：本次会话上次选的目录 → <c>D:\GyroDatamotor</c> → 「文档」。
    /// <para>只做存在性判断，不自动创建目录，避免在意外位置留下空文件夹。</para>
    /// </summary>
    private string? ResolveExportDirectory()
    {
        if (!string.IsNullOrEmpty(_lastExportDirectory) && Directory.Exists(_lastExportDirectory))
        {
            return _lastExportDirectory;
        }

        if (Directory.Exists(PreferredExportDirectory))
        {
            return PreferredExportDirectory;
        }

        string documents = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
        return Directory.Exists(documents) ? documents : null;
    }

    /// <summary>
    /// 刷新「可保存范围」文字，并在「有/无数据」状态翻转时通知按钮可用性。
    /// <para>每次刷新只读首尾两条（O(1)），不遍历历史。</para>
    /// </summary>
    private void RefreshExportState()
    {
        int angleCount = _dataHistory.Count;
        int rateCount = _rateHistory.Count;
        int rawCount = _rawHistory.Count;
        int motorCount = _motorHistory.Count;

        bool hasData = angleCount > 0 || rateCount > 0 || rawCount > 0 || motorCount > 0;
        if (hasData != _exportHasData)
        {
            _exportHasData = hasData;
            SaveSegmentCommand.RaiseCanExecuteChanged();
        }

        if (!hasData)
        {
            ExportRangeText = "可保存：暂无数据（连接并接收后即可保存）";
            return;
        }

        double first = double.NaN;
        double last = double.NaN;
        if (angleCount > 0)
        {
            first = _dataHistory[0].TimeSeconds;
            last = _dataHistory[^1].TimeSeconds;
        }

        if (rateCount > 0)
        {
            double rateFirst = _rateHistory[0].TimeSeconds;
            double rateLast = _rateHistory[^1].TimeSeconds;
            first = double.IsNaN(first) ? rateFirst : Math.Min(first, rateFirst);
            last = double.IsNaN(last) ? rateLast : Math.Max(last, rateLast);
        }

        if (rawCount > 0)
        {
            double rawFirst = _rawHistory[0].TimeSeconds;
            double rawLast = _rawHistory[^1].TimeSeconds;
            first = double.IsNaN(first) ? rawFirst : Math.Min(first, rawFirst);
            last = double.IsNaN(last) ? rawLast : Math.Max(last, rawLast);
        }

        if (motorCount > 0)
        {
            double motorFirst = _motorHistory[0].TimeSeconds;
            double motorLast = _motorHistory[^1].TimeSeconds;
            first = double.IsNaN(first) ? motorFirst : Math.Min(first, motorFirst);
            last = double.IsNaN(last) ? motorLast : Math.Max(last, motorLast);
        }

        ExportRangeText = string.Format(
            CultureInfo.InvariantCulture,
            "可保存：角度 {0} 点、角速度 {1} 点、原始采样 {2} 点、电机 {3} 点（{4:F1}–{5:F1} s；每路上限 {6} 点，超出时丢弃最早的点）",
            angleCount,
            rateCount,
            rawCount,
            motorCount,
            first,
            last,
            MaxHistorySamples);
    }

    private void ApplyDefaultView()
    {
        _hasAppliedWindow = false;

        foreach (LiveChart chart in GyroCharts)
        {
            chart.ResetYRange();
        }

        AutoFollow = true;
        UpdateChartWindows();
        FlushCharts();
    }

    private void FlushCharts()
    {
        foreach (LiveChart chart in GyroCharts)
        {
            chart.Flush();
        }
    }

    public void Dispose()
    {
        if (_disposed)
        {
            return;
        }

        _disposed = true;
        _timer.Stop();
        TearDownSession();
    }

    // -------------------------------------------------------------- 内部

    /// <summary>建立连接（真实串口或模拟数据源）。失败时把原因显示到界面，不抛异常。</summary>
    public async Task ConnectAsync()
    {
        if (_session is not null || _isConnecting)
        {
            return;
        }

        _isConnecting = true;
        ConnectCommand.RaiseCanExecuteChanged();
        try
        {
            ICanConnection connection;
            if (_injectedConnection is not null)
            {
                connection = _injectedConnection;
            }
            else if (UseSimulation)
            {
                connection = new SimulationConnection();
            }
            else
            {
                if (string.IsNullOrWhiteSpace(SelectedPort))
                {
                    SetConnectionStatus(false, "未选择串口", "请先刷新并选择串口，或勾选“模拟模式”。");
                    return;
                }

                connection = new SlcanConnection(new SlcanConnectionOptions
                {
                    PortName = SelectedPort,
                    SerialBaudRate = SelectedBaudRate,
                    CanBitRateKbps = SelectedCanBitRateKbps,
                });
            }

            // 断线续接：只要用户没点“清零”，下一次连接就接着上一段画，不留空档、不清旧图线。
            bool resume = HasHistory;
            MotorResumeState? resumeState = resume
                ? new MotorResumeState(_latestTimeSeconds, _motorGeneration, _motorP0Raw)
                : null;

            SlcanSession session = new(connection, _injectedMotorDecoder, resumeState);

            // 电机设置必须在 ConnectAsync 之前生效：轮询任务连接后立即开始。
            session.MotorDeviceId = _motorDeviceId;
            session.MotorWheelDiameterMm = _motorWheelDiameterMm;
            session.MotorPollIntervalMs = _motorPollIntervalMs;
            session.MotorPollingEnabled = _motorPollingEnabled;

            // 先挂到字段上：即使 ConnectAsync 抛异常，TearDownSession 也能把串口释放掉。
            _connection = connection;
            _session = session;

            await session.ConnectAsync();

            _displayResetAt = null;
            _motorResetAt = null;
            if (resume)
            {
                // 只清实时数值，历史曲线与 P0 保留；接续处的断点由会话层置 BreakBefore。
                AppendLogLine(
                    $"断线续接：时间轴从 {_latestTimeSeconds:0.###} s 继续，旧图线与电机 P0" +
                    $"（{(_motorP0Raw is { } p0 ? p0.ToString(CultureInfo.InvariantCulture) : "未建立")}）保留，接续处断开。");
                ClearMotorLiveValues("等待位置反馈");
            }
            else
            {
                ResetMotorChartAndValues();
            }

            IsConnected = true;
            SetConnectionStatus(true, "已连接", connection.Description);
            RestoreAutoFollow("连接建立");
            AppendLogLine($"已连接：{connection.Description}");
            AppendLogLine(_motorPollingEnabled
                ? $"电机位置轮询已启动：设备 {_motorDeviceId:00}，周期 {_motorPollIntervalMs} ms，轮径 {_motorWheelDiameterMm:0.###} mm。"
                : "电机位置轮询未启用（可在电机卡片中勾选）。");
        }
        catch (Exception ex)
        {
            AppendLogLine($"连接失败：{ex.Message}");
            SetConnectionStatus(false, "连接失败", ex.Message);
            TearDownSession();
        }
        finally
        {
            _isConnecting = false;
            ConnectCommand.RaiseCanExecuteChanged();
            DisconnectCommand.RaiseCanExecuteChanged();
            ResetDisplayCommand.RaiseCanExecuteChanged();
            RefreshCommandStates();
        }
    }

    /// <summary>断开连接并释放串口。</summary>
    public async Task DisconnectAsync()
    {
        SlcanSession? session = _session;
        if (session is not null)
        {
            try
            {
                await session.DisconnectAsync();
            }
            catch (Exception ex)
            {
                AppendLogLine($"断开时出现异常：{ex.Message}");
            }
        }

        TearDownSession();
        SetConnectionStatus(false, "已断开", "串口与后台接收已关闭。");
        AppendLogLine("已断开连接，串口已释放。");
        RefreshCommandStates();
    }

    /// <summary>发送板端角度清零命令（0x322 = 02）。</summary>
    public async Task SendZeroAngleAsync()
    {
        if (_motion.IsRunning)
        {
            AppendLogLine("往复运动期间不允许清零，以保留返回残差。");
            return;
        }
        SlcanSession? session = _session;
        if (session is null)
        {
            return;
        }

        try
        {
            await session.SendZeroAngleAsync();
        }
        catch (Exception ex)
        {
            AppendLogLine($"发送角度清零命令失败：{ex.Message}");
        }
    }

    /// <summary>发送重新零偏校准命令（0x322 = 01）。</summary>
    public async Task SendRecalibrateAsync()
    {
        if (_motion.IsRunning)
        {
            AppendLogLine("往复运动期间不允许重新校准，请在本批结束并静止后操作。");
            return;
        }
        SlcanSession? session = _session;
        if (session is null)
        {
            return;
        }

        try
        {
            await session.SendRecalibrateAsync();
            CalibrationStateText = "已发送，等待板端状态变化...";
        }
        catch (Exception ex)
        {
            AppendLogLine($"发送重新校准命令失败：{ex.Message}");
        }
    }

    private void TearDownSession()
    {
        SlcanSession? session = _session;
        ICanConnection? connection = _connection;
        _session = null;
        _connection = null;

        CancelMotorCycle();

        if (session is not null)
        {
            try
            {
                session.Dispose();
            }
            catch (Exception ex)
            {
                AppendLogLine($"释放会话时出现异常：{ex.Message}");
            }
        }
        else
        {
            try
            {
                connection?.Dispose();
            }
            catch (Exception ex)
            {
                AppendLogLine($"释放连接时出现异常：{ex.Message}");
            }
        }

        IsConnected = false;
        ClearMotorLiveValues("未连接");
        RefreshCommandStates();
    }

    private void RefreshCommandStates()
    {
        ConnectCommand.RaiseCanExecuteChanged();
        DisconnectCommand.RaiseCanExecuteChanged();
        ResetDisplayCommand.RaiseCanExecuteChanged();
        ZeroAngleCommand.RaiseCanExecuteChanged();
        RecalibrateCommand.RaiseCanExecuteChanged();
        MotorZeroCommand.RaiseCanExecuteChanged();
        RunMotorCycleCommand.RaiseCanExecuteChanged();
        CancelMotorCycleCommand.RaiseCanExecuteChanged();
        RaiseMotorInitCommandStatesChanged();
    }

    private void OnRefreshTick()
    {
        SlcanSession? session = _session;
        if (session is null)
        {
            return;
        }

        SessionSnapshot snapshot = session.TakeSnapshot();

        // 连接已经失效（设备拔出、串口错误、对端断开）：清理会话，允许重新连接。
        if (snapshot.LinkState is LinkState.Faulted or LinkState.Disconnected && _connection is { IsConnected: false })
        {
            SetConnectionStatus(false, snapshot.LinkState == LinkState.Faulted ? "连接错误" : "已断开", snapshot.LinkMessage);
            TearDownSession();
            return;
        }

        AppendToCharts(snapshot);
        UpdateMotor(snapshot);
        UpdateValues(snapshot);
        UpdateCounters(snapshot);
        AppendLogLines(snapshot.LogLines);
        TickMotorMotion(snapshot.Motor);
    }

    private void AppendToCharts(SessionSnapshot snapshot)
    {
        bool hasNewPoint = false;

        foreach (DataSample sample in snapshot.Data)
        {
            if (_motion.IsRunning &&
                ((sample.Status & (GyroStatus.AngleValid | GyroStatus.CalibrationDone)) !=
                 (GyroStatus.AngleValid | GyroStatus.CalibrationDone) ||
                 (sample.Status & GyroStatus.DegradedCalibration) != 0))
                _cycleGyroQualityBad = true;
            _dataHistory.Add(sample);

            // 角度图始终折返显示 [-180°,180°)：纵轴固定 ±180°，累计角度看左侧数值面板。
            AngleChart.Append(sample.TimeSeconds, WrapTo180(sample.AngleDeg), sample.BreakBefore);

            // 温度无效时不上屏：曲线自然断开，而不是画一个假的 0℃。
            if ((sample.Status & GyroStatus.TemperatureValid) != 0)
            {
                TemperatureChart.Append(sample.TimeSeconds, sample.TemperatureC);
            }

            _latestTimeSeconds = Math.Max(_latestTimeSeconds, sample.TimeSeconds);
            hasNewPoint = true;
        }

        foreach (RateSample sample in snapshot.Rate)
        {
            _rateHistory.Add(sample);
            RateChart.Append(sample.TimeSeconds, sample.RateDps);
            _latestTimeSeconds = Math.Max(_latestTimeSeconds, sample.TimeSeconds);
            hasNewPoint = true;
        }

        // 原始采样只进历史（供 CSV 导出），不上曲线：它是未标定的 16 bit 码，
        // 和 °/s 曲线混在一张图上量纲不一致。
        foreach (RawSample sample in snapshot.Raw)
        {
            _rawHistory.Add(sample);
            _latestTimeSeconds = Math.Max(_latestTimeSeconds, sample.TimeSeconds);
            hasNewPoint = true;
        }

        TrimHistory(_dataHistory, static sample => sample.TimeSeconds);
        TrimHistory(_rateHistory, static sample => sample.TimeSeconds);
        TrimHistory(_rawHistory, static sample => sample.TimeSeconds);

        if (hasNewPoint)
        {
            RefreshExportState();

            // 先算好坐标轴窗口再刷新：否则这一帧渲染的还是旧范围，要等下一个周期才生效。
            UpdateChartWindows();
            FlushCharts();
        }
    }

    private void UpdateValues(SessionSnapshot snapshot)
    {
        TimeSpan now = MonotonicClock.Now;

        bool dataFresh = snapshot.LastDataArrival is { } dataArrival && now - dataArrival <= FrameTimeout;
        bool rateFresh = snapshot.LastRateArrival is { } rateArrival && now - rateArrival <= FrameTimeout;

        // 重置之后必须等新一帧到达才刷新数值，避免把重置前残留的旧值显示成当前值。
        bool dataAfterReset = IsArrivalAfterReset(snapshot.LastDataArrival);
        bool rateAfterReset = IsArrivalAfterReset(snapshot.LastRateArrival);

        bool angleUsable = dataFresh && dataAfterReset && (snapshot.LastData.Status & GyroStatus.AngleValid) != 0;
        _lastGyroArrival = snapshot.LastDataArrival;
        _lastGyroAngle = snapshot.LastData.AngleDeg;
        _lastGyroDegraded = (snapshot.LastData.Status & GyroStatus.DegradedCalibration) != 0;
        if (_motion.IsRunning && !dataFresh)
            _cycleGyroQualityBad = true;
        _lastGyroReady = angleUsable &&
            (snapshot.LastData.Status & (GyroStatus.CalibrationDone | GyroStatus.StillLocked)) ==
            (GyroStatus.CalibrationDone | GyroStatus.StillLocked);
        bool temperatureUsable = dataFresh && dataAfterReset && (snapshot.LastData.Status & GyroStatus.TemperatureValid) != 0;
        bool rateUsable = rateFresh && rateAfterReset && snapshot.HasRate;

        AngleWrappedText = angleUsable ? WrapTo180(snapshot.LastData.AngleDeg).ToString(AngleFormat) : "--";
        AngleCumulativeText = angleUsable ? snapshot.LastData.AngleDeg.ToString(AngleFormat) : "--";
        TemperatureText = temperatureUsable ? snapshot.LastData.TemperatureC.ToString("F2") : "--";
        RateText = rateUsable ? snapshot.LastRateDps.ToString(RateFormat) : "--";
        SampleIntervalText = dataFresh && dataAfterReset ? $"{snapshot.LastData.IntervalMs} ms" : "--";

        AngleValidText = angleUsable
            ? (snapshot.LastData.Status & GyroStatus.DegradedCalibration) != 0
                ? "有效（校准降级）"
                : "有效"
            : dataFresh && dataAfterReset
                ? "板端标记无效"
                : "无有效数据";

        StillLockText = angleUsable
            ? (snapshot.LastData.Status & GyroStatus.ContinuousIntegration) != 0
                ? (snapshot.LastData.Status & GyroStatus.StillLocked) != 0
                    ? "判定静止（连续积分）" : "运动中（连续积分）"
                : (snapshot.LastData.Status & GyroStatus.StillLocked) != 0 ? "已锁定（静止）" : "运动中"
            : "--";

        CalibrationStateText = DescribeCalibration(snapshot);

        ZeroAngleStateText = snapshot.ZeroAngle switch
        {
            ZeroAnglePhase.Sent => "命令已发送，等待回传数据确认",
            ZeroAnglePhase.Verified => "已由回传数据确认归零",
            ZeroAnglePhase.NoChange => "命令已发送，但未观察到角度归零",
            _ => "未执行",
        };

        TimeoutText = $"0x321 {(dataFresh ? "正常" : "超时")} / 0x323 {(rateFresh ? "正常" : "超时")}";

        UpdateMotorFreshnessText(snapshot.Motor);
    }

    private void UpdateMotorFreshnessText(MotorSnapshot motor)
    {
        if (!motor.DecoderConfirmed)
        {
            return;
        }

        TimeoutText += motor.IsFresh ? " / 电机正常" : " / 电机超时";
    }

    private void UpdateMotor(SessionSnapshot snapshot)
    {
        MotorSnapshot motor = snapshot.Motor;

        AppendMotorRawLogLines(motor.RawLogLines);

        if (motor.Generation != _motorGeneration)
        {
            _motorGeneration = motor.Generation;
            _motorHistory.Clear();

            // 只清电机那一条曲线；陀螺仪角度曲线继续保留并沿用同一时间轴。
            AngleChart.ClearSecondary();
        }

        bool afterReset = IsMotorArrivalAfterReset(motor.LastPositionArrival);
        bool hasLivePosition = motor.DecoderConfirmed && motor.HasPosition && motor.IsFresh && afterReset;
        bool hasAnyPosition = motor.DecoderConfirmed && motor.HasPosition && afterReset;

        // 状态机与按钮的可用性都依赖这一份“最新是否新鲜”的判断。
        if (_lastMotorLive != hasLivePosition)
        {
            _lastMotorLive = hasLivePosition;
            RunMotorCycleCommand.RaiseCanExecuteChanged();
        }

        if (hasLivePosition)
        {
            _lastMotorRaw = motor.RawPosition;
        }

        if (hasLivePosition)
        {
            _motorP0Raw = motor.ReferenceRawPosition;
        }

        bool hasNewPoint = false;
        foreach (MotorPositionSample sample in motor.Samples)
        {
            if (!motor.DecoderConfirmed)
            {
                // 未确认布局时会话层不会产生样本；这里再兜一层，避免以后改错把猜测画上屏。
                continue;
            }

            if (!IsMotorArrivalAfterReset(sample.Arrival))
            {
                // 重置调用与界面时间戳之间仍有极小窗口；这条属于重置前到达的旧数据。
                continue;
            }

            _motorHistory.Add(sample);

            // 与陀螺仪曲线共用同一时间基准（ChartTimeSeconds），并按同样的 ±180° 折返，
            // 两条曲线才能画在同一根纵轴上（720° 会在图上绕 2 圈，真实累计角度看数值面板）。
            AngleChart.AppendSecondary(
                sample.ChartTimeSeconds,
                WrapTo180(sample.RelativeAngleDeg),
                sample.BreakBefore);
            _latestTimeSeconds = Math.Max(_latestTimeSeconds, sample.ChartTimeSeconds);
            hasNewPoint = true;
        }

        TrimHistory(_motorHistory, static sample => sample.ChartTimeSeconds);

        if (hasNewPoint)
        {
            AngleChart.Flush();
            UpdateChartWindows();
        }

        if (!motor.DecoderConfirmed)
        {
            MotorProtocolStatusText = motor.AllRepliesReceived > 0
                ? $"待确认协议：已收到 {motor.AllRepliesReceived} 个 0x0409 原始回复，但布局未完全确认，不能判定为位置。"
                : "待确认协议：0x0409 回复布局未完全确认，位置数值不可用。";
            MotorMissingFieldsText = motor.MissingReplyFields.Count == 0
                ? string.Empty
                : "缺失字段：" + string.Join("；", motor.MissingReplyFields);
            MotorRawPositionText = "待确认协议";
            MotorDisplacementText = "--";
            MotorAngleText = "--";
            MotorP0Text = "--";
            MotorValidText = "待确认协议";
            MotorLastFeedbackText = "--";
            MotorRemainingDisplacementText = "--";
        }
        else
        {
            MotorProtocolStatusText = "位置解码已启用（待实机验证）：" + motor.DecoderDescription;
            MotorMissingFieldsText = MotorStillUnverifiedText;
            MotorRawPositionText = hasLivePosition
                ? motor.RawPosition.ToString(CultureInfo.InvariantCulture)
                : "--";
            MotorDisplacementText = hasLivePosition
                ? motor.DisplacementMm.ToString("F1", CultureInfo.InvariantCulture)
                : "--";
            MotorAngleText = hasLivePosition
                ? motor.RelativeAngleDeg.ToString("F" + MotorAngleDecimalPlaces, CultureInfo.InvariantCulture)
                : "--";
            MotorP0Text = motor.HasReference
                ? motor.ReferenceRawPosition.ToString(CultureInfo.InvariantCulture)
                : "--";
            MotorValidText = hasLivePosition
                ? "有效"
                : motor.HasPosition
                    ? afterReset ? "超时 / 已失效" : "等待新反馈"
                    : "无有效数据";
            MotorLastFeedbackText = hasLivePosition && motor.LastPositionArrival is { } lastAt
                ? FormatWallClock(lastAt)
                : "--";
            MotorRemainingDisplacementText = DescribeRemainingDisplacement(motor);
        }

        MotorLastRawReplyText = motor.LastReplyArrival is { } rawAt
            ? $"{FormatWallClock(rawAt)}  DLC={motor.LastReplyDlc}  {motor.LastReplyHex ?? "-"}"
            : "--";
        MotorErrorText = motor.LastErrorCode is { } errorCode &&
                         motor.LastErrorArrival is { } errorAt &&
                         IsMotorArrivalAfterReset(errorAt)
            ? $"0x{errorCode:X4} {MotorProtocol.DescribeErrorCode(errorCode)}  {FormatWallClock(errorAt)}"
            : "--";
        MotorQueryStateText = DescribeMotorQuery(motor);
        MotorCountersText =
            $"查询 {motor.QueriesSent} / 位置回复 {motor.PositionReplies} / 0x0409 {motor.AllRepliesReceived} / " +
            $"超时 {motor.Timeouts} / 未确认 {motor.UnconfirmedReplies} / 拒绝 {motor.RejectedReplies} / 错误 {motor.ErrorFrames} / " +
            $"运动命令 {motor.TargetCommandsSent} / ACK {motor.ParameterAcks}";
        MotorAckText = motor.LastAckArrival is { } ackAt
            ? $"{FormatWallClock(ackAt)}  " +
              (motor.LastAckEchoId is { } echoId
                  ? $"可能回显 0x{echoId:X4}（待实机确认）"
                  : "回显命令未知（协议表 DLC 自相矛盾）")
            : "--";

        if (_motorCanZero != hasLivePosition)
        {
            _motorCanZero = hasLivePosition;
            MotorZeroCommand.RaiseCanExecuteChanged();
        }

        AngleChart.SetSubtitle(motor.DecoderConfirmed
            ? (hasAnyPosition ? string.Empty : "电机角度：尚未收到有效的 0x0409 位置反馈")
            : "电机角度：0x0409 布局未确认，不绘制电机曲线");
    }

    private void AppendMotorRawLogLines(IReadOnlyList<string> lines)
    {
        foreach (string line in lines)
        {
            MotorRawLogLines.Add(line);
            while (MotorRawLogLines.Count > MaxMotorRawLogLines)
            {
                MotorRawLogLines.RemoveAt(0);
            }
        }
    }

    private static string DescribeMotorQuery(MotorSnapshot motor)
    {
        if (!motor.DecoderConfirmed)
        {
            return motor.AllRepliesReceived > 0
                ? $"待确认协议：已收到 {motor.AllRepliesReceived} 个 0x0409，无法确认是否为位置"
                : "待确认协议：轮询中，等待 0x0409 原始回复";
        }

        return motor.RequestState switch
        {
            MotorRequestState.WaitingForReply => "等待位置应答",
            MotorRequestState.Received => motor.IsFresh ? "已收到位置反馈" : "位置反馈已超时",
            MotorRequestState.TimedOut => "位置应答超时",
            _ => motor.HasPosition ? "空闲（保留最后有效值）" : "空闲",
        };
    }

    private string DescribeRemainingDisplacement(MotorSnapshot motor)
    {
        if (!motor.HasRemainingDisplacement ||
            !IsMotorArrivalAfterReset(motor.RemainingDisplacementArrival) ||
            motor.RemainingDisplacementArrival is not { } arrival ||
            MonotonicClock.Now - arrival > FrameTimeout)
        {
            return "--";
        }

        return $"{motor.RemainingDisplacementMm:F1} mm（主通道，非当前位置）";
    }

    /// <summary>把单调时钟时刻换算成墙钟时间，仅用于显示“最新反馈时间”。</summary>
    private static string FormatWallClock(TimeSpan monotonicArrival)
    {
        DateTimeOffset wallClock = DateTimeOffset.Now - (MonotonicClock.Now - monotonicArrival);
        return wallClock.ToString("HH:mm:ss.fff", CultureInfo.InvariantCulture);
    }

    private bool IsMotorArrivalAfterReset(TimeSpan? arrival)
    {
        if (_motorResetAt is not { } resetAt)
        {
            return true;
        }

        return arrival is { } value && value >= resetAt;
    }

    private void ResetMotorChartAndValues()
    {
        _motorHistory.Clear();
        _motorP0Raw = null;
        AngleChart.ClearSecondary();
        _motorResetAt = MonotonicClock.Now;
        ClearMotorLiveValues(_session is null ? "未连接" : "等待位置反馈");
        MotorProtocolStatusText = _session is null
            ? "位置解码已启用（待实机验证）：等待连接。"
            : "位置解码已启用（待实机验证）：轮询已启动，等待 0x0409 位置回复。";
        MotorMissingFieldsText = MotorStillUnverifiedText;
    }

    private void ClearMotorLiveValues(string state)
    {
        _lastMotorLive = false;
        MotorRawPositionText = "--";
        MotorDisplacementText = "--";
        MotorAngleText = "--";
        MotorP0Text = "--";
        MotorValidText = "--";
        MotorQueryStateText = state;
        MotorLastFeedbackText = "--";
        MotorLastRawReplyText = "--";
        MotorRemainingDisplacementText = "--";
        MotorErrorText = "--";
        MotorAckText = "--";
        MotorCountersText = "查询 0 / 0x0409 0 / 超时 0";
        if (_motorCanZero)
        {
            _motorCanZero = false;
            MotorZeroCommand.RaiseCanExecuteChanged();
        }
    }

    /// <summary>轮径变化后按原始位置重算电机曲线；状态与 P0 不变，时间轴仍是共用的那一条。</summary>
    private void RebuildMotorChart()
    {
        if (_motorP0Raw is not { } p0 || _motorHistory.Count == 0)
        {
            AngleChart.ClearSecondary();
            return;
        }

        List<(double TimeSeconds, double Value, bool BreakBefore)> points = new(_motorHistory.Count);
        foreach (MotorPositionSample sample in _motorHistory)
        {
            double value = WrapTo180(
                MotorProtocol.PositionDeltaLsbToDegrees(sample.RawPosition - p0, _motorWheelDiameterMm));
            points.Add((sample.ChartTimeSeconds, value, sample.BreakBefore));
        }

        AngleChart.RebuildSecondary(points);
        UpdateChartWindows();
        AngleChart.Flush();
    }

    private void UpdateMotorResolutionText()
    {
        double degreesPerLsb = MotorProtocol.DegreesPerPositionLsb(_motorWheelDiameterMm);
        MotorResolutionText =
            $"轮径 {_motorWheelDiameterMm:0.###} mm 时，1 个位置单位（0.1 mm）≈ {degreesPerLsb:F5}°；" +
            "角度显示 5 位小数，但真实分辨率受位置单位限制，不能按 0.00001° 苛求。";
    }

    private static bool TryParseMotorNumber(string text, out double value)
    {
        return double.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out value) ||
               double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value);
    }

    private void ZeroMotorDisplay()
    {
        SlcanSession? session = _session;
        if (session is null)
        {
            return;
        }

        if (!session.ZeroMotorDisplay())
        {
            AppendLogLine("电机显示归零失败：当前没有有效的电机位置反馈（或协议未确认）。");
            return;
        }

        _motorHistory.Clear();
        _motorP0Raw = null;
        AngleChart.ClearSecondary();
        _motorResetAt = MonotonicClock.Now;

        // 不把旧角度显示成 0：等下一帧真实反馈再从 0 开始。
        MotorAngleText = "--";
        MotorValidText = "显示已归零，等待新反馈";
        _motorCanZero = false;
        MotorZeroCommand.RaiseCanExecuteChanged();
        AppendLogLine("电机显示已归零：只更新上位机 P0 与电机曲线起点，未向电机发送归零命令。");
    }

    private void CopyMotorRawLog()
    {
        try
        {
            Clipboard.SetText(string.Join(Environment.NewLine, MotorRawLogLines));
            AppendLogLine($"已复制 {MotorRawLogLines.Count} 条电机原始日志到剪贴板。");
        }
        catch (Exception ex)
        {
            AppendLogLine($"复制电机原始日志失败：{ex.Message}");
        }
    }

    // ------------------------------------------------------------ 电机初始化

    /// <summary>初始化类命令的可用条件：已连接、有会话，且没有正在进行的往复运动。</summary>
    private bool CanSendMotorInitCommand() => _session is not null && _isConnected && !_isMotorCycleRunning;

    private void RaiseMotorInitCommandStatesChanged()
    {
        InitMotorParametersCommand.RaiseCanExecuteChanged();
        WriteMotorWheelDiameterCommand.RaiseCanExecuteChanged();
        MotorEnableCommand.RaiseCanExecuteChanged();
        MotorReleaseCommand.RaiseCanExecuteChanged();
    }

    /// <summary>发送 <c>0x0316</c>：参数初始化 / 建立 PID 基准。无应答。</summary>
    private async Task SendMotorParameterInitAsync()
    {
        SlcanSession? session = _session;
        if (session is null)
        {
            return;
        }

        byte[] payload = MotorProtocol.BuildParameterInit(_motorDeviceId);
        await session.SendMotorParameterInitAsync().ConfigureAwait(true);
        MotorInitStatusText = $"0x0316 参数初始化已发送（设备 {_motorDeviceId:00}）";
        AppendLogLine(
            $"已发送 0x0316 参数初始化 / 建立 PID 基准：DLC={MotorProtocol.ParameterInitDlc}、" +
            $"数据 {Convert.ToHexString(payload)},设备 {_motorDeviceId:00}。" +
            "该帧无应答，发送成功只表示已交给适配器。");
    }

    /// <summary>发送 <c>0x0408 / 0x24</c>：写入轮径，连发 3 次。无应答。</summary>
    private async Task SendMotorWheelDiameterConfigAsync()
    {
        SlcanSession? session = _session;
        if (session is null)
        {
            return;
        }

        ushort raw;
        try
        {
            raw = MotorProtocol.MillimetersToWheelDiameterRaw(_motorWheelDiameterMm);
        }
        catch (ArgumentOutOfRangeException ex)
        {
            MotorInitStatusText = $"轮径无法写入：{ex.Message}";
            AppendLogLine($"未发送配置轮径：{ex.Message}");
            return;
        }

        byte[] payload = MotorProtocol.BuildWheelDiameterConfig(_motorDeviceId, _motorWheelDiameterMm);
        await session.SendMotorWheelDiameterConfigAsync(_motorWheelDiameterMm).ConfigureAwait(true);
        MotorInitStatusText =
            $"0x0408 配置轮径已发送 ×{MotorProtocol.WheelConfigRepeatCount}（设备 {_motorDeviceId:00}，" +
            $"{_motorWheelDiameterMm:0.##} mm = {raw} = 0x{raw:X4}）";
        AppendLogLine(
            $"已发送 0x0408 配置轮径 {MotorProtocol.WheelConfigRepeatCount} 次：DLC={MotorProtocol.WheelConfigDlc}、" +
            $"数据 {Convert.ToHexString(payload)}（{_motorWheelDiameterMm:0.##} mm，单位 0.01 mm），设备 {_motorDeviceId:00}。" +
            "该帧无应答，发送成功只表示已交给适配器。");
    }

    /// <summary>发送 <c>0x0413 = 0x5A</c>：上电使能（锁定）。</summary>
    private async Task SendMotorEnableAsync()
    {
        SlcanSession? session = _session;
        if (session is null)
        {
            return;
        }

        await session.SendMotorEnableAsync(true).ConfigureAwait(true);
        MotorInitStatusText = $"0x0413 = 0x5A 上电使能已发送（设备 {_motorDeviceId:00}）";
        AppendLogLine(
            $"已发送 0x0413 上电使能（0x5A），设备 {_motorDeviceId:00}：电机应保持锁定/有锁力。" +
            "该帧无应答，结果以电机实际动作与位置反馈为准。");
    }

    /// <summary>发送 <c>0x0413 = 0x4A</c>：下电释放。</summary>
    private async Task SendMotorReleaseAsync()
    {
        SlcanSession? session = _session;
        if (session is null)
        {
            return;
        }

        await session.SendMotorEnableAsync(false).ConfigureAwait(true);
        MotorInitStatusText = $"0x0413 = 0x4A 下电释放已发送（设备 {_motorDeviceId:00}）";
        AppendLogLine(
            $"已发送 0x0413 下电释放（0x4A），设备 {_motorDeviceId:00}：电机将失去锁力，可能自由滑动。" +
            "该帧无应答，结果以电机实际动作与位置反馈为准。");
    }

    // ------------------------------------------------------------ 电机往复运动

    private bool CanRunMotorCycle(string angleText)
        => _session is not null && _isConnected && !_isMotorCycleRunning;

    /// <summary>
    /// 以指定角度执行“正转 → 返回”往复运动：先发 0x0413 锁定 + 0x0415 速度 + 0x0414 加速度，
    /// 再逐段发 0x0416 相对目标，最后回到起点。每轮正转到位后先按“停顿时长”停留再返程。
    /// 整个过程由 <see cref="MotorMotionController"/> 驱动。
    /// </summary>
    private void RunMotorCycle(string angleText)
    {
        if (!_lastGyroReady || _lastGyroArrival is not { } gyroArrival ||
            MonotonicClock.Now - gyroArrival > FrameTimeout)
        {
            AppendLogLine($"暂不能开始往复：请等待陀螺仪完成约{GyroProtocol.CalibrationDurationSeconds:0}秒静止校准（60秒估计＋20秒独立验证）、角度有效且已判定静止。");
            return;
        }
        SlcanSession? session = _session;
        if (session is null)
        {
            AppendLogLine("尚未连接，无法执行电机运动。");
            return;
        }

        if (!int.TryParse(angleText, NumberStyles.Integer, CultureInfo.InvariantCulture, out int angle) || angle <= 0)
        {
            AppendLogLine($"目标角度“{angleText}”无效，必须是正整数（°）。");
            return;
        }

        if (!TryParseMotorNumber(_motorSpeedText, out double speed) || speed <= 0.0)
        {
            AppendLogLine($"速度“{_motorSpeedText}”无效，必须大于 0（mm/s）。");
            return;
        }

        if (!TryParseMotorNumber(_motorAccelerationText, out double acceleration) || acceleration <= 0.0)
        {
            AppendLogLine($"加速度“{_motorAccelerationText}”无效，必须大于 0（mm/s²）。");
            return;
        }

        if (!TryParseMotorNumber(_motorDwellSecondsText, out double dwellSeconds) ||
            dwellSeconds < 0.0 ||
            dwellSeconds > MotorMotionController.MaxDwellSeconds)
        {
            AppendLogLine(
                $"停顿时长“{_motorDwellSecondsText}”无效，必须是 0..{MotorMotionController.MaxDwellSeconds:0} s" +
                "（0 表示到位后不停顿）。");
            return;
        }

        if (!_motion.IsRunning)
        {
            _motion.Reset();
        }

        _motion.SpeedMmPerSecond = speed;
        _motion.AccelerationMmPerSecondSquared = acceleration;
        _motion.DwellSeconds = dwellSeconds;

        if (!_motion.Start(
                MonotonicClock.Now,
                angle,
                _motorCycleRounds,
                _motorWheelDiameterMm,
                _lastMotorLive,
                _lastMotorRaw,
                out string? error))
        {
            AppendLogLine($"无法开始往复运动：{error}");
            MotorMotionStatusText = $"未开始：{error}";
            return;
        }

        _motionWasRunning = true;
        _cycleGyroOrigin = _lastGyroAngle;
        _cycleGyroQualityBad = _lastGyroDegraded;
        _residualRound = _reportedResidualRound = 0;
        IsMotorCycleRunning = true;
        MotorMotionStatusText = _motion.StatusText;
        AppendLogLine(
            $"开始电机往复运动：目标 {angle}°，{_motorCycleRounds} 轮（正转→到位停顿→返回），每段 {_motion.SegmentDeltaLsb} LSB；" +
            $"速度 {speed:0.#} mm/s、加速度 {acceleration:0.#} mm/s²、到位停顿 {dwellSeconds:0.##} s。" +
            "将发送 0x0413/0x0415/0x0414 与 0x0416（相对目标），不发送归零命令。");
    }

    private void CancelMotorCycle()
    {
        if (!_motion.IsRunning)
        {
            return;
        }

        _motion.Cancel();
        MotorMotionStatusText = _motion.StatusText;
        FinishMotorCycle();
        AppendLogLine("已取消电机往复运动（已发出的指令不会撤回）。");
    }

    /// <summary>每个刷新周期推进一次状态机，并把产出的命令真正发到 CAN 上。</summary>
    private void TickMotorMotion(MotorSnapshot motor)
    {
        if (!_motion.IsRunning || _motionSendInFlight)
        {
            return;
        }

        bool fresh = motor.DecoderConfirmed &&
                     motor.HasPosition &&
                     motor.IsFresh &&
                     IsMotorArrivalAfterReset(motor.LastPositionArrival);

        // 使用未折返累计角度；整批只保存一次原点，不按轮清零。
        if (_motion.Phase == MotorMotionPhase.DwellAfterRound)
        {
            if (_residualRound != _motion.CompletedRounds)
            {
                _residualRound = _motion.CompletedRounds;
                _residualWaitStarted = MonotonicClock.Now;
            }
            if (_reportedResidualRound != _residualRound && _lastGyroReady &&
                MonotonicClock.Now - _residualWaitStarted >= TimeSpan.FromSeconds(1))
                ReportReturnResidual();
        }

        IReadOnlyList<MotorCommand> commands = _motion.Tick(
            MonotonicClock.Now,
            fresh,
            motor.RawPosition,
            motor.LastPositionArrival);

        MotorMotionStatusText = _motion.StatusText;

        if (_residualRound > _reportedResidualRound &&
            _motion.Phase != MotorMotionPhase.DwellAfterRound)
        {
            AppendLogLine($"第 {_residualRound} 轮返回残差未验收：停顿不足或未收到有效静止角度。");
            _reportedResidualRound = _residualRound;
        }

        if (commands.Count > 0)
        {
            _ = SendMotorCommandsAsync(commands);
        }

        if (!_motion.IsRunning)
        {
            FinishMotorCycle();
        }
    }

    private async Task SendMotorCommandsAsync(IReadOnlyList<MotorCommand> commands)
    {
        SlcanSession? session = _session;
        _motionSendInFlight = true;
        try
        {
            if (session is null)
            {
                _motion.Abort("连接已断开");
                return;
            }

            foreach (MotorCommand command in commands)
            {
                switch (command.Kind)
                {
                    case MotorCommandKind.Enable:
                        await session.SendMotorEnableAsync(command.Enable);
                        break;
                    case MotorCommandKind.SetSpeed:
                        await session.SendMotorSpeedAsync(command.RawValue);
                        break;
                    case MotorCommandKind.SetAcceleration:
                        await session.SendMotorAccelerationAsync(command.RawValue);
                        break;
                    case MotorCommandKind.TargetPosition:
                        await session.SendMotorTargetPositionAsync(command.RawPosition, command.TargetType);
                        break;
                    case MotorCommandKind.RecalibrateGyro:
                        await session.SendRecalibrateAsync();
                        CalibrationStateText = "已发送，等待板端状态变化...";
                        AppendLogLine("已发送板端重新零偏校准（0x322 = 01），保留累计角度；此批不用于往复验收。");
                        _cycleGyroQualityBad = true;
                        break;
                }
            }
        }
        catch (Exception ex)
        {
            AppendLogLine($"发送电机运动命令失败：{ex.Message}");
            _motion.Abort(ex.Message);
        }
        finally
        {
            _motionSendInFlight = false;
            MotorMotionStatusText = _motion.StatusText;
            if (!_motion.IsRunning)
            {
                FinishMotorCycle();
            }
        }
    }

    private void FinishMotorCycle()
    {
        if (!_motionWasRunning)
        {
            return;
        }

        _motionWasRunning = false;
        IsMotorCycleRunning = false;
        MotorMotionStatusText = _motion.StatusText;

        switch (_motion.State)
        {
            case MotorMotionState.Completed:
                AppendLogLine("电机往复运动完成：" + _motion.StatusText);
                break;
            case MotorMotionState.Failed:
                AppendLogLine("电机往复运动失败：" + (_motion.FailureReason ?? "原因未知"));
                break;
            case MotorMotionState.Cancelled:
                AppendLogLine("电机往复运动已取消。");
                break;
        }
    }

    private void ReportReturnResidual()
    {
        double residual = _lastGyroAngle - _cycleGyroOrigin;
        string result = _cycleGyroQualityBad || !double.IsFinite(residual)
            ? "质量不足，不能判定合格"
            : Math.Abs(residual) < 0.5 ? "陀螺仪返回读数小于0.5°" : "陀螺仪返回读数超限";
        AppendLogLine($"第 {_residualRound} 轮返回残差：{residual:F5}°，{result}；" +
            "相对整批初始累计角度，实际机械回零仍需独立核验。");
        _reportedResidualRound = _residualRound;
    }

    private static string DescribeCalibration(SessionSnapshot snapshot)
    {
        switch (snapshot.Calibration)
        {
            case CalibrationPhase.Requested:
                return "命令已发送，等待板端响应";
            case CalibrationPhase.InProgress:
                return "校准中，请保持静止";
            case CalibrationPhase.Completed:
                return (snapshot.LastData.Status & GyroStatus.DegradedCalibration) != 0
                    ? "本次校准完成（降级，不能判定精度合格）" : "本次校准完成";
            case CalibrationPhase.Failed:
                return "本次校准失败";
            case CalibrationPhase.NoResponse:
                return "命令已发送，但未观察到状态变化";
        }

        if ((snapshot.LastData.Status & GyroStatus.CalibrationFailed) != 0)
        {
            return "本轮校准失败（板端 2s 后自动重试）";
        }

        if ((snapshot.LastData.Status & GyroStatus.CalibrationDone) != 0)
        {
            return (snapshot.LastData.Status & GyroStatus.DegradedCalibration) != 0
                ? "补偿校准已完成（降级，不能判定精度合格）"
                : "补偿校准已完成";
        }

        return snapshot.HasData ? "板端尚未完成校准" : "--";
    }

    private void UpdateCounters(SessionSnapshot snapshot)
    {
        SessionCounters counters = snapshot.Counters;
        FrameCountText =
            $"0x321 {counters.DataFrames} 帧 / 0x323 {counters.RateFrames} 帧 / 0x324+0x325 {counters.RawFrames} 帧 / 0x0409 {snapshot.Motor.AllRepliesReceived} 帧";

        long errors = counters.BadDlcFrames + counters.MalformedLines + counters.UnsupportedFrames + counters.RawPairMismatches;
        ErrorCountText =
            $"{errors}（DLC {counters.BadDlcFrames} / 解析 {counters.MalformedLines} / 未知 {counters.UnsupportedFrames} / 原始帧未配对 {counters.RawPairMismatches}）";
        DroppedText = $"{counters.DroppedBeforeReset}（重置前到达被丢弃）";
    }

    private void AppendLogLines(IReadOnlyList<string> lines)
    {
        foreach (string line in lines)
        {
            AppendLogLine(line);
        }
    }

    private void AppendLogLine(string message)
    {
        LogLines.Add(message);
        while (LogLines.Count > MaxLogLines)
        {
            LogLines.RemoveAt(0);
        }
    }

    private void SetConnectionStatus(bool connected, string state, string message)
    {
        IsConnected = connected;
        ConnectionStateText = state;
        ConnectionMessageText = message;
        ConnectionBrush = connected
            ? Brushes.MediumSeaGreen
            : state.Contains("错误", StringComparison.Ordinal) || state.Contains("失败", StringComparison.Ordinal)
                ? Brushes.OrangeRed
                : Brushes.Gray;
    }

    private void OnCommandError(Exception ex) => AppendLogLine($"操作失败：{ex.Message}");

    private bool IsArrivalAfterReset(TimeSpan? arrival)
    {
        if (_displayResetAt is not { } resetAt)
        {
            return true;
        }

        return arrival is { } value && value >= resetAt;
    }

    /// <summary>
    /// 维护曲线横轴。
    /// <para>
    /// 规则很简单，也很确定：
    /// 1) 跟随时，每 tick 把三张曲线一起滑到“最近 N 秒”（角度图里的电机曲线共用同一对坐标轴，自动同步）；
    /// 2) 暂停时不碰任何范围——用户鼠标缩放/拖动后不会再被抢回去；
    ///    跟随的开关由 <see cref="PauseAutoFollow"/>（鼠标操作）和 <see cref="RestoreAutoFollow"/> 明确控制。
    /// </para>
    /// 纵轴不在这里动，只在恢复默认视角和清零时 <see cref="LiveChart.ResetYRange"/>。
    /// </summary>
    private void UpdateChartWindows()
    {
        if (!AutoFollow)
        {
            return;
        }

        // 不依赖鼠标事件的暂停判定：OxyPlot 的缩放/平移写的是 ViewMinimum/ViewMaximum，
        // 实际显示范围一变就说明用户动过视角（滚轮、右键拖动、触控板手势都覆盖到）。
        if (HasUserAdjustedView())
        {
            PauseAutoFollow("鼠标缩放/拖动");
            return;
        }

        ApplyFollowWindow();
    }

    /// <summary>实际显示的范围是否已经偏离我们上一次写进去的窗口。</summary>
    private bool HasUserAdjustedView()
    {
        if (!_hasAppliedWindow)
        {
            return false;
        }

        double tolerance = Math.Max(1e-6, (_appliedWindowEnd - _appliedWindowStart) * 0.005);

        foreach (LiveChart chart in GyroCharts)
        {
            if (Math.Abs(chart.ActualRangeMinimum - _appliedWindowStart) > tolerance ||
                Math.Abs(chart.ActualRangeMaximum - _appliedWindowEnd) > tolerance)
            {
                return true;
            }
        }

        return false;
    }

    private void ApplyFollowWindow()
    {
        double windowSeconds = SelectedTimeWindowSeconds;
        double start = Math.Max(0.0, _latestTimeSeconds - windowSeconds);
        double end = _latestTimeSeconds + (windowSeconds * 0.02);

        foreach (LiveChart chart in GyroCharts)
        {
            chart.SetRange(start, end);
        }

        _appliedWindowStart = start;
        _appliedWindowEnd = end;
        _hasAppliedWindow = true;
    }

    private static void TrimHistory<T>(List<T> history, Func<T, double> timeSelector)
    {
        if (history.Count <= MaxHistorySamples)
        {
            return;
        }

        history.RemoveRange(0, history.Count - MaxHistorySamples);
    }

    /// <summary>把任意角度折算到 [-180°, 180°)。</summary>
    private static double WrapTo180(double angleDeg)
    {
        double wrapped = (angleDeg + 180.0) % 360.0;
        if (wrapped < 0.0)
        {
            wrapped += 360.0;
        }

        return wrapped - 180.0;
    }
}
