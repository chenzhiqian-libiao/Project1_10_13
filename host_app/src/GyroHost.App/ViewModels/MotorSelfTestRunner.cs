using System.Globalization;

namespace GyroHost.App.ViewModels;

/// <summary>
/// 电机显示链路自检（<c>GyroHost.exe --selftest-motor</c>）。
/// <para>
/// 用 <see cref="ScriptedMotorConnection"/> 的**合成报文**驱动真实会话、默认解码器、
/// 仓库、视图模型和 OxyPlot，验证位置显示、轮径换算、P0、显示归零、超时失效和恢复断点。
/// 电机角度与陀螺仪角度共用同一张角度图和同一条时间轴。
/// </para>
/// <para>
/// ⚠️ 合成报文不等于真实 0x0409 协议验证；它只验证代码路径，不能替代实机报文验证。
/// </para>
/// </summary>
internal sealed class MotorSelfTestRunner
{
    private readonly MainViewModel _viewModel;
    private readonly ScriptedMotorConnection _connection;
    private readonly List<string> _report = new();
    private bool _passed = true;

    private MotorSelfTestRunner(MainViewModel viewModel, ScriptedMotorConnection connection)
    {
        _viewModel = viewModel;
        _connection = connection;
    }

    public static async Task<(bool Passed, string Report)> RunAsync(
        MainViewModel viewModel,
        ScriptedMotorConnection connection)
    {
        MotorSelfTestRunner runner = new(viewModel, connection);
        await runner.ExecuteAsync();
        return (runner._passed, string.Join(Environment.NewLine, runner._report));
    }

    private async Task ExecuteAsync()
    {
        try
        {
            await _viewModel.ConnectAsync();
            Check("连接建立（脚本化电机通道）", _viewModel.IsConnected);

            await Task.Delay(700);

            Check("位置查询已发送（0x0408）",
                _viewModel.MotorRawLogLines.Any(line => line.Contains("TX 0x0408", StringComparison.Ordinal)));
            Check("0x0409 回复已接收",
                _viewModel.MotorRawLogLines.Any(line => line.Contains("RX 0x0409", StringComparison.Ordinal)));
            Check("默认解码器已启用（不再显示待确认）",
                !_viewModel.MotorProtocolStatusText.Contains("待确认协议", StringComparison.Ordinal));
            Check("位置原始值已刷新", _viewModel.MotorRawPositionText != "--");
            Check("位移已刷新", _viewModel.MotorDisplacementText != "--");
            Check("相对角度已刷新", _viewModel.MotorAngleText != "--");
            Check("相对角度保留 5 位小数", HasDecimalPlaces(_viewModel.MotorAngleText, 5));
            Check("角度图有两条曲线", _viewModel.AngleChart.Model.Series.Count == 2);
            Check("角度图带图例", _viewModel.AngleChart.Model.Legends.Count == 1);
            Check("电机曲线已落点", _viewModel.AngleChart.SecondaryPointCount > 2);
            Check("两路曲线共用同一对坐标轴（时间轴 + 角度轴）",
                _viewModel.AngleChart.Model.Axes.Count == 2);
            Check("电机曲线与陀螺仪共用时间基准（从全局时间 0 起画）",
                _viewModel.AngleChart.SecondarySeries!.Points[0].X is >= 0.0 and < 1.0);
            Check("电机角度曲线悬浮读数保留 5 位小数",
                _viewModel.AngleChart.SecondaryTrackerFormat.Contains("F5", StringComparison.Ordinal));
            Check("数据有效", _viewModel.MotorValidText == "有效");
            Check("位置应答计数在增长", ParseCount(_viewModel.MotorCountersText, "位置回复 ") > 0);
            Check("显示归零可用", _viewModel.MotorZeroCommand.CanExecute(null));
            Check("P0 为第一次有效位置", _viewModel.MotorP0Text == "1000");
            Check("位移 = P × 0.1 mm", Math.Abs(
                ParseDouble(_viewModel.MotorDisplacementText) - (ParseInt(_viewModel.MotorRawPositionText) * 0.1)) < 1e-9);
            CheckAngleMatches(100.0, "轮径 100 mm 角度换算");
            Check("分辨率提示 0.11459°/单位",
                _viewModel.MotorResolutionText.Contains("0.11459", StringComparison.Ordinal));
            Check("原始回复包含 DLC=8",
                _viewModel.MotorLastRawReplyText.Contains("DLC=8", StringComparison.Ordinal));

            // ---- 修改轮径：只影响上位机换算，曲线保留并重算 ----
            _viewModel.MotorWheelDiameterText = "200";
            await Task.Delay(300);
            Check("修改轮径后立即重算分辨率",
                _viewModel.MotorResolutionText.Contains("0.05730", StringComparison.Ordinal));
            CheckAngleMatches(200.0, "轮径 200 mm 角度换算");
            Check("修改轮径保留曲线", _viewModel.AngleChart.SecondaryPointCount > 0);

            // ---- 电机显示归零：只改 P0/电机曲线，不发 CAN ----
            string p0Before = _viewModel.MotorP0Text;
            _viewModel.MotorZeroCommand.Execute(null);
            await Task.Delay(350);
            Check("显示归零更新 P0", _viewModel.MotorP0Text != p0Before);
            Check("显示归零后电机曲线重新落点",
                _viewModel.AngleChart.SecondaryPointCount > 0 && _viewModel.AngleChart.SecondaryPointCount <= 6);
            Check("显示归零不清除陀螺仪角度曲线（共用同一张图）",
                _viewModel.AngleChart.Model.Series.Count == 2);
            CheckAngleMatches(200.0, "显示归零后的角度换算");
            Check("显示归零未发送 CAN 命令",
                _connection.GetSentLines().All(line => line.StartsWith("t408", StringComparison.OrdinalIgnoreCase)));

            // ---- 停止应答：数据失效、显示 --、按钮禁用 ----
            _connection.StopResponding = true;
            await Task.Delay(1200);
            Check("超时后位置数值显示 --",
                _viewModel.MotorAngleText == "--" && _viewModel.MotorRawPositionText == "--");
            Check("超时状态明确",
                _viewModel.MotorValidText.Contains("超时", StringComparison.Ordinal) ||
                _viewModel.MotorValidText.Contains("失效", StringComparison.Ordinal));
            Check("超时后禁止显示归零", !_viewModel.MotorZeroCommand.CanExecute(null));

            // ---- 恢复应答：数值恢复，曲线在恢复点断开 ----
            _connection.StopResponding = false;
            await Task.Delay(600);
            Check("恢复后数值重新有效",
                _viewModel.MotorValidText == "有效" && _viewModel.MotorAngleText != "--");
            Check("恢复点插入断点",
                _viewModel.AngleChart.SecondarySeries!.Points.Any(point => double.IsNaN(point.Y)));
            Check("电机曲线也折返到 [-180°,180°)",
                _viewModel.AngleChart.SecondarySeries!.Points
                    .Where(static p => !double.IsNaN(p.Y))
                    .All(static p => p.Y >= -180.0 && p.Y < 180.0));

            // ---- 往复运动：只验证命令确实按协议帧发出、状态机可取消 ----
            Check("位置有效时往复按钮可用", _viewModel.RunMotorCycleCommand.CanExecute("90"));
            // 前面把轮径改成 200 mm 验证换算，这里先改回 100 mm，才能按固定 LSB 核对报文。
            _viewModel.MotorWheelDiameterText = "100";
            await Task.Delay(200);

            // ---- 电机初始化（0x0316 / 0x0408 配轮径 / 0x0413）：只验证发出的 SLCAN 行 ----
            Check("轮径写入预览为 0x2710 × 3", _viewModel.MotorWheelConfigText.Contains("0x2710", StringComparison.Ordinal));

            int sentBeforeInit = _connection.GetSentLines().Count;
            _viewModel.InitMotorParametersCommand.Execute(null);
            await Task.Delay(120);
            _viewModel.WriteMotorWheelDiameterCommand.Execute(null);
            await Task.Delay(150);
            _viewModel.MotorEnableCommand.Execute(null);
            await Task.Delay(120);
            _viewModel.MotorReleaseCommand.Execute(null);
            await Task.Delay(120);

            string[] initLines = _connection.GetSentLines().Skip(sentBeforeInit).ToArray();
            Check("初始化发送 0x0316（固定前缀 + 设备号）",
                initLines.Contains("t3168B4BFCFAA005F0101", StringComparer.OrdinalIgnoreCase));
            Check("配置轮径连发 3 次 0x0408",
                initLines.Count(line => line.Equals("t408401241027", StringComparison.OrdinalIgnoreCase)) == 3,
                $"实际：{string.Join(" | ", initLines)}");
            Check("上电使能 0x0413 = 0x5A",
                initLines.Contains("t41355A00000001", StringComparer.OrdinalIgnoreCase));
            Check("下电释放 0x0413 = 0x4A",
                initLines.Contains("t41354A00000001", StringComparer.OrdinalIgnoreCase));
            Check("初始化原始日志已记录", _viewModel.MotorRawLogLines.Any(
                line => line.Contains("TX 0x0316", StringComparison.Ordinal)));
            Check("初始化状态有说明文字", _viewModel.MotorInitStatusText != "未执行");

            // ---- 停顿时长：默认 3 s（须覆盖固件 2 s 校准窗口），非法值必须拒绝启动 ----
            Check("默认停顿时长 3 s", _viewModel.MotorDwellSecondsText == "3");
            Check("摘要说明包含停顿时长", _viewModel.MotorCycleSummaryText.Contains("停顿", StringComparison.Ordinal));
            Check("摘要说明保留返回残差", _viewModel.MotorCycleSummaryText.Contains("保留返回残差", StringComparison.Ordinal));

            _viewModel.MotorDwellSecondsText = "-1";
            int sentBeforeInvalidDwell = CountMotionLines(_connection);
            _viewModel.RunMotorCycleCommand.Execute("90");
            await Task.Delay(200);
            Check("非法停顿时长拒绝启动",
                !_viewModel.IsMotorCycleRunning &&
                CountMotionLines(_connection) == sentBeforeInvalidDwell &&
                _viewModel.LogLines.Any(line => line.Contains("停顿时长", StringComparison.Ordinal)),
                $"实际：{_viewModel.MotorMotionStatusText}");
            _viewModel.MotorDwellSecondsText = "3";

            int sentBeforeMotion = CountMotionLines(_connection);
            _viewModel.MotorCycleRounds = 1;
            _viewModel.MotorSpeedText = "100";
            _viewModel.MotorAccelerationText = "500";
            _viewModel.RunMotorCycleCommand.Execute("90");
            await Task.Delay(800);

            string[] motionLines = _connection.GetSentLines()
                .Where(IsMotionLine)
                .Skip(sentBeforeMotion)
                .ToArray();
            Check("运动前发送 0x0413 锁定",
                motionLines.Any(line => line.StartsWith("t4135", StringComparison.OrdinalIgnoreCase)));
            Check("运动前发送 0x0415 速度（100 mm/s → 1000）",
                motionLines.Contains("t4155E803000001", StringComparer.OrdinalIgnoreCase));
            Check("运动前发送 0x0414 加速度（500 mm/s² → 5000）",
                motionLines.Contains("t41458813000001", StringComparer.OrdinalIgnoreCase));
            Check("发送 0x0416 相对目标（90° → 785 LSB）",
                motionLines.Contains("t4166110300000100", StringComparer.OrdinalIgnoreCase),
                $"实际运动帧：{string.Join(" | ", motionLines)}；状态：{_viewModel.MotorMotionStatusText}");
            Check("运动状态有说明文字", _viewModel.MotorMotionStatusText != "未运行");
            Check("运动中禁止重复启动",
                !_viewModel.RunMotorCycleCommand.CanExecute("90") &&
                _viewModel.CancelMotorCycleCommand.CanExecute(null));
            Check("运动中禁用初始化命令",
                !_viewModel.InitMotorParametersCommand.CanExecute(null) &&
                !_viewModel.WriteMotorWheelDiameterCommand.CanExecute(null) &&
                !_viewModel.MotorEnableCommand.CanExecute(null) &&
                !_viewModel.MotorReleaseCommand.CanExecute(null));

            _viewModel.CancelMotorCycleCommand.Execute(null);
            int motionAfterCancel = CountMotionLines(_connection);
            await Task.Delay(500);
            Check("取消后状态复位",
                !_viewModel.IsMotorCycleRunning && !_viewModel.CancelMotorCycleCommand.CanExecute(null));
            Check("取消后不再发送运动命令", CountMotionLines(_connection) == motionAfterCancel);
            Check("取消写进运行日志",
                _viewModel.LogLines.Any(line => line.Contains("已取消电机往复运动", StringComparison.Ordinal)));

            // ---- 断开：数值清空，曲线保留 ----
            await _viewModel.DisconnectAsync();
            Check("断开后电机数值显示 --", _viewModel.MotorAngleText == "--");
            Check("断开后电机曲线保留", _viewModel.AngleChart.SecondaryPointCount > 0);
        }
        catch (Exception ex)
        {
            _report.Add($"自检过程中出现异常：{ex}");
            _passed = false;
        }

        int passedCount = _report.Count(line => line.StartsWith("[通过]", StringComparison.Ordinal));
        int failedCount = _report.Count(line => line.StartsWith("[失败]", StringComparison.Ordinal));
        _report.Add($"合计：通过 {passedCount} 项，失败 {failedCount} 项");
    }

    private void Check(string name, bool condition, string? detail = null)
    {
        string suffix = condition || string.IsNullOrEmpty(detail) ? string.Empty : $"（{detail}）";
        _report.Add(condition ? $"[通过] {name}" : $"[失败] {name}{suffix}");
        if (!condition)
        {
            _passed = false;
        }
    }

    private void CheckAngleMatches(double wheelDiameterMm, string name)
    {
        int raw = ParseInt(_viewModel.MotorRawPositionText);
        int p0 = ParseInt(_viewModel.MotorP0Text);
        double expected = (raw - p0) * 0.1 * 360.0 / (Math.PI * wheelDiameterMm);
        double actual = ParseDouble(_viewModel.MotorAngleText);
        Check($"{name}（期望 {expected:F5}°，实际 {actual:F5}°）", Math.Abs(actual - expected) < 1e-4);
    }

    private static int ParseInt(string text) => int.Parse(text, CultureInfo.InvariantCulture);

    /// <summary>运动控制帧：0x0413 / 0x0414 / 0x0415 / 0x0416 / 0x041A。</summary>
    private static bool IsMotionLine(string line)
        => line.StartsWith("t41", StringComparison.OrdinalIgnoreCase);

    private static int CountMotionLines(ScriptedMotorConnection connection)
        => connection.GetSentLines().Count(IsMotionLine);

    /// <summary>从“位置回复 12 / …”这类计数文本里取出数字；取不到返回 -1。</summary>
    private static long ParseCount(string text, string key)
    {
        int start = text.IndexOf(key, StringComparison.Ordinal);
        if (start < 0)
        {
            return -1;
        }

        start += key.Length;
        int end = text.IndexOf(' ', start);
        ReadOnlySpan<char> number = end < 0 ? text.AsSpan(start) : text.AsSpan(start, end - start);
        return long.TryParse(number, NumberStyles.Integer, CultureInfo.InvariantCulture, out long value) ? value : -1;
    }

    private static double ParseDouble(string text) => double.Parse(text, CultureInfo.InvariantCulture);

    /// <summary>检查文本的小数位数（兼容以逗号作小数点的区域设置）。</summary>
    private static bool HasDecimalPlaces(string text, int places)
    {
        int separator = text.LastIndexOfAny(new[] { '.', ',' });
        return separator >= 0 && text.Length - separator - 1 == places && separator > 0;
    }
}
