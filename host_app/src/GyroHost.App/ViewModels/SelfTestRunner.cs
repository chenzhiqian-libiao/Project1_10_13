using System.Text;

namespace GyroHost.App.ViewModels;

/// <summary>
/// 无人值守自检（<c>GyroHost.exe --selftest</c>）。
/// <para>
/// 用模拟数据源驱动真实的视图模型与真实曲线对象，验证界面层确实能跑起来：
/// 连接、三路数值刷新、曲线落点、重置显示、板端清零、断开。
/// 结果写入文本文件并以退出码返回，便于命令行验证。
/// </para>
/// </summary>
internal sealed class SelfTestRunner
{
    private readonly MainViewModel _viewModel;
    private readonly List<string> _report = new();
    private bool _passed = true;

    private SelfTestRunner(MainViewModel viewModel) => _viewModel = viewModel;

    public static async Task<(bool Passed, string Report)> RunAsync(MainViewModel viewModel)
    {
        SelfTestRunner runner = new(viewModel);
        await runner.ExecuteAsync();
        return (runner._passed, string.Join(Environment.NewLine, runner._report));
    }

    private async Task ExecuteAsync()
    {
        try
        {
            _viewModel.UseSimulation = true;
            await _viewModel.ConnectAsync();
            await Task.Delay(1600);

            Check("连接建立（模拟模式）", _viewModel.IsConnected);
            Check("0x321 角度曲线已落点", _viewModel.AngleChart.PointCount > 5);
            Check("温度曲线已落点", _viewModel.TemperatureChart.PointCount > 0);
            Check("0x323 角速度曲线已落点", _viewModel.RateChart.PointCount > 5);
            Check("角度数值已刷新", _viewModel.AngleWrappedText != "--");
            Check("角度显示保留 5 位小数", HasDecimalPlaces(_viewModel.AngleWrappedText, 5));
            Check("累计角度显示保留 5 位小数", HasDecimalPlaces(_viewModel.AngleCumulativeText, 5));
            Check("角度曲线悬浮读数使用 5 位小数", _viewModel.AngleChart.TrackerFormat.Contains("F5", StringComparison.Ordinal));
            Check("温度数值已刷新（不是伪造的 0℃）", _viewModel.TemperatureText != "--");
            Check("角速度数值已刷新", _viewModel.RateText != "--");
            Check("角速度显示保留 5 位小数", HasDecimalPlaces(_viewModel.RateText, 5));
            Check("角速度曲线悬浮读数使用 5 位小数", _viewModel.RateChart.TrackerFormat.Contains("F5", StringComparison.Ordinal));
            Check("采样间隔已刷新", _viewModel.SampleIntervalText != "--");
            Check("帧计数在增长", _viewModel.FrameCountText.Contains("0x321 ", StringComparison.Ordinal));
            Check("无异常帧", _viewModel.ErrorCountText.StartsWith("0", StringComparison.Ordinal));
            Check("自动跟随已开启", _viewModel.AutoFollow);
            Check("角度图纵轴固定 ±180°",
                _viewModel.AngleChart.YRangeMinimum == -180.0 && _viewModel.AngleChart.YRangeMaximum == 180.0);
            Check("角度曲线全部折返到 [-180°,180°)",
                AllPointsWrapped(_viewModel.AngleChart.Series.Points));

            // ---- 数据保存：有数据才可点，且范围文字如实显示点数 ----
            Check("有数据时「保存本段数据」可用", _viewModel.SaveSegmentCommand.CanExecute(null));
            Check("可保存范围显示角度、角速度、原始采样与电机点数",
                _viewModel.ExportRangeText.Contains("角度 ", StringComparison.Ordinal) &&
                _viewModel.ExportRangeText.Contains("角速度 ", StringComparison.Ordinal) &&
                _viewModel.ExportRangeText.Contains("原始采样 ", StringComparison.Ordinal) &&
                _viewModel.ExportRangeText.Contains("电机 ", StringComparison.Ordinal) &&
                _viewModel.ExportRangeText.Contains("200000 点", StringComparison.Ordinal));
            Check("尚未保存时状态文字为初始值",
                _viewModel.ExportStatusText.Contains("尚未保存", StringComparison.Ordinal));

            // ---- 电机位置：解码已接入，但模拟通道没有 0x0409，不能伪造位置 ----
            Check("电机位置查询已发送（0x0408）",
                _viewModel.MotorRawLogLines.Any(line => line.Contains("TX 0x0408", StringComparison.Ordinal)));
            Check("电机位置解码已启用",
                !_viewModel.MotorProtocolStatusText.Contains("待确认协议", StringComparison.Ordinal));
            Check("尚无有效 0x0409 时电机角度显示 --",
                _viewModel.MotorAngleText == "--" && _viewModel.MotorRawPositionText == "--");
            Check("尚无有效 0x0409 时电机曲线保持空白", _viewModel.AngleChart.SecondaryPointCount == 0);
            Check("尚无有效 0x0409 时禁止显示归零", !_viewModel.MotorZeroCommand.CanExecute(null));

            int pointsBeforeReset = _viewModel.AngleChart.PointCount;

            // ---- 重置显示数据：接收过程中进行 ----
            _viewModel.ResetDisplay();
            Check("重置后角度曲线被清空", _viewModel.AngleChart.PointCount == 0);
            Check("重置后温/速曲线被清空", _viewModel.TemperatureChart.PointCount == 0 && _viewModel.RateChart.PointCount == 0);
            Check("重置后电机曲线仍为空", _viewModel.AngleChart.SecondaryPointCount == 0);
            Check("重置后数值等待下一帧",
                _viewModel.AngleWrappedText == "--" && _viewModel.TemperatureText == "--" && _viewModel.MotorAngleText == "--");

            await Task.Delay(1300);

            Check("重置后重新开始接收", _viewModel.AngleChart.PointCount > 3);
            Check("重置后相对时间从 0 重新计时", _viewModel.AngleChart.RangeMaximum < 5.0);
            Check("重置后数值恢复刷新", _viewModel.TemperatureText != "--" && _viewModel.RateText != "--");
            Check("重置未中断连接", _viewModel.IsConnected);
            Check("重置前排队的旧数据未重现", _viewModel.AngleChart.PointCount < pointsBeforeReset + 40);
            Check("重置后纵轴回到 ±180°",
                _viewModel.AngleChart.YRangeMinimum == -180.0 && _viewModel.AngleChart.YRangeMaximum == 180.0);
            Check("重置后自动跟随已恢复", _viewModel.AutoFollow);
            Check("重置后「保存本段数据」重新变为可用（已收到新数据）", _viewModel.SaveSegmentCommand.CanExecute(null));
            Check("重置后保存状态文字已更新",
                _viewModel.ExportStatusText.Contains("重置", StringComparison.Ordinal));

            // ---- 鼠标操作后暂停跟随 / 一键恢复跟随 ----
            _viewModel.PauseAutoFollow("自检：模拟鼠标缩放");
            await Task.Delay(400);
            double pausedMin = _viewModel.AngleChart.RangeMinimum;
            double pausedMax = _viewModel.AngleChart.RangeMaximum;
            await Task.Delay(500);
            Check("暂停跟随：自动跟随已关闭", !_viewModel.AutoFollow);
            Check("暂停跟随：横轴不再被刷新抢走",
                _viewModel.AngleChart.RangeMinimum == pausedMin && _viewModel.AngleChart.RangeMaximum == pausedMax);
            Check("暂停跟随：三张曲线横轴一致",
                _viewModel.TemperatureChart.RangeMaximum == _viewModel.AngleChart.RangeMaximum &&
                _viewModel.RateChart.RangeMaximum == _viewModel.AngleChart.RangeMaximum);

            _viewModel.RestoreAutoFollow("自检：恢复");
            await Task.Delay(500);
            Check("恢复跟随：自动跟随重新开启", _viewModel.AutoFollow);
            Check("恢复跟随：视角已追到最新数据",
                _viewModel.AngleChart.RangeMaximum > pausedMax);

            _viewModel.RestoreDefaultView();
            Check("恢复默认视角：纵轴回到 ±180°",
                _viewModel.AngleChart.YRangeMinimum == -180.0 && _viewModel.AngleChart.YRangeMaximum == 180.0);
            Check("恢复默认视角：仍保持跟随", _viewModel.AutoFollow);

            // ---- OxyPlot 真实缩放路径（写的是 ViewMinimum/ViewMaximum）----
            // 这是“按钮没反应”的根因回归测试：只设 Minimum/Maximum 会被 View* 覆盖。
            _viewModel.AngleChart.SimulateZoom(1000.0, 1010.0);
            _viewModel.AngleChart.SimulateYAxisZoom(10.0, 20.0);
            Check("模拟鼠标缩放：实际显示范围已改变",
                Near(_viewModel.AngleChart.ActualRangeMinimum, 1000.0) &&
                Near(_viewModel.AngleChart.ActualYRangeMaximum, 20.0));

            await Task.Delay(500);
            Check("鼠标缩放后：自动跟随被自动识别并暂停", !_viewModel.AutoFollow);

            _viewModel.RestoreAutoFollowCommand.Execute(null);
            await Task.Delay(200);
            Check("跟随按钮：横轴实际范围回到自动窗口（View 覆盖已清除）",
                Near(_viewModel.AngleChart.ActualRangeMinimum, _viewModel.AngleChart.RangeMinimum) &&
                Near(_viewModel.AngleChart.ActualRangeMaximum, _viewModel.AngleChart.RangeMaximum) &&
                _viewModel.AngleChart.ActualRangeMinimum < 1000.0);
            Check("跟随按钮：自动跟随保持开启", _viewModel.AutoFollow);

            _viewModel.AngleChart.SimulateYAxisZoom(10.0, 20.0);
            Check("模拟纵轴缩放：实际显示范围已改变",
                Near(_viewModel.AngleChart.ActualYRangeMaximum, 20.0));

            _viewModel.RestoreDefaultViewCommand.Execute(null);
            await Task.Delay(200);
            Check("恢复默认视角按钮：实际纵轴确实回到 ±180°",
                Near(_viewModel.AngleChart.ActualYRangeMinimum, -180.0) &&
                Near(_viewModel.AngleChart.ActualYRangeMaximum, 180.0));

            // ---- 板端角度清零 ----
            await _viewModel.SendZeroAngleAsync();
            await Task.Delay(900);
            Check("清零命令已发送", _viewModel.ZeroAngleStateText != "未执行");
            Check("清零结果以回传数据为准", _viewModel.ZeroAngleStateText == "已由回传数据确认归零");

            // ---- 重新零偏校准 ----
            await _viewModel.SendRecalibrateAsync();
            await Task.Delay(1200);
            Check("重新校准已观察到板端状态变化",
                _viewModel.CalibrationStateText.Contains("校准中", StringComparison.Ordinal)
                || _viewModel.CalibrationStateText.Contains("完成", StringComparison.Ordinal));

            // ---- 断开 ----
            await _viewModel.DisconnectAsync();
            Check("断开后状态为未连接", !_viewModel.IsConnected);
            Check("断开后曲线仍在（保留已采集数据）", _viewModel.AngleChart.PointCount > 0);
            Check("断开后电机数值显示 --",
                _viewModel.MotorAngleText == "--" && _viewModel.MotorRawPositionText == "--");
            Check("断开后电机显示归零不可用", !_viewModel.MotorZeroCommand.CanExecute(null));

            // ---- 断线续接：不清零就重连，时间轴接着上一段，接续处断开 ----
            List<OxyPlot.DataPoint> beforeReconnect = _viewModel.AngleChart.Series.Points.ToList();
            int pointsBeforeReconnect = beforeReconnect.Count;
            double lastX = beforeReconnect.Where(static p => !double.IsNaN(p.Y)).Max(static p => p.X);

            await _viewModel.ConnectAsync();
            await Task.Delay(1200);

            Check("重连成功", _viewModel.IsConnected);
            Check("续接：旧图线未被清空", _viewModel.AngleChart.PointCount > pointsBeforeReconnect);
            double firstNewX = _viewModel.AngleChart.Series.Points
                .Skip(pointsBeforeReconnect)
                .FirstOrDefault(static p => !double.IsNaN(p.Y), new OxyPlot.DataPoint(double.NaN, double.NaN)).X;
            Check("续接：横轴紧接上一段最后一点（不留空档）",
                !double.IsNaN(firstNewX) && firstNewX >= lastX - 1e-6 && firstNewX <= lastX + 1.0);
            Check("续接：接续处曲线断开",
                _viewModel.AngleChart.Series.Points.Skip(pointsBeforeReconnect).Any(static p => double.IsNaN(p.Y)));
            Check("续接：仍按 ±180° 显示",
                _viewModel.AngleChart.YRangeMinimum == -180.0 && AllPointsWrapped(_viewModel.AngleChart.Series.Points));

            _report.Add($"日志尾部：{(_viewModel.LogLines.Count > 0 ? _viewModel.LogLines[^1] : "（空）")}");
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

    private void Check(string name, bool condition)
    {
        _report.Add(condition ? $"[通过] {name}" : $"[失败] {name}");
        if (!condition)
        {
            _passed = false;
        }
    }

    /// <summary>检查文本的小数位数（兼容以逗号作小数点的区域设置）。</summary>
    private static bool HasDecimalPlaces(string text, int places)
    {
        int separator = text.LastIndexOfAny(new[] { '.', ',' });
        return separator >= 0 && text.Length - separator - 1 == places && separator > 0;
    }

    /// <summary>浮点近似相等（坐标轴范围比较用，避免二进制误差误判）。</summary>
    private static bool Near(double actual, double expected) => Math.Abs(actual - expected) < 1e-6;

    /// <summary>所有有效点都必须落在 [-180°, 180°)：角度图统一折返显示。</summary>
    private static bool AllPointsWrapped(IEnumerable<OxyPlot.DataPoint> points)
    {
        bool any = false;
        foreach (OxyPlot.DataPoint point in points)
        {
            if (double.IsNaN(point.Y))
            {
                continue;
            }

            any = true;
            if (point.Y < -180.0 || point.Y >= 180.0)
            {
                return false;
            }
        }

        return any;
    }

    /// <summary>把报告打成字符串（供写文件）。</summary>
    public override string ToString()
    {
        StringBuilder builder = new();
        foreach (string line in _report)
        {
            builder.AppendLine(line);
        }

        return builder.ToString();
    }
}
