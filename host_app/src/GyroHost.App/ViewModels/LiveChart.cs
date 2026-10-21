using OxyPlot;
using OxyPlot.Axes;
using OxyPlot.Legends;
using OxyPlot.Series;

namespace GyroHost.App.ViewModels;

/// <summary>
/// 一条实时曲线：封装 OxyPlot 的模型、坐标轴与分批追加逻辑。
/// <para>
/// 关键约束：
/// 1) 点数有上限，长时间运行内存不会无限增长；
/// 2) 横轴为相对时间，同一组曲线用同一个 <see cref="SetWindow"/> 保持时间轴同步；
///    <see cref="SecondarySeries"/> 与主曲线共用同一对坐标轴，因此两条曲线时间基准一致；
/// 3) 用 NaN 断开曲线，跨越 ±180° 或板端清零时不会画出虚假的连线。
/// </para>
/// </summary>
public sealed class LiveChart
{
    private readonly double _jumpThreshold;
    private readonly LinearAxis _xAxis;
    private readonly LinearAxis _yAxis;
    private readonly double _defaultYMinimum;
    private readonly double _defaultYMaximum;
    private bool _hasPreviousValue;
    private double _previousValue;
    private bool _hasPreviousSecondaryValue;
    private double _previousSecondaryValue;

    /// <param name="title">图表标题。</param>
    /// <param name="yTitle">纵轴标题（带单位）。</param>
    /// <param name="color">主曲线颜色。</param>
    /// <param name="maxPoints">
    /// 最大保留点数，超出后丢弃最旧的点。
    /// 默认 60000：板端 10 ms 一帧时约 10 分钟回滚窗口；再大下去 OxyPlot 每条
    /// 曲线的渲染与命中测试成本会明显上升，而导出用的长历史在视图模型里另有缓存。
    /// </param>
    /// <param name="jumpThreshold">相邻两点变化超过该值时自动断开曲线；NaN 表示不启用。</param>
    /// <param name="trackerFormat">主曲线悬浮读数的格式串；null = 用 OxyPlot 默认值。</param>
    /// <param name="secondaryTitle">副曲线图例名；null = 不启用副曲线。</param>
    /// <param name="secondaryColor">副曲线颜色。</param>
    /// <param name="secondaryTrackerFormat">副曲线悬浮读数的格式串；null = 用 OxyPlot 默认值。</param>
    /// <param name="seriesTitle">主曲线图例名；null = 与图表标题相同。</param>
    /// <param name="yMinimum">纵轴默认下限；NaN = 自动范围。</param>
    /// <param name="yMaximum">纵轴默认上限；NaN = 自动范围。</param>
    public LiveChart(
        string title,
        string yTitle,
        OxyColor color,
        int maxPoints = 60000,
        double jumpThreshold = double.NaN,
        string? trackerFormat = null,
        string? secondaryTitle = null,
        OxyColor? secondaryColor = null,
        string? secondaryTrackerFormat = null,
        string? seriesTitle = null,
        double yMinimum = double.NaN,
        double yMaximum = double.NaN)
    {
        MaxPoints = maxPoints;
        _jumpThreshold = jumpThreshold;
        _defaultYMinimum = yMinimum;
        _defaultYMaximum = yMaximum;

        _xAxis = new LinearAxis
        {
            Position = AxisPosition.Bottom,
            Title = "相对时间 (s)",
            MajorGridlineStyle = LineStyle.Dot,
            MinorGridlineStyle = LineStyle.None,
            MajorGridlineColor = OxyColor.Parse("#D8D8E0"),
            TextColor = OxyColor.Parse("#40404A"),
            TitleColor = OxyColor.Parse("#40404A"),
            TicklineColor = OxyColor.Parse("#90909C"),
            AxislineColor = OxyColor.Parse("#90909C"),
        };

        LinearAxis yAxis = new()
        {
            Position = AxisPosition.Left,
            Title = yTitle,
            MajorGridlineStyle = LineStyle.Dot,
            MinorGridlineStyle = LineStyle.None,
            MajorGridlineColor = OxyColor.Parse("#D8D8E0"),
            TextColor = OxyColor.Parse("#40404A"),
            TitleColor = OxyColor.Parse("#40404A"),
            TicklineColor = OxyColor.Parse("#90909C"),
            AxislineColor = OxyColor.Parse("#90909C"),
        };

        if (!double.IsNaN(_defaultYMinimum) && !double.IsNaN(_defaultYMaximum))
        {
            yAxis.Minimum = _defaultYMinimum;
            yAxis.Maximum = _defaultYMaximum;
        }

        _yAxis = yAxis;

        Series = new LineSeries
        {
            Color = color,
            StrokeThickness = 1.6,
            Title = seriesTitle ?? title,
        };

        if (!string.IsNullOrEmpty(trackerFormat))
        {
            // 默认格式串不带精度说明符，悬浮读数会打印 double 的完整精度
            // （例如 31.560000000000002），这里显式指定，与左侧数值显示保持一致。
            Series.TrackerFormatString = trackerFormat;
        }

        Model = new PlotModel
        {
            Title = title,
            TitleColor = OxyColor.Parse("#1F1F28"),
            TextColor = OxyColor.Parse("#40404A"),
            Background = OxyColors.White,
            PlotAreaBorderColor = OxyColor.Parse("#D0D0D8"),
            TitleFontSize = 13,
        };

        Model.Axes.Add(_xAxis);
        Model.Axes.Add(yAxis);
        Model.Series.Add(Series);

        if (!string.IsNullOrEmpty(secondaryTitle) && secondaryColor is { } secondary)
        {
            SecondarySeries = new LineSeries
            {
                Color = secondary,
                StrokeThickness = 1.6,
                Title = secondaryTitle,
            };

            if (!string.IsNullOrEmpty(secondaryTrackerFormat))
            {
                SecondarySeries.TrackerFormatString = secondaryTrackerFormat;
            }

            Model.Series.Add(SecondarySeries);
            Model.Legends.Add(new Legend
            {
                LegendPlacement = LegendPlacement.Inside,
                LegendPosition = LegendPosition.TopRight,
                LegendOrientation = LegendOrientation.Vertical,
                LegendBackground = OxyColor.Parse("#F7FFFFFF"),
                LegendBorder = OxyColor.Parse("#D0D0D8"),
                LegendTextColor = OxyColor.Parse("#40404A"),
                LegendFontSize = 11,
            });
        }
    }

    public PlotModel Model { get; }

    public LineSeries Series { get; }

    /// <summary>与主曲线共用坐标轴的第二条曲线；未启用时为 null。</summary>
    public LineSeries? SecondarySeries { get; }

    public int MaxPoints { get; }

    /// <summary>已绘制的点数。</summary>
    public int PointCount => Series.Points.Count;

    /// <summary>副曲线已绘制的点数。</summary>
    public int SecondaryPointCount => SecondarySeries?.Points.Count ?? 0;

    /// <summary>鼠标悬浮读数的格式串。</summary>
    public string TrackerFormat => Series.TrackerFormatString;

    /// <summary>副曲线鼠标悬浮读数的格式串。</summary>
    public string SecondaryTrackerFormat => SecondarySeries?.TrackerFormatString ?? string.Empty;

    /// <summary>当前横轴下限（NaN = 自动范围）。</summary>
    public double RangeMinimum => _xAxis.Minimum;

    /// <summary>当前横轴上限（NaN = 自动范围）。</summary>
    public double RangeMaximum => _xAxis.Maximum;

    /// <summary>当前纵轴下限（NaN = 自动范围）。</summary>
    public double YRangeMinimum => _yAxis.Minimum;

    /// <summary>当前纵轴上限（NaN = 自动范围）。</summary>
    public double YRangeMaximum => _yAxis.Maximum;

    /// <summary>
    /// 屏幕上实际显示时的横轴下限（含鼠标缩放/平移的效果）。
    /// <para>
    /// 与 <see cref="RangeMinimum"/> 的区别很关键：OxyPlot 的鼠标操作写的是
    /// <c>ViewMinimum</c>/<c>ViewMaximum</c>，而 <c>ActualMinimum</c> 的优先级是
    /// View* &gt; Minimum/Maximum &gt; 数据范围，所以“设了 Minimum 却没生效”只能从实际值上看出来。
    /// </para>
    /// </summary>
    public double ActualRangeMinimum => _xAxis.ActualMinimum;

    /// <summary>实际显示的横轴上限（含鼠标缩放/平移的效果）。</summary>
    public double ActualRangeMaximum => _xAxis.ActualMaximum;

    /// <summary>实际显示的纵轴下限（含鼠标缩放/平移的效果）。</summary>
    public double ActualYRangeMinimum => _yAxis.ActualMinimum;

    /// <summary>实际显示的纵轴上限（含鼠标缩放/平移的效果）。</summary>
    public double ActualYRangeMaximum => _yAxis.ActualMaximum;

    /// <summary>
    /// 恢复纵轴默认范围。角度图为 ±180°；其它图为自动范围（NaN）。
    /// <para>只在初始化、恢复默认视角、清零时调用，不每 tick 覆盖，用户纵向拉伸才不会被抢回。</para>
    /// </summary>
    public void ResetYRange()
    {
        _yAxis.Minimum = _defaultYMinimum;
        _yAxis.Maximum = _defaultYMaximum;

        // 必须先写 Minimum/Maximum 再 Reset：Reset 只清 View* 覆盖，
        // 不清 View* 的话鼠标纵向缩放会一直压住这里的默认值。
        _yAxis.Reset();
    }

    /// <summary>
    /// 追加一个点。<paramref name="breakBefore"/> = true 或变化超过阈值时，先插入 NaN 断开曲线。
    /// </summary>
    public void Append(double timeSeconds, double value, bool breakBefore = false)
    {
        bool needsBreak = breakBefore
            || (_hasPreviousValue && !double.IsNaN(_jumpThreshold) && Math.Abs(value - _previousValue) > _jumpThreshold);

        if (needsBreak)
        {
            InsertGap(Series, timeSeconds);
        }

        Series.Points.Add(new DataPoint(timeSeconds, value));
        TrimToLimit(Series);

        _previousValue = value;
        _hasPreviousValue = true;
    }

    /// <summary>向副曲线追加一个点（与主曲线共用时间轴和角度轴）。</summary>
    public void AppendSecondary(double timeSeconds, double value, bool breakBefore = false)
    {
        if (SecondarySeries is not { } series)
        {
            return;
        }

        // 与主曲线同样的跳变判据：±180° 折返时自动断开，不会画出横穿整幅图的假线。
        bool needsBreak = breakBefore
            || (_hasPreviousSecondaryValue &&
                !double.IsNaN(_jumpThreshold) &&
                Math.Abs(value - _previousSecondaryValue) > _jumpThreshold);

        if (needsBreak)
        {
            InsertGap(series, timeSeconds);
        }

        series.Points.Add(new DataPoint(timeSeconds, value));
        TrimToLimit(series);

        _previousSecondaryValue = value;
        _hasPreviousSecondaryValue = true;
    }

    /// <summary>插入一个 NaN 断点（该处曲线断开）。</summary>
    public void InsertGap(double timeSeconds) => InsertGap(Series, timeSeconds);

    /// <summary>副曲线插入一个 NaN 断点。</summary>
    public void InsertGapSecondary(double timeSeconds)
    {
        if (SecondarySeries is { } series)
        {
            InsertGap(series, timeSeconds);
        }
    }

    /// <summary>用给定的点集重建主曲线（切换角度显示模式时使用）。</summary>
    public void Rebuild(IReadOnlyList<(double TimeSeconds, double Value, bool BreakBefore)> points)
    {
        Series.Points.Clear();
        _hasPreviousValue = false;

        foreach ((double timeSeconds, double value, bool breakBefore) in points)
        {
            Append(timeSeconds, value, breakBefore);
        }

        Flush();
    }

    /// <summary>用给定的点集重建副曲线。</summary>
    public void RebuildSecondary(IReadOnlyList<(double TimeSeconds, double Value, bool BreakBefore)> points)
    {
        if (SecondarySeries is not { } series)
        {
            return;
        }

        series.Points.Clear();
        _hasPreviousSecondaryValue = false;
        foreach ((double timeSeconds, double value, bool breakBefore) in points)
        {
            AppendSecondary(timeSeconds, value, breakBefore);
        }

        Flush();
    }

    /// <summary>清空主曲线与副曲线。</summary>
    public void Clear()
    {
        Series.Points.Clear();
        SecondarySeries?.Points.Clear();
        _hasPreviousValue = false;
        _hasPreviousSecondaryValue = false;
        Flush();
    }

    /// <summary>只清空副曲线（例如“电机显示归零”后重新起画，不影响陀螺仪曲线）。</summary>
    public void ClearSecondary()
    {
        if (SecondarySeries is not { } series)
        {
            return;
        }

        series.Points.Clear();
        _hasPreviousSecondaryValue = false;
        Flush();
    }

    /// <summary>
    /// 设置横轴显示范围（同一组曲线用同一范围保持时间轴同步）。
    /// <para>
    /// 这里必须调用 <see cref="Axis.Reset()"/>：OxyPlot 的鼠标缩放/拖动把范围写进了
    /// <c>ViewMinimum</c>/<c>ViewMaximum</c>，而 <c>ActualMinimum</c> 优先取 View*，
    /// 不清掉的话“跟随最新数据/恢复默认视角”设了 Minimum/Maximum 也不会有任何变化。
    /// </para>
    /// </summary>
    public void SetRange(double minimum, double maximum)
    {
        _xAxis.Minimum = minimum;
        _xAxis.Maximum = maximum;
        _xAxis.Reset();
    }

    /// <summary>自检用：模拟鼠标滚轮/拖动缩放横向（OxyPlot 会写 ViewMinimum/ViewMaximum）。</summary>
    internal void SimulateZoom(double xMinimum, double xMaximum) => _xAxis.Zoom(xMinimum, xMaximum);

    /// <summary>自检用：模拟鼠标纵向缩放。</summary>
    internal void SimulateYAxisZoom(double yMinimum, double yMaximum) => _yAxis.Zoom(yMinimum, yMaximum);

    /// <summary>设置图表副标题（用于“协议未确认/尚无数据”等状态说明）。</summary>
    public void SetSubtitle(string? subtitle)
    {
        string value = subtitle ?? string.Empty;
        if (string.Equals(Model.Subtitle, value, StringComparison.Ordinal))
        {
            return;
        }

        Model.Subtitle = value;
        Model.SubtitleColor = OxyColor.Parse("#8A5A00");
        Model.InvalidatePlot(false);
    }

    /// <summary>把最新数据刷到界面上（每个刷新周期调用一次，而不是每个点调用）。</summary>
    public void Flush() => Model.InvalidatePlot(true);

    private void InsertGap(LineSeries series, double timeSeconds)
    {
        if (series.Points.Count == 0)
        {
            return;
        }

        series.Points.Add(new DataPoint(timeSeconds, double.NaN));
        TrimToLimit(series);
    }

    private void TrimToLimit(LineSeries series)
    {
        int overflow = series.Points.Count - MaxPoints;
        if (overflow > 0)
        {
            series.Points.RemoveRange(0, overflow);
        }
    }
}
