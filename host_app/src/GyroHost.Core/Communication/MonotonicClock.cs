using System.Diagnostics;

namespace GyroHost.Core.Communication;

/// <summary>
/// 单调时钟。相对时间基准与"重置前旧数据"判定都必须用单调时间，
/// 否则系统时间被调整时曲线和重置边界都会出错。
/// </summary>
public static class MonotonicClock
{
    private static readonly Stopwatch Stopwatch = Stopwatch.StartNew();

    /// <summary>自进程启动以来的时间。</summary>
    public static TimeSpan Now => Stopwatch.Elapsed;
}
