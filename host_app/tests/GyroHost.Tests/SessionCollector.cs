using GyroHost.Core.Services;

namespace GyroHost.Tests;

/// <summary>
/// 模拟界面的取数方式：周期性调用 <see cref="SlcanSession.TakeSnapshot"/> 并累积采样点。
/// <para>
/// 这一点很关键 —— 会话层每次取数都会清空待绘制队列，测试里若直接把单次快照
/// 当成"全部数据"，就会漏点。用累加器才能真实反映界面侧的行为。
/// </para>
/// </summary>
internal sealed class SessionCollector
{
    public List<DataSample> Data { get; } = new();

    public List<RateSample> Rate { get; } = new();

    public List<RawSample> Raw { get; } = new();

    public SessionSnapshot Last { get; private set; } = null!;

    public SessionCollector(SlcanSession session)
    {
        Poll(session);
    }

    /// <summary>取一次数据并累加。</summary>
    public void Poll(SlcanSession session)
    {
        SessionSnapshot snapshot = session.TakeSnapshot();
        Data.AddRange(snapshot.Data);
        Rate.AddRange(snapshot.Rate);
        Raw.AddRange(snapshot.Raw);
        Last = snapshot;
    }

    public static async Task<bool> WaitUntilAsync(Func<bool> condition, int timeoutMs)
    {
        System.Diagnostics.Stopwatch stopwatch = System.Diagnostics.Stopwatch.StartNew();
        while (stopwatch.ElapsedMilliseconds < timeoutMs)
        {
            if (condition())
            {
                return true;
            }

            await Task.Delay(20);
        }

        return condition();
    }
}
