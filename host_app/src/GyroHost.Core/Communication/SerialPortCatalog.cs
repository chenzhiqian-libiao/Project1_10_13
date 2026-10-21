using System.IO.Ports;

namespace GyroHost.Core.Communication;

/// <summary>
/// 串口枚举与常用参数。<see cref="SerialPort.GetPortNames"/> 枚举当前可用端口，
/// 不写死 COM 号，换机器/换适配器直接刷新即可。
/// </summary>
public static class SerialPortCatalog
{
    /// <summary>界面可选的串口波特率。默认 115200，是多数 SLCAN 适配器的默认值。</summary>
    public static IReadOnlyList<int> StandardBaudRates { get; } = new[]
    {
        9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000, 2000000,
    };

    /// <summary>默认串口波特率（注意：这是串口参数，与 CAN 总线的 1 Mbps 无关）。</summary>
    public const int DefaultBaudRate = 115200;

    /// <summary>枚举可用串口，按 COM 号数值排序（COM3 排在 COM10 前面）。</summary>
    public static IReadOnlyList<string> GetPortNames()
    {
        try
        {
            return SerialPort.GetPortNames()
                .Distinct(StringComparer.OrdinalIgnoreCase)
                .OrderBy(static name => name, PortNameComparer.Instance)
                .ToArray();
        }
        catch (Exception)
        {
            // 枚举失败（驱动异常等）不抛给界面，返回空列表由用户手动刷新。
            return Array.Empty<string>();
        }
    }

    /// <summary>把 "COM10" 这类端口名按数值比较，避免字母序把 COM10 排在 COM3 之前。</summary>
    private sealed class PortNameComparer : IComparer<string>
    {
        public static PortNameComparer Instance { get; } = new();

        public int Compare(string? x, string? y)
        {
            int nx = ExtractNumber(x);
            int ny = ExtractNumber(y);
            if (nx >= 0 && ny >= 0)
            {
                int byNumber = nx.CompareTo(ny);
                if (byNumber != 0)
                {
                    return byNumber;
                }
            }

            return string.CompareOrdinal(x, y);
        }

        private static int ExtractNumber(string? name)
        {
            if (string.IsNullOrEmpty(name))
            {
                return -1;
            }

            int index = 0;
            while (index < name.Length && !char.IsDigit(name[index]))
            {
                index++;
            }

            return index < name.Length && int.TryParse(name.AsSpan(index), out int value) ? value : -1;
        }
    }
}
