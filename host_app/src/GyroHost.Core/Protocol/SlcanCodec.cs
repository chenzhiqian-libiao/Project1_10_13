using System.Text;

namespace GyroHost.Core.Protocol;

/// <summary>
/// LAWICEL / CANABLE SLCAN ASCII 协议的编码部分。
/// <para>
/// 关键点：串口上跑的是 ASCII 文本命令与文本帧，不是裸 CAN 二进制。
/// 例如标准数据帧 0x322、DLC=1、数据 02 在串口上是字符串 <c>"t322102\r"</c>。
/// </para>
/// </summary>
public static class SlcanCodec
{
    /// <summary>关闭 CAN 通道（进入配置状态）。</summary>
    public const string CloseChannel = "C\r";

    /// <summary>以正常模式打开 CAN 通道。</summary>
    public const string OpenChannel = "O\r";

    /// <summary>读取状态标志。</summary>
    public const string ReadStatus = "F\r";

    /// <summary>读取固件版本。</summary>
    public const string ReadVersion = "V\r";

    /// <summary>UART 波特率 → SLCAN 命令行参数（LAWICEL 标准 S0..S8）。</summary>
    private static readonly (int Kbps, string Command)[] BitRateTable =
    {
        (10, "S0"),
        (20, "S1"),
        (50, "S2"),
        (100, "S3"),
        (125, "S4"),
        (250, "S5"),
        (500, "S6"),
        (800, "S7"),
        (1000, "S8"),
    };

    /// <summary>界面可选的 CAN 波特率（kbps），按常用程度排序，1000 为当前课题值。</summary>
    public static IReadOnlyList<int> SupportedBitRatesKbps { get; } =
        BitRateTable.Select(entry => entry.Kbps).ToArray();

    /// <summary>默认 CAN 波特率：本课题固定 1 Mbps。</summary>
    public const int DefaultBitRateKbps = 1000;

    /// <summary>
    /// 生成设置 CAN 波特率的命令。适配器不支持该速率时返回 false，
    /// 由调用方把原因显示到界面，而不是静默发送一个错误命令。
    /// </summary>
    public static bool TryBuildBitRateCommand(int kbps, out string command)
    {
        foreach ((int Kbps, string Command) entry in BitRateTable)
        {
            if (entry.Kbps == kbps)
            {
                command = entry.Command + "\r";
                return true;
            }
        }

        command = string.Empty;
        return false;
    }

    /// <summary>
    /// 把一条标准数据帧编码成 SLCAN 行。
    /// 格式：'t' + 3 位十六进制 ID + 1 位 DLC + 数据十六进制 + '\r'。
    /// </summary>
    public static string EncodeStandardDataFrame(uint id, ReadOnlySpan<byte> data)
    {
        if (id > 0x7FFU)
        {
            throw new ArgumentOutOfRangeException(nameof(id), id, "标准帧 ID 必须在 0x000..0x7FF 之间");
        }

        if (data.Length > 8)
        {
            throw new ArgumentOutOfRangeException(nameof(data), data.Length, "经典 CAN 数据长度不能超过 8 字节");
        }

        StringBuilder builder = new(5 + (data.Length * 2) + 1);
        builder.Append('t');
        builder.Append(id.ToString("X3"));
        builder.Append((char)('0' + data.Length));

        foreach (byte value in data)
        {
            builder.Append(value.ToString("X2"));
        }

        builder.Append('\r');
        return builder.ToString();
    }
}
