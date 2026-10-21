using System.Text;

namespace GyroHost.Core.Protocol;

/// <summary>
/// 增量式 SLCAN 解析器：把串口字节流切成完整行再解析。
/// <para>
/// 必须处理三种真实情况：分包（一行被切成多次到达）、粘包（一次到达含多行）、
/// 残缺/异常输入（截断行、非法十六进制、非法 DLC、适配器 BEL）。
/// 因此解析器内部带缓冲区，只有遇到行结束符才产出事件。
/// </para>
/// <para>此类不是线程安全的：规定只由单一接收线程调用。</para>
/// </summary>
public sealed class SlcanFrameParser
{
    /// <summary>单行最大长度。超过即判为噪声并丢弃，避免异常输入把缓冲区撑爆。</summary>
    private const int MaxLineLength = 128;

    private readonly List<byte> _buffer = new(256);

    /// <summary>true = 已判定当前行超长，丢弃到行结束符为止。</summary>
    private bool _discarding;

    /// <summary>累计丢弃的超长行数量，便于界面显示异常。</summary>
    public long OverlongLineCount { get; private set; }

    /// <summary>投喂一段串口字节，返回本次能解析出的全部事件。</summary>
    public IReadOnlyList<SlcanEvent> Feed(ReadOnlySpan<byte> chunk, TimeSpan arrival)
    {
        List<SlcanEvent> events = new();

        foreach (byte value in chunk)
        {
            if (value == (byte)'\r' || value == (byte)'\n')
            {
                if (_discarding)
                {
                    // 超长行在这里结束，行结束符本身被吃掉，不再产生第二个残缺行。
                    _discarding = false;
                    continue;
                }

                if (value == (byte)'\r' || _buffer.Count > 0)
                {
                    Flush(events, arrival);
                }

                continue;
            }

            if (_discarding)
            {
                continue;
            }

            if (_buffer.Count >= MaxLineLength)
            {
                _buffer.Clear();
                _discarding = true;
                OverlongLineCount++;
                events.Add(new SlcanParseError("单行超过 128 字节，已丢弃", string.Empty, arrival));
                continue;
            }

            _buffer.Add(value);
        }

        return events;
    }

    /// <summary>丢弃半截行。断开连接或重置时调用。</summary>
    public void Reset()
    {
        _buffer.Clear();
        _discarding = false;
    }

    private void Flush(List<SlcanEvent> events, TimeSpan arrival)
    {
        if (_buffer.Count == 0)
        {
            return;
        }

        string line = Encoding.ASCII.GetString(_buffer.ToArray());
        _buffer.Clear();
        ParseLine(line, events, arrival);
    }

    private static void ParseLine(string line, List<SlcanEvent> events, TimeSpan arrival)
    {
        if (line.Length == 0)
        {
            return;
        }

        if (line.Length == 1)
        {
            switch (line[0])
            {
                case 'z':
                    events.Add(new SlcanTxAck(true, arrival));
                    return;
                case 'Z':
                    events.Add(new SlcanTxAck(false, arrival));
                    return;
                case '\a':
                    events.Add(new SlcanParseError("适配器返回 BEL(0x07)：上一帧或命令被拒绝", line, arrival));
                    return;
            }
        }

        switch (line[0])
        {
            case 't':
                ParseFrame(line, isExtended: false, isRemote: false, events, arrival);
                return;
            case 'r':
                ParseFrame(line, isExtended: false, isRemote: true, events, arrival);
                return;
            case 'T':
                ParseFrame(line, isExtended: true, isRemote: false, events, arrival);
                return;
            case 'R':
                ParseFrame(line, isExtended: true, isRemote: true, events, arrival);
                return;
            case 'C':
            case 'O':
            case 'S':
            case 'L':
            case 'U':
                // 适配器对配置命令的回显（C 关闭、Sx 设波特率、O 打开、L/U 时间戳开关）。
                // 这些不是数据也不是错误，按普通信息处理，避免误计异常帧。
                events.Add(new SlcanText(line, arrival));
                return;
            case 'F':
                // F + 2 位十六进制状态标志
                if (line.Length >= 3 && TryParseHexByte(line.AsSpan(1, 2), out byte flags))
                {
                    events.Add(new SlcanStatusFlags(flags, arrival));
                }
                else
                {
                    events.Add(new SlcanParseError("状态行格式错误", line, arrival));
                }

                return;
            case 'V':
            case 'v':
            case 'N':
            case 'M':
                events.Add(new SlcanText(line, arrival));
                return;
            default:
                events.Add(new SlcanParseError($"未知的 SLCAN 行: {line}", line, arrival));
                return;
        }
    }

    private static void ParseFrame(string line, bool isExtended, bool isRemote, List<SlcanEvent> events, TimeSpan arrival)
    {
        int idLength = isExtended ? 8 : 3;
        int headerLength = 1 + idLength + 1;

        if (line.Length < headerLength)
        {
            events.Add(new SlcanParseError("帧头不完整", line, arrival));
            return;
        }

        if (!TryParseHexUInt(line.AsSpan(1, idLength), out uint id))
        {
            events.Add(new SlcanParseError("帧 ID 不是合法十六进制", line, arrival));
            return;
        }

        char dlcChar = line[headerLength - 1];
        if (dlcChar is < '0' or > '8')
        {
            events.Add(new SlcanParseError($"非法 DLC '{dlcChar}'（经典 CAN 只允许 0..8）", line, arrival));
            return;
        }

        int dlc = dlcChar - '0';

        // 远程帧不带数据字节；数据帧后面必须正好有 dlc*2 个十六进制字符。
        int expectedLength = isRemote ? headerLength : headerLength + (dlc * 2);
        if (line.Length != expectedLength)
        {
            events.Add(new SlcanParseError(
                $"帧长度与 DLC 不符：期望 {expectedLength} 字符，实际 {line.Length} 字符",
                line,
                arrival));
            return;
        }

        if (isRemote)
        {
            events.Add(new SlcanCanFrame(
                new CanFrame(id, isExtended, true, dlc, Array.Empty<byte>()),
                arrival));
            return;
        }

        byte[] data = new byte[dlc];
        for (int i = 0; i < dlc; i++)
        {
            if (!TryParseHexByte(line.AsSpan(headerLength + (i * 2), 2), out data[i]))
            {
                events.Add(new SlcanParseError("数据区不是合法十六进制", line, arrival));
                return;
            }
        }

        events.Add(new SlcanCanFrame(new CanFrame(id, isExtended, false, dlc, data), arrival));
    }

    private static bool TryParseHexByte(ReadOnlySpan<char> text, out byte value)
    {
        value = 0;
        if (text.Length != 2)
        {
            return false;
        }

        int high = HexValue(text[0]);
        int low = HexValue(text[1]);
        if (high < 0 || low < 0)
        {
            return false;
        }

        value = (byte)((high << 4) | low);
        return true;
    }

    private static bool TryParseHexUInt(ReadOnlySpan<char> text, out uint value)
    {
        value = 0U;
        if (text.Length == 0)
        {
            return false;
        }

        uint result = 0U;
        foreach (char c in text)
        {
            int nibble = HexValue(c);
            if (nibble < 0)
            {
                return false;
            }

            result = (result << 4) | (uint)nibble;
        }

        value = result;
        return true;
    }

    private static int HexValue(char c) => c switch
    {
        >= '0' and <= '9' => c - '0',
        >= 'a' and <= 'f' => c - 'a' + 10,
        >= 'A' and <= 'F' => c - 'A' + 10,
        _ => -1,
    };
}
