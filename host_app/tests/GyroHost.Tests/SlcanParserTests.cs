using System.Text;
using GyroHost.Core.Protocol;

namespace GyroHost.Tests;

/// <summary>
/// SLCAN 行解析测试：分包、粘包、残缺帧、异常输入。串口字节不是裸 CAN 数据，
/// 必须先切成行再解析，这里逐条覆盖。
/// </summary>
public sealed class SlcanParserTests
{
    private static readonly TimeSpan Arrival = TimeSpan.FromSeconds(1);

    private static IReadOnlyList<SlcanEvent> Feed(SlcanFrameParser parser, string text)
        => parser.Feed(Encoding.ASCII.GetBytes(text), Arrival);

    /// <summary>单条完整帧。</summary>
    [Fact]
    public void SingleFrame_ShouldParse()
    {
        SlcanFrameParser parser = new();

        IReadOnlyList<SlcanEvent> events = Feed(parser, "t321800000000540C170A\r");

        SlcanCanFrame frame = Assert.IsType<SlcanCanFrame>(Assert.Single(events));
        Assert.Equal(0x321U, frame.Frame.Id);
        Assert.Equal(8, frame.Frame.Dlc);
        Assert.False(frame.Frame.IsExtended);
        Assert.False(frame.Frame.IsRemote);
        Assert.Equal(new byte[] { 0x00, 0x00, 0x00, 0x00, 0x54, 0x0C, 0x17, 0x0A }, frame.Frame.Data);
    }

    /// <summary>粘包：一次到达包含多行，必须全部解析。</summary>
    [Fact]
    public void MultipleLinesInOneChunk_ShouldAllParse()
    {
        SlcanFrameParser parser = new();

        IReadOnlyList<SlcanEvent> events = Feed(parser, "t321800000000540C170A\rt3234FFFFFFFF\r");

        Assert.Equal(2, events.Count);
        SlcanCanFrame data = Assert.IsType<SlcanCanFrame>(events[0]);
        SlcanCanFrame rate = Assert.IsType<SlcanCanFrame>(events[1]);
        Assert.Equal(0x321U, data.Frame.Id);
        Assert.Equal(0x323U, rate.Frame.Id);
        Assert.Equal(4, rate.Frame.Dlc);
    }

    /// <summary>分包：一行被切成多次到达，只有凑齐行结束符才产出事件。</summary>
    [Fact]
    public void FragmentedLine_ShouldEmitOnlyAfterTerminator()
    {
        SlcanFrameParser parser = new();

        Assert.Empty(Feed(parser, "t32"));
        Assert.Empty(Feed(parser, "34FF"));
        Assert.Empty(Feed(parser, "FFFF"));

        SlcanCanFrame frame = Assert.IsType<SlcanCanFrame>(Assert.Single(Feed(parser, "FF\r")));
        Assert.Equal(0x323U, frame.Frame.Id);
        Assert.Equal(new byte[] { 0xFF, 0xFF, 0xFF, 0xFF }, frame.Frame.Data);
    }

    /// <summary>粘包 + 分包混合：行边界落在任意位置都必须正确。</summary>
    [Fact]
    public void MixedFragmentationAndBatching_ShouldParseAllFrames()
    {
        SlcanFrameParser parser = new();
        List<SlcanEvent> collected = new();

        // 两帧被切成了 5 个不规则的字节块。
        string[] chunks =
        {
            "t3218000000",
            "00540C170A\rt323",
            "402",
            "000000\r",
            string.Empty,
        };

        foreach (string chunk in chunks)
        {
            collected.AddRange(Feed(parser, chunk));
        }

        Assert.Equal(2, collected.Count);
        Assert.Equal(0x321U, ((SlcanCanFrame)collected[0]).Frame.Id);
        SlcanCanFrame rate = (SlcanCanFrame)collected[1];
        Assert.Equal(0x323U, rate.Frame.Id);
        Assert.Equal(new byte[] { 0x02, 0x00, 0x00, 0x00 }, rate.Frame.Data);
    }

    /// <summary>\r\n 结尾不应产生多余的空行事件。</summary>
    [Fact]
    public void CrLfTerminator_ShouldNotProduceExtraEvents()
    {
        SlcanFrameParser parser = new();

        IReadOnlyList<SlcanEvent> events = Feed(parser, "t323402000000\r\n");

        Assert.Single(events);
    }

    /// <summary>DLC 与数据长度不符 → 异常帧，不能按有效帧处理。</summary>
    [Theory]
    [InlineData("t3217FFFFFFFF\r")]      // 声明 7 字节却给了 8 字节
    [InlineData("t3232FFFFFF\r")]        // 声明 2 字节却给了 3 字节
    [InlineData("t3218FFFFFFFF\r")]      // 声明 8 字节却给了 4 字节
    [InlineData("t321\r")]               // 帧头不完整
    public void LengthMismatch_ShouldBeParseError(string line)
    {
        SlcanFrameParser parser = new();

        IReadOnlyList<SlcanEvent> events = Feed(parser, line);

        Assert.IsType<SlcanParseError>(Assert.Single(events));
    }

    /// <summary>经典 CAN 不允许 DLC > 8。</summary>
    [Fact]
    public void InvalidDlc_ShouldBeParseError()
    {
        SlcanFrameParser parser = new();

        IReadOnlyList<SlcanEvent> events = Feed(parser, "t3219FFFFFFFFFFFF\r");

        SlcanParseError error = Assert.IsType<SlcanParseError>(Assert.Single(events));
        Assert.Contains("DLC", error.Reason);
    }

    /// <summary>非法十六进制与未知行必须被识别为异常，而不是被当成数据。</summary>
    [Theory]
    [InlineData("t32Z800000000540C170A\r", "ID")]
    [InlineData("t32180000ZZ00540C170A\r", "十六进制")]
    [InlineData("hello world\r", "未知")]
    [InlineData("\a\r", "BEL")]
    public void MalformedInput_ShouldBeParseError(string line, string expectedFragment)
    {
        SlcanFrameParser parser = new();

        IReadOnlyList<SlcanEvent> events = Feed(parser, line);

        SlcanParseError error = Assert.IsType<SlcanParseError>(Assert.Single(events));
        Assert.Contains(expectedFragment, error.Reason, StringComparison.Ordinal);
    }

    /// <summary>适配器对发送的应答：z = 已写入总线，Z = 被拒绝。</summary>
    [Fact]
    public void TxAcks_ShouldBeReported()
    {
        SlcanFrameParser parser = new();

        IReadOnlyList<SlcanEvent> events = Feed(parser, "z\rZ\r");

        Assert.Equal(2, events.Count);
        Assert.True(Assert.IsType<SlcanTxAck>(events[0]).Accepted);
        Assert.False(Assert.IsType<SlcanTxAck>(events[1]).Accepted);
    }

    /// <summary>适配器状态标志 F + 2 位十六进制。</summary>
    [Fact]
    public void StatusFlags_ShouldParse()
    {
        SlcanFrameParser parser = new();

        IReadOnlyList<SlcanEvent> events = Feed(parser, "F03\r");

        Assert.Equal(0x03, Assert.IsType<SlcanStatusFlags>(Assert.Single(events)).Flags);
    }

    /// <summary>命令回显（C / S8 / O）不能被计成异常帧。</summary>
    [Theory]
    [InlineData("V1013\r")]
    [InlineData("C\r")]
    [InlineData("O\r")]
    [InlineData("S8\r")]
    public void CommandAndInfoEchoes_ShouldNotBeErrors(string line)
    {
        SlcanFrameParser parser = new();

        IReadOnlyList<SlcanEvent> events = Feed(parser, line);

        Assert.IsType<SlcanText>(Assert.Single(events));
    }

    /// <summary>扩展帧与远程帧要能解析出来（本课题用不到，但必须能识别并计数）。</summary>
    [Fact]
    public void ExtendedAndRemoteFrames_ShouldParse()
    {
        SlcanFrameParser parser = new();

        IReadOnlyList<SlcanEvent> events = Feed(parser, "T00000321800000000540C170A\rr3218\r");

        Assert.Equal(2, events.Count);
        SlcanCanFrame extended = Assert.IsType<SlcanCanFrame>(events[0]);
        Assert.True(extended.Frame.IsExtended);
        Assert.Equal(0x321U, extended.Frame.Id);

        SlcanCanFrame remote = Assert.IsType<SlcanCanFrame>(events[1]);
        Assert.True(remote.Frame.IsRemote);
        Assert.Equal(8, remote.Frame.Dlc);
        Assert.Empty(remote.Frame.Data);
    }

    /// <summary>超长噪声行必须被丢弃并计入异常，且不影响后续正常帧。</summary>
    [Fact]
    public void OverlongLine_ShouldBeDroppedWithoutBreakingFollowingFrames()
    {
        SlcanFrameParser parser = new();

        IReadOnlyList<SlcanEvent> first = Feed(parser, new string('x', 200) + "\r");
        Assert.IsType<SlcanParseError>(Assert.Single(first));
        Assert.Equal(1, parser.OverlongLineCount);

        IReadOnlyList<SlcanEvent> second = Feed(parser, "t323402000000\r");
        SlcanCanFrame frame = Assert.IsType<SlcanCanFrame>(Assert.Single(second));
        Assert.Equal(0x323U, frame.Frame.Id);
    }

    /// <summary>到达时刻必须原样带到事件上，供界面用真实接收时间绘图。</summary>
    [Fact]
    public void Arrival_ShouldBePropagatedToEvents()
    {
        SlcanFrameParser parser = new();
        TimeSpan arrival = TimeSpan.FromSeconds(12.5);

        IReadOnlyList<SlcanEvent> events = parser.Feed(Encoding.ASCII.GetBytes("t323402000000\r"), arrival);

        Assert.Equal(arrival, Assert.Single(events).Arrival);
    }
}
