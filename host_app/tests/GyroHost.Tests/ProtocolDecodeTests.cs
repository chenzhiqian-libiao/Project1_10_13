using GyroHost.Core.Protocol;

namespace GyroHost.Tests;

/// <summary>
/// 协议解码测试。使用需求中给定的报文向量，确保端序、符号和比例正确。
/// </summary>
public sealed class ProtocolDecodeTests
{
    /// <summary>
    /// 三个比例常量是板端与上位机的<b>硬约定</b>，必须与固件逐一对齐：
    /// 角度 → <c>CAN_GYRO_ANGLE_SCALE</c>、温度 → <c>GyroScaleToInt16(temp, 100)</c>、
    /// 角速度 → <c>CAN_GYRO_RATE_SCALE</c>。
    /// <para>
    /// 这里显式锁死数值而不是引用常量，是为了在有人改动比例时让测试失败并提示
    /// "固件也要一起改"。比例不匹配不会报任何错，只会让角度或角速度整体差整数倍。
    /// </para>
    /// </summary>
    [Fact]
    public void ScaleConstants_ShouldMatchFirmwareContract()
    {
        Assert.Equal(100000.0, GyroProtocol.AngleLsbPerDegree);
        Assert.Equal(100.0, GyroProtocol.TemperatureLsbPerDegree);
        Assert.Equal(100000.0, GyroProtocol.RateLsbPerDps);
        Assert.Equal(1000.0, GyroProtocol.BiasLsbPerDps);
        Assert.Equal(70.0, GyroProtocol.RawLsbPerDps);
    }

    /// <summary>
    /// 编码方向的字节级契约：这些字节就是固件 <c>task_can_tx()</c> 在同一角度下发出的
    /// 前 4 个字节，可以直接拿 CAN 抓包结果对照。
    /// </summary>
    [Theory]
    [InlineData(90.0, 0x40, 0x54, 0x89, 0x00)]
    [InlineData(-90.0, 0xC0, 0xAB, 0x76, 0xFF)]
    [InlineData(12.34567, 0x87, 0xD6, 0x12, 0x00)]
    [InlineData(-0.12346, 0xC6, 0xCF, 0xFF, 0xFF)]
    [InlineData(0.00001, 0x01, 0x00, 0x00, 0x00)]
    public void EncodeAngle_ShouldProduceFirmwareBytes(
        double angleDeg, byte b0, byte b1, byte b2, byte b3)
    {
        byte[] data = GyroProtocol.BuildDataFrame(angleDeg, 31.56, GyroStatus.AngleValid, 10);

        Assert.Equal(new[] { b0, b1, b2, b3 }, data[..4]);
    }

    /// <summary>0x321：00 00 00 00 54 0C 17 0A → 0°、31.56℃、状态 0x17、间隔 10ms。</summary>
    [Fact]
    public void DecodeDataFrame_ShouldMatchRequiredVector()
    {
        byte[] data = { 0x00, 0x00, 0x00, 0x00, 0x54, 0x0C, 0x17, 0x0A };

        Assert.True(GyroProtocol.TryDecodeData(data, out DataPayload payload));

        Assert.Equal(0.0, payload.AngleDeg, 6);
        Assert.Equal(31.56, payload.TemperatureC, 6);
        Assert.Equal(0x17, (byte)payload.Status);
        Assert.Equal(10, payload.IntervalMs);
    }

    /// <summary>状态位含义必须与固件一致（0x17 = 有效 + 温度有效 + 校准完成 + 静止锁定）。</summary>
    [Fact]
    public void DecodeDataFrame_StatusBits_ShouldMatchFirmwareMeaning()
    {
        byte[] data = { 0x00, 0x00, 0x00, 0x00, 0x54, 0x0C, 0x17, 0x0A };
        Assert.True(GyroProtocol.TryDecodeData(data, out DataPayload payload));

        Assert.True(payload.Status.HasFlag(GyroStatus.AngleValid));
        Assert.True(payload.Status.HasFlag(GyroStatus.TemperatureValid));
        Assert.True(payload.Status.HasFlag(GyroStatus.CalibrationDone));
        Assert.True(payload.Status.HasFlag(GyroStatus.StillLocked));
        Assert.False(payload.Status.HasFlag(GyroStatus.CalibrationFailed));
        Assert.False(payload.Status.HasFlag(GyroStatus.DegradedCalibration));
    }

    /// <summary>
    /// bit5 = 校准降级：峰峰值超限但均值可用时板端仍报角度有效。
    /// 0x27 = bit0 有效 + bit1 温度 + bit2 校准完成 + bit5 降级（无 bit4 静止锁定）。
    /// </summary>
    [Fact]
    public void DecodeDataFrame_DegradedCalibration_ShouldStillReportAngleValid()
    {
        byte[] data = { 0x00, 0x00, 0x00, 0x00, 0x54, 0x0C, 0x27, 0x0A };
        Assert.True(GyroProtocol.TryDecodeData(data, out DataPayload payload));

        Assert.True(payload.Status.HasFlag(GyroStatus.AngleValid));
        Assert.True(payload.Status.HasFlag(GyroStatus.CalibrationDone));
        Assert.True(payload.Status.HasFlag(GyroStatus.DegradedCalibration));
        Assert.False(payload.Status.HasFlag(GyroStatus.CalibrationFailed));
        Assert.False(payload.Status.HasFlag(GyroStatus.StillLocked));
    }

    /// <summary>bit5 与 bit4 互相独立：0x37 = 有效 + 温度 + 校准完成 + 静止锁定 + 降级。</summary>
    [Fact]
    public void DecodeDataFrame_DegradedAndStillLocked_ShouldCoexist()
    {
        byte[] data = { 0x00, 0x00, 0x00, 0x00, 0x54, 0x0C, 0x37, 0x0A };
        Assert.True(GyroProtocol.TryDecodeData(data, out DataPayload payload));

        Assert.True(payload.Status.HasFlag(GyroStatus.DegradedCalibration));
        Assert.True(payload.Status.HasFlag(GyroStatus.StillLocked));
    }

    [Fact]
    public void DecodeDataFrame_ContinuousIntegrationCanReportStillness()
    {
        byte[] data = { 0, 0, 0, 0, 0x54, 0x0C, 0x57, 0x0A };
        Assert.True(GyroProtocol.TryDecodeData(data, out DataPayload payload));
        Assert.True(payload.Status.HasFlag(GyroStatus.ContinuousIntegration));
        Assert.True(payload.Status.HasFlag(GyroStatus.StillLocked));
        Assert.True(payload.Status.HasFlag(GyroStatus.AngleValid));
    }

    /// <summary>0x323：FF FF FF FF → raw = -1 → -0.00001°/s（协议最小负步进）。</summary>
    [Fact]
    public void DecodeRateFrame_MinusOneRaw_ShouldBeNegativeMinimumStep()
    {
        byte[] data = { 0xFF, 0xFF, 0xFF, 0xFF };

        Assert.True(GyroProtocol.TryDecodeRate(data, out double rate));

        Assert.Equal(-0.00001, rate, 6);
    }

    /// <summary>0x323：02 00 00 00 → raw = 2 → +0.00002°/s。</summary>
    [Fact]
    public void DecodeRateFrame_TwoRaw_ShouldBeTwoMinimumSteps()
    {
        byte[] data = { 0x02, 0x00, 0x00, 0x00 };

        Assert.True(GyroProtocol.TryDecodeRate(data, out double rate));

        Assert.Equal(0.00002, rate, 6);
    }

    /// <summary>0x323：01 00 00 00 → raw = 1 → +0.00001°/s（协议最小正步进）。</summary>
    [Fact]
    public void DecodeRateFrame_OneRaw_ShouldBeOneMinimumStep()
    {
        byte[] data = { 0x01, 0x00, 0x00, 0x00 };

        Assert.True(GyroProtocol.TryDecodeRate(data, out double rate));

        Assert.Equal(0.00001, rate, 6);
    }

    /// <summary>
    /// 角速度编码方向的字节级契约：与角度同理，这 4 个字节就是固件
    /// <c>task_can_tx()</c> 在该角速度下发出的 0x323 载荷，可直接对照 CAN 抓包。
    /// </summary>
    [Theory]
    [InlineData(0.0, 0x00, 0x00, 0x00, 0x00)]
    [InlineData(0.00001, 0x01, 0x00, 0x00, 0x00)]
    [InlineData(1.234, 0x08, 0xE2, 0x01, 0x00)]
    [InlineData(1.25, 0x48, 0xE8, 0x01, 0x00)]
    [InlineData(12.34567, 0x87, 0xD6, 0x12, 0x00)]
    [InlineData(-0.00001, 0xFF, 0xFF, 0xFF, 0xFF)]
    [InlineData(-100.0, 0x80, 0x69, 0x67, 0xFF)]
    public void EncodeRate_ShouldProduceFirmwareBytes(
        double rateDps, byte b0, byte b1, byte b2, byte b3)
    {
        byte[] data = GyroProtocol.BuildRateFrame(rateDps);

        Assert.Equal(new[] { b0, b1, b2, b3 }, data);
    }

    /// <summary>
    /// 角速度分辨率 0.00001°/s：解析得回 5 位小数，
    /// 以及传感器满量程 ±100°/s 在协议里不饱和（余量约 214 倍）。
    /// </summary>
    [Theory]
    [InlineData(0x87, 0xD6, 0x12, 0x00, 12.34567)]
    [InlineData(0x80, 0x96, 0x98, 0x00, 100.0)]
    [InlineData(0x80, 0x69, 0x67, 0xFF, -100.0)]
    public void DecodeRateFrame_ShouldResolveFiveDecimalPlaces(byte b0, byte b1, byte b2, byte b3, double expected)
    {
        Assert.True(GyroProtocol.TryDecodeRate(new[] { b0, b1, b2, b3 }, out double rate));

        Assert.Equal(expected, rate, 5);
    }

    /// <summary>往返一致性：角速度编码后再解码应还原（误差小于 1 个 LSB）。</summary>
    [Theory]
    [InlineData(0.0)]
    [InlineData(1.234)]
    [InlineData(-1.234)]
    [InlineData(99.99999)]
    [InlineData(-100.0)]
    public void EncodeThenDecodeRate_ShouldRoundTrip(double rateDps)
    {
        byte[] data = GyroProtocol.BuildRateFrame(rateDps);

        Assert.True(GyroProtocol.TryDecodeRate(data, out double rate));

        Assert.Equal(rateDps, rate, 5);
    }

    /// <summary>负角度：60 79 FE FF → raw = -100000 → -1.00000°。</summary>
    [Fact]
    public void DecodeDataFrame_NegativeAngle_ShouldBeSigned()
    {
        byte[] data = { 0x60, 0x79, 0xFE, 0xFF, 0x00, 0x00, 0x00, 0x00 };

        Assert.True(GyroProtocol.TryDecodeData(data, out DataPayload payload));

        Assert.Equal(-1.0, payload.AngleDeg, 6);
    }

    /// <summary>负温度：38 FF → -2.00℃。</summary>
    [Fact]
    public void DecodeDataFrame_NegativeTemperature_ShouldBeSigned()
    {
        byte[] data = { 0x00, 0x00, 0x00, 0x00, 0x38, 0xFF, 0x00, 0x0A };

        Assert.True(GyroProtocol.TryDecodeData(data, out DataPayload payload));

        Assert.Equal(-2.0, payload.TemperatureC, 6);
    }

    /// <summary>大角度：0x12345678 = 305419896 → 3054.19896°（比例 1e5，不再溢出）。</summary>
    [Fact]
    public void DecodeDataFrame_LargeAngle_ShouldNotOverflow()
    {
        byte[] data = { 0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 0x00, 0x0A };

        Assert.True(GyroProtocol.TryDecodeData(data, out DataPayload payload));

        Assert.Equal(3054.19896, payload.AngleDeg, 5);
    }

    /// <summary>角度分辨率 0.00001°：每个 LSB 都要能在 5 位小数上体现出来。</summary>
    [Theory]
    [InlineData(0x01, 0x00, 0x00, 0x00, 0.00001)]
    [InlineData(0x64, 0x00, 0x00, 0x00, 0.00100)]
    [InlineData(0x39, 0x30, 0x00, 0x00, 0.12345)]
    [InlineData(0xC6, 0xCF, 0xFF, 0xFF, -0.12346)]
    public void DecodeDataFrame_ShouldResolveFiveDecimalPlaces(byte b0, byte b1, byte b2, byte b3, double expected)
    {
        byte[] data = { b0, b1, b2, b3, 0x00, 0x00, 0x00, 0x0A };

        Assert.True(GyroProtocol.TryDecodeData(data, out DataPayload payload));

        Assert.Equal(expected, payload.AngleDeg, 5);
    }

    /// <summary>DLC 不足时必须返回 false，由上层计入异常帧，不能把残缺数据当有效值。</summary>
    [Fact]
    public void DecodeDataFrame_ShortPayload_ShouldFail()
    {
        Assert.False(GyroProtocol.TryDecodeData(new byte[] { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 }, out _));
        Assert.False(GyroProtocol.TryDecodeRate(new byte[] { 0x01, 0x02, 0x03 }, out _));
    }

    /// <summary>往返一致性：编码后再解码应还原原值。</summary>
    [Theory]
    [InlineData(0.0, 25.0)]
    [InlineData(-179.99, -12.34)]
    [InlineData(123.45, 31.56)]
    [InlineData(-0.01, 0.0)]
    public void EncodeThenDecode_ShouldRoundTrip(double angleDeg, double temperatureC)
    {
        byte[] data = GyroProtocol.BuildDataFrame(angleDeg, temperatureC, GyroStatus.AngleValid, 10);

        Assert.True(GyroProtocol.TryDecodeData(data, out DataPayload payload));
        Assert.Equal(angleDeg, payload.AngleDeg, 6);
        Assert.Equal(temperatureC, payload.TemperatureC, 6);
    }

    /// <summary>
    /// 0x324：D0–D1 原始码 int16、D2–D5 采样时刻 uint32、D6–D7 零偏 int16 ×1000。
    /// 负原始码与负零偏都要正确还原（补码 + 小端）。
    /// </summary>
    [Fact]
    public void DecodeRawFrame_ShouldDecodeSignedCodeTickAndBias()
    {
        // raw = 0x1234 = 4660；tick = 0x00000539 = 1337；bias raw = 0xFFF4 = -12 → -0.012°/s
        byte[] data = { 0x34, 0x12, 0x39, 0x05, 0x00, 0x00, 0xF4, 0xFF };

        Assert.True(GyroProtocol.TryDecodeRaw(data, out RawPayload payload));

        Assert.Equal(4660, payload.RawCode);
        Assert.Equal(1337U, payload.SampleTickMs);
        Assert.Equal(-0.012, payload.BiasDps, 6);
    }

    /// <summary>0x324 编码方向的字节契约：可直接与固件 task_can_tx() 的载荷对照。</summary>
    [Fact]
    public void EncodeRawFrame_ShouldProduceFirmwareBytes()
    {
        byte[] data = GyroProtocol.BuildRawFrame(rawCode: -2, sampleTickMs: 0x11223344U, biasDps: 0.001);

        Assert.Equal(
            new byte[] { 0xFE, 0xFF, 0x44, 0x33, 0x22, 0x11, 0x01, 0x00 },
            data);
    }

    /// <summary>
    /// 0x325：D0–D3 滤波后角速度 int32 ×100000、D4–D7 采样时刻 uint32。
    /// 时间戳必须与 0x324 相同才允许配对。
    /// </summary>
    [Fact]
    public void DecodeRawDetailFrame_ShouldDecodeFilteredRateAndTick()
    {
        // 1.234°/s → 123400 = 0x0001E208；tick = 1337
        byte[] data = { 0x08, 0xE2, 0x01, 0x00, 0x39, 0x05, 0x00, 0x00 };

        Assert.True(GyroProtocol.TryDecodeRawDetail(data, out RawDetailPayload payload));

        Assert.Equal(1.234, payload.FilteredRateDps, 6);
        Assert.Equal(1337U, payload.SampleTickMs);
    }

    /// <summary>原始码/零偏的编码→解码往返一致性（误差不超过 1 个 LSB）。</summary>
    [Theory]
    [InlineData(0, 0.0)]
    [InlineData(-1234, -32.0)]
    [InlineData(7000, 1.234)]
    [InlineData(-7000, -1.234)]
    [InlineData(32767, 32.767)]
    [InlineData(-32768, -32.768)]
    public void EncodeThenDecodeRaw_ShouldRoundTrip(short rawCode, double biasDps)
    {
        byte[] data = GyroProtocol.BuildRawFrame(rawCode, 987654321U, biasDps);

        Assert.True(GyroProtocol.TryDecodeRaw(data, out RawPayload payload));

        Assert.Equal(rawCode, payload.RawCode);
        Assert.Equal(987654321U, payload.SampleTickMs);
        Assert.Equal(biasDps, payload.BiasDps, 3);
    }

    /// <summary>0x324/0x325 的 DLC 不足时同样必须返回 false。</summary>
    [Fact]
    public void DecodeRawFrames_ShortPayload_ShouldFail()
    {
        Assert.False(GyroProtocol.TryDecodeRaw(new byte[] { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 }, out _));
        Assert.False(GyroProtocol.TryDecodeRawDetail(new byte[] { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 }, out _));
    }

    /// <summary>命令帧编码：0x322 + DLC=1 + 数据 02 → "t322102\r"。</summary>
    [Theory]
    [InlineData(0x01, "t322101\r")]
    [InlineData(0x02, "t322102\r")]
    public void EncodeCommandFrame_ShouldProduceExpectedSlcanLine(byte command, string expected)
    {
        string line = SlcanCodec.EncodeStandardDataFrame(GyroProtocol.CommandId, GyroProtocol.BuildCommandFrame(command));

        Assert.Equal(expected, line);
    }

    /// <summary>SLCAN 波特率命令：1 Mbps → S8（LAWICEL 标准）。</summary>
    [Theory]
    [InlineData(1000, "S8\r")]
    [InlineData(500, "S6\r")]
    [InlineData(250, "S5\r")]
    [InlineData(125, "S4\r")]
    public void BuildBitRateCommand_ShouldMapStandardRates(int kbps, string expected)
    {
        Assert.True(SlcanCodec.TryBuildBitRateCommand(kbps, out string command));
        Assert.Equal(expected, command);
    }

    /// <summary>不支持的波特率必须显式失败，而不是静默发送错误命令。</summary>
    [Fact]
    public void BuildBitRateCommand_UnsupportedRate_ShouldFail()
    {
        Assert.False(SlcanCodec.TryBuildBitRateCommand(1234, out _));
    }
}
