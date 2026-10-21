using System.Buffers.Binary;
using GyroHost.Core.Protocol;

namespace GyroHost.Tests;

/// <summary>
/// 电机查询帧、单位换算、解码器布局与反馈帧解析的纯协议测试。
/// 这里的 <see cref="TestLayout"/> 只是测试夹具，**不是**真实协议结论。
/// </summary>
public sealed class MotorProtocolTests
{
    /// <summary>测试夹具：DLC=8，Data[0]=设备号，Data[1]=0x08，Data[2..5]=int32 小端位置。</summary>
    internal static MotorPositionReplyLayout TestLayout { get; } = new(
        expectedDlc: 8,
        deviceIdOffset: 0,
        queryCodeOffset: 1,
        positionOffset: 2,
        positionLength: 4,
        positionIsLittleEndian: true,
        positionIsSigned: true,
        description: "测试夹具（不是真实协议）");

    internal static byte[] BuildTestReply(byte deviceId, byte queryCode, int rawPosition)
    {
        byte[] data = new byte[8];
        data[0] = deviceId;
        data[1] = queryCode;
        BinaryPrimitives.WriteInt32LittleEndian(data.AsSpan(2, 4), rawPosition);
        return data;
    }

    /// <summary>位置查询帧必须是 0x0408 / DLC=2 / [设备号, 0x08]。</summary>
    [Fact]
    public void BuildPositionQuery_ShouldMatchConfirmedFrame()
    {
        byte[] payload = MotorProtocol.BuildPositionQuery(0x01);

        Assert.Equal(new byte[] { 0x01, 0x08 }, payload);
        Assert.Equal("t40820108\r", SlcanCodec.EncodeStandardDataFrame(MotorProtocol.QueryId, payload));
    }

    /// <summary>0.1 mm/单位 → 角度：轮径 100 mm 时 1 个位置单位约 0.11459°。</summary>
    [Fact]
    public void PositionDeltaLsbToDegrees_ShouldUseWheelDiameter()
    {
        double oneLsb = MotorProtocol.PositionDeltaLsbToDegrees(1, 100.0);
        Assert.Equal(0.1 * 360.0 / (Math.PI * 100.0), oneLsb, 12);
        Assert.InRange(oneLsb, 0.11459, 0.11460);

        Assert.Equal(-oneLsb, MotorProtocol.PositionDeltaLsbToDegrees(-1, 100.0), 12);
        Assert.Equal(5.0 * oneLsb, MotorProtocol.PositionDeltaLsbToDegrees(5, 100.0), 12);

        // 轮径加倍 → 每个位置单位对应角度减半。
        Assert.Equal(oneLsb / 2.0, MotorProtocol.PositionDeltaLsbToDegrees(1, 200.0), 12);
    }

    /// <summary>轮径非法时必须报错，不能悄悄按 0 或负数换算。</summary>
    [Theory]
    [InlineData(0.0)]
    [InlineData(-100.0)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    public void PositionDeltaLsbToDegrees_WithInvalidDiameter_ShouldThrow(double diameter)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => MotorProtocol.PositionDeltaLsbToDegrees(1, diameter));
    }

    /// <summary>
    /// 默认解码器已按实机报文接入（DLC=8、Data[0]=设备号、Data[1]=0x08、Data[2..5]=s32 小端）。
    /// 位置字段长度/符号仍待更多实机报文验证，但**普通真实连接就走这一个解码器**。
    /// </summary>
    [Fact]
    public void DefaultDecoder_ShouldBeEnabledWithCapturedLayout()
    {
        IMotorPositionDecoder decoder = MotorProtocol.CreateDefaultPositionDecoder();

        Assert.True(decoder.IsConfirmed);
        Assert.Empty(decoder.MissingFields);
        Assert.Equal(8, ((MotorPositionReplyLayout)decoder).ExpectedDlc);
        Assert.Equal(2, ((MotorPositionReplyLayout)decoder).PositionOffset);
        Assert.Equal(4, ((MotorPositionReplyLayout)decoder).PositionLength);
        Assert.True(((MotorPositionReplyLayout)decoder).PositionIsLittleEndian);
        Assert.True(((MotorPositionReplyLayout)decoder).PositionIsSigned);

        Assert.True(decoder.TryDecode(BuildTestReply(1, 8, 1234), 1, out MotorPositionReply reply));
        Assert.Equal(1234, reply.RawPosition);

        // 共享 0x0409：查询码不是 0x08（温度/状态/版本）或设备号不符时必须拒绝。
        Assert.False(decoder.TryDecode(BuildTestReply(1, 6, 1234), 1, out _));
        Assert.False(decoder.TryDecode(BuildTestReply(2, 8, 1234), 1, out _));
    }

    /// <summary>
    /// 真实报文核对记录 1（2026-10-17，设备 01）：
    /// TX <c>0x0408 [01 08]</c> → RX <c>0x0409 [01 08 BC 18 00 00 FC FF]</c>。
    /// <para>
    /// 这里证明默认解码器能按接入的布局读出这段**真实字节**（6332 → 633.2 mm），
    /// 同时用断言固定住“本帧**不能**区分长度/符号”：2 字节与 4 字节解释结果相同。
    /// <c>Data[6..7]</c> 含义未知，测试不做任何解释。
    /// </para>
    /// </summary>
    [Fact]
    public void RealCapture_20261017_ShouldDecode6332ButCannotDisambiguateLength()
    {
        byte[] captured = { 0x01, 0x08, 0xBC, 0x18, 0x00, 0x00, 0xFC, 0xFF };

        MotorPositionReplyLayout fourByte = new(8, 0, 1, 2, 4, true, true, "候选：Data[2..5] int32 LE（未确认）");
        MotorPositionReplyLayout twoByte = new(8, 0, 1, 2, 2, true, true, "候选：Data[2..3] int16 LE（未确认）");

        Assert.True(fourByte.TryDecode(captured, 0x01, out MotorPositionReply four));
        Assert.True(twoByte.TryDecode(captured, 0x01, out MotorPositionReply two));

        // 本帧两种长度解释都得到 6332，因此这一帧无法区分长度。
        Assert.Equal(6332, four.RawPosition);
        Assert.Equal(6332, two.RawPosition);
        Assert.Equal(633.2, MotorProtocol.PositionLsbToMillimeters(four.RawPosition), 9);

        // 默认解码器就是这套布局：真实报文必须能解出 6332。
        Assert.True(MotorProtocol.CreateDefaultPositionDecoder().TryDecode(captured, 0x01, out MotorPositionReply actual));
        Assert.Equal(6332, actual.RawPosition);
        Assert.Equal(633.2, MotorProtocol.PositionLsbToMillimeters(actual.RawPosition), 9);

        // Data[6..7] 不参与计算：只改这两个字节，解析结果必须不变。
        byte[] withOtherTail = { 0x01, 0x08, 0xBC, 0x18, 0x00, 0x00, 0x12, 0x34 };
        Assert.True(fourByte.TryDecode(withOtherTail, 0x01, out MotorPositionReply tailChanged));
        Assert.Equal(6332, tailChanged.RawPosition);

        // 共享 ID：设备号/查询码不匹配时必须拒绝，温度等其他回复不能被当成位置。
        Assert.False(fourByte.TryDecode(captured, 0x02, out _));
        Assert.False(fourByte.TryDecode(new byte[] { 0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 0x01, out _));
    }

    /// <summary>已确认布局：设备号、查询码、DLC 全部匹配才解码。</summary>
    [Fact]
    public void ConfirmedLayout_ShouldValidateDeviceQueryAndDlc()
    {
        IMotorPositionDecoder decoder = TestLayout;

        Assert.True(decoder.TryDecode(BuildTestReply(1, 8, -1234), 1, out MotorPositionReply reply));
        Assert.Equal(1, reply.DeviceId);
        Assert.Equal(8, reply.QueryCode);
        Assert.Equal(-1234, reply.RawPosition);

        Assert.False(decoder.TryDecode(BuildTestReply(2, 8, 100), 1, out _));
        Assert.False(decoder.TryDecode(BuildTestReply(1, 0x06, 100), 1, out _));
        Assert.False(decoder.TryDecode(new byte[7], 1, out _));
    }

    /// <summary>大端、2 字节、无符号布局都要按声明解析。</summary>
    [Fact]
    public void ConfirmedLayout_ShouldHonorEndiannessAndSignedness()
    {
        MotorPositionReplyLayout layout = new(
            expectedDlc: 4,
            deviceIdOffset: 0,
            queryCodeOffset: 1,
            positionOffset: 2,
            positionLength: 2,
            positionIsLittleEndian: false,
            positionIsSigned: false,
            description: "测试夹具：大端 u16");

        byte[] data = { 1, 8, 0x12, 0x34 };
        Assert.True(layout.TryDecode(data, 1, out MotorPositionReply reply));
        Assert.Equal(0x1234, reply.RawPosition);

        MotorPositionReplyLayout signed = new(
            expectedDlc: 4,
            deviceIdOffset: 0,
            queryCodeOffset: 1,
            positionOffset: 2,
            positionLength: 2,
            positionIsLittleEndian: false,
            positionIsSigned: true,
            description: "测试夹具：大端 s16");

        byte[] negative = { 1, 8, 0xFF, 0xFE };
        Assert.True(signed.TryDecode(negative, 1, out MotorPositionReply negativeReply));
        Assert.Equal(-2, negativeReply.RawPosition);
    }

    /// <summary>回复不含设备号时按单设备处理：不检查设备号，但仍在结果里带出当前设备号。</summary>
    [Fact]
    public void LayoutWithoutDeviceId_ShouldTreatAsSingleDevice()
    {
        MotorPositionReplyLayout layout = new(
            expectedDlc: 4,
            deviceIdOffset: -1,
            queryCodeOffset: 0,
            positionOffset: 1,
            positionLength: 2,
            positionIsLittleEndian: true,
            positionIsSigned: true,
            description: "测试夹具：无设备号");

        Assert.False(layout.HasDeviceId);
        Assert.Contains("单设备", layout.Description, StringComparison.Ordinal);

        byte[] data = { 0x08, 0x34, 0x12, 0x00 };
        Assert.True(layout.TryDecode(data, 7, out MotorPositionReply reply));
        Assert.Equal(7, reply.DeviceId);
        Assert.Equal(0x1234, reply.RawPosition);

        // 查询码仍然必须匹配，否则温度/状态/版本回复会被误当成位置。
        byte[] wrongQuery = { 0x06, 0x34, 0x12, 0x00 };
        Assert.False(layout.TryDecode(wrongQuery, 7, out _));
    }

    /// <summary>布局参数非法时必须拒绝构造，避免把错误偏移带到运行期。</summary>
    [Fact]
    public void Layout_WithInvalidOffsets_ShouldThrow()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new MotorPositionReplyLayout(8, 8, 1, 2, 4, true, true, "x"));
        Assert.Throws<ArgumentOutOfRangeException>(() => new MotorPositionReplyLayout(8, -2, 1, 2, 4, true, true, "x"));
        Assert.Throws<ArgumentOutOfRangeException>(() => new MotorPositionReplyLayout(8, 0, 8, 2, 4, true, true, "x"));
        Assert.Throws<ArgumentOutOfRangeException>(() => new MotorPositionReplyLayout(8, 0, 1, 6, 4, true, true, "x"));
        Assert.Throws<ArgumentOutOfRangeException>(() => new MotorPositionReplyLayout(8, 0, 1, 2, 3, true, true, "x"));
        Assert.Throws<ArgumentException>(() => new MotorPositionReplyLayout(8, 0, 1, 2, 4, true, true, " "));
    }

    /// <summary>0x0421 是剩余位移（主/从两路 s32），不是当前位置。</summary>
    [Fact]
    public void RemainingDisplacement_ShouldDecodeTwoChannels()
    {
        byte[] data = new byte[8];
        BinaryPrimitives.WriteInt32LittleEndian(data.AsSpan(0, 4), 1234);
        BinaryPrimitives.WriteInt32LittleEndian(data.AsSpan(4, 4), -56);

        Assert.True(MotorProtocol.TryDecodeRemainingDisplacement(data, out int main, out int slave));
        Assert.Equal(1234, main);
        Assert.Equal(-56, slave);
        Assert.Equal(123.4, MotorProtocol.PositionLsbToMillimeters(main), 6);

        Assert.False(MotorProtocol.TryDecodeRemainingDisplacement(new byte[7], out _, out _));
    }

    /// <summary>错误码段与常见错误码含义来自协议表第 4 节。</summary>
    [Fact]
    public void ErrorCodes_ShouldBeRecognized()
    {
        Assert.True(MotorProtocol.IsErrorId(0x0100));
        Assert.True(MotorProtocol.IsErrorId(0x0104));
        Assert.True(MotorProtocol.IsErrorId(0x017F));
        Assert.False(MotorProtocol.IsErrorId(0x00FF));
        Assert.False(MotorProtocol.IsErrorId(0x0180));

        Assert.Equal("任务未完成，未达到目标点", MotorProtocol.DescribeErrorCode(0x0102));
        Assert.Equal("堵转", MotorProtocol.DescribeErrorCode(0x0104));
        Assert.Equal("超温", MotorProtocol.DescribeErrorCode(0x0105));
        Assert.Equal("欠压", MotorProtocol.DescribeErrorCode(0x0107));
        Assert.Equal("断线", MotorProtocol.DescribeErrorCode(0x0108));
        Assert.Equal("未知错误码", MotorProtocol.DescribeErrorCode(0x0170));
    }

    /// <summary>0x0441 力矩在 Data[2..3]；实测 DLC=8，协议表写 4，两种都应能读出 Data[2..3]。</summary>
    [Fact]
    public void Torque_ShouldReadData2And3()
    {
        byte[] data = { 0x00, 0x00, 0x34, 0x12, 0xAA, 0xBB, 0xCC, 0xDD };
        Assert.True(MotorProtocol.TryDecodeTorque(data, out short torque));
        Assert.Equal(0x1234, torque);

        Assert.False(MotorProtocol.TryDecodeTorque(new byte[3], out _));
    }

    // -------------------------------------------------- 运动控制指令（协议表 V11）

    /// <summary>0x0416：DLC=6，s32 小端目标位置 + 设备号 + 目标类型。</summary>
    [Fact]
    public void BuildTargetPosition_ShouldMatchProtocolTable()
    {
        byte[] payload = MotorProtocol.BuildTargetPosition(785, 0x01, MotorTargetType.Relative);

        Assert.Equal(MotorProtocol.TargetPositionDlc, payload.Length);
        Assert.Equal(new byte[] { 0x11, 0x03, 0x00, 0x00, 0x01, 0x00 }, payload);
        Assert.Equal("t4166110300000100\r", SlcanCodec.EncodeStandardDataFrame(MotorProtocol.TargetPositionId, payload));

        // 负的相对目标（返回行程）必须按二进制补码写进去。
        byte[] backward = MotorProtocol.BuildTargetPosition(-785, 0x01, MotorTargetType.Relative);
        Assert.Equal(new byte[] { 0xEF, 0xFC, 0xFF, 0xFF, 0x01, 0x00 }, backward);

        // 绝对 / 归零 / 设当前位置的目标类型字节。
        Assert.Equal(0x05, MotorProtocol.BuildTargetPosition(0, 1, MotorTargetType.Absolute)[5]);
        Assert.Equal(0x0A, MotorProtocol.BuildTargetPosition(0, 1, MotorTargetType.Zero)[5]);
        Assert.Equal(0x0B, MotorProtocol.BuildTargetPosition(0, 1, MotorTargetType.SetCurrent)[5]);
    }

    /// <summary>0x0413 / 0x0415 / 0x0414 都是 DLC=5，最后 1 字节是设备号。</summary>
    [Fact]
    public void BuildEnableSpeedAcceleration_ShouldMatchProtocolTable()
    {
        Assert.Equal(new byte[] { 0x5A, 0x00, 0x00, 0x00, 0x01 }, MotorProtocol.BuildEnable(0x01, true));
        Assert.Equal(new byte[] { 0x4A, 0x00, 0x00, 0x00, 0x02 }, MotorProtocol.BuildEnable(0x02, false));
        Assert.Equal(
            "t41355A00000001\r",
            SlcanCodec.EncodeStandardDataFrame(MotorProtocol.EnableId, MotorProtocol.BuildEnable(0x01, true)));

        Assert.Equal(new byte[] { 0xE8, 0x03, 0x00, 0x00, 0x01 }, MotorProtocol.BuildSpeed(0x01, 1000));
        Assert.Equal(new byte[] { 0x88, 0x13, 0x00, 0x00, 0x01 }, MotorProtocol.BuildAcceleration(0x01, 5000));

        Assert.Equal(MotorProtocol.SpeedAccelerationDlc, MotorProtocol.BuildSpeed(1, 0).Length);
        Assert.Equal(MotorProtocol.SpeedAccelerationDlc, MotorProtocol.BuildAcceleration(1, 0).Length);
        Assert.Equal(MotorProtocol.EnableDlc, MotorProtocol.BuildEnable(1, true).Length);
    }

    // -------------------------------------------------- 初始化指令（补充资料，不在 V11 表内）

    /// <summary>
    /// 0x0316：DLC=8，前 7 字节固定 B4 BF CF AA 00 5F 01，末字节为设备号。
    /// </summary>
    [Fact]
    public void BuildParameterInit_ShouldUseFixedPrefixPlusDeviceId()
    {
        byte[] payload = MotorProtocol.BuildParameterInit(0x01);

        Assert.Equal(MotorProtocol.ParameterInitDlc, payload.Length);
        Assert.Equal(new byte[] { 0xB4, 0xBF, 0xCF, 0xAA, 0x00, 0x5F, 0x01, 0x01 }, payload);
        Assert.Equal(
            "t3168B4BFCFAA005F0101\r",
            SlcanCodec.EncodeStandardDataFrame(MotorProtocol.ParameterInitId, payload));

        // 换设备号只改末字节，其余 7 字节保持不变。
        byte[] device2 = MotorProtocol.BuildParameterInit(0x02);
        Assert.Equal(payload[..7], device2[..7]);
        Assert.Equal(0x02, device2[7]);
        Assert.Equal("t3168B4BFCFAA005F0102\r", SlcanCodec.EncodeStandardDataFrame(MotorProtocol.ParameterInitId, device2));
    }

    /// <summary>
    /// 0x0408 / 0x24：DLC=4，设备号 + 0x24 + u16 小端轮径（单位 0.01 mm）。
    /// 100 mm → 0x2710（用户给的 <c>01 24 10 27</c>）。
    /// </summary>
    [Theory]
    [InlineData(100.0, 0x10, 0x27)]
    [InlineData(200.0, 0x20, 0x4E)]
    [InlineData(0.01, 0x01, 0x00)]
    public void BuildWheelDiameterConfig_ShouldWriteLittleEndianHundredthsOfMillimeter(
        double diameterMm,
        byte low,
        byte high)
    {
        byte[] payload = MotorProtocol.BuildWheelDiameterConfig(0x01, diameterMm);

        Assert.Equal(MotorProtocol.WheelConfigDlc, payload.Length);
        Assert.Equal(new byte[] { 0x01, MotorProtocol.WheelConfigCode, low, high }, payload);
        Assert.Equal(0x24, payload[1]);
        Assert.Equal("t408401241027\r", SlcanCodec.EncodeStandardDataFrame(
            MotorProtocol.QueryId,
            MotorProtocol.BuildWheelDiameterConfig(0x01, 100.0)));
    }

    /// <summary>轮径原始值是 0.01 mm/LSB，与读取方向互逆；10000 → 100.00 mm。</summary>
    [Fact]
    public void WheelDiameterRaw_ShouldRoundTrip()
    {
        Assert.Equal(10000, MotorProtocol.MillimetersToWheelDiameterRaw(100.0));
        Assert.Equal(100.0, MotorProtocol.WheelDiameterRawToMillimeters(10000), 12);
        Assert.Equal(MotorProtocol.MaxWritableWheelDiameterMm, 655.35, 12);
        Assert.Equal(ushort.MaxValue, MotorProtocol.MillimetersToWheelDiameterRaw(MotorProtocol.MaxWritableWheelDiameterMm));

        // 0.005 mm 以下按四舍五入进位（AwayFromZero），不会悄悄截断成 0。
        Assert.Equal(1, MotorProtocol.MillimetersToWheelDiameterRaw(0.005));
    }

    /// <summary>轮径超范围 / 非有限值必须报错，不能静默写入错误配置。</summary>
    [Theory]
    [InlineData(0.0)]
    [InlineData(-100.0)]
    [InlineData(655.36)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    public void BuildWheelDiameterConfig_WithInvalidDiameter_ShouldThrow(double diameterMm)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => MotorProtocol.BuildWheelDiameterConfig(0x01, diameterMm));
    }

    /// <summary>协议要求配置轮径连发三次。</summary>
    [Fact]
    public void WheelConfigRepeatCount_ShouldBeThree()
    {
        Assert.Equal(3, MotorProtocol.WheelConfigRepeatCount);
    }

    /// <summary>轮径 100 mm 时四个目标角度换算出的位置增量（0.1 mm/单位）。</summary>
    [Theory]
    [InlineData(90, 785)]
    [InlineData(180, 1571)]
    [InlineData(360, 3142)]
    [InlineData(720, 6283)]
    public void DegreesToPositionDelta_ShouldMatchWheelCircumference(int angleDeg, int expectedLsb)
    {
        Assert.Equal(expectedLsb, MotorProtocol.DegreesToPositionDelta(angleDeg, 100.0));

        // 与反向换算互逆：误差不超过 1 个 LSB 对应的角度。
        double back = MotorProtocol.PositionDeltaLsbToDegrees(expectedLsb, 100.0);
        Assert.InRange(back, angleDeg - 0.2, angleDeg + 0.2);
    }

    /// <summary>负角度按对称处理，0° 得 0；轮径非法时报错而不是悄悄算成 0。</summary>
    [Fact]
    public void DegreesToPositionDelta_ShouldHandleEdgeCases()
    {
        Assert.Equal(-785, MotorProtocol.DegreesToPositionDelta(-90, 100.0));
        Assert.Equal(0, MotorProtocol.DegreesToPositionDelta(0, 100.0));

        // 轮径减半 → 同样的角度只走一半的位移。
        Assert.Equal(1571, MotorProtocol.DegreesToPositionDelta(360, 50.0));

        Assert.Throws<ArgumentOutOfRangeException>(() => MotorProtocol.DegreesToPositionDelta(90, 0.0));
        Assert.Throws<ArgumentOutOfRangeException>(() => MotorProtocol.DegreesToPositionDelta(90, double.NaN));
    }

    /// <summary>到位容差换算成 LSB 时至少 1 个 LSB，避免容差 0 导致永远不到位。</summary>
    [Fact]
    public void AngleToleranceToPositionLsb_ShouldBeAtLeastOneLsb()
    {
        Assert.Equal(17, MotorProtocol.AngleToleranceToPositionLsb(2.0, 100.0));
        Assert.Equal(1, MotorProtocol.AngleToleranceToPositionLsb(0.0, 100.0));
        Assert.Equal(1, MotorProtocol.AngleToleranceToPositionLsb(double.NaN, 100.0));
        Assert.Equal(44, MotorProtocol.AngleToleranceToPositionLsb(-5.0, 100.0));
    }

    /// <summary>速度/加速度按 0.1 单位换算；非法值一律 0，不抛异常，避免运动中途崩。</summary>
    [Fact]
    public void SpeedAndAccelerationConversion_ShouldClampInvalidValues()
    {
        Assert.Equal(1000U, MotorProtocol.MillimetersPerSecondToRawSpeed(100.0));
        Assert.Equal(0U, MotorProtocol.MillimetersPerSecondToRawSpeed(-1.0));
        Assert.Equal(0U, MotorProtocol.MillimetersPerSecondToRawSpeed(double.NaN));
        Assert.Equal(0U, MotorProtocol.MillimetersPerSecondToRawSpeed(double.PositiveInfinity));

        Assert.Equal(5000U, MotorProtocol.MillimetersPerSecondSquaredToRawAcceleration(500.0));
        Assert.Equal(0U, MotorProtocol.MillimetersPerSecondSquaredToRawAcceleration(-1.0));
        Assert.Equal(uint.MaxValue, MotorProtocol.MillimetersPerSecondToRawSpeed(1e12));
    }

    /// <summary>目标类型在日志里的中文说明。</summary>
    [Fact]
    public void DescribeTargetType_ShouldCoverAllKinds()
    {
        Assert.Equal("相对当前位置", MotorProtocol.DescribeTargetType(MotorTargetType.Relative));
        Assert.Equal("绝对位置", MotorProtocol.DescribeTargetType(MotorTargetType.Absolute));
        Assert.Equal("归零", MotorProtocol.DescribeTargetType(MotorTargetType.Zero));
        Assert.Equal("设为当前位置", MotorProtocol.DescribeTargetType(MotorTargetType.SetCurrent));
        Assert.Contains("未知", MotorProtocol.DescribeTargetType((MotorTargetType)0x77));
    }
}
