using System.Buffers.Binary;
using System.Text;
using GyroHost.Core.Communication;
using GyroHost.Core.Protocol;
using GyroHost.Core.Services;

namespace GyroHost.Tests;

/// <summary>
/// 电机位置监视的会话层测试：查询、单请求、超时、设备/查询码区分、原始日志、
/// 剩余位移/力矩/心跳不被当成位置、显示归零。
/// <para>
/// 需要“布局已确认”的测试使用 <see cref="MotorProtocolTests.TestLayout"/>（测试夹具）；
/// 默认解码器已按一帧实机报文接入（<see cref="MotorProtocol.CreateDefaultPositionDecoder"/>），
/// 但位置字段长度/符号仍待更多实机报文验证。
/// </para>
/// </summary>
public sealed class MotorSessionTests
{
    private static (SlcanSession Session, FakeCanConnection Connection) CreateSession(
        IMotorPositionDecoder? decoder = null)
    {
        FakeCanConnection connection = new();
        SlcanSession session = decoder is null
            ? new SlcanSession(connection)
            : new SlcanSession(connection, decoder);
        return (session, connection);
    }

    private static void EmitReply(FakeCanConnection connection, byte deviceId, byte queryCode, int rawPosition)
    {
        byte[] data = MotorProtocolTests.BuildTestReply(deviceId, queryCode, rawPosition);
        connection.EmitText(SlcanCodec.EncodeStandardDataFrame(MotorProtocol.ReplyId, data));
    }

    private static void EmitRemainingDisplacement(FakeCanConnection connection, uint id, int main, int slave)
    {
        byte[] data = new byte[8];
        BinaryPrimitives.WriteInt32LittleEndian(data.AsSpan(0, 4), main);
        BinaryPrimitives.WriteInt32LittleEndian(data.AsSpan(4, 4), slave);
        connection.EmitText(SlcanCodec.EncodeStandardDataFrame(id, data));
    }

    /// <summary>
    /// 显式使用“未确认布局”解码器时：查询照发、0x0409 原文照记，但绝不产生位置。
    /// 这条路径仍需保留，用于以后协议改动后安全回退。
    /// </summary>
    [Fact]
    public async Task UnconfirmedDecoder_ShouldLogRawReplyWithoutFakingPosition()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession(UnconfirmedMotorPositionDecoder.Instance);
        using (session)
        {
            await session.ConnectAsync();

            Assert.True(await session.QueryMotorPositionOnceAsync());
            Assert.Equal("t40820108\r", connection.SentLines[^1]);

            EmitReply(connection, 1, MotorProtocol.QueryPositionCode, 1234);

            SessionSnapshot snapshot = session.TakeSnapshot();
            Assert.False(snapshot.Motor.DecoderConfirmed);
            Assert.NotEmpty(snapshot.Motor.MissingReplyFields);
            Assert.False(snapshot.Motor.HasPosition);
            Assert.Equal(0, snapshot.Motor.PositionReplies);
            Assert.Equal(1, snapshot.Motor.AllRepliesReceived);
            Assert.Equal(1, snapshot.Motor.UnconfirmedReplies);
            Assert.Equal(MotorRequestState.WaitingForReply, snapshot.Motor.RequestState);
            Assert.Contains(snapshot.Motor.RawLogLines, line => line.Contains("0x0409", StringComparison.Ordinal));
            Assert.Contains(snapshot.Motor.RawLogLines, line => line.Contains("布局未确认", StringComparison.Ordinal));
        }
    }

    /// <summary>
    /// 默认解码器（按实机报文接入）在会话层解出真实帧：
    /// RX <c>0x0409 [01 08 BC 18 00 00 FC FF]</c> → 位置 6332 → 633.2 mm，
    /// 且第一帧就是 P0（相对角度 0）。本帧不区分 2/4 字节长度，符号仍待验证。
    /// </summary>
    [Fact]
    public async Task DefaultDecoder_ShouldDecodeCapturedRealFrame()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();
        using (session)
        {
            session.MotorPollingEnabled = false;
            await session.ConnectAsync();
            Assert.True(await session.QueryMotorPositionOnceAsync());

            connection.EmitText(SlcanCodec.EncodeStandardDataFrame(
                MotorProtocol.ReplyId,
                new byte[] { 0x01, 0x08, 0xBC, 0x18, 0x00, 0x00, 0xFC, 0xFF }));

            SessionSnapshot snapshot = session.TakeSnapshot();
            Assert.True(snapshot.Motor.DecoderConfirmed);
            Assert.Empty(snapshot.Motor.MissingReplyFields);
            Assert.True(snapshot.Motor.HasPosition);
            Assert.Equal(6332, snapshot.Motor.RawPosition);
            Assert.Equal(633.2, snapshot.Motor.DisplacementMm, 9);
            Assert.Equal(0.0, snapshot.Motor.RelativeAngleDeg, 9);
            Assert.Equal(6332, snapshot.Motor.ReferenceRawPosition);
            Assert.Single(snapshot.Motor.Samples);
            Assert.Equal(1, snapshot.Motor.PositionReplies);
            Assert.Equal(0, snapshot.Motor.UnconfirmedReplies);
        }
    }

    /// <summary>
    /// 重置前已到达、重置后才解析的旧位置回复必须被丢弃，不能更新实时值或 P0；
    /// 重置后新到达的回复仍要正常成为 P0。
    /// </summary>
    [Fact]
    public async Task ResetDisplay_ShouldDropReplyThatArrivedBeforeReset()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession(MotorProtocolTests.TestLayout);
        using (session)
        {
            session.MotorPollingEnabled = false;
            await session.ConnectAsync();
            Assert.True(await session.QueryMotorPositionOnceAsync());

            TimeSpan oldArrival = MonotonicClock.Now;
            session.ResetDisplay();

            byte[] oldReplyData = MotorProtocolTests.BuildTestReply(1, MotorProtocol.QueryPositionCode, 1234);
            string oldReply = SlcanCodec.EncodeStandardDataFrame(MotorProtocol.ReplyId, oldReplyData);
            connection.Emit(Encoding.ASCII.GetBytes(oldReply), oldArrival);

            SessionSnapshot snapshot = session.TakeSnapshot();
            Assert.False(snapshot.Motor.HasPosition);
            Assert.False(snapshot.Motor.HasReference);
            Assert.Equal(0, snapshot.Motor.ReferenceRawPosition);
            Assert.Contains(snapshot.Motor.RawLogLines, line => line.Contains("早于显示重置", StringComparison.Ordinal));

            Assert.True(await session.QueryMotorPositionOnceAsync());
            EmitReply(connection, 1, MotorProtocol.QueryPositionCode, 2000);

            snapshot = session.TakeSnapshot();
            Assert.True(snapshot.Motor.HasPosition);
            Assert.True(snapshot.Motor.HasReference);
            Assert.Equal(2000, snapshot.Motor.ReferenceRawPosition);
        }
    }

    /// <summary>同一时刻只能有一个待应答位置查询，超时后才能发下一条。</summary>
    [Fact]
    public async Task Query_ShouldKeepSingleOutstandingRequest()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();
        using (session)
        {
            session.MotorReplyTimeoutMs = 30;
            await session.ConnectAsync();

            Assert.True(await session.QueryMotorPositionOnceAsync());
            Assert.False(await session.QueryMotorPositionOnceAsync());
            Assert.Single(connection.SentLines);

            await Task.Delay(80);

            Assert.True(await session.QueryMotorPositionOnceAsync());
            SessionSnapshot snapshot = session.TakeSnapshot();
            Assert.Equal(2, snapshot.Motor.QueriesSent);
            Assert.True(snapshot.Motor.Timeouts >= 1);
            Assert.Equal(2, connection.SentLines.Count);
        }
    }

    /// <summary>启用轮询后应按周期自动发送 0x0408，并且不会堆积请求。</summary>
    [Fact]
    public async Task Polling_ShouldSendQueriesWithoutStacking()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();
        using (session)
        {
            session.MotorPollingEnabled = true;
            session.MotorPollIntervalMs = 40;
            session.MotorReplyTimeoutMs = 60;

            await session.ConnectAsync();
            await Task.Delay(300);

            SessionSnapshot snapshot = session.TakeSnapshot();
            Assert.True(snapshot.Motor.QueriesSent >= 2, $"实际查询次数：{snapshot.Motor.QueriesSent}");
            Assert.True(snapshot.Motor.QueriesSent <= 10, $"查询次数过多，疑似请求堆积：{snapshot.Motor.QueriesSent}");
            Assert.True(snapshot.Motor.Timeouts >= 1);
            Assert.All(connection.SentLines, line => Assert.Equal("t40820108\r", line));
        }
    }

    /// <summary>已确认布局：设备号/查询码匹配时，P0 取首帧，后续按轮径换算相对角度。</summary>
    [Fact]
    public async Task ConfirmedProtocol_ShouldDecodePositionAndRelativeAngle()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession(MotorProtocolTests.TestLayout);
        using (session)
        {
            await session.ConnectAsync();

            Assert.True(await session.QueryMotorPositionOnceAsync());
            EmitReply(connection, 1, MotorProtocol.QueryPositionCode, 1000);

            SessionSnapshot first = session.TakeSnapshot();
            Assert.True(first.Motor.DecoderConfirmed);
            Assert.True(first.Motor.HasPosition);
            Assert.True(first.Motor.IsFresh);
            Assert.True(first.Motor.HasReference);
            Assert.Equal(1000, first.Motor.RawPosition);
            Assert.Equal(1000, first.Motor.ReferenceRawPosition);
            Assert.Equal(100.0, first.Motor.DisplacementMm, 6);
            Assert.Equal(0.0, first.Motor.RelativeAngleDeg, 12);
            Assert.Equal(1, first.Motor.PositionReplies);

            Assert.True(await session.QueryMotorPositionOnceAsync());
            EmitReply(connection, 1, MotorProtocol.QueryPositionCode, 1001);

            SessionSnapshot second = session.TakeSnapshot();
            double expected = 0.1 * 360.0 / (Math.PI * 100.0);
            Assert.Equal(expected, second.Motor.RelativeAngleDeg, 9);
            Assert.Single(second.Motor.Samples);
            Assert.Equal(1001, second.Motor.Samples[0].RawPosition);
            Assert.Equal(100.1, second.Motor.Samples[0].DisplacementMm, 6);
        }
    }

    /// <summary>设备号或查询码不匹配的 0x0409 不能清掉待应答请求，也不能当成位置。</summary>
    [Fact]
    public async Task ConfirmedProtocol_ShouldRejectWrongDeviceThenAcceptCorrectReply()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession(MotorProtocolTests.TestLayout);
        using (session)
        {
            await session.ConnectAsync();

            Assert.True(await session.QueryMotorPositionOnceAsync());
            EmitReply(connection, 2, MotorProtocol.QueryPositionCode, 555);

            SessionSnapshot rejected = session.TakeSnapshot();
            Assert.False(rejected.Motor.HasPosition);
            Assert.Equal(1, rejected.Motor.RejectedReplies);
            Assert.Equal(MotorRequestState.WaitingForReply, rejected.Motor.RequestState);

            // 正确的回复仍然被接收，说明错误的 0x0409 没有清掉请求。
            EmitReply(connection, 1, MotorProtocol.QueryPositionCode, 777);
            SessionSnapshot accepted = session.TakeSnapshot();
            Assert.True(accepted.Motor.HasPosition);
            Assert.Equal(777, accepted.Motor.RawPosition);
            Assert.Equal(MotorRequestState.Received, accepted.Motor.RequestState);
        }
    }

    /// <summary>温度/状态/版本回复共用 0x0409，查询码 0x06 不能被当位置。</summary>
    [Fact]
    public async Task ConfirmedProtocol_ShouldRejectNonPositionQueryCode()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession(MotorProtocolTests.TestLayout);
        using (session)
        {
            await session.ConnectAsync();

            Assert.True(await session.QueryMotorPositionOnceAsync());
            EmitReply(connection, 1, MotorProtocol.QueryTemperatureCode, 888);

            SessionSnapshot snapshot = session.TakeSnapshot();
            Assert.False(snapshot.Motor.HasPosition);
            Assert.Equal(1, snapshot.Motor.RejectedReplies);
            Assert.Equal(MotorRequestState.WaitingForReply, snapshot.Motor.RequestState);
        }
    }

    /// <summary>应答超时后必须报告超时，不能用旧值冒充有效。</summary>
    [Fact]
    public async Task ReplyTimeout_ShouldBeReported()
    {
        (SlcanSession session, _) = CreateSession(MotorProtocolTests.TestLayout);
        using (session)
        {
            session.MotorReplyTimeoutMs = 30;
            await session.ConnectAsync();

            Assert.True(await session.QueryMotorPositionOnceAsync());
            await Task.Delay(80);

            SessionSnapshot snapshot = session.TakeSnapshot();
            Assert.Equal(MotorRequestState.TimedOut, snapshot.Motor.RequestState);
            Assert.True(snapshot.Motor.Timeouts >= 1);
            Assert.False(snapshot.Motor.IsFresh);
            Assert.Contains(snapshot.LogLines, line => line.Contains("超时", StringComparison.Ordinal));
        }
    }

    /// <summary>位置反馈中断超过数据超时后失效，恢复时曲线必须断开。</summary>
    [Fact]
    public async Task DataTimeout_ShouldMarkInvalidAndBreakCurveOnRecovery()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession(MotorProtocolTests.TestLayout);
        using (session)
        {
            session.MotorDataTimeoutMs = 100;
            session.MotorReplyTimeoutMs = 500;
            await session.ConnectAsync();

            Assert.True(await session.QueryMotorPositionOnceAsync());
            EmitReply(connection, 1, MotorProtocol.QueryPositionCode, 100);
            session.TakeSnapshot();

            await Task.Delay(180);
            SessionSnapshot stale = session.TakeSnapshot();
            Assert.True(stale.Motor.HasPosition);
            Assert.False(
                stale.Motor.IsFresh,
                $"last={stale.Motor.LastPositionArrival}, now={MonotonicClock.Now}, timeout={stale.Motor.DataTimeoutMs}, state={stale.Motor.RequestState}");

            Assert.True(await session.QueryMotorPositionOnceAsync());
            EmitReply(connection, 1, MotorProtocol.QueryPositionCode, 200);

            SessionSnapshot recovered = session.TakeSnapshot();
            Assert.True(recovered.Motor.IsFresh);
            Assert.Single(recovered.Motor.Samples);
            Assert.True(recovered.Motor.Samples[0].BreakBefore);
            Assert.Equal(
                MotorProtocol.PositionDeltaLsbToDegrees(100, 100.0),
                recovered.Motor.Samples[0].RelativeAngleDeg,
                9);
        }
    }

    /// <summary>“电机显示归零”只改 P0、清空电机曲线并重新计时，不发任何 CAN 命令。</summary>
    [Fact]
    public async Task ZeroMotorDisplay_ShouldOnlyChangeP0AndRestartMotorTimeline()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession(MotorProtocolTests.TestLayout);
        using (session)
        {
            await session.ConnectAsync();

            Assert.True(await session.QueryMotorPositionOnceAsync());
            EmitReply(connection, 1, MotorProtocol.QueryPositionCode, 500);
            session.TakeSnapshot();

            Assert.True(await session.QueryMotorPositionOnceAsync());
            EmitReply(connection, 1, MotorProtocol.QueryPositionCode, 510);
            SessionSnapshot beforeZero = session.TakeSnapshot();
            double expected = MotorProtocol.PositionDeltaLsbToDegrees(10, 100.0);
            Assert.Equal(expected, beforeZero.Motor.RelativeAngleDeg, 9);

            int sentBeforeZero = connection.SentLines.Count;
            Assert.True(session.ZeroMotorDisplay());
            Assert.Equal(sentBeforeZero, connection.SentLines.Count);

            Assert.True(await session.QueryMotorPositionOnceAsync());
            EmitReply(connection, 1, MotorProtocol.QueryPositionCode, 510);

            SessionSnapshot afterZero = session.TakeSnapshot();
            Assert.Equal(510, afterZero.Motor.ReferenceRawPosition);
            Assert.Equal(0.0, afterZero.Motor.RelativeAngleDeg, 12);
            Assert.Single(afterZero.Motor.Samples);
            Assert.Equal(0.0, afterZero.Motor.Samples[0].RelativeAngleDeg, 12);
            Assert.InRange(afterZero.Motor.Samples[0].TimeSeconds, 0.0, 1.0);
        }
    }

    /// <summary>没有有效位置时“显示归零”必须返回 false，不能凭空建立 P0。</summary>
    [Fact]
    public async Task ZeroMotorDisplay_WithoutValidPosition_ShouldReturnFalse()
    {
        (SlcanSession session, _) = CreateSession(MotorProtocolTests.TestLayout);
        using (session)
        {
            await session.ConnectAsync();
            Assert.False(session.ZeroMotorDisplay());
        }
    }

    /// <summary>0x0100..0x017F 是错误上报，必须识别并显示协议表含义，不能算“未知帧”。</summary>
    [Fact]
    public void ErrorFrame_ShouldBeRecognizedAndDescribed()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();
        using (session)
        {
            connection.EmitText(SlcanCodec.EncodeStandardDataFrame(
                MotorProtocol.ErrorIdMin + 0x04,
                new byte[] { 0x01, 0x04 }));

            SessionSnapshot snapshot = session.TakeSnapshot();
            Assert.False(snapshot.Motor.HasPosition);
            Assert.Equal(1, snapshot.Motor.ErrorFrames);
            Assert.True(snapshot.Motor.LastErrorCode.HasValue);
            Assert.Equal((ushort)0x0104, snapshot.Motor.LastErrorCode.Value);
            Assert.Equal(0, snapshot.Counters.UnsupportedFrames);
            Assert.Contains(snapshot.Motor.RawLogLines, line => line.Contains("堵转", StringComparison.Ordinal));
            Assert.Contains(snapshot.LogLines, line => line.Contains("0x0104", StringComparison.Ordinal));
        }
    }

    /// <summary>0x0421 是剩余位移，必须单独记录，不能当成当前位置。</summary>
    [Fact]
    public async Task RemainingDisplacementFrame_ShouldNotBecomePosition()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();
        using (session)
        {
            EmitRemainingDisplacement(connection, MotorProtocol.RemainingDisplacementIdBase + 1, 1234, -56);

            SessionSnapshot snapshot = session.TakeSnapshot();
            Assert.False(snapshot.Motor.HasPosition);
            Assert.True(snapshot.Motor.HasRemainingDisplacement);
            Assert.Equal(1, snapshot.Motor.RemainingDisplacementFrames);
            Assert.Equal(1234, snapshot.Motor.RemainingDisplacementRaw);
            Assert.Equal(123.4, snapshot.Motor.RemainingDisplacementMm, 6);
            Assert.Equal(0, snapshot.Counters.UnsupportedFrames);
            Assert.Contains(snapshot.Motor.RawLogLines, line => line.Contains("剩余位移", StringComparison.Ordinal));

            EmitRemainingDisplacement(connection, MotorProtocol.RemainingDisplacementIdBase + 2, 999, 0);
            SessionSnapshot otherDevice = session.TakeSnapshot();
            Assert.Equal(1, otherDevice.Motor.RemainingDisplacementFrames);
            Assert.Contains(otherDevice.Motor.RawLogLines, line => line.Contains("其它设备", StringComparison.Ordinal));
        }
    }

    /// <summary>0x0441 力矩反馈：实测 DLC=8，协议表写 4，两者都读取 Data[2..3]；它也不是位置。</summary>
    [Fact]
    public void TorqueFrame_ShouldAcceptMeasuredDlcAndNotBecomePosition()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();
        using (session)
        {
            connection.EmitText(SlcanCodec.EncodeStandardDataFrame(
                MotorProtocol.TorqueFeedbackIdBase + 1,
                new byte[] { 0, 0, 0x34, 0x12, 0xAA, 0xBB, 0xCC, 0xDD }));
            connection.EmitText(SlcanCodec.EncodeStandardDataFrame(
                MotorProtocol.TorqueFeedbackIdBase + 1,
                new byte[] { 0, 0, 0x78, 0x56 }));

            SessionSnapshot snapshot = session.TakeSnapshot();
            Assert.False(snapshot.Motor.HasPosition);
            Assert.Equal(2, snapshot.Motor.TorqueFrames);
            Assert.Equal(0, snapshot.Counters.BadDlcFrames);

            connection.EmitText(SlcanCodec.EncodeStandardDataFrame(
                MotorProtocol.TorqueFeedbackIdBase + 1,
                new byte[] { 0, 0 }));
            Assert.Equal(1, session.TakeSnapshot().Counters.BadDlcFrames);
        }
    }

    /// <summary>0x0381 心跳只计数，不能被当成位置。</summary>
    [Fact]
    public void HeartbeatFrame_ShouldNotBecomePosition()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();
        using (session)
        {
            connection.EmitText(SlcanCodec.EncodeStandardDataFrame(
                MotorProtocol.HeartbeatIdBase + 1,
                new byte[] { 0x01, 0x02 }));

            SessionSnapshot snapshot = session.TakeSnapshot();
            Assert.False(snapshot.Motor.HasPosition);
            Assert.Equal(1, snapshot.Motor.HeartbeatFrames);
            Assert.Equal(0, snapshot.Counters.UnsupportedFrames);
        }
    }

    /// <summary>收到 0x0408 说明是查询方向/回显，不能当位置反馈。</summary>
    [Fact]
    public void QueryEcho_ShouldNotBecomePosition()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession();
        using (session)
        {
            connection.EmitText("t40820108\r");

            SessionSnapshot snapshot = session.TakeSnapshot();
            Assert.False(snapshot.Motor.HasPosition);
            Assert.Equal(1, snapshot.Motor.OtherQueryFrames);
            Assert.Equal(0, snapshot.Motor.AllRepliesReceived);
            Assert.Equal(0, snapshot.Counters.UnsupportedFrames);
        }
    }

    /// <summary>修改设备号会重置 P0 和统计，下一条回复按新设备解码。</summary>
    [Fact]
    public async Task ChangingDeviceId_ShouldResetReferenceAndUseNewIdInQuery()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession(MotorProtocolTests.TestLayout);
        using (session)
        {
            await session.ConnectAsync();

            Assert.True(await session.QueryMotorPositionOnceAsync());
            EmitReply(connection, 1, MotorProtocol.QueryPositionCode, 1000);
            session.TakeSnapshot();

            session.MotorDeviceId = 2;
            Assert.True(await session.QueryMotorPositionOnceAsync());
            Assert.Equal("t40820208\r", connection.SentLines[^1]);

            EmitReply(connection, 2, MotorProtocol.QueryPositionCode, 500);
            SessionSnapshot snapshot = session.TakeSnapshot();
            Assert.Equal(500, snapshot.Motor.ReferenceRawPosition);
            Assert.Equal(0.0, snapshot.Motor.RelativeAngleDeg, 12);
            Assert.Equal(1, snapshot.Motor.QueriesSent);
        }
    }

    /// <summary>设备号必须限制在反馈 ID 低 4 位范围内，避免和相邻反馈段重叠。</summary>
    [Fact]
    public void DeviceId_OutOfRange_ShouldThrow()
    {
        (SlcanSession session, _) = CreateSession();
        using (session)
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => session.MotorDeviceId = 0);
            Assert.Throws<ArgumentOutOfRangeException>(() => session.MotorDeviceId = 16);
        }
    }

    /// <summary>轮径只影响后续换算，不改写电机配置；改大轮径后同样位置增量对应更小角度。</summary>
    [Fact]
    public async Task ChangingWheelDiameter_ShouldAffectAngleConversion()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession(MotorProtocolTests.TestLayout);
        using (session)
        {
            await session.ConnectAsync();

            Assert.True(await session.QueryMotorPositionOnceAsync());
            EmitReply(connection, 1, MotorProtocol.QueryPositionCode, 1000);
            session.TakeSnapshot();

            session.MotorWheelDiameterMm = 200.0;

            Assert.True(await session.QueryMotorPositionOnceAsync());
            EmitReply(connection, 1, MotorProtocol.QueryPositionCode, 1150);

            SessionSnapshot snapshot = session.TakeSnapshot();
            Assert.Equal(150, snapshot.Motor.RawPosition - snapshot.Motor.ReferenceRawPosition);
            Assert.Equal(
                MotorProtocol.PositionDeltaLsbToDegrees(150, 200.0),
                snapshot.Motor.RelativeAngleDeg,
                9);
        }
    }

    // -------------------------------------------------- 运动控制指令（0x0413/0x0416）

    /// <summary>运动命令要真的按协议帧发到总线上：0x0413 / 0x0415 / 0x0414 / 0x0416。</summary>
    [Fact]
    public async Task MotionCommands_ShouldEncodeProtocolFrames()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession(MotorProtocolTests.TestLayout);
        using (session)
        {
            session.MotorPollingEnabled = false;
            await session.ConnectAsync();
            int baseline = connection.SentLines.Count;

            await session.SendMotorEnableAsync(true);
            await session.SendMotorSpeedAsync(1000);
            await session.SendMotorAccelerationAsync(5000);
            await session.SendMotorTargetPositionAsync(785, MotorTargetType.Relative);

            Assert.Equal(
                new[]
                {
                    "t41355A00000001\r",
                    "t4155E803000001\r",
                    "t41458813000001\r",
                    "t4166110300000100\r",
                },
                connection.SentLines.Skip(baseline).ToArray());

            SessionSnapshot snapshot = session.TakeSnapshot();
            Assert.Equal(1, snapshot.Motor.TargetCommandsSent);

            // 发送的每一帧都要写进原始日志，便于实机核对。
            Assert.Contains(snapshot.Motor.RawLogLines, line => line.Contains("TX 0x0416", StringComparison.Ordinal));
            Assert.Contains(snapshot.Motor.RawLogLines, line => line.Contains("设置目标位置", StringComparison.Ordinal));
        }
    }

    /// <summary>未连接时发送运动命令必须报错，不能悄悄丢掉。</summary>
    [Fact]
    public async Task MotionCommands_WithoutConnection_ShouldThrow()
    {
        (SlcanSession session, _) = CreateSession();
        using (session)
        {
            await Assert.ThrowsAsync<InvalidOperationException>(
                () => session.SendMotorTargetPositionAsync(100, MotorTargetType.Relative));
        }
    }

    // -------------------------------------------------- 初始化指令（0x0316 / 0x0408 配轮径 / 0x0413）

    /// <summary>
    /// 初始化帧必须原样发到总线：0x0316 一次、0x0408 配轮径连发三次、0x0413 使能/释放各一次。
    /// </summary>
    [Fact]
    public async Task InitCommands_ShouldEncodeProtocolFrames()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession(MotorProtocolTests.TestLayout);
        using (session)
        {
            session.MotorPollingEnabled = false;
            await session.ConnectAsync();
            int baseline = connection.SentLines.Count;

            await session.SendMotorParameterInitAsync();
            await session.SendMotorWheelDiameterConfigAsync(100.0);
            await session.SendMotorEnableAsync(true);
            await session.SendMotorEnableAsync(false);

            Assert.Equal(
                new[]
                {
                    "t3168B4BFCFAA005F0101\r",
                    "t408401241027\r",
                    "t408401241027\r",
                    "t408401241027\r",
                    "t41355A00000001\r",
                    "t41354A00000001\r",
                },
                connection.SentLines.Skip(baseline).ToArray());

            SessionSnapshot snapshot = session.TakeSnapshot();
            Assert.Contains(snapshot.Motor.RawLogLines, line => line.Contains("TX 0x0316", StringComparison.Ordinal));
            Assert.Contains(snapshot.Motor.RawLogLines, line => line.Contains("参数初始化", StringComparison.Ordinal));

            // 连发的 3 条配置轮径都要各自留下日志（便于实机核对次数）。
            Assert.Equal(
                3,
                snapshot.Motor.RawLogLines.Count(line =>
                    line.Contains("TX 0x0408", StringComparison.Ordinal) &&
                    line.Contains("配置轮径", StringComparison.Ordinal)));
        }
    }

    /// <summary>换设备号后，初始化帧的设备号字节必须跟着变（0x0316 末字节 / 0x0408 Data[0]）。</summary>
    [Fact]
    public async Task InitCommands_ShouldUseCurrentDeviceId()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession(MotorProtocolTests.TestLayout);
        using (session)
        {
            session.MotorPollingEnabled = false;
            await session.ConnectAsync();
            session.MotorDeviceId = 0x02;
            await Task.Delay(60);
            int baseline = connection.SentLines.Count;

            await session.SendMotorParameterInitAsync();
            await session.SendMotorWheelDiameterConfigAsync(200.0);

            Assert.Equal(
                new[]
                {
                    "t3168B4BFCFAA005F0102\r",
                    "t40840224204E\r",
                    "t40840224204E\r",
                    "t40840224204E\r",
                },
                connection.SentLines.Skip(baseline).ToArray());
        }
    }

    /// <summary>轮径超出可写入范围时必须报错，不能把错误配置写到电机上。</summary>
    [Fact]
    public async Task WheelConfig_WithInvalidDiameter_ShouldThrowWithoutSending()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession(MotorProtocolTests.TestLayout);
        using (session)
        {
            session.MotorPollingEnabled = false;
            await session.ConnectAsync();
            int baseline = connection.SentLines.Count;

            await Assert.ThrowsAsync<ArgumentOutOfRangeException>(
                () => session.SendMotorWheelDiameterConfigAsync(700.0));

            Assert.Equal(baseline, connection.SentLines.Count);
        }
    }

    /// <summary>
    /// 电机命令回显（0x0316 / 0x0413..0x0416）不能被当成未知 ID 刷日志，也不参与位置解码。
    /// </summary>
    [Fact]
    public async Task MotorCommandEcho_ShouldBeReportedAsEchoNotUnknown()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession(MotorProtocolTests.TestLayout);
        using (session)
        {
            session.MotorPollingEnabled = false;
            await session.ConnectAsync();
            connection.EmitText("t3168B4BFCFAA005F0101\r");
            connection.EmitText("t41355A00000001\r");
            await Task.Delay(120);

            SessionSnapshot snapshot = session.TakeSnapshot();
            Assert.Equal(0, snapshot.Counters.UnsupportedFrames);
            Assert.DoesNotContain(snapshot.LogLines, line => line.Contains("[未知 ID]", StringComparison.Ordinal));
            Assert.Contains(snapshot.Motor.RawLogLines, line => line.Contains("命令回显", StringComparison.Ordinal));
            Assert.False(snapshot.Motor.HasPosition);
        }
    }

    /// <summary>
    /// 0x041A 参数设置 ACK：只计数 + 记原始字节；回显 ID 按小端解释并与已发命令匹配时才记下来。
    /// </summary>
    [Fact]
    public async Task ParameterAck_ShouldBeCountedWithoutAssumingSuccess()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession(MotorProtocolTests.TestLayout);
        using (session)
        {
            session.MotorPollingEnabled = false;
            await session.ConnectAsync();
            await session.SendMotorTargetPositionAsync(785, MotorTargetType.Relative);

            byte[] ack = { 0x16, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
            connection.EmitText(SlcanCodec.EncodeStandardDataFrame(MotorProtocol.ParameterAckId, ack));

            SessionSnapshot snapshot = session.TakeSnapshot();
            Assert.Equal(1, snapshot.Motor.ParameterAcks);
            Assert.Equal(0x0416U, snapshot.Motor.LastAckEchoId);
            Assert.NotNull(snapshot.Motor.LastAckArrival);
            Assert.Contains(snapshot.Motor.RawLogLines, line => line.Contains("参数设置 ACK", StringComparison.Ordinal));

            // 回显 ID 对不上任何已知命令时，只记原始字节，不硬猜。
            byte[] unknown = { 0x99, 0x88 };
            connection.EmitText(SlcanCodec.EncodeStandardDataFrame(MotorProtocol.ParameterAckId, unknown));

            snapshot = session.TakeSnapshot();
            Assert.Equal(2, snapshot.Motor.ParameterAcks);
            Assert.Null(snapshot.Motor.LastAckEchoId);
        }
    }

    /// <summary>
    /// 断线续接：<see cref="MotorResumeState"/> 给出上一段的 P0 后，重连的第一帧位置不能再当新起点，
    /// 电机曲线必须和上一段画在同一根零点上。
    /// </summary>
    [Fact]
    public async Task Resume_ShouldKeepPreviousReferenceP0()
    {
        FakeCanConnection connection = new();
        SlcanSession session = new(
            connection,
            MotorProtocolTests.TestLayout,
            new MotorResumeState(TimeOffsetSeconds: 42.5, MotorGeneration: 7, ReferenceRawPosition: 1000));

        using (session)
        {
            session.MotorPollingEnabled = false;
            await session.ConnectAsync();

            Assert.True(await session.QueryMotorPositionOnceAsync());
            EmitReply(connection, 1, MotorProtocol.QueryPositionCode, 1150);

            SessionSnapshot snapshot = session.TakeSnapshot();
            Assert.True(snapshot.Motor.HasReference);
            Assert.Equal(1000, snapshot.Motor.ReferenceRawPosition);
            Assert.Equal(150, snapshot.Motor.RawPosition - snapshot.Motor.ReferenceRawPosition);
            Assert.Equal(MotorProtocol.PositionDeltaLsbToDegrees(150, 100.0), snapshot.Motor.RelativeAngleDeg, 9);

            // 时间轴从 42.5 s 接着走（第一个点正好落在上一段的最后一点上，不留空档）。
            Assert.Single(snapshot.Motor.Samples);
            Assert.Equal(42.5, snapshot.Motor.Samples[0].ChartTimeSeconds, 6);
            Assert.True(snapshot.Motor.Samples[0].BreakBefore);

            // 批次号不变：界面据此保留旧曲线。
            Assert.Equal(7, snapshot.Motor.Generation);
        }
    }

    /// <summary>重连时没有给 P0（上一段没建立过），第一帧位置照旧当新起点。</summary>
    [Fact]
    public async Task ResumeWithoutReference_ShouldEstablishP0FromFirstReply()
    {
        FakeCanConnection connection = new();
        SlcanSession session = new(
            connection,
            MotorProtocolTests.TestLayout,
            new MotorResumeState(TimeOffsetSeconds: 10.0, MotorGeneration: 2, ReferenceRawPosition: null));

        using (session)
        {
            session.MotorPollingEnabled = false;
            await session.ConnectAsync();

            Assert.True(await session.QueryMotorPositionOnceAsync());
            EmitReply(connection, 1, MotorProtocol.QueryPositionCode, 4321);

            SessionSnapshot snapshot = session.TakeSnapshot();
            Assert.True(snapshot.Motor.HasReference);
            Assert.Equal(4321, snapshot.Motor.ReferenceRawPosition);
            Assert.Equal(0.0, snapshot.Motor.RelativeAngleDeg, 9);
            Assert.Equal(2, snapshot.Motor.Generation);
        }
    }

    /// <summary>“清零”后 P0 要重新建立：清参考点后下一帧位置成为新的 0°。</summary>
    [Fact]
    public async Task ResetMotorReference_ShouldRebuildP0FromNextReply()
    {
        (SlcanSession session, FakeCanConnection connection) = CreateSession(MotorProtocolTests.TestLayout);
        using (session)
        {
            session.MotorPollingEnabled = false;
            await session.ConnectAsync();

            Assert.True(await session.QueryMotorPositionOnceAsync());
            EmitReply(connection, 1, MotorProtocol.QueryPositionCode, 1000);
            Assert.Equal(1000, session.TakeSnapshot().Motor.ReferenceRawPosition);

            session.ResetMotorReference();
            Assert.True(await session.QueryMotorPositionOnceAsync());
            EmitReply(connection, 1, MotorProtocol.QueryPositionCode, 1600);

            SessionSnapshot snapshot = session.TakeSnapshot();
            Assert.Equal(1600, snapshot.Motor.ReferenceRawPosition);
            Assert.Equal(0.0, snapshot.Motor.RelativeAngleDeg, 9);
        }
    }
}
