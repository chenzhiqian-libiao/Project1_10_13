using GyroHost.Core.Protocol;
using GyroHost.Core.Services;

namespace GyroHost.Tests;

/// <summary>
/// 电机往复运动状态机的确定性测试：这里用“瞬时到位”的假电机（收到 0x0416 就地移动），
/// 因此不涉及真实硬件、也不假装实机行为已验证。
/// </summary>
public sealed class MotorMotionControllerTests
{
    private const double WheelDiameterMm = 100.0;

    [Theory]
    [InlineData(90)]
    [InlineData(180)]
    [InlineData(360)]
    [InlineData(720)]
    public void AcceptanceRun_PreservesCalibrationAcrossAllStops(int target)
    {
        var (controller, motor, now) = StartThreeRounds(target, 3);
        Assert.False(controller.RecalibrateOnStop);
        for (int i = 0; controller.IsRunning && i < 1000; i++)
            Step(controller, motor, ref now);
        Assert.Equal(MotorMotionState.Completed, controller.State);
        Assert.Equal(3, controller.CompletedRounds);
        Assert.Equal(controller.StartRawPosition, motor.Position);
        Assert.DoesNotContain(motor.Commands, c => c.Kind == MotorCommandKind.RecalibrateGyro);
        Assert.Equal(6, motor.TargetCommandCount);
        double actualTolerance = MotorProtocol.PositionDeltaLsbToDegrees(
            MotorProtocol.AngleToleranceToPositionLsb(controller.ToleranceDeg, WheelDiameterMm), WheelDiameterMm);
        Assert.True(actualTolerance < 0.5);
    }

    /// <summary>假电机：收到相对目标就立刻移动到位。</summary>
    private sealed class FakeMotor
    {
        public int Position { get; set; }

        public List<MotorCommand> Commands { get; } = new();

        public int TargetCommandCount => Commands.Count(command => command.Kind == MotorCommandKind.TargetPosition);

        public void Apply(IReadOnlyList<MotorCommand> commands)
        {
            foreach (MotorCommand command in commands)
            {
                Commands.Add(command);
                if (command.Kind == MotorCommandKind.TargetPosition)
                {
                    Position += command.RawPosition;
                }
            }
        }
    }

    private static (MotorMotionController Controller, FakeMotor Motor, TimeSpan Now) StartThreeRounds(
        int angleDeg = 90,
        int rounds = 3)
    {
        MotorMotionController controller = new();
        FakeMotor motor = new() { Position = 1000 };
        TimeSpan now = TimeSpan.FromMilliseconds(100);

        Assert.True(controller.Start(
            now,
            angleDeg,
            rounds,
            WheelDiameterMm,
            hasFreshPosition: true,
            rawPosition: motor.Position,
            out string? error));
        Assert.Null(error);

        return (controller, motor, now);
    }

    /// <summary>推进状态机；<paramref name="positionOverride"/> 可模拟“电机没动到位”或回弹。</summary>
    private static void Step(
        MotorMotionController controller,
        FakeMotor motor,
        ref TimeSpan now,
        Func<TimeSpan, MotorMotionController, int?>? positionOverride = null,
        Action<MotorMotionController>? afterTick = null)
    {
        now += TimeSpan.FromMilliseconds(100);
        int position = positionOverride?.Invoke(now, controller) ?? motor.Position;
        motor.Apply(controller.Tick(now, true, position, now));
        afterTick?.Invoke(controller);
    }

    /// <summary>3 轮“正转 → 返回”后必须回到起点，且只发 3 条配置命令 + 6 条目标命令。</summary>
    [Fact]
    public void ThreeRounds_ShouldReturnToStartAndComplete()
    {
        (MotorMotionController controller, FakeMotor motor, TimeSpan now) = StartThreeRounds();

        int guard = 0;
        while (controller.IsRunning && guard++ < 500)
        {
            Step(controller, motor, ref now);
        }

        Assert.Equal(MotorMotionState.Completed, controller.State);
        Assert.Equal(3, controller.CompletedRounds);
        Assert.Equal(controller.StartRawPosition, motor.Position);
        Assert.Equal(785, controller.SegmentDeltaLsb);
        Assert.Equal(0, controller.CommandedOffsetLsb);

        Assert.Equal(1, motor.Commands.Count(c => c.Kind == MotorCommandKind.Enable));
        Assert.Equal(1, motor.Commands.Count(c => c.Kind == MotorCommandKind.SetSpeed));
        Assert.Equal(1, motor.Commands.Count(c => c.Kind == MotorCommandKind.SetAcceleration));
        Assert.Equal(6, motor.TargetCommandCount);

        // 正转 +785、返回 −785 交替出现，最后一段一定是返回。
        int[] targets = motor.Commands
            .Where(c => c.Kind == MotorCommandKind.TargetPosition)
            .Select(c => c.RawPosition)
            .ToArray();
        Assert.Equal(new[] { 785, -785, 785, -785, 785, -785 }, targets);

        // 使能是“锁定”（0x5A），速度/加速度按 0.1 单位换算。
        Assert.True(motor.Commands[0].Enable);
        Assert.Equal(1000U, motor.Commands[1].RawValue);
        Assert.Equal(5000U, motor.Commands[2].RawValue);
    }

    /// <summary>720° 时每段位置增量翻倍（4 个 90°），仍然回到起点。</summary>
    [Fact]
    public void LargeAngle_ShouldScaleSegmentAndStillReturnToStart()
    {
        (MotorMotionController controller, FakeMotor motor, TimeSpan now) = StartThreeRounds(angleDeg: 720, rounds: 1);

        int guard = 0;
        while (controller.IsRunning && guard++ < 500)
        {
            Step(controller, motor, ref now);
        }

        Assert.Equal(MotorMotionState.Completed, controller.State);
        Assert.Equal(6283, controller.SegmentDeltaLsb);
        Assert.Equal(controller.StartRawPosition, motor.Position);
        Assert.Equal(2, motor.TargetCommandCount);
    }

    /// <summary>位置反馈一直中断（&gt;600 ms）时必须判失败并停止发命令，不能当成到位。</summary>
    [Fact]
    public void PositionLoss_ShouldFailAndStopSendingCommands()
    {
        (MotorMotionController controller, FakeMotor motor, TimeSpan now) = StartThreeRounds(rounds: 1);

        // 先把配置与正转命令发出去，然后不再给新鲜位置。
        for (int i = 0; i < 3; i++)
        {
            Step(controller, motor, ref now);
        }

        int commandsBefore = motor.Commands.Count;
        for (int i = 0; i < 20; i++)
        {
            now += TimeSpan.FromMilliseconds(100);
            motor.Apply(controller.Tick(now, false, motor.Position, null));
        }

        Assert.Equal(MotorMotionState.Failed, controller.State);
        Assert.Contains("位置反馈中断", controller.FailureReason, StringComparison.Ordinal);
        Assert.Equal(commandsBefore, motor.Commands.Count);
        Assert.Equal(0, controller.CompletedRounds);
    }

    /// <summary>位置一直新鲜但电机就是不动：单段 30 s 超时后判失败。</summary>
    [Fact]
    public void StuckMotor_ShouldTimeoutAfterSegmentLimit()
    {
        (MotorMotionController controller, FakeMotor motor, TimeSpan now) = StartThreeRounds(rounds: 1);

        int guard = 0;
        while (controller.IsRunning && guard++ < 1000)
        {
            Step(controller, motor, ref now, positionOverride: (_, _) => 1000);
        }

        Assert.Equal(MotorMotionState.Failed, controller.State);
        Assert.Contains("未到位", controller.FailureReason, StringComparison.Ordinal);
        Assert.True(now.TotalSeconds > MotorMotionController.DefaultSegmentTimeoutSeconds);
    }

    /// <summary>停止后回弹（偏离起点超过容差）时，只发一次相对修正，修正到位就算完成。</summary>
    [Fact]
    public void EndDrift_ShouldSendSingleCorrection()
    {
        (MotorMotionController controller, FakeMotor motor, TimeSpan now) = StartThreeRounds(rounds: 1);

        bool drifted = false;
        int guard = 0;
        while (controller.IsRunning && guard++ < 500)
        {
            Step(controller, motor, ref now, afterTick: state =>
            {
                if (drifted || state.Phase != MotorMotionPhase.VerifyEnd)
                {
                    return;
                }

                int tolerance = MotorProtocol.AngleToleranceToPositionLsb(
                    MotorMotionController.DefaultToleranceDeg,
                    WheelDiameterMm);

                // 模拟停车后机械回弹：偏离起点 2 倍容差（超出判据）。
                motor.Position = state.StartRawPosition + (2 * tolerance);
                drifted = true;
            });
        }

        Assert.True(drifted);
        Assert.Equal(MotorMotionState.Completed, controller.State);
        Assert.Equal(controller.StartRawPosition, motor.Position);
        Assert.Equal(3, motor.TargetCommandCount);

        MotorCommand correction = motor.Commands.Last(c => c.Kind == MotorCommandKind.TargetPosition);
        Assert.True(correction.RawPosition < 0);
    }

    /// <summary>取消后不再产生任何命令，状态为 Cancelled。</summary>
    [Fact]
    public void Cancel_ShouldStopProducingCommands()
    {
        (MotorMotionController controller, FakeMotor motor, TimeSpan now) = StartThreeRounds();

        for (int i = 0; i < 4; i++)
        {
            Step(controller, motor, ref now);
        }

        controller.Cancel();
        Assert.Equal(MotorMotionState.Cancelled, controller.State);
        Assert.Contains("已取消", controller.StatusText, StringComparison.Ordinal);

        int commandsBefore = motor.Commands.Count;
        for (int i = 0; i < 10; i++)
        {
            Step(controller, motor, ref now);
        }

        Assert.Equal(commandsBefore, motor.Commands.Count);
        Assert.Equal(MotorMotionState.Cancelled, controller.State);
    }

    /// <summary>没有新鲜位置 / 参数非法 / 正在运行 时都必须拒绝启动，并给出可读原因。</summary>
    [Fact]
    public void Start_ShouldRejectInvalidInput()
    {
        MotorMotionController controller = new();

        Assert.False(controller.Start(TimeSpan.Zero, 90, 3, WheelDiameterMm, false, 0, out string? noPosition));
        Assert.Contains("位置反馈", noPosition, StringComparison.Ordinal);

        Assert.False(controller.Start(TimeSpan.Zero, 0, 3, WheelDiameterMm, true, 0, out string? zeroAngle));
        Assert.Contains("角度", zeroAngle, StringComparison.Ordinal);

        Assert.False(controller.Start(TimeSpan.Zero, 90, 0, WheelDiameterMm, true, 0, out string? zeroRounds));
        Assert.Contains("轮数", zeroRounds, StringComparison.Ordinal);

        Assert.True(controller.Start(TimeSpan.Zero, 90, 3, WheelDiameterMm, true, 0, out _));

        Assert.False(controller.Start(TimeSpan.Zero, 90, 3, WheelDiameterMm, true, 0, out string? busy));
        Assert.Contains("尚未结束", busy, StringComparison.Ordinal);
    }

    /// <summary>同一个位置样本不能重复计入到位判据（否则一帧就能“到位”）。</summary>
    [Fact]
    public void SameSampleRepeated_ShouldNotSatisfyArrival()
    {
        (MotorMotionController controller, FakeMotor motor, TimeSpan now) = StartThreeRounds(rounds: 1);

        // 发完配置和正转目标，并让假电机立刻到位。
        for (int i = 0; i < 3; i++)
        {
            Step(controller, motor, ref now);
        }

        Assert.Equal(MotorMotionPhase.WaitForwardArrival, controller.Phase);

        TimeSpan frozenArrival = now;
        int position = motor.Position;
        for (int i = 0; i < 5; i++)
        {
            now += TimeSpan.FromMilliseconds(100);
            motor.Apply(controller.Tick(now, true, position, frozenArrival));
        }

        // 同一个到达时刻重复 5 次只算 1 个样本，仍未到位。
        Assert.Equal(MotorMotionPhase.WaitForwardArrival, controller.Phase);
        Assert.Empty(motor.Commands.Where(c => c.Kind == MotorCommandKind.TargetPosition).Skip(1));
    }

    /// <summary>
    /// 每个停靠点（正转到位、每轮回到起点）都必须先发 0x322=01 重新校准零偏，
    /// 且停顿时间不短于固件校准窗口，保证校准不被下一段运动打断。
    /// </summary>
    [Fact]
    public void EachStop_ShouldRequestGyroRecalibration()
    {
        (MotorMotionController controller, FakeMotor motor, TimeSpan now) = StartThreeRounds(rounds: 2);
        controller.RecalibrateOnStop = true; // 仅显式校准实验启用。

        int guard = 0;
        int maxTicks = (int)Math.Ceiling(4 * MotorMotionController.GyroRecalibrationSeconds * 10) + 100;
        while (controller.IsRunning && guard++ < maxTicks)
        {
            Step(controller, motor, ref now);
        }

        Assert.Equal(MotorMotionState.Completed, controller.State);

        // 2 轮 × (正转到位 + 回到起点) = 4 个停靠点。
        Assert.Equal(4, motor.Commands.Count(c => c.Kind == MotorCommandKind.RecalibrateGyro));
    }

    /// <summary>
    /// 停顿时长填 0 也不能跳过完整校准与验证窗口，
    /// 否则下一段运动会污染零偏估计或独立验证。
    /// </summary>
    [Fact]
    public void ZeroDwellWithRecalibration_ShouldStillWaitForCalibrationWindow()
    {
        (MotorMotionController controller, FakeMotor motor, TimeSpan now) = StartThreeRounds(rounds: 1);
        controller.DwellSeconds = 0;
        controller.RecalibrateOnStop = true;

        int guard = 0;
        while (controller.Phase != MotorMotionPhase.DwellAtTarget && guard++ < 100)
        {
            Step(controller, motor, ref now);
        }

        Assert.Equal(MotorMotionPhase.DwellAtTarget, controller.Phase);

        // 校准窗口结束前不能起步。
        int waitTicks = (int)(MotorMotionController.GyroRecalibrationSeconds * 10);
        for (int i = 0; i < waitTicks - 1; i++)
        {
            Step(controller, motor, ref now);
            Assert.Equal(MotorMotionPhase.DwellAtTarget, controller.Phase);
        }

        Step(controller, motor, ref now);
        Assert.Equal(MotorMotionPhase.WaitBackwardArrival, controller.Phase);
    }

    /// <summary>每轮正转到位后必须先停在目标角度（默认 3 s）才发返回命令。</summary>
    [Fact]
    public void ForwardArrival_ShouldDwellBeforeReturning()
    {
        (MotorMotionController controller, FakeMotor motor, TimeSpan now) = StartThreeRounds(rounds: 1);

        int guard = 0;
        while (controller.Phase != MotorMotionPhase.DwellAtTarget && guard++ < 100)
        {
            Step(controller, motor, ref now);
        }

        Assert.Equal(MotorMotionPhase.DwellAtTarget, controller.Phase);
        Assert.Equal(1, motor.TargetCommandCount);
        Assert.DoesNotContain(motor.Commands, c => c.Kind == MotorCommandKind.RecalibrateGyro);

        // 默认停靠 3 s：29 个 100 ms 的 tick 后仍在停顿，第 30 个（正好 3.0 s）才返程。
        for (int i = 0; i < 29; i++)
        {
            Step(controller, motor, ref now);
            Assert.Equal(MotorMotionPhase.DwellAtTarget, controller.Phase);
            Assert.Equal(1, motor.TargetCommandCount);
        }

        Assert.DoesNotContain("校准", controller.StatusText, StringComparison.Ordinal);

        Step(controller, motor, ref now);
        Assert.Equal(MotorMotionPhase.WaitBackwardArrival, controller.Phase);

        MotorCommand backward = motor.Commands.Last(c => c.Kind == MotorCommandKind.TargetPosition);
        Assert.Equal(-controller.SegmentDeltaLsb, backward.RawPosition);

        while (controller.IsRunning && guard++ < 800)
        {
            Step(controller, motor, ref now);
        }

        Assert.Equal(MotorMotionState.Completed, controller.State);
        Assert.Equal(1, controller.CompletedRounds);
        Assert.Equal(2, motor.TargetCommandCount);
    }

    /// <summary>关掉停靠校准、停顿时长为 0 时回到旧行为：正转到位立即返程。</summary>
    [Fact]
    public void ZeroDwellWithoutRecalibration_ShouldReturnImmediately()
    {
        (MotorMotionController controller, FakeMotor motor, TimeSpan now) = StartThreeRounds(rounds: 1);
        controller.DwellSeconds = 0;
        controller.RecalibrateOnStop = false;

        // 先把配置与正转目标发出去。
        int guard = 0;
        while (motor.TargetCommandCount == 0 && guard++ < 20)
        {
            Step(controller, motor, ref now);
        }

        Assert.Equal(1, motor.TargetCommandCount);

        // 假电机瞬时到位：攒够到位样本的当次 tick 就返程。
        int ticksAfterForward = 0;
        while (controller.Phase != MotorMotionPhase.WaitBackwardArrival && guard++ < 100)
        {
            Step(controller, motor, ref now);
            ticksAfterForward++;
        }

        Assert.Equal(MotorMotionPhase.WaitBackwardArrival, controller.Phase);
        Assert.Equal(MotorMotionController.RequiredSettleSamples, ticksAfterForward);
        Assert.Equal(2, motor.TargetCommandCount);
        Assert.DoesNotContain(motor.Commands, c => c.Kind == MotorCommandKind.RecalibrateGyro);
    }

    /// <summary>停顿期间位置反馈中断超过 600 ms 仍要判失败，不能盲目返程。</summary>
    [Fact]
    public void PositionLossDuringDwell_ShouldFail()
    {
        (MotorMotionController controller, FakeMotor motor, TimeSpan now) = StartThreeRounds(rounds: 1);

        int guard = 0;
        while (controller.Phase != MotorMotionPhase.DwellAtTarget && guard++ < 100)
        {
            Step(controller, motor, ref now);
        }

        Assert.Equal(MotorMotionPhase.DwellAtTarget, controller.Phase);
        int commandsBefore = motor.Commands.Count;

        for (int i = 0; i < 10; i++)
        {
            now += TimeSpan.FromMilliseconds(100);
            motor.Apply(controller.Tick(now, false, motor.Position, null));
        }

        Assert.Equal(MotorMotionState.Failed, controller.State);
        Assert.Contains("位置反馈中断", controller.FailureReason, StringComparison.Ordinal);
        Assert.Equal(commandsBefore, motor.Commands.Count);
    }
}
