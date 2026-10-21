using System.IO;
using System.Windows;
using GyroHost.App.ViewModels;

namespace GyroHost.App;

/// <summary>
/// 应用入口。
/// <para>
/// 命令行参数：
/// <c>--simulate</c> 启动后用模拟数据源自动连接（无硬件演示）；
/// <c>--selftest</c> 不显示窗口，跑一轮界面层自检，写出报告文件并以退出码返回结果；
/// <c>--selftest-motor</c> 用脚本化连接 + 合成 0x0409 报文验证电机显示链路
/// （不代表真实协议已确认）。
/// </para>
/// </summary>
public partial class App : Application
{
    /// <summary>陀螺仪界面自检报告文件名，写在程序输出目录下。</summary>
    public const string SelfTestReportFileName = "selftest-result.txt";

    /// <summary>电机显示链路自检报告文件名。</summary>
    public const string MotorSelfTestReportFileName = "selftest-motor-result.txt";

    protected override void OnStartup(StartupEventArgs e)
    {
        base.OnStartup(e);

        bool selfTest = e.Args.Any(static arg => arg.Equals("--selftest", StringComparison.OrdinalIgnoreCase));
        bool motorSelfTest = e.Args.Any(static arg => arg.Equals("--selftest-motor", StringComparison.OrdinalIgnoreCase));
        bool simulate = selfTest || motorSelfTest ||
                        e.Args.Any(static arg => arg.Equals("--simulate", StringComparison.OrdinalIgnoreCase));

        MainWindow window;
        ScriptedMotorConnection? scriptedConnection = null;

        if (motorSelfTest)
        {
            scriptedConnection = new ScriptedMotorConnection();
            // 不注入解码器：走与真实串口连接相同的默认解码器。
            window = new MainWindow(new MainViewModel(scriptedConnection, null));
        }
        else
        {
            window = new MainWindow();
        }

        MainWindow = window;

        if (selfTest)
        {
            RunSelfTestAsync(window, SelfTestRunner.RunAsync, SelfTestReportFileName);
            return;
        }

        if (motorSelfTest && scriptedConnection is not null)
        {
            ScriptedMotorConnection connection = scriptedConnection;
            RunSelfTestAsync(
                window,
                viewModel => MotorSelfTestRunner.RunAsync(viewModel, connection),
                MotorSelfTestReportFileName);
            return;
        }

        window.Show();

        if (simulate)
        {
            window.StartSimulation();
        }
    }

    private async void RunSelfTestAsync(
        MainWindow window,
        Func<MainViewModel, Task<(bool Passed, string Report)>> runner,
        string reportFileName)
    {
        int exitCode = 2;
        string report;

        try
        {
            (bool passed, report) = await runner(window.ViewModel);
            exitCode = passed ? 0 : 3;
        }
        catch (Exception ex)
        {
            report = $"自检运行失败：{ex}";
        }

        try
        {
            File.WriteAllText(
                Path.Combine(AppContext.BaseDirectory, reportFileName),
                report);
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine(ex.Message);
        }

        Shutdown(exitCode);
    }
}
