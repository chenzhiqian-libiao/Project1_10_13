using System.ComponentModel;
using System.Windows;
using GyroHost.App.ViewModels;

namespace GyroHost.App;

/// <summary>
/// 主窗口。视图模型在构造时建立，窗口关闭时确保串口与后台接收任务被释放。
/// </summary>
public partial class MainWindow : Window
{
    public MainWindow()
        : this(new MainViewModel())
    {
    }

    /// <summary>自检用：注入自定义视图模型（脚本化连接 + 测试解码器）。</summary>
    internal MainWindow(MainViewModel viewModel)
    {
        ViewModel = viewModel;
        InitializeComponent();
        DataContext = ViewModel;
    }

    /// <summary>界面视图模型。</summary>
    public MainViewModel ViewModel { get; }

    /// <summary>以模拟模式自动连接（无硬件演示 / 自检用）。</summary>
    public void StartSimulation()
    {
        ViewModel.UseSimulation = true;
        _ = ViewModel.ConnectAsync();
    }

    /// <summary>
    /// 鼠标在图上按下（拖动平移的起点）：暂停自动跟随，之后的缩放/拖动才不会被刷新覆盖。
    /// </summary>
    private void OnPlotPreviewMouseDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        => ViewModel.PauseAutoFollow("鼠标拖动/缩放");

    /// <summary>鼠标滚轮缩放：同样暂停自动跟随。</summary>
    private void OnPlotPreviewMouseWheel(object sender, System.Windows.Input.MouseWheelEventArgs e)
        => ViewModel.PauseAutoFollow("鼠标滚轮缩放");

    protected override void OnClosing(CancelEventArgs e)
    {
        // 关窗即释放：停止刷新定时器、关闭串口、结束接收线程。
        ViewModel.Dispose();
        base.OnClosing(e);
    }
}
