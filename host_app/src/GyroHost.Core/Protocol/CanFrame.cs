namespace GyroHost.Core.Protocol;

/// <summary>
/// 一条从 SLCAN ASCII 行解析出来的经典 CAN 帧。
/// </summary>
/// <param name="Id">11 位（标准帧）或 29 位（扩展帧）标识符。</param>
/// <param name="IsExtended">true = 扩展帧（SLCAN 的 T/R 行）。</param>
/// <param name="IsRemote">true = 远程帧（RTR）。</param>
/// <param name="Dlc">数据长度码，经典 CAN 为 0..8。</param>
/// <param name="Data">数据字节，长度为 <paramref name="Dlc"/>。</param>
public readonly record struct CanFrame(
    uint Id,
    bool IsExtended,
    bool IsRemote,
    int Dlc,
    byte[] Data)
{
    /// <summary>用于日志显示的标识符文本。</summary>
    public string IdText => IsExtended ? $"0x{Id:X8}" : $"0x{Id:X3}";

    /// <summary>用于日志显示的数据文本。</summary>
    public string DataText => Data.Length == 0 ? "-" : Convert.ToHexString(Data);
}
