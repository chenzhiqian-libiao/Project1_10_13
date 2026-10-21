using System.Buffers.Binary;

namespace GyroHost.Core.Protocol;

/// <summary>一条已经通过确认布局解析出来的位置回复。</summary>
/// <param name="DeviceId">回复中的设备号。</param>
/// <param name="QueryCode">回复中的查询码（位置查询应为 <see cref="MotorProtocol.QueryPositionCode"/>）。</param>
/// <param name="RawPosition">位置原始值，单位 0.1 mm（有符号）。</param>
public readonly record struct MotorPositionReply(byte DeviceId, byte QueryCode, int RawPosition);

/// <summary>
/// <c>0x0409</c> 位置回复解码器接口。
/// <para>
/// 协议表没有定义位置回复布局，因此默认实现是
/// <see cref="UnconfirmedMotorPositionDecoder"/>：<see cref="TryDecode"/> 永远返回 false。
/// 这样查询、原始日志、界面和解码接口都能工作，但不会把猜测当成真实位置。
/// </para>
/// </summary>
public interface IMotorPositionDecoder
{
    /// <summary>是否已经按确认过的协议字段启用解码。</summary>
    bool IsConfirmed { get; }

    /// <summary>给界面显示的解码器说明。</summary>
    string Description { get; }

    /// <summary>尚未确认的字段；已确认时为空。</summary>
    IReadOnlyList<string> MissingFields { get; }

    /// <summary>
    /// 尝试把一条 <c>0x0409</c> 数据解析成位置回复。
    /// 设备号或查询码与本次请求不匹配时必须返回 false，不能把温度/状态/版本回复当成位置。
    /// </summary>
    bool TryDecode(ReadOnlySpan<byte> data, byte expectedDeviceId, out MotorPositionReply reply);
}

/// <summary>
/// 未确认布局的占位解码器。它**不会**猜测位置，只让上层继续记录原始报文并报告缺失字段。
/// </summary>
public sealed class UnconfirmedMotorPositionDecoder : IMotorPositionDecoder
{
    /// <summary>无状态，可共享。</summary>
    public static UnconfirmedMotorPositionDecoder Instance { get; } = new();

    private UnconfirmedMotorPositionDecoder()
    {
    }

    /// <inheritdoc />
    public bool IsConfirmed => false;

    /// <inheritdoc />
    public string Description => "待确认协议：Excel 未定义 0x0409 位置回复的数据长度与字段布局";

    /// <inheritdoc />
    public IReadOnlyList<string> MissingFields => MotorProtocol.MissingReplyLayoutFields;

    /// <inheritdoc />
    public bool TryDecode(ReadOnlySpan<byte> data, byte expectedDeviceId, out MotorPositionReply reply)
    {
        reply = default;
        return false;
    }
}

/// <summary>
/// 明确写死字段布局的位置回复解码器。
/// <para>
/// 只有在拿到完整协议或真实报文、确认每个偏移/符号/字节序之后才应该实例化它。
/// 构造参数本身就是“协议确认结果”，不要把未确认的值填进来。
/// </para>
/// </summary>
public sealed class MotorPositionReplyLayout : IMotorPositionDecoder
{
    /// <summary>
    /// 创建已确认的布局。
    /// </summary>
    /// <param name="expectedDlc">该回复固定的 DLC（1..8）。</param>
    /// <param name="deviceIdOffset">
    /// 设备号所在字节（0..7）；<c>-1</c> 表示回复**不含设备号**，按单设备处理。
    /// </param>
    /// <param name="queryCodeOffset">查询码所在字节（0..7）。</param>
    /// <param name="positionOffset">位置字段起始字节（0..7）。</param>
    /// <param name="positionLength">位置字段长度：2 或 4 字节。</param>
    /// <param name="positionIsLittleEndian">位置字段是否小端。</param>
    /// <param name="positionIsSigned">位置字段是否有符号。</param>
    /// <param name="description">给界面/日志显示的布局说明，例如“DLC=6, Data[2..5] int32 LE”。</param>
    public MotorPositionReplyLayout(
        int expectedDlc,
        int deviceIdOffset,
        int queryCodeOffset,
        int positionOffset,
        int positionLength,
        bool positionIsLittleEndian,
        bool positionIsSigned,
        string description)
    {
        if (expectedDlc is < 1 or > 8)
        {
            throw new ArgumentOutOfRangeException(nameof(expectedDlc), expectedDlc, "DLC 必须在 1..8 之间。");
        }

        if (deviceIdOffset < -1 || deviceIdOffset >= expectedDlc)
        {
            throw new ArgumentOutOfRangeException(nameof(deviceIdOffset), deviceIdOffset, "设备号偏移必须落在 DLC 范围内，或为 -1 表示无设备号。");
        }

        if (queryCodeOffset < 0 || queryCodeOffset >= expectedDlc)
        {
            throw new ArgumentOutOfRangeException(nameof(queryCodeOffset), queryCodeOffset, "查询码偏移必须落在 DLC 范围内。");
        }

        if (positionLength is not (2 or 4))
        {
            throw new ArgumentOutOfRangeException(nameof(positionLength), positionLength, "位置字段只能是 2 或 4 字节。");
        }

        if (positionOffset < 0 || positionOffset + positionLength > expectedDlc)
        {
            throw new ArgumentOutOfRangeException(nameof(positionOffset), positionOffset, "位置字段必须完整落在 DLC 范围内。");
        }

        if (string.IsNullOrWhiteSpace(description))
        {
            throw new ArgumentException("必须写明该布局的来源/确认依据。", nameof(description));
        }

        ExpectedDlc = expectedDlc;
        DeviceIdOffset = deviceIdOffset;
        QueryCodeOffset = queryCodeOffset;
        PositionOffset = positionOffset;
        PositionLength = positionLength;
        PositionIsLittleEndian = positionIsLittleEndian;
        PositionIsSigned = positionIsSigned;
        Description = deviceIdOffset < 0
            ? description + "；回复不含设备号，按单设备处理"
            : description;
    }

    /// <summary>该回复固定的 DLC。</summary>
    public int ExpectedDlc { get; }

    /// <summary>设备号字节偏移；-1 表示回复不含设备号。</summary>
    public int DeviceIdOffset { get; }

    /// <summary>回复是否包含设备号。false 时按单设备处理，不做设备号匹配。</summary>
    public bool HasDeviceId => DeviceIdOffset >= 0;

    /// <summary>查询码字节偏移。</summary>
    public int QueryCodeOffset { get; }

    /// <summary>位置字段起始偏移。</summary>
    public int PositionOffset { get; }

    /// <summary>位置字段长度（2 或 4）。</summary>
    public int PositionLength { get; }

    /// <summary>位置字段是否小端。</summary>
    public bool PositionIsLittleEndian { get; }

    /// <summary>位置字段是否有符号。</summary>
    public bool PositionIsSigned { get; }

    /// <inheritdoc />
    public bool IsConfirmed => true;

    /// <inheritdoc />
    public string Description { get; }

    /// <inheritdoc />
    public IReadOnlyList<string> MissingFields => Array.Empty<string>();

    /// <inheritdoc />
    public bool TryDecode(ReadOnlySpan<byte> data, byte expectedDeviceId, out MotorPositionReply reply)
    {
        reply = default;
        if (data.Length != ExpectedDlc)
        {
            return false;
        }

        if (HasDeviceId && data[DeviceIdOffset] != expectedDeviceId)
        {
            return false;
        }

        if (data[QueryCodeOffset] != MotorProtocol.QueryPositionCode)
        {
            return false;
        }

        ReadOnlySpan<byte> position = data.Slice(PositionOffset, PositionLength);
        int rawPosition;
        if (PositionLength == 2)
        {
            rawPosition = PositionIsSigned
                ? (PositionIsLittleEndian
                    ? BinaryPrimitives.ReadInt16LittleEndian(position)
                    : BinaryPrimitives.ReadInt16BigEndian(position))
                : (PositionIsLittleEndian
                    ? BinaryPrimitives.ReadUInt16LittleEndian(position)
                    : BinaryPrimitives.ReadUInt16BigEndian(position));
        }
        else
        {
            if (PositionIsSigned)
            {
                rawPosition = PositionIsLittleEndian
                    ? BinaryPrimitives.ReadInt32LittleEndian(position)
                    : BinaryPrimitives.ReadInt32BigEndian(position);
            }
            else
            {
                uint value = PositionIsLittleEndian
                    ? BinaryPrimitives.ReadUInt32LittleEndian(position)
                    : BinaryPrimitives.ReadUInt32BigEndian(position);

                // 协议里的位置是 s32；无符号 32 位放不进 int 时说明布局/协议判断有误。
                if (value > int.MaxValue)
                {
                    return false;
                }

                rawPosition = (int)value;
            }
        }

        reply = new MotorPositionReply(
            HasDeviceId ? data[DeviceIdOffset] : expectedDeviceId,
            data[QueryCodeOffset],
            rawPosition);
        return true;
    }
}
