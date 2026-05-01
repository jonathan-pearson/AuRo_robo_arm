#!/usr/bin/env python3
"""Read DYNAMIXEL X-Series hardware error status registers."""

from argparse import ArgumentParser

from dynamixel_sdk import PacketHandler, PortHandler


COMM_SUCCESS = 0
HARDWARE_ERROR_STATUS_ADDR = 70
HARDWARE_ERROR_STATUS_LEN = 1

ERROR_BITS = {
    0x01: 'input voltage',
    0x04: 'overheating',
    0x08: 'motor encoder',
    0x10: 'electrical shock',
    0x20: 'overload',
}


def _parse_id_range(value: str) -> range:
    if '-' in value:
        start_text, end_text = value.split('-', 1)
        start = int(start_text)
        end = int(end_text)
    else:
        start = 0
        end = int(value)

    if start < 0 or end < start or end > 252:
        raise ValueError('ID range must be between 0 and 252, for example 1-8.')
    return range(start, end + 1)


def _decode_error(value: int) -> str:
    if value == 0:
        return 'none'
    names = [name for bit, name in ERROR_BITS.items() if value & bit]
    unknown = value & ~sum(ERROR_BITS)
    if unknown:
        names.append(f'unknown bits 0x{unknown:02x}')
    return ', '.join(names)


def main() -> int:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyDXL')
    parser.add_argument('--baud', type=int, default=1000000)
    parser.add_argument('--protocol', type=float, default=2.0)
    parser.add_argument('--ids', default='1-8')
    parser.add_argument(
        '--reboot-alerting',
        action='store_true',
        help='Reboot motors with nonzero hardware error status.',
    )
    args = parser.parse_args()

    port_handler = PortHandler(args.port)
    packet_handler = PacketHandler(args.protocol)

    if not port_handler.openPort():
        raise RuntimeError(f'Failed to open {args.port}. Check permissions and cable.')
    if not port_handler.setBaudRate(args.baud):
        raise RuntimeError(f'Failed to set baud rate {args.baud} on {args.port}.')

    try:
        for dxl_id in _parse_id_range(args.ids):
            value, result, error = packet_handler.read1ByteTxRx(
                port_handler,
                dxl_id,
                HARDWARE_ERROR_STATUS_ADDR,
            )
            if result != COMM_SUCCESS:
                print(
                    f'ID {dxl_id:3d}: read failed: '
                    f'{packet_handler.getTxRxResult(result)}'
                )
                continue

            suffix = ''
            if error:
                suffix = f' | status: {packet_handler.getRxPacketError(error)}'

            print(
                f'ID {dxl_id:3d}: hardware_error_status=0x{value:02x} '
                f'({_decode_error(value)}){suffix}'
            )

            if args.reboot_alerting and value:
                result, error = packet_handler.reboot(port_handler, dxl_id)
                if result == COMM_SUCCESS and error == 0:
                    print(f'ID {dxl_id:3d}: reboot command sent')
                else:
                    print(
                        f'ID {dxl_id:3d}: reboot failed: '
                        f'{packet_handler.getTxRxResult(result)} '
                        f'{packet_handler.getRxPacketError(error)}'
                    )
    finally:
        port_handler.closePort()

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
