#!/usr/bin/env python3
"""Scan a DYNAMIXEL bus for responding servo IDs."""

from argparse import ArgumentParser
from typing import Iterable

from dynamixel_sdk import PacketHandler, PortHandler


COMM_SUCCESS = 0


def _parse_id_range(value: str) -> range:
    if '-' in value:
        start_text, end_text = value.split('-', 1)
        start = int(start_text)
        end = int(end_text)
    else:
        start = 0
        end = int(value)

    if start < 0 or end < start or end > 252:
        raise ValueError('ID range must be between 0 and 252, for example 0-36.')
    return range(start, end + 1)


def _parse_int_list(value: str) -> list[int]:
    return [int(item.strip()) for item in value.split(',') if item.strip()]


def _parse_float_list(value: str) -> list[float]:
    return [float(item.strip()) for item in value.split(',') if item.strip()]


def scan(
    port: str,
    baud_rates: Iterable[int],
    protocol_versions: Iterable[float],
    ids: range,
) -> int:
    found = 0
    port_handler = PortHandler(port)

    if not port_handler.openPort():
        raise RuntimeError(f'Failed to open {port}. Check permissions and cable.')

    try:
        for baud_rate in baud_rates:
            if not port_handler.setBaudRate(baud_rate):
                print(f'Failed to set baud rate {baud_rate} on {port}')
                continue

            for protocol_version in protocol_versions:
                packet_handler = PacketHandler(protocol_version)
                print(
                    f'Scanning {port} at {baud_rate} bps, '
                    f'Protocol {protocol_version:.1f}, IDs {ids.start}-{ids.stop - 1}'
                )

                for dxl_id in ids:
                    model_number, result, error = packet_handler.ping(
                        port_handler,
                        dxl_id,
                    )
                    if result == COMM_SUCCESS and error == 0:
                        found += 1
                        print(
                            f'  ID {dxl_id:3d}: model {model_number} responded'
                        )
                print()
    finally:
        port_handler.closePort()

    if found == 0:
        print('No DYNAMIXELs responded.')
    else:
        print(f'Found {found} responding DYNAMIXEL(s).')
    return found


def main() -> int:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', default='1000000,57600')
    parser.add_argument('--protocol', default='2.0,1.0')
    parser.add_argument('--ids', default='0-36')
    args = parser.parse_args()

    found = scan(
        port=args.port,
        baud_rates=_parse_int_list(args.baud),
        protocol_versions=_parse_float_list(args.protocol),
        ids=_parse_id_range(args.ids),
    )
    return 0 if found else 1


if __name__ == '__main__':
    raise SystemExit(main())
