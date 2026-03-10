#!/usr/bin/env python3
"""
Raspberry Pi I2C master reference for the ESP32 shared-state frame.

Frame format (92 bytes, little-endian):
  [0]      magic (0xA5)
  [1]      version (0x01)
  [2..5]   seq (uint32)
  [6]      volume (uint8)
  [7]      battery (uint8)
  [8..25]  BLE address string (18 bytes, null-terminated/padded)
  [26..57] param1 string (32 bytes, null-terminated/padded)
  [58..89] param2 string (32 bytes, null-terminated/padded)
  [90..91] crc16-ccitt over bytes [0..89], little-endian
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass

from smbus2 import SMBus, i2c_msg

FRAME_MAGIC = 0xA5
FRAME_VERSION = 0x01
WRITE_SETTLE_SEC = 0.01
READ_RETRY_COUNT = 3

VP_MAGIC_BIT_LEN = 1
VP_FRAME_VERSION_LEN = 1
VP_SEQ_MAX_LEN = 4
VP_VOLUME_LEN = 1
VP_BATTERY_LEN = 1
VP_BLE_ADDR_MAX_LEN = 18
VP_PARAM_MAX_LEN = 32
VP_CRC16_LEN = 2
BLE_ADDR_MAX_LEN = 18
PARAM_MAX_LEN = 32

FRAME_SIZE = (
    VP_MAGIC_BIT_LEN
    + VP_FRAME_VERSION_LEN
    + VP_SEQ_MAX_LEN
    + VP_VOLUME_LEN
    + VP_BATTERY_LEN
    + VP_BLE_ADDR_MAX_LEN
    + VP_PARAM_MAX_LEN
    + VP_PARAM_MAX_LEN
    + VP_CRC16_LEN
)
DEFAULT_READ_WINDOW = FRAME_SIZE * 3


@dataclass
class DeviceState:
    seq: int
    volume: int
    battery: int
    ble_addr: str
    param1: str
    param2: str


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def decode_fixed_string(raw: bytes) -> str:
    return raw.split(b"\x00", 1)[0].decode("utf-8", errors="replace")


def validate_frame(frame: bytes) -> None:
    if len(frame) != FRAME_SIZE:
        raise ValueError(f"invalid frame size {len(frame)} (expected {FRAME_SIZE})")

    if frame[0] != FRAME_MAGIC:
        raise ValueError(f"bad magic 0x{frame[0]:02X}")

    if frame[1] != FRAME_VERSION:
        raise ValueError(f"bad version {frame[1]}")

    expected_crc = int.from_bytes(frame[-2:], byteorder="little", signed=False)
    computed_crc = crc16_ccitt(frame[:-2])
    if computed_crc != expected_crc:
        raise ValueError(
            f"CRC mismatch expected=0x{expected_crc:04X} got=0x{computed_crc:04X}"
        )


def parse_frame(frame: bytes) -> DeviceState:
    validate_frame(frame)

    seq = int.from_bytes(frame[2:6], byteorder="little", signed=False)
    volume = frame[6]
    battery = frame[7]

    p = 8
    ble_addr = decode_fixed_string(frame[p : p + BLE_ADDR_MAX_LEN])
    p += BLE_ADDR_MAX_LEN
    param1 = decode_fixed_string(frame[p : p + PARAM_MAX_LEN])
    p += PARAM_MAX_LEN
    param2 = decode_fixed_string(frame[p : p + PARAM_MAX_LEN])

    return DeviceState(
        seq=seq,
        volume=volume,
        battery=battery,
        ble_addr=ble_addr,
        param1=param1,
        param2=param2,
    )


def find_valid_frame(raw: bytes) -> bytes:
    if len(raw) < FRAME_SIZE:
        raise ValueError(f"read window too small: {len(raw)} bytes")

    for offset in range(0, len(raw) - FRAME_SIZE + 1):
        if raw[offset] != FRAME_MAGIC or raw[offset + 1] != FRAME_VERSION:
            continue

        candidate = raw[offset : offset + FRAME_SIZE]
        expected_crc = int.from_bytes(candidate[-2:], byteorder="little", signed=False)
        computed_crc = crc16_ccitt(candidate[:-2])
        if computed_crc == expected_crc:
            return candidate

    prefix = " ".join(f"{b:02X}" for b in raw[:8])
    raise ValueError(f"no valid frame found in read window, prefix={prefix}")


def i2c_read_window(bus: SMBus, address: int, size: int) -> bytes:
    read_msg = i2c_msg.read(address, size)
    bus.i2c_rdwr(read_msg)
    return bytes(read_msg)


def i2c_read_frame(bus: SMBus, address: int, read_window: int, retry_count: int) -> bytes:
    last_error: Exception | None = None

    for _ in range(retry_count):
        raw = i2c_read_window(bus, address, read_window)
        try:
            return find_valid_frame(raw)
        except ValueError as exc:
            last_error = exc

    if last_error is not None:
        raise last_error

    raise ValueError("unable to recover a valid frame")


def i2c_write_tokens(bus: SMBus, address: int, token_string: str) -> None:
    data = token_string.encode("utf-8")
    write_msg = i2c_msg.write(address, data)
    bus.i2c_rdwr(write_msg)


def main() -> int:
    parser = argparse.ArgumentParser(description="Poll ESP32 shared-state frame over I2C.")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus number (default: 1)")
    parser.add_argument(
        "--address",
        type=lambda x: int(x, 0),
        default=0x42,
        help="7-bit I2C slave address (default: 0x42)",
    )
    parser.add_argument(
        "--interval-ms",
        type=int,
        default=50,
        help="Polling interval in milliseconds (default: 50)",
    )
    parser.add_argument(
        "--write",
        type=str,
        default="",
        help="Optional token payload to write before each read, e.g. 'VOL=35;P1=hello'",
    )
    parser.add_argument(
        "--read-window",
        type=int,
        default=DEFAULT_READ_WINDOW,
        help=f"Number of bytes to read for frame resynchronization (default: {DEFAULT_READ_WINDOW})",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=READ_RETRY_COUNT,
        help=f"Number of read windows to scan before failing (default: {READ_RETRY_COUNT})",
    )
    parser.add_argument("--json", action="store_true", help="Print decoded state as JSON")
    args = parser.parse_args()

    if args.read_window < FRAME_SIZE:
        raise SystemExit(f"--read-window must be at least {FRAME_SIZE}")

    print(
        f"Polling I2C bus={args.bus} addr=0x{args.address:02X} "
        f"interval={args.interval_ms}ms frame={FRAME_SIZE} bytes window={args.read_window}"
    )

    last_seq: int | None = None
    with SMBus(args.bus) as bus:
        while True:
            try:
                if args.write:
                    i2c_write_tokens(bus, args.address, args.write)
                    time.sleep(WRITE_SETTLE_SEC)

                frame = i2c_read_frame(bus, args.address, args.read_window, args.retries)
                state = parse_frame(frame)

                if last_seq != state.seq:
                    if args.json:
                        print(json.dumps(asdict(state), separators=(",", ":")))
                    else:
                        print(
                            f"seq={state.seq} volume={state.volume} battery={state.battery} "
                            f"addr='{state.ble_addr}' p1='{state.param1}' p2='{state.param2}'"
                        )
                    last_seq = state.seq
            except KeyboardInterrupt:
                print("\nStopped.")
                return 0
            except Exception as exc:
                print(f"read error: {exc}")

            time.sleep(args.interval_ms / 1000.0)


if __name__ == "__main__":
    raise SystemExit(main())
