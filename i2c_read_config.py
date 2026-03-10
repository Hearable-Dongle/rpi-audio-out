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
from typing import Any, Callable, Iterator

try:
    from smbus2 import SMBus, i2c_msg
except ModuleNotFoundError:
    SMBus = Any  # type: ignore[assignment]
    i2c_msg = None

FRAME_MAGIC = 0xA5
FRAME_VERSION = 0x01
WRITE_SETTLE_SEC = 0.01
READ_RETRY_COUNT = 1
DEFAULT_BUS = 1
DEFAULT_ADDRESS = 0x42
DEFAULT_INTERVAL_MS = 50

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
DEFAULT_READ_WINDOW = FRAME_SIZE
DEFAULT_RESYNC_READ_WINDOW = FRAME_SIZE * 3


@dataclass(frozen=True)
class I2CReaderConfig:
    bus: int = DEFAULT_BUS
    address: int = DEFAULT_ADDRESS
    interval_ms: int = DEFAULT_INTERVAL_MS
    write: str = ""
    read_window: int = DEFAULT_READ_WINDOW
    retries: int = READ_RETRY_COUNT
    resync_read: bool = False


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

    offset = 8
    ble_addr = decode_fixed_string(frame[offset : offset + BLE_ADDR_MAX_LEN])
    offset += BLE_ADDR_MAX_LEN
    param1 = decode_fixed_string(frame[offset : offset + PARAM_MAX_LEN])
    offset += PARAM_MAX_LEN
    param2 = decode_fixed_string(frame[offset : offset + PARAM_MAX_LEN])

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


def validate_reader_config(config: I2CReaderConfig) -> None:
    if config.bus < 0:
        raise ValueError("bus must be non-negative")
    if not 0 <= config.address <= 0x7F:
        raise ValueError("address must be in range 0x00..0x7F")
    if config.interval_ms < 0:
        raise ValueError("interval_ms must be non-negative")
    if config.read_window < FRAME_SIZE:
        raise ValueError(f"read_window must be at least {FRAME_SIZE}")
    if config.retries < 1:
        raise ValueError("retries must be at least 1")
    if not config.resync_read and config.read_window != FRAME_SIZE:
        raise ValueError(
            f"read_window must be exactly {FRAME_SIZE} unless resync_read is enabled"
        )
    if not config.resync_read and config.retries != 1:
        raise ValueError("retries must be 1 unless resync_read is enabled")


def require_smbus2() -> None:
    if i2c_msg is None:
        raise RuntimeError(
            "smbus2 is required for I2C access. Install it on the Raspberry Pi with "
            "'python3 -m pip install smbus2'."
        )


def i2c_read_window(bus: SMBus, address: int, size: int) -> bytes:
    require_smbus2()
    read_msg = i2c_msg.read(address, size)
    bus.i2c_rdwr(read_msg)
    return bytes(read_msg)


def i2c_read_exact_frame(bus: SMBus, address: int) -> bytes:
    return i2c_read_window(bus, address, FRAME_SIZE)


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
    require_smbus2()
    write_msg = i2c_msg.write(address, token_string.encode("utf-8"))
    bus.i2c_rdwr(write_msg)


def read_device_state(bus: SMBus, config: I2CReaderConfig) -> DeviceState:
    if config.write:
        i2c_write_tokens(bus, config.address, config.write)
        time.sleep(WRITE_SETTLE_SEC)

    if config.resync_read:
        frame = i2c_read_frame(bus, config.address, config.read_window, config.retries)
    else:
        frame = i2c_read_exact_frame(bus, config.address)
    return parse_frame(frame)


def poll_device_states(
    config: I2CReaderConfig,
    *,
    dedupe_seq: bool = True,
    on_error: Callable[[Exception], None] | None = None,
) -> Iterator[DeviceState]:
    validate_reader_config(config)
    require_smbus2()

    last_seq: int | None = None
    with SMBus(config.bus) as bus:
        while True:
            try:
                state = read_device_state(bus, config)
                if not dedupe_seq or state.seq != last_seq:
                    last_seq = state.seq
                    yield state
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                if on_error is None:
                    raise
                on_error(exc)

            time.sleep(config.interval_ms / 1000.0)


def format_device_state(state: DeviceState) -> str:
    return (
        f"seq={state.seq} volume={state.volume} battery={state.battery} "
        f"addr='{state.ble_addr}' p1='{state.param1}' p2='{state.param2}'"
    )


def add_reader_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--bus",
        type=int,
        default=DEFAULT_BUS,
        help=f"I2C bus number (default: {DEFAULT_BUS})",
    )
    parser.add_argument(
        "--address",
        type=lambda value: int(value, 0),
        default=DEFAULT_ADDRESS,
        help=f"7-bit I2C slave address (default: 0x{DEFAULT_ADDRESS:02X})",
    )
    parser.add_argument(
        "--interval-ms",
        type=int,
        default=DEFAULT_INTERVAL_MS,
        help=f"Polling interval in milliseconds (default: {DEFAULT_INTERVAL_MS})",
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
        help=(
            "Number of bytes to read when --resync-read is enabled "
            f"(default: {DEFAULT_READ_WINDOW})"
        ),
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=READ_RETRY_COUNT,
        help=f"Number of resync read attempts before failing (default: {READ_RETRY_COUNT})",
    )
    parser.add_argument(
        "--resync-read",
        action="store_true",
        help=(
            "Enable larger read-window scanning for frame resynchronization. "
            "Disabled by default to preserve the original direct 92-byte read behavior."
        ),
    )


def reader_config_from_args(args: argparse.Namespace) -> I2CReaderConfig:
    return I2CReaderConfig(
        bus=args.bus,
        address=args.address,
        interval_ms=args.interval_ms,
        write=args.write,
        read_window=args.read_window,
        retries=args.retries,
        resync_read=args.resync_read,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Poll ESP32 shared-state frame over I2C.")
    add_reader_arguments(parser)
    parser.add_argument("--json", action="store_true", help="Print decoded state as JSON")
    args = parser.parse_args()

    config = reader_config_from_args(args)
    validate_reader_config(config)

    print(
        f"Polling I2C bus={config.bus} addr=0x{config.address:02X} "
        f"interval={config.interval_ms}ms frame={FRAME_SIZE} bytes"
    )
    if config.resync_read:
        print(
            f"Resync mode enabled: window={config.read_window} retries={config.retries}"
        )

    try:
        for state in poll_device_states(
            config,
            on_error=lambda exc: print(f"read error: {exc}"),
        ):
            if args.json:
                print(json.dumps(asdict(state), separators=(",", ":")))
            else:
                print(format_device_state(state))
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
