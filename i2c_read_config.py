#!/usr/bin/env python3
"""
Raspberry Pi I2C master for the ESP32 VocalPoint shared-state frame.

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

ESP32 TX buffer holds up to 4 frames. The read window is set to 4x the
frame size so find_latest_frame() can scan all buffered frames and return
the one with the highest sequence number, rather than the oldest.
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Callable, Iterator

try:
    from smbus2 import SMBus, i2c_msg
except ModuleNotFoundError:
    SMBus = Any  # type: ignore[assignment]
    i2c_msg = None

from device_state_store import DEFAULT_STATE_PATH, save_shared_state

FRAME_MAGIC = 0xA5
FRAME_VERSION = 0x01
WRITE_SETTLE_SEC = 0.01
READ_RETRY_COUNT = 3
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
DEFAULT_READ_WINDOW = FRAME_SIZE * 4


@dataclass(frozen=True)
class I2CReaderConfig:
    bus: int = DEFAULT_BUS
    address: int = DEFAULT_ADDRESS
    interval_ms: int = DEFAULT_INTERVAL_MS
    write: str = ""
    read_window: int = DEFAULT_READ_WINDOW
    retries: int = READ_RETRY_COUNT
    state_path: Path = DEFAULT_STATE_PATH


@dataclass
class DeviceState:
    seq: int
    volume: int
    battery: int
    ble_addr: str
    param1: str
    param2: str


def require_smbus2() -> None:
    if i2c_msg is None:
        raise RuntimeError(
            "smbus2 is required for I2C access. Install it on the Raspberry Pi with "
            "'python3 -m pip install smbus2'."
        )


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


def parse_frame(frame: bytes) -> DeviceState:
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

    seq = int.from_bytes(frame[2:6], byteorder="little", signed=False)
    volume = frame[6]
    battery = frame[7]

    offset = 8
    ble_addr = decode_fixed_string(frame[offset : offset + VP_BLE_ADDR_MAX_LEN])
    offset += VP_BLE_ADDR_MAX_LEN
    param1 = decode_fixed_string(frame[offset : offset + VP_PARAM_MAX_LEN])
    offset += VP_PARAM_MAX_LEN
    param2 = decode_fixed_string(frame[offset : offset + VP_PARAM_MAX_LEN])

    return DeviceState(
        seq=seq,
        volume=volume,
        battery=battery,
        ble_addr=ble_addr,
        param1=param1,
        param2=param2,
    )


def find_latest_frame(raw: bytes) -> bytes:
    if len(raw) < FRAME_SIZE:
        raise ValueError(f"read window too small: {len(raw)} bytes")

    best: bytes | None = None
    best_seq = -1

    for offset in range(0, len(raw) - FRAME_SIZE + 1):
        if raw[offset] != FRAME_MAGIC or raw[offset + 1] != FRAME_VERSION:
            continue

        candidate = raw[offset : offset + FRAME_SIZE]
        expected_crc = int.from_bytes(candidate[-2:], byteorder="little", signed=False)
        if crc16_ccitt(candidate[:-2]) != expected_crc:
            continue

        seq = int.from_bytes(candidate[2:6], byteorder="little", signed=False)
        if seq > best_seq:
            best_seq = seq
            best = candidate

    if best is None:
        prefix = " ".join(f"{b:02X}" for b in raw[:8])
        raise ValueError(f"no valid frame found in read window, prefix={prefix}")

    return best


def i2c_read_window(bus: SMBus, address: int, size: int) -> bytes:
    require_smbus2()
    read_msg = i2c_msg.read(address, size)
    bus.i2c_rdwr(read_msg)
    return bytes(read_msg)


def i2c_read_frame(bus: SMBus, address: int, read_window: int, retry_count: int) -> bytes:
    last_error: Exception | None = None

    for _ in range(retry_count):
        raw = i2c_read_window(bus, address, read_window)
        try:
            return find_latest_frame(raw)
        except ValueError as exc:
            last_error = exc

    if last_error is not None:
        raise last_error

    raise ValueError("unable to recover a valid frame")


def i2c_write_tokens(bus: SMBus, address: int, token_string: str) -> None:
    require_smbus2()
    data = token_string.encode("utf-8")
    write_msg = i2c_msg.write(address, data)
    bus.i2c_rdwr(write_msg)


def read_device_state(bus: SMBus, config: I2CReaderConfig) -> DeviceState:
    if config.write:
        i2c_write_tokens(bus, config.address, config.write)
        time.sleep(WRITE_SETTLE_SEC)

    frame = i2c_read_frame(bus, config.address, config.read_window, config.retries)
    return parse_frame(frame)


def save_device_state(state: DeviceState, state_path: Path) -> None:
    save_shared_state(asdict(state), state_path)


def format_device_state(state: DeviceState) -> str:
    return (
        f"seq={state.seq} volume={state.volume} battery={state.battery} "
        f"addr='{state.ble_addr}' p1='{state.param1}' p2='{state.param2}'"
    )


def poll_device_states(
    config: I2CReaderConfig,
    *,
    dedupe_seq: bool = True,
    on_error: Callable[[Exception], None] | None = None,
) -> Iterator[DeviceState]:
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
            except ValueError:
                pass
            except Exception as exc:
                if on_error is None:
                    raise
                on_error(exc)

            time.sleep(config.interval_ms / 1000.0)


def add_reader_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--bus", type=int, default=DEFAULT_BUS, help="I2C bus number (default: 1)")
    parser.add_argument(
        "--address",
        type=lambda value: int(value, 0),
        default=DEFAULT_ADDRESS,
        help="7-bit I2C slave address (default: 0x42)",
    )
    parser.add_argument(
        "--interval-ms",
        type=int,
        default=DEFAULT_INTERVAL_MS,
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
        help=f"Number of bytes to read per poll (default: {DEFAULT_READ_WINDOW})",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=READ_RETRY_COUNT,
        help=f"Number of read attempts before reporting an error (default: {READ_RETRY_COUNT})",
    )
    parser.add_argument(
        "--state-path",
        type=Path,
        default=DEFAULT_STATE_PATH,
        help=f"Path to save the latest decoded state (default: {DEFAULT_STATE_PATH})",
    )


def reader_config_from_args(args: argparse.Namespace) -> I2CReaderConfig:
    return I2CReaderConfig(
        bus=args.bus,
        address=args.address,
        interval_ms=args.interval_ms,
        write=args.write,
        read_window=args.read_window,
        retries=args.retries,
        state_path=args.state_path,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Poll ESP32 shared-state frame over I2C.")
    add_reader_arguments(parser)
    parser.add_argument("--json", action="store_true", help="Print decoded state as JSON")
    args = parser.parse_args()

    if args.read_window < FRAME_SIZE:
        raise SystemExit(f"--read-window must be at least {FRAME_SIZE}")

    config = reader_config_from_args(args)

    print(
        f"Polling I2C bus={config.bus} addr=0x{config.address:02X} "
        f"interval={config.interval_ms}ms frame={FRAME_SIZE}B window={config.read_window}B"
    )
    print(f"Saving latest state to {config.state_path}")

    last_seq: int | None = None
    with SMBus(config.bus) as bus:
        while True:
            try:
                state = read_device_state(bus, config)

                if last_seq != state.seq:
                    save_device_state(state, config.state_path)
                    if args.json:
                        print(json.dumps(asdict(state), separators=(",", ":")))
                    else:
                        print(format_device_state(state))
                    last_seq = state.seq

            except KeyboardInterrupt:
                print("\nStopped.")
                return 0
            except ValueError:
                pass
            except Exception as exc:
                print(f"read error: {exc}")

            time.sleep(config.interval_ms / 1000.0)


if __name__ == "__main__":
    raise SystemExit(main())
