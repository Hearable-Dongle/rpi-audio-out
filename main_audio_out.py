#!/usr/bin/env python3
"""
Bridge I2C state updates to Bluetooth audio-output connection attempts.
"""

from __future__ import annotations

import argparse

from i2c_read_config import (
    DeviceState,
    add_reader_arguments,
    poll_device_states,
    reader_config_from_args,
)
from rpi_set_audio_out import (
    BluetoothAudioConfig,
    add_bluetooth_arguments,
    bluetooth_config_from_args,
    connect_audio_output,
    normalize_ble_address,
)


def connect_from_state(
    state: DeviceState,
    *,
    last_address: str | None,
    retry_same_address: bool,
    bt_config: BluetoothAudioConfig,
) -> str | None:
    raw_address = state.ble_addr.strip()
    if not raw_address:
        print(f"seq={state.seq} missing BLE address, skipping")
        return last_address

    try:
        normalized_address = normalize_ble_address(raw_address)
    except ValueError as exc:
        print(f"seq={state.seq} invalid BLE address {raw_address!r}: {exc}")
        return last_address

    if not retry_same_address and normalized_address == last_address:
        return last_address

    print(f"seq={state.seq} attempting Bluetooth connect to {normalized_address}")
    try:
        result = connect_audio_output(normalized_address, config=bt_config)
    except Exception as exc:
        print(f"connection error for {normalized_address}: {exc}")
        return last_address

    if result.success:
        status = "already connected" if result.already_connected else "connected"
        print(f"{status}: {normalized_address}")
        return normalized_address

    print(f"connection failed: {normalized_address}")
    return last_address


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Poll ESP32 state over I2C and attempt Bluetooth audio connections "
            "using the BLE address carried in each frame."
        )
    )
    add_reader_arguments(parser)
    add_bluetooth_arguments(parser)
    parser.add_argument(
        "--retry-same-address",
        action="store_true",
        help="Attempt a Bluetooth reconnect even if the BLE address has not changed",
    )
    args = parser.parse_args()

    i2c_config = reader_config_from_args(args)
    bt_config = bluetooth_config_from_args(args)

    last_address: str | None = None
    try:
        for state in poll_device_states(
            i2c_config,
            on_error=lambda exc: print(f"read error: {exc}"),
        ):
            last_address = connect_from_state(
                state,
                last_address=last_address,
                retry_same_address=args.retry_same_address,
                bt_config=bt_config,
            )
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
