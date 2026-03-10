#!/usr/bin/env python3
"""
Bluetooth audio-output helper for Raspberry Pi.
"""

from __future__ import annotations

import argparse
import re
import subprocess
from dataclasses import dataclass

BLE_ADDRESS_RE = re.compile(r"^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$")
DEFAULT_BLUETOOTHCTL = "bluetoothctl"
DEFAULT_COMMAND_TIMEOUT_SEC = 30.0


@dataclass(frozen=True)
class BluetoothAudioConfig:
    bluetoothctl_path: str = DEFAULT_BLUETOOTHCTL
    command_timeout_sec: float = DEFAULT_COMMAND_TIMEOUT_SEC
    pair: bool = True
    trust: bool = True
    power_on: bool = True


@dataclass(frozen=True)
class BluetoothConnectionResult:
    address: str
    success: bool
    already_connected: bool
    output: str


def normalize_ble_address(address: str) -> str:
    normalized = address.strip().upper()
    if not BLE_ADDRESS_RE.fullmatch(normalized):
        raise ValueError(f"invalid BLE address: {address!r}")
    return normalized


def run_bluetoothctl(
    commands: list[str],
    config: BluetoothAudioConfig,
) -> subprocess.CompletedProcess[str]:
    command_input = "\n".join(commands + ["quit"]) + "\n"
    return subprocess.run(
        [config.bluetoothctl_path],
        input=command_input,
        capture_output=True,
        text=True,
        timeout=config.command_timeout_sec,
        check=False,
    )


def get_device_info(address: str, config: BluetoothAudioConfig) -> str:
    result = run_bluetoothctl([f"info {address}"], config)
    return result.stdout + result.stderr


def is_connected_output(output: str) -> bool:
    return "Connected: yes" in output or "Connection successful" in output


def connect_audio_output(
    address: str,
    config: BluetoothAudioConfig | None = None,
) -> BluetoothConnectionResult:
    resolved_config = config or BluetoothAudioConfig()
    normalized_address = normalize_ble_address(address)

    try:
        info_output = get_device_info(normalized_address, resolved_config)
        if "Connected: yes" in info_output:
            return BluetoothConnectionResult(
                address=normalized_address,
                success=True,
                already_connected=True,
                output=info_output,
            )

        commands: list[str] = []
        if resolved_config.power_on:
            commands.extend(["power on", "agent on", "default-agent"])
        if resolved_config.pair:
            commands.append(f"pair {normalized_address}")
        if resolved_config.trust:
            commands.append(f"trust {normalized_address}")
        commands.extend(
            [
                f"connect {normalized_address}",
                f"info {normalized_address}",
            ]
        )
        result = run_bluetoothctl(commands, resolved_config)
    except FileNotFoundError as exc:
        raise RuntimeError(
            f"bluetoothctl not found at {resolved_config.bluetoothctl_path!r}"
        ) from exc
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError(
            f"bluetoothctl timed out after {resolved_config.command_timeout_sec}s"
        ) from exc

    output = result.stdout + result.stderr
    success = is_connected_output(output)
    return BluetoothConnectionResult(
        address=normalized_address,
        success=success,
        already_connected=False,
        output=output,
    )


def add_bluetooth_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--bluetoothctl-path",
        default=DEFAULT_BLUETOOTHCTL,
        help=f"Path to bluetoothctl (default: {DEFAULT_BLUETOOTHCTL})",
    )
    parser.add_argument(
        "--bt-timeout-sec",
        type=float,
        default=DEFAULT_COMMAND_TIMEOUT_SEC,
        help=(
            "Timeout for each bluetoothctl interaction in seconds "
            f"(default: {DEFAULT_COMMAND_TIMEOUT_SEC})"
        ),
    )
    parser.add_argument(
        "--no-pair",
        action="store_true",
        help="Skip bluetoothctl pair before connect",
    )
    parser.add_argument(
        "--no-trust",
        action="store_true",
        help="Skip bluetoothctl trust before connect",
    )
    parser.add_argument(
        "--no-power-on",
        action="store_true",
        help="Skip bluetoothctl power on / agent setup",
    )


def bluetooth_config_from_args(args: argparse.Namespace) -> BluetoothAudioConfig:
    return BluetoothAudioConfig(
        bluetoothctl_path=args.bluetoothctl_path,
        command_timeout_sec=args.bt_timeout_sec,
        pair=not args.no_pair,
        trust=not args.no_trust,
        power_on=not args.no_power_on,
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Attempt to connect Raspberry Pi audio output to a Bluetooth device."
    )
    parser.add_argument(
        "address",
        help="Bluetooth MAC address to connect to, e.g. AA:BB:CC:DD:EE:FF",
    )
    add_bluetooth_arguments(parser)
    parser.add_argument(
        "--show-output",
        action="store_true",
        help="Print raw bluetoothctl output after the connection attempt",
    )
    args = parser.parse_args()

    result = connect_audio_output(
        args.address,
        config=bluetooth_config_from_args(args),
    )

    if result.success:
        if result.already_connected:
            print(f"{result.address} already connected")
        else:
            print(f"connected to {result.address}")
    else:
        print(f"failed to connect to {result.address}")

    if args.show_output:
        print(result.output, end="" if result.output.endswith("\n") else "\n")

    return 0 if result.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
