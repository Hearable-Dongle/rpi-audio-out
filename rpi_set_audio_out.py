#!/usr/bin/env python3
"""
Bluetooth audio-output helper for Raspberry Pi.
"""

from __future__ import annotations

import argparse
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path

from device_state_store import DEFAULT_STATE_PATH, load_saved_ble_address, load_shared_state

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


@dataclass(frozen=True)
class BluetoothDevice:
    address: str
    name: str


@dataclass(frozen=True)
class DeviceTarget:
    name: str
    uuid: str | None = None


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


def normalize_identifier(value: str) -> str:
    return re.sub(r"[^0-9a-z]", "", value.lower())


def parse_param1_target(param1: str) -> DeviceTarget:
    raw = param1.strip()
    if not raw:
        raise ValueError("param1 is empty")

    parts = [part.strip() for part in raw.split(";") if part.strip()]
    keyed: dict[str, str] = {}
    for part in parts:
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        keyed[key.strip().lower()] = value.strip()

    if "name" in keyed:
        return DeviceTarget(name=keyed["name"], uuid=keyed.get("uuid"))

    for separator in ("|", ","):
        if separator in raw:
            left, right = (part.strip() for part in raw.split(separator, 1))
            if left and right:
                return DeviceTarget(name=right, uuid=left)

    raise ValueError(
        "param1 must contain a device name and optional UUID, for example "
        "'UUID=<id>;NAME=<device>' or '<uuid>|<name>'"
    )


def list_bluetooth_devices(config: BluetoothAudioConfig) -> list[BluetoothDevice]:
    result = run_bluetoothctl(["devices"], config)
    output = result.stdout + result.stderr
    devices: list[BluetoothDevice] = []

    for line in output.splitlines():
        line = line.strip()
        if not line.startswith("Device "):
            continue
        parts = line.split(" ", 2)
        if len(parts) < 3:
            continue
        devices.append(BluetoothDevice(address=parts[1], name=parts[2].strip()))

    return devices


def find_device_by_target(
    target: DeviceTarget,
    config: BluetoothAudioConfig,
) -> tuple[BluetoothDevice, bool]:
    devices = list_bluetooth_devices(config)
    name_matches = [
        device
        for device in devices
        if device.name.strip().casefold() == target.name.strip().casefold()
    ]

    if not name_matches:
        raise LookupError(f"no Bluetooth device found with name {target.name!r}")

    if target.uuid:
        normalized_uuid = normalize_identifier(target.uuid)
        uuid_matches: list[BluetoothDevice] = []
        for device in name_matches:
            info_output = get_device_info(device.address, config)
            if normalized_uuid and normalized_uuid in normalize_identifier(info_output):
                uuid_matches.append(device)

        if len(uuid_matches) == 1:
            return uuid_matches[0], True
        if len(uuid_matches) > 1:
            raise LookupError(
                f"multiple Bluetooth devices matched name {target.name!r} and UUID {target.uuid!r}"
            )

    if len(name_matches) == 1:
        return name_matches[0], False

    raise LookupError(
        f"multiple Bluetooth devices matched name {target.name!r}; UUID verification did not disambiguate"
    )


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


def resolve_saved_target(state_path: Path) -> DeviceTarget | None:
    state = load_shared_state(state_path)
    param1 = str(state.get("param1", "")).strip()
    if param1:
        return parse_param1_target(param1)

    ble_addr = str(state.get("ble_addr", "")).strip()
    if ble_addr:
        return None

    raise ValueError(f"saved state at {state_path} does not contain param1 or ble_addr")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Attempt to connect Raspberry Pi audio output to a Bluetooth device."
    )
    parser.add_argument(
        "address",
        nargs="?",
        help="Bluetooth MAC address to connect to, e.g. AA:BB:CC:DD:EE:FF",
    )
    parser.add_argument(
        "--name",
        help="Bluetooth device name to match manually",
    )
    parser.add_argument(
        "--uuid",
        help="Optional device UUID or identifier to verify while matching by name",
    )
    parser.add_argument(
        "--state-path",
        type=Path,
        default=DEFAULT_STATE_PATH,
        help=f"Path to the saved device state from i2c_read_config.py (default: {DEFAULT_STATE_PATH})",
    )
    add_bluetooth_arguments(parser)
    parser.add_argument(
        "--show-output",
        action="store_true",
        help="Print raw bluetoothctl output after the connection attempt",
    )
    args = parser.parse_args()

    try:
        if args.address:
            target_address = args.address
        else:
            if args.name:
                target = DeviceTarget(name=args.name, uuid=args.uuid)
            else:
                target = resolve_saved_target(args.state_path)

            if target is None:
                target_address = load_saved_ble_address(args.state_path)
            else:
                device, uuid_verified = find_device_by_target(
                    target,
                    bluetooth_config_from_args(args),
                )
                verification = " with UUID verification" if uuid_verified else ""
                print(
                    f"resolved {device.name!r} to {device.address}{verification}"
                )
                target_address = device.address

        result = connect_audio_output(
            target_address,
            config=bluetooth_config_from_args(args),
        )
    except Exception as exc:
        print(f"connection setup failed: {exc}")
        return 1

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
