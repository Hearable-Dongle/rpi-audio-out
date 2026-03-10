#!/usr/bin/env python3
"""
Bluetooth audio-output helper for Raspberry Pi.
"""

from __future__ import annotations

import argparse
import re
import select
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

from device_state_store import DEFAULT_STATE_PATH, load_shared_state

BLE_ADDRESS_RE = re.compile(r"^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$")
DEFAULT_BLUETOOTHCTL = "bluetoothctl"
DEFAULT_COMMAND_TIMEOUT_SEC = 30.0
DEFAULT_SCAN_TIMEOUT_SEC = 10.0
DEVICE_LINE_RE = re.compile(
    r"(?:\[[^\]]+\]\s+)?Device\s+([0-9A-Fa-f:]{17})\s+(.+)$"
)


@dataclass(frozen=True)
class BluetoothAudioConfig:
    bluetoothctl_path: str = DEFAULT_BLUETOOTHCTL
    command_timeout_sec: float = DEFAULT_COMMAND_TIMEOUT_SEC
    scan_timeout_sec: float = DEFAULT_SCAN_TIMEOUT_SEC
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


def normalize_device_name(value: str) -> str:
    return normalize_identifier(value)


def start_bluetoothctl_session(config: BluetoothAudioConfig) -> subprocess.Popen[str]:
    try:
        return subprocess.Popen(
            [config.bluetoothctl_path],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
    except FileNotFoundError as exc:
        raise RuntimeError(
            f"bluetoothctl not found at {config.bluetoothctl_path!r}"
        ) from exc


def write_session_command(process: subprocess.Popen[str], command: str) -> None:
    assert process.stdin is not None
    process.stdin.write(command + "\n")
    process.stdin.flush()


def read_session_output(
    process: subprocess.Popen[str],
    duration_sec: float,
) -> str:
    assert process.stdout is not None
    output_chunks: list[str] = []
    deadline = time.monotonic() + duration_sec

    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        ready, _, _ = select.select([process.stdout], [], [], min(0.2, remaining))
        if not ready:
            continue
        line = process.stdout.readline()
        if not line:
            break
        output_chunks.append(line)

    return "".join(output_chunks)


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

    return DeviceTarget(name=raw)


def parse_bluetooth_devices(output: str) -> list[BluetoothDevice]:
    devices: list[BluetoothDevice] = []
    seen: dict[str, BluetoothDevice] = {}

    for line in output.splitlines():
        line = line.strip()
        match = DEVICE_LINE_RE.search(line)
        if match is None:
            continue
        device = BluetoothDevice(address=match.group(1), name=match.group(2).strip())
        seen[device.address] = device

    devices.extend(seen.values())
    return devices


def match_devices_by_target(
    devices: list[BluetoothDevice],
    target: DeviceTarget,
    config: BluetoothAudioConfig,
) -> tuple[BluetoothDevice, bool]:
    name_matches = [
        device
        for device in devices
        if normalize_device_name(device.name) == normalize_device_name(target.name)
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


def run_bluetoothctl_scan(config: BluetoothAudioConfig) -> str:
    try:
        process = subprocess.Popen(
            [config.bluetoothctl_path],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
    except FileNotFoundError as exc:
        raise RuntimeError(
            f"bluetoothctl not found at {config.bluetoothctl_path!r}"
        ) from exc

    assert process.stdin is not None
    assert process.stdout is not None

    try:
        if config.power_on:
            process.stdin.write("power on\n")
        process.stdin.write("scan on\n")
        process.stdin.flush()
        time.sleep(config.scan_timeout_sec)
        process.stdin.write("devices\n")
        process.stdin.write("scan off\n")
        process.stdin.write("quit\n")
        process.stdin.flush()
        output, _ = process.communicate(timeout=config.command_timeout_sec)
    except subprocess.TimeoutExpired as exc:
        process.kill()
        output, _ = process.communicate()
        raise RuntimeError(
            f"bluetoothctl scan timed out after {config.command_timeout_sec}s"
        ) from exc

    return output


def list_bluetooth_devices(config: BluetoothAudioConfig) -> list[BluetoothDevice]:
    output = run_bluetoothctl_scan(config)
    devices = parse_bluetooth_devices(output)
    if devices:
        return devices

    result = run_bluetoothctl(["devices"], config)
    return parse_bluetooth_devices(result.stdout + result.stderr)


def connect_audio_output(
    address: str,
    config: BluetoothAudioConfig | None = None,
) -> BluetoothConnectionResult:
    resolved_config = config or BluetoothAudioConfig()
    normalized_address = normalize_ble_address(address)
    process = start_bluetoothctl_session(resolved_config)
    transcript: list[str] = []

    try:
        if resolved_config.power_on:
            write_session_command(process, "power on")
            transcript.append(read_session_output(process, 0.5))

        write_session_command(process, "agent NoInputNoOutput")
        transcript.append(read_session_output(process, 0.5))
        write_session_command(process, "default-agent")
        transcript.append(read_session_output(process, 0.5))

        write_session_command(process, f"info {normalized_address}")
        info_output = read_session_output(process, 1.0)
        transcript.append(info_output)
        if "Connected: yes" in info_output:
            write_session_command(process, "quit")
            transcript.append(read_session_output(process, 0.5))
            return BluetoothConnectionResult(
                address=normalized_address,
                success=True,
                already_connected=True,
                output="".join(transcript),
            )

        if resolved_config.pair:
            write_session_command(process, f"pair {normalized_address}")
            transcript.append(read_session_output(process, 8.0))
        if resolved_config.trust:
            write_session_command(process, f"trust {normalized_address}")
            transcript.append(read_session_output(process, 1.0))

        write_session_command(process, f"connect {normalized_address}")
        transcript.append(read_session_output(process, 8.0))
        write_session_command(process, f"info {normalized_address}")
        transcript.append(read_session_output(process, 1.5))
        write_session_command(process, "quit")
        transcript.append(read_session_output(process, 0.5))
        process.wait(timeout=resolved_config.command_timeout_sec)
    except subprocess.TimeoutExpired as exc:
        process.kill()
        raise RuntimeError(
            f"bluetoothctl timed out after {resolved_config.command_timeout_sec}s"
        ) from exc
    finally:
        if process.poll() is None:
            process.kill()

    output = "".join(transcript)
    success = is_connected_output(output)
    return BluetoothConnectionResult(
        address=normalized_address,
        success=success,
        already_connected=False,
        output=output,
    )


def connect_audio_output_by_target(
    target: DeviceTarget,
    config: BluetoothAudioConfig | None = None,
) -> tuple[BluetoothDevice, bool, BluetoothConnectionResult]:
    resolved_config = config or BluetoothAudioConfig()
    process = start_bluetoothctl_session(resolved_config)
    transcript: list[str] = []

    try:
        if resolved_config.power_on:
            write_session_command(process, "power on")
            transcript.append(read_session_output(process, 0.5))

        write_session_command(process, "agent NoInputNoOutput")
        transcript.append(read_session_output(process, 0.5))
        write_session_command(process, "default-agent")
        transcript.append(read_session_output(process, 0.5))

        write_session_command(process, "scan on")
        transcript.append(read_session_output(process, resolved_config.scan_timeout_sec))
        write_session_command(process, "devices")
        transcript.append(read_session_output(process, 1.0))

        devices = parse_bluetooth_devices("".join(transcript))
        device, uuid_verified = match_devices_by_target(devices, target, resolved_config)

        if resolved_config.pair:
            write_session_command(process, f"pair {device.address}")
            transcript.append(read_session_output(process, 8.0))
        if resolved_config.trust:
            write_session_command(process, f"trust {device.address}")
            transcript.append(read_session_output(process, 1.0))

        write_session_command(process, f"connect {device.address}")
        transcript.append(read_session_output(process, 8.0))
        write_session_command(process, f"info {device.address}")
        transcript.append(read_session_output(process, 1.5))
        write_session_command(process, "scan off")
        transcript.append(read_session_output(process, 0.5))
        write_session_command(process, "quit")
        transcript.append(read_session_output(process, 0.5))
        process.wait(timeout=resolved_config.command_timeout_sec)
    except subprocess.TimeoutExpired as exc:
        process.kill()
        raise RuntimeError(
            f"bluetoothctl timed out after {resolved_config.command_timeout_sec}s"
        ) from exc
    finally:
        if process.poll() is None:
            process.kill()

    output = "".join(transcript)
    result = BluetoothConnectionResult(
        address=device.address,
        success=is_connected_output(output),
        already_connected="Connected: yes" in output and "Connection successful" not in output,
        output=output,
    )
    return device, uuid_verified, result


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
        "--scan-timeout-sec",
        type=float,
        default=DEFAULT_SCAN_TIMEOUT_SEC,
        help=(
            "How long to scan for nearby Bluetooth devices before matching by name "
            f"(default: {DEFAULT_SCAN_TIMEOUT_SEC})"
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
        scan_timeout_sec=args.scan_timeout_sec,
        pair=not args.no_pair,
        trust=not args.no_trust,
        power_on=not args.no_power_on,
    )


def resolve_saved_target(state_path: Path) -> DeviceTarget | None:
    state = load_shared_state(state_path)
    param1 = str(state.get("param1", "")).strip()
    if param1:
        return parse_param1_target(param1)

    raise ValueError(f"saved state at {state_path} does not contain param1")


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
            result = connect_audio_output(
                target_address,
                config=bluetooth_config_from_args(args),
            )
        else:
            if args.name:
                target = DeviceTarget(name=args.name, uuid=args.uuid)
            else:
                target = resolve_saved_target(args.state_path)

            device, uuid_verified, result = connect_audio_output_by_target(
                target,
                config=bluetooth_config_from_args(args),
            )
            verification = " with UUID verification" if uuid_verified else ""
            print(f"resolved {device.name!r} to {device.address}{verification}")
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
