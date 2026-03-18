#!/usr/bin/env python3
"""
Raspberry Pi I2C master for the ESP32 VocalPoint shared-state bridge.

Protocol overview
-----------------
All communication is RPi-initiated (RPi = master, ESP32 = slave at 0x42).
The RPi always WRITES a 4-byte request first, waits SETTLE_SEC, then READs
the response.  The ESP32 never streams proactively.

Request register (4 bytes, little-endian uint32):
  0x00000000            → read status register
  VP_REQ_DATA | <param> → read a specific parameter

Status register response (4 bytes, little-endian uint32):
  bit 0  VP_FLAG_CHANGED   at least one field changed since last fetch
  bit 2  VP_FLAG_VOL       volume changed
  bit 3  VP_FLAG_BAT       battery changed
  bit 4  VP_FLAG_ADDR      BLE address changed
  bit 5  VP_FLAG_P1        param1 changed
  bit 6  VP_FLAG_P2        param2 changed

Param response (4-byte flags header + N-byte payload):
  [0..3]  uint32 flags (VP_REQ_DATA | param_bit)
  [4..]   raw value (sizes: vol/bat=1, addr=40, p1/p2=32 bytes)
"""

from __future__ import annotations

import argparse
import json
import struct
import time
from dataclasses import asdict, dataclass

from smbus2 import SMBus, i2c_msg

from device_state_store import DEFAULT_STATE_PATH, save_shared_state

# ── Protocol constants (must match i2c_protocol.h) ──────────────────────────

VP_FLAG_CHANGED = 1 << 0
VP_FLAG_VOL     = 1 << 2
VP_FLAG_BAT     = 1 << 3
VP_FLAG_ADDR    = 1 << 4
VP_FLAG_P1      = 1 << 5
VP_FLAG_P2      = 1 << 6

VP_REQ_DATA = 1 << 1
VP_REQ_VOL  = 1 << 2
VP_REQ_BAT  = 1 << 3
VP_REQ_ADDR = 1 << 4
VP_REQ_P1   = 1 << 5
VP_REQ_P2   = 1 << 6

VP_PARAM_BITS = VP_FLAG_VOL | VP_FLAG_BAT | VP_FLAG_ADDR | VP_FLAG_P1 | VP_FLAG_P2

VP_STATUS_LEN   = 4
VP_RESP_HDR_LEN = 4

# Bytes of payload for each param bit (excluding the 4-byte header)
PARAM_PAYLOAD_SIZES: dict[int, int] = {
    VP_FLAG_VOL:  1,
    VP_FLAG_BAT:  1,
    VP_FLAG_ADDR: 40,
    VP_FLAG_P1:   32,
    VP_FLAG_P2:   32,
}

# Maps a param flag bit to the DeviceState field name
PARAM_FLAG_TO_FIELD: dict[int, str] = {
    VP_FLAG_VOL:  "volume",
    VP_FLAG_BAT:  "battery",
    VP_FLAG_ADDR: "ble_addr",
    VP_FLAG_P1:   "param1",
    VP_FLAG_P2:   "param2",
}

# How long to wait after writing a request before reading the response.
# The ESP32 bridge task wakes on the incoming write and prepares its TX buffer.
# A slightly larger gap is more robust when BLE activity and I2C handling share
# a single ESP32-C3 core.
SETTLE_SEC = 0.050

# How many times to retry reads if the response header doesn't match.
# Stale bytes can leave the slave TX FIFO misaligned by one transaction.
STATUS_RETRY_COUNT = 2
PARAM_RETRY_COUNT = 2

# Maximum response the ESP32 will ever send (VP_RESP_ADDR_LEN = 4 + 40 = 44).
# Used by the startup drain to flush any leftover bytes from a previous session.
VP_RESP_MAX_LEN = 44


# ── State dataclass ──────────────────────────────────────────────────────────

@dataclass
class DeviceState:
    volume:   int  = 0
    battery:  int  = 0
    ble_addr: str  = ""
    param1:   str  = ""
    param2:   str  = ""


# ── Low-level I2C helpers ────────────────────────────────────────────────────

def _write_request(bus: SMBus, address: int, flags: int) -> None:
    """Write a 4-byte little-endian request register to the ESP32."""
    data = list(struct.pack("<I", flags))
    bus.i2c_rdwr(i2c_msg.write(address, data))


def _read_bytes(bus: SMBus, address: int, n: int) -> bytes:
    msg = i2c_msg.read(address, n)
    bus.i2c_rdwr(msg)
    return bytes(msg)


def drain_tx_buffer(bus: SMBus, address: int, attempts: int = 4) -> None:
    """
    Discard any bytes left in the ESP32's TX FIFO from a previous session.

    The ESP32 TX buffer is a ring FIFO.  If the RPi process was interrupted
    mid-read (crash, restart), leftover bytes sit at the head of the FIFO and
    will prefix the next read, corrupting framing.  Reading VP_RESP_MAX_LEN
    bytes consumes at most one full leftover response.  If the buffer is
    already empty the slave will NACK and smbus2 raises OSError — we ignore
    that here because an empty buffer is the desired end-state.
    """
    for _ in range(attempts):
        try:
            _read_bytes(bus, address, VP_RESP_MAX_LEN)
        except OSError:
            break  # buffer was already empty — that's fine


def _expect_exact_flags(resp_flags: int, expected_flags: int, kind: str) -> None:
    if resp_flags != expected_flags:
        raise ValueError(
            f"{kind} flags mismatch: expected 0x{expected_flags:08X} got 0x{resp_flags:08X}"
        )


# ── Protocol helpers ─────────────────────────────────────────────────────────

def read_status(bus: SMBus, address: int) -> int:
    """
    Request and read the ESP32 status register.

    Returns the 32-bit flags value.  VP_FLAG_CHANGED (bit 0) indicates that
    at least one parameter has changed; the individual VP_FLAG_* bits say which.
    """
    last_err: ValueError | None = None

    for attempt in range(STATUS_RETRY_COUNT + 1):
        if attempt > 0:
            drain_tx_buffer(bus, address)

        _write_request(bus, address, 0x00000000)
        time.sleep(SETTLE_SEC)
        raw = _read_bytes(bus, address, VP_STATUS_LEN)
        flags = struct.unpack("<I", raw)[0]

        # Status replies must not contain the param-response marker.
        if flags & VP_REQ_DATA:
            last_err = ValueError(f"status read returned param header: 0x{flags:08X}")
            continue

        return flags

    raise last_err  # type: ignore[misc]


def read_param(bus: SMBus, address: int, param_bit: int) -> bytes:
    """
    Request and read a single parameter from the ESP32.

    param_bit must be one of the VP_FLAG_* / VP_REQ_* constants (they share
    the same bit positions for bits 2-6).

    Returns the raw payload bytes (length per PARAM_PAYLOAD_SIZES).

    Retries once on header mismatch to handle the TX-buffer contamination
    edge case: the ESP32 clears the dirty bit as soon as it queues the
    response; if the RPi reads stale leftover bytes instead, a single retry
    re-issues the same request and gets a fresh response.

    Raises ValueError if the header still doesn't match after the retry.
    """
    payload_size = PARAM_PAYLOAD_SIZES[param_bit]
    total = VP_RESP_HDR_LEN + payload_size
    last_err: ValueError | None = None

    for attempt in range(PARAM_RETRY_COUNT + 1):
        # Drain any late or stale bytes before issuing the next request.
        drain_tx_buffer(bus, address)

        req = VP_REQ_DATA | param_bit
        _write_request(bus, address, req)
        time.sleep(SETTLE_SEC)

        raw = _read_bytes(bus, address, total)

        resp_flags = struct.unpack("<I", raw[:VP_RESP_HDR_LEN])[0]
        try:
            _expect_exact_flags(resp_flags, req, "param response")
        except ValueError as exc:
            last_err = exc
            continue

        return raw[VP_RESP_HDR_LEN:]

    raise last_err  # type: ignore[misc]


# ── Payload decoders ─────────────────────────────────────────────────────────

def _decode_u8(raw: bytes) -> int:
    return raw[0]


def _decode_string(raw: bytes) -> str:
    return raw.split(b"\x00", 1)[0].decode("utf-8", errors="replace")


def apply_param(state: DeviceState, param_bit: int, raw: bytes) -> None:
    """Parse a raw param payload and update the corresponding DeviceState field."""
    if param_bit == VP_FLAG_VOL:
        state.volume = _decode_u8(raw)
    elif param_bit == VP_FLAG_BAT:
        state.battery = _decode_u8(raw)
    elif param_bit == VP_FLAG_ADDR:
        state.ble_addr = _decode_string(raw)
    elif param_bit == VP_FLAG_P1:
        state.param1 = _decode_string(raw)
    elif param_bit == VP_FLAG_P2:
        state.param2 = _decode_string(raw)


# ── Main polling loop ────────────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Poll ESP32 VocalPoint state over I2C (request/response protocol)."
    )
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
        default=100,
        help="Status poll interval in milliseconds (default: 100)",
    )
    parser.add_argument("--json", action="store_true", help="Print decoded state as JSON")
    args = parser.parse_args()

    print(
        f"VocalPoint I2C  bus={args.bus} addr=0x{args.address:02X} "
        f"interval={args.interval_ms}ms  protocol=request_response"
    )

    state = DeviceState()

    with SMBus(args.bus) as bus:
        # Flush any bytes left in the ESP32 TX FIFO from a previous session.
        drain_tx_buffer(bus, args.address)

        while True:
            cycle_start = time.monotonic()
            try:
                flags = read_status(bus, args.address)

                if flags & VP_FLAG_CHANGED:
                    dirty = flags & VP_PARAM_BITS
                    changed = False

                    for param_bit in (VP_FLAG_VOL, VP_FLAG_BAT,
                                      VP_FLAG_ADDR, VP_FLAG_P1, VP_FLAG_P2):
                        if not (dirty & param_bit):
                            continue
                        try:
                            raw = read_param(bus, args.address, param_bit)
                            apply_param(state, param_bit, raw)
                            changed = True
                        except (ValueError, OSError) as exc:
                            field_name = PARAM_FLAG_TO_FIELD.get(param_bit, f"0x{param_bit:02X}")
                            print(f"param fetch error ({field_name}): {exc}")

                    if changed:
                        save_shared_state(asdict(state), DEFAULT_STATE_PATH)
                        if args.json:
                            print(json.dumps(asdict(state), separators=(",", ":")))
                        else:
                            print(
                                f"volume={state.volume} battery={state.battery} "
                                f"addr='{state.ble_addr}' "
                                f"p1='{state.param1}' p2='{state.param2}'"
                            )

            except KeyboardInterrupt:
                print("\nStopped.")
                return 0
            except OSError as exc:
                print(f"I2C error: {exc}")
            except Exception as exc:
                print(f"unexpected error: {exc}")

            elapsed = time.monotonic() - cycle_start
            remaining = args.interval_ms / 1000.0 - elapsed
            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    raise SystemExit(main())
