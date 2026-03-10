#!/usr/bin/env python3
"""
Helpers for sharing the latest decoded device state between scripts.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

DEFAULT_STATE_PATH = Path(__file__).with_name("latest_device_state.json")


def save_shared_state(state: Mapping[str, Any], path: Path = DEFAULT_STATE_PATH) -> None:
    resolved_path = Path(path)
    resolved_path.parent.mkdir(parents=True, exist_ok=True)
    resolved_path.write_text(json.dumps(dict(state), indent=2, sort_keys=True) + "\n")


def load_shared_state(path: Path = DEFAULT_STATE_PATH) -> dict[str, Any]:
    resolved_path = Path(path)
    if not resolved_path.exists():
        raise FileNotFoundError(f"saved state file not found: {resolved_path}")
    return json.loads(resolved_path.read_text())


def load_saved_ble_address(path: Path = DEFAULT_STATE_PATH) -> str:
    state = load_shared_state(path)
    address = str(state.get("ble_addr", "")).strip()
    if not address:
        raise ValueError(f"saved state at {path} does not contain a BLE address")
    return address
