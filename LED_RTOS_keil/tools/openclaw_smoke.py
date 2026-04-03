#!/usr/bin/env python3
"""
OpenClaw/W800 TCP smoke test client.

Usage:
  python tools/openclaw_smoke.py --host 192.168.31.120
  python tools/openclaw_smoke.py --host 192.168.31.120 --cmd '{"cmd":"status"}'
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
from typing import Iterable, List


DEFAULT_COMMANDS = [
    {"cmd": "status"},
    {"cmd": "home"},
    {"cmd": "move_xyz", "x": 120, "y": 0, "z": 80},
    {"cmd": "stop"},
]


def recv_line(sock: socket.socket, timeout_s: float) -> str:
    sock.settimeout(timeout_s)
    buf = bytearray()
    while True:
        chunk = sock.recv(1)
        if not chunk:
            if not buf:
                raise ConnectionError("peer closed")
            break
        buf += chunk
        if chunk == b"\n":
            break
    return buf.decode("utf-8", errors="replace").strip()


def normalize_cmds(raw_cmds: Iterable[str]) -> List[dict]:
    cmds: List[dict] = []
    for idx, raw in enumerate(raw_cmds, start=1):
        try:
            cmd = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise ValueError(f"--cmd #{idx} is not valid JSON: {exc}") from exc
        if not isinstance(cmd, dict):
            raise ValueError(f"--cmd #{idx} must be a JSON object")
        cmds.append(cmd)
    return cmds


def run(host: str, port: int, timeout_s: float, commands: List[dict]) -> int:
    with socket.create_connection((host, port), timeout=timeout_s) as sock:
        print(f"[INFO] connected to {host}:{port}")

        for idx, cmd in enumerate(commands, start=1):
            line = json.dumps(cmd, ensure_ascii=False, separators=(",", ":"))
            payload = (line + "\r\n").encode("utf-8")

            print(f"[{idx}/{len(commands)}] -> {line}")
            sock.sendall(payload)

            try:
                rsp = recv_line(sock, timeout_s)
            except TimeoutError:
                print(f"[{idx}/{len(commands)}] !! timeout waiting response", file=sys.stderr)
                return 2
            except ConnectionError as exc:
                print(f"[{idx}/{len(commands)}] !! connection error: {exc}", file=sys.stderr)
                return 3

            print(f"[{idx}/{len(commands)}] <- {rsp}")
            if not rsp.startswith("{"):
                print(f"[{idx}/{len(commands)}] !! non-JSON response", file=sys.stderr)
                return 4

    print("[PASS] smoke test completed")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="OpenClaw/W800 TCP smoke test")
    parser.add_argument("--host", required=True, help="W800 IP address")
    parser.add_argument("--port", type=int, default=8080, help="W800 TCP port (default: 8080)")
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="socket timeout in seconds (default: 5.0)",
    )
    parser.add_argument(
        "--cmd",
        action="append",
        default=[],
        help='custom JSON command, can be used multiple times, e.g. --cmd \'{"cmd":"status"}\'',
    )
    args = parser.parse_args()

    try:
        commands = normalize_cmds(args.cmd) if args.cmd else list(DEFAULT_COMMANDS)
    except ValueError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    return run(args.host, args.port, args.timeout, commands)


if __name__ == "__main__":
    raise SystemExit(main())
