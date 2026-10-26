#!/usr/bin/env python3
"""vc.py — command-line control for a running VinylCraft GUI.

The GUI is an eframe app, so it cannot be driven by importing it. It listens on a
localhost control socket (127.0.0.1:47777 by default, or an ephemeral port written to
<temp>/vinylcraft.ctl) and speaks newline-delimited JSON-RPC 2.0 — the same protocol as
the headless --mcp server. This script is that client: send commands and read status
from a terminal instead of clicking and reading pixels.

Usage:
    vc.py state                       # full live state as JSON
    vc.py tools                       # list tool names
    vc.py jog DX DY                   # move the carriage by millimetres (pen up)
    vc.py home                        # send the carriage to the origin
    vc.py load PATH                   # load an SVG or DXF
    vc.py demo                        # load the built-in 300x150 sticker
    vc.py emit [--pen-up] [--out P]   # generate the command stream, optionally write it
    vc.py send [--pen-up]             # send the current design to the cutter
    vc.py call NAME '{"k":v}'         # any tool, raw JSON arguments
    vc.py quit                        # close the GUI

Exit code is 0 on success, 1 when the call reports an error. Status goes to stdout as
JSON so it can be piped into jq or another program; diagnostics go to stderr.

Port resolution order: --port, $VINYLCRAFT_PORT, <temp>/vinylcraft.ctl, then 47777.
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import tempfile
from pathlib import Path

DEFAULT_PORT = 47777
TIMEOUT_S = 120.0


def control_port_file() -> Path:
    return Path(tempfile.gettempdir()) / "vinylcraft.ctl"


def resolve_port(explicit: int | None) -> int:
    if explicit:
        return explicit
    env = os.environ.get("VINYLCRAFT_PORT")
    if env:
        try:
            return int(env)
        except ValueError:
            print(f"ignoring VINYLCRAFT_PORT={env!r}: not a number", file=sys.stderr)
    try:
        return int(control_port_file().read_text().strip())
    except (OSError, ValueError):
        return DEFAULT_PORT


class Client:
    """One JSON-RPC connection to the running GUI."""

    def __init__(self, port: int, timeout: float = TIMEOUT_S):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=timeout)
        self.f = self.sock.makefile("rwb")
        self._id = 0

    def rpc(self, method: str, params: dict | None = None) -> dict:
        self._id += 1
        msg = {"jsonrpc": "2.0", "id": self._id, "method": method}
        if params is not None:
            msg["params"] = params
        self.f.write(json.dumps(msg).encode() + b"\n")
        self.f.flush()
        line = self.f.readline()
        if not line:
            raise RuntimeError("control socket closed without a reply")
        return json.loads(line)

    def call(self, name: str, **args) -> dict:
        """Invoke a tool and unwrap the MCP-shaped result."""
        reply = self.rpc("tools/call", {"name": name, "arguments": args})
        if "error" in reply:
            err = reply["error"]
            raise RuntimeError(f"{name}: {err.get('message', err)}")
        res = reply.get("result", {})
        if isinstance(res, dict) and res.get("isError"):
            raise RuntimeError(f"{name}: {res}")
        # Tool payloads arrive as content[0].text holding JSON.
        for block in res.get("content", []) if isinstance(res, dict) else []:
            if block.get("type") == "text":
                try:
                    return json.loads(block["text"])
                except json.JSONDecodeError:
                    return {"text": block["text"]}
        return res if isinstance(res, dict) else {"result": res}

    def close(self):
        try:
            self.f.close()
            self.sock.close()
        except OSError:
            pass


def print_json(obj):
    print(json.dumps(obj, indent=2, sort_keys=True))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Drive a running VinylCraft GUI over its control socket.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    # dest is deliberately not "port": subcommands take a serial port named "port", and a
    # shared dest makes argparse overwrite the socket port with "COM3" and then try to
    # connect to 127.0.0.1:COM3.
    ap.add_argument("--port", dest="control_port", type=int, default=None,
                    help="control socket port (default: port file, then 47777)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("state", help="print live GUI state")
    sub.add_parser("tools", help="list tool names")
    sub.add_parser("demo", help="load the built-in sticker")
    sub.add_parser("home", help="send the carriage to the origin")
    sub.add_parser("ports", help="list serial ports and which look like cutter bridges")

    p = sub.add_parser("set-port", help="change the serial port without restarting")
    p.add_argument("port")
    p.add_argument("--baud", type=int, default=None)

    p = sub.add_parser("jog", help="move the carriage by dx,dy millimetres")
    p.add_argument("dx", type=float)
    p.add_argument("dy", type=float)

    p = sub.add_parser("load", help="load an SVG or DXF")
    p.add_argument("path")

    p = sub.add_parser("emit", help="generate the cutter command stream")
    p.add_argument("--pen-up", action="store_true", help="dry run: no pen-down moves")
    p.add_argument("--out", default=None, help="write the stream to this path")

    p = sub.add_parser("send", help="send the current design to the cutter")
    p.add_argument("--pen-up", action="store_true", help="dry run: no pen-down moves")

    p = sub.add_parser("call", help="invoke any tool with raw JSON arguments")
    p.add_argument("name")
    p.add_argument("args", nargs="?", default="{}")

    sub.add_parser("quit", help="close the GUI")

    a = ap.parse_args(argv)
    port = resolve_port(a.control_port)
    try:
        c = Client(port)
    except OSError as e:
        print(f"no control socket on 127.0.0.1:{port} ({e})", file=sys.stderr)
        print("start the GUI first: target/debug/vinylcraft.exe", file=sys.stderr)
        return 1

    try:
        if a.cmd == "state":
            print_json(c.rpc("state")["result"])
        elif a.cmd == "tools":
            for t in c.rpc("tools/list")["result"]["tools"]:
                print(t["name"])
        elif a.cmd == "ports":
            print_json(c.call("vinyl_ports"))
        elif a.cmd == "set-port":
            kw = {"port": a.port}
            if a.baud is not None:
                kw["baud"] = a.baud
            print_json(c.call("vinyl_set_port", **kw))
        elif a.cmd == "jog":
            print_json(c.call("vinyl_jog", dx=a.dx, dy=a.dy))
        elif a.cmd == "home":
            print_json(c.call("vinyl_home"))
        elif a.cmd == "demo":
            print_json(c.call("vinyl_demo"))
        elif a.cmd == "load":
            print_json(c.call("vinyl_load", path=str(Path(a.path).resolve())))
        elif a.cmd == "emit":
            kw = {"pen_up": bool(a.pen_up)}
            if a.out:
                kw["path"] = str(Path(a.out).resolve())
            print_json(c.call("vinyl_emit", **kw))
        elif a.cmd == "send":
            print_json(c.call("vinyl_send", pen_up=bool(a.pen_up)))
        elif a.cmd == "call":
            print_json(c.call(a.name, **json.loads(a.args)))
        elif a.cmd == "quit":
            print_json(c.rpc("quit")["result"])
        return 0
    except RuntimeError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    finally:
        c.close()


if __name__ == "__main__":
    sys.exit(main())
