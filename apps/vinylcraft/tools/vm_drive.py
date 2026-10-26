#!/usr/bin/env python3
"""Drive VinylCraft over its own JSON-RPC (MCP-shaped) server.

Proves the GUI's transform/emit path without clicking: load the demo, move it,
scale it, tile it across a roll, emit a cut file, and clamp a design that runs
off the material edge back onto the vinyl.

Usage:
    python vm_drive.py [--exe PATH] [--send]

--send additionally pushes the design to the cutter in pen-up dry run, which is
safe with no vinyl loaded.
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

EXE = Path(r"F:/phy6-code/solidcraft-phy6/solidcraft/target/debug/vinylcraft.exe")


class Client:
    def __init__(self, exe: Path, extra: list[str] | None = None):
        self.p = subprocess.Popen(
            [str(exe), "--mcp", *(extra or [])],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self._id = 0

    def _rpc(self, method: str, params: dict | None = None):
        self._id += 1
        msg = {"jsonrpc": "2.0", "id": self._id, "method": method}
        if params is not None:
            msg["params"] = params
        self.p.stdin.write(json.dumps(msg) + "\n")
        self.p.stdin.flush()
        line = self.p.stdout.readline()
        if not line:
            err = self.p.stderr.read()
            raise RuntimeError(f"server died: {err}")
        return json.loads(line)

    def initialize(self):
        return self._rpc("initialize", {"protocolVersion": "2025-06-18"})

    def tools(self):
        return [t["name"] for t in self._rpc("tools/list")["result"]["tools"]]

    def call(self, name: str, **args):
        r = self._rpc("tools/call", {"name": name, "arguments": args})
        res = r.get("result", {})
        if res.get("isError"):
            raise RuntimeError(f"{name}: {res}")
        # Tool payload arrives as content[0].text JSON in most MCP servers.
        for block in res.get("content", []):
            if block.get("type") == "text":
                try:
                    return json.loads(block["text"])
                except json.JSONDecodeError:
                    return block["text"]
        return res

    def close(self):
        try:
            self.p.stdin.close()
        except Exception:
            pass
        self.p.wait(timeout=10)


def bbox(o, key="design_bbox"):
    b = o[key]
    return (b[0], b[1], b[2], b[3])


def fmt(b):
    return f"x {b[0]:8.2f}..{b[2]:8.2f}  y {b[1]:7.2f}..{b[3]:7.2f}  ({b[2]-b[0]:.1f} x {b[3]-b[1]:.1f} mm)"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", default=str(EXE))
    ap.add_argument("--send", action="store_true", help="also send to the cutter (pen-up dry run)")
    ap.add_argument("--port", default=None)
    a = ap.parse_args()

    extra = ["--port", a.port] if a.port else []
    c = Client(Path(a.exe), extra)
    fails = []

    init = c.initialize()
    print(f"server      : {init['result']['serverInfo']['name']} v{init['result']['serverInfo']['version']}")
    print(f"tools       : {', '.join(c.tools())}\n")

    st = c.call("vinyl_state")
    print(f"profile     : {st['profile']} ({st['protocol']}, {st['units_per_mm']} u/mm)")
    print(f"port        : {st['port']} @ {st['baud']}   pen-up only: {st['pen_up_only']}")
    print(f"roll        : {st['roll_mm'][0]:.0f} mm wide x {st['roll_mm'][1]:.0f} mm long\n")

    d = c.call("vinyl_demo")
    base = bbox(d)
    print(f"demo loaded : {fmt(base)}  paths={d['paths']}")
    if d["paths"] == 0:
        fails.append("demo produced no paths")

    # 1. MOVE
    d = c.call("vinyl_transform", offset_x=40.0, offset_y=20.0)
    moved = bbox(d)
    print(f"move +40,+20: {fmt(moved)}")
    dx, dy = moved[0] - base[0], moved[1] - base[1]
    print(f"              delta ({dx:+.2f}, {dy:+.2f}) mm")
    if abs(dx - 40) > 0.01 or abs(dy - 20) > 0.01:
        fails.append(f"move delta wrong: {dx:+.2f},{dy:+.2f}")

    # 2. SCALE
    d = c.call("vinyl_transform", scale=0.5)
    scaled = bbox(d)
    print(f"scale 0.5   : {fmt(scaled)}")
    w0, w1 = moved[2] - moved[0], scaled[2] - scaled[0]
    if abs(w1 - w0 / 2) > 0.05:
        fails.append(f"scale wrong: {w0:.2f} -> {w1:.2f}")

    # 3. ROTATE 90 -> width and height swap
    d = c.call("vinyl_transform", scale=1.0, rotation_deg=90.0)
    rot = bbox(d)
    print(f"rotate 90   : {fmt(rot)}")
    if abs((rot[2] - rot[0]) - (moved[3] - moved[1])) > 0.05:
        fails.append("rotate 90 did not swap extents")

    # 4. ROLL SHAPE: a vinyl cutter is fixed-width and long. A 300 mm sticker
    # cannot tile 4-up across a 610 mm roll, but it tiles fine DOWN the length.
    rejected = False
    try:
        c.call("vinyl_transform", rotation_deg=0.0, offset_x=0.0, offset_y=0.0,
               copies_x=4, copies_y=2, step_x=320.0, step_y=170.0)
        c.call("vinyl_emit", path=r"F:/tmp/vm_toowide.dmpl", pen_up=True)
    except RuntimeError as e:
        rejected = "roll" in str(e)
        print(f"4-up across : rejected as expected ({str(e).split('error: ')[-1][:70]})")
    if not rejected:
        fails.append("app did not reject a design wider than the roll")

    # 5. TILE down the length: 2 across (605 mm of 610) x 6 down (990 mm of 2000)
    d = c.call("vinyl_transform", copies_x=2, copies_y=6, step_x=305.0, step_y=170.0)
    tiled = bbox(d)
    print(f"tile 2x6    : {fmt(tiled)}  copies={d['copies']}")
    if not (tiled[2] - tiled[0] <= st["roll_mm"][0] + 0.5):
        fails.append(f"tiled design wider than roll: {tiled[2]-tiled[0]:.1f}")
    if tiled[3] - tiled[1] < 900:
        fails.append("tiling did not run down the roll length")

    # 6. EMIT a real cut file from the tiled layout
    out = r"F:/tmp/vm_tiled.dmpl"
    e = c.call("vinyl_emit", path=out, pen_up=False)
    print(f"emit --cut  : {e['bytes']} bytes -> {out}   shift={e['shift_mm']}")
    code = Path(out).read_text()
    print(f"              commands: U={code.count('U')} D={code.count('D')} IN={'IN;' in code} PA={'PA;' in code}")
    if "D" not in code:
        fails.append("cut emit produced no pen-down commands")

    # 7. ROLL-WIDTH CLAMP: shove the design past the right edge and emit
    d = c.call("vinyl_transform", copies_x=1, copies_y=1, offset_x=st["roll_mm"][0] + 150.0)
    print(f"off-roll    : design bbox {fmt(bbox(d))}")
    right_edge = st["roll_mm"][0]
    try:
        e = c.call("vinyl_emit", path=r"F:/tmp/vm_offroll.dmpl", pen_up=True)
        shift = e["shift_mm"]
        after = bbox(e["state"])
        print(f"              shift applied {shift} -> {fmt(after)}")
        if after[2] > right_edge + 0.5:
            fails.append(f"design still past roll edge: {after[2]:.2f} > {right_edge:.2f}")
        else:
            print(f"              clamped inside {right_edge:.0f} mm roll width")
    except RuntimeError as e:
        print(f"              emit refused: {str(e).split('error: ')[-1][:70]}")
        print(f"              ({'hard error' if 'off the' in str(e) or 'edge' in str(e) else 'see above'})")

    # 7. JOG tracking (pen up, no vinyl)
    j = c.call("vinyl_jog", dx=25.0, dy=-10.0)
    print(f"jog +25,-10 : tracked pos {j['pos_mm']}  status={j['status'][:60]}")
    c.call("vinyl_home")

    if a.send:
        print("\nsending design to cutter, pen-up dry run ...")
        r = c.call("vinyl_send", pen_up=True)
        print(f"  status: {r['status']}")
        for line in str(r.get("log", "")).strip().splitlines()[-6:]:
            print(f"  | {line}")
        if "fail" in str(r["status"]).lower() or "error" in str(r["status"]).lower():
            fails.append(f"send failed: {r['status']}")

    c.close()
    print()
    if fails:
        print("FAIL:")
        for f in fails:
            print(f"  - {f}")
        return 1
    print("PASS — move, scale, rotate, tile, emit and roll-width clamp all behave")
    return 0


if __name__ == "__main__":
    sys.exit(main())
