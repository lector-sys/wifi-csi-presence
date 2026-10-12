#!/usr/bin/env python3
"""
CSI presence bridge: takes JSON telemetry from the ESP32 and serves the
dashboard.

  ESP32 --UDP JSON--> bridge --HTTP + Server-Sent Events--> browser

Standard library only, so there is nothing to pip install. The dashboard needs
a one-way live stream, which is exactly what SSE is; a WebSocket would need a
dependency to do the same job. Swap in `websockets` only if you later need the
browser to push data back.

  python bridge.py                          # UDP in on 5566, dashboard on 8080
  python bridge.py --node-ip 192.168.1.42   # enables the Calibrate button
  python bridge.py --serial COM5            # serial instead of UDP (pyserial)
  python bridge.py --no-simulate            # never fake data, even with no node
"""

import argparse
import json
import socket
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

WEB_DIR = Path(__file__).resolve().parent.parent / "web"

HISTORY_SECONDS = 3600
HISTORY_STEP = 5           # one sparkline point per 5 s -> 720 points per hour
OFFLINE_AFTER = 15.0       # seconds of silence before a node reads offline



class State:
    """Latest reading plus a rolling hour of scores, per location."""

    def __init__(self):
        self._lock = threading.Lock()
        self._nodes = {}
        self.last_real_packet = 0.0

    def update(self, msg, simulated=False):
        loc = str(msg.get("location", "unknown"))
        now = time.time()
        with self._lock:
            if not simulated:
                # A real node showed up: drop the placeholder so the dashboard
                # never shows fake and real cards side by side.
                for key in [k for k, v in self._nodes.items()
                            if v.get("latest", {}).get("simulated")]:
                    del self._nodes[key]

            node = self._nodes.get(loc)
            if node is None:
                node = {
                    "history": deque(maxlen=HISTORY_SECONDS // HISTORY_STEP),
                    "last_history": 0.0,
                }
                self._nodes[loc] = node

            node["latest"] = {
                "location": loc,
                "activity_score": int(msg.get("activity_score", 0)),
                "calibrated": bool(msg.get("calibrated", False)),
                "calibrating": bool(msg.get("calibrating", False)),
                "pattern": str(msg.get("pattern", "quiet")),
                "raw_variance": msg.get("raw_variance"),
                "packets": msg.get("packets"),
                "simulated": simulated,
            }
            node["seen"] = now

            if now - node["last_history"] >= HISTORY_STEP:
                node["last_history"] = now
                node["history"].append(int(msg.get("activity_score", 0)))

        if not simulated:
            self.last_real_packet = now

    def snapshot(self):
        now = time.time()
        with self._lock:
            return {
                "now": now,
                "nodes": [
                    dict(
                        n["latest"],
                        online=(now - n["seen"]) < OFFLINE_AFTER,
                        age=round(now - n["seen"], 1),
                        history=list(n["history"]),
                        history_step=HISTORY_STEP,
                    )
                    for n in self._nodes.values()
                    if "latest" in n
                ],
            }


STATE = State()


# --------------------------------------------------------------- receivers

def udp_receiver(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    print(f"[bridge] listening for CSI telemetry on UDP :{port}")
    while True:
        data, addr = sock.recvfrom(2048)
        try:
            STATE.update(json.loads(data.decode("utf-8", "replace")))
        except (ValueError, TypeError) as exc:
            print(f"[bridge] bad packet from {addr[0]}: {exc}")


def serial_receiver(port, baud):
    try:
        import serial  # pyserial, only needed for this mode
    except ImportError:
        raise SystemExit("--serial needs pyserial:  pip install pyserial")

    with serial.Serial(port, baud, timeout=2) as ser:
        print(f"[bridge] reading telemetry from {port} @ {baud}")
        while True:
            line = ser.readline().decode("utf-8", "replace").strip()
            if not line.startswith("{"):
                continue          # ESP-IDF log lines share the port, skip them
            try:
                STATE.update(json.loads(line))
            except ValueError:
                pass


def simulator(location):
    """Plausible fluctuating data so the dashboard is usable with no hardware.
    Every reading it produces is flagged simulated:true and labelled as such."""
    import math
    import random

    t = 0.0
    score = 3.0
    print("[bridge] no node yet - simulating (dashboard will say so)")
    while True:
        time.sleep(0.5)
        if STATE.last_real_packet:
            return                # a real node exists; if it dies, show it
                                  # offline rather than papering over it

        t += 0.5
        # Slow wander plus occasional bursts of "activity".
        target = 4 + 3 * math.sin(t / 37)
        if int(t / 45) % 3 == 0:
            target = 45 + 25 * math.sin(t / 4) + random.uniform(-8, 8)
        score += (target - score) * 0.15 + random.uniform(-1.5, 1.5)
        score = max(0.0, min(100.0, score))
        STATE.update(
            {
                "location": location,
                "activity_score": round(score),
                "calibrated": True,
                "calibrating": False,
                "pattern": "sustained" if score > 30 else ("brief" if score > 15 else "quiet"),
            },
            simulated=True,
        )


# ------------------------------------------------------------------ server

class Server(ThreadingHTTPServer):
    # On Windows SO_REUSEADDR lets a second process bind the same port and
    # silently shadow the first. Fail loudly on a port clash instead.
    allow_reuse_address = False
    daemon_threads = True


class Handler(BaseHTTPRequestHandler):
    node_ip = None
    command_port = 5567

    def log_message(self, *args):
        pass

    def _send(self, code, body, ctype="application/json"):
        body = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.startswith("/events"):
            return self._events()
        if self.path.startswith("/api/state"):
            return self._send(200, json.dumps(STATE.snapshot()))

        name = "index.html" if self.path in ("/", "") else self.path.lstrip("/")
        path = (WEB_DIR / name).resolve()
        if WEB_DIR not in path.parents or not path.is_file():
            return self._send(404, "not found", "text/plain")
        ctype = {"html": "text/html", "css": "text/css", "js": "text/javascript"}.get(
            path.suffix.lstrip("."), "application/octet-stream"
        )
        self._send(200, path.read_bytes(), ctype)

    def do_POST(self):
        if not self.path.startswith("/api/calibrate"):
            return self._send(404, "not found", "text/plain")
        if not self.node_ip:
            return self._send(
                409,
                json.dumps({"ok": False, "error": "no --node-ip set, cannot reach the node"}),
            )
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(b"CAL", (self.node_ip, self.command_port))
            sock.close()
            self._send(200, json.dumps({"ok": True}))
        except OSError as exc:
            self._send(502, json.dumps({"ok": False, "error": str(exc)}))

    def _events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            while True:
                payload = json.dumps(STATE.snapshot())
                self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                self.wfile.flush()
                time.sleep(1.0)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass          # browser navigated away; EventSource will reconnect


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--udp-port", type=int, default=5566)
    p.add_argument("--http-port", type=int, default=8080)
    p.add_argument("--node-ip", help="ESP32 IP, needed for the Calibrate button")
    p.add_argument("--command-port", type=int, default=5567)
    p.add_argument("--serial", help="serial port instead of UDP, e.g. COM5")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--no-simulate", action="store_true")
    p.add_argument("--sim-location", default="living room (simulated)")
    args = p.parse_args()

    Handler.node_ip = args.node_ip
    Handler.command_port = args.command_port

    if args.serial:
        threading.Thread(target=serial_receiver, args=(args.serial, args.baud), daemon=True).start()
    else:
        threading.Thread(target=udp_receiver, args=(args.udp_port,), daemon=True).start()

    if not args.no_simulate:
        threading.Thread(target=simulator, args=(args.sim_location,), daemon=True).start()

    try:
        server = Server(("0.0.0.0", args.http_port), Handler)
    except OSError as exc:
        raise SystemExit(f"[bridge] cannot listen on :{args.http_port} ({exc}) "
                         f"- another bridge is probably already running")
    print(f"[bridge] dashboard on http://localhost:{args.http_port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[bridge] bye")


if __name__ == "__main__":
    main()
