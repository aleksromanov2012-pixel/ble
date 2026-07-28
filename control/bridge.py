#!/usr/bin/env python3
"""
Пульт МеталИнспектор — USB/BLE + живая карта автообхода.

  python3 control/bridge.py
  → http://127.0.0.1:8765/
"""
from __future__ import annotations

import asyncio
import json
import math
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

ROOT = Path(__file__).resolve().parent
HOST = "127.0.0.1"
PORT = 8765
BAUD = 115200

DEVICE_NAME = "ESP32_ROBOT"
SERVICE = "12345678-1234-1234-1234-1234567890ab"
CMD_UUID = "abcdefab-1234-5678-1234-abcdefabcdef"
TELEM_UUID = "fedcbaab-1234-5678-1234-abcdefabcdef"

state = {
    "ble": False,
    "link": "none",
    "scanning": True,
    "name": DEVICE_NAME,
    "telem": "",
    "error": "",
    "updated": 0.0,
    "odo_mm": 0.0,
    "auto": False,
    "nav": "idle",
    "phase": "idle",
    "event": "",
    "cliff": None,
    "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    "path": [[0.0, 0.0]],
    "cliffs": [],
    "steps": 0,
}
_lock = threading.Lock()
_ser = None
_ser_lock = threading.Lock()
_stop = threading.Event()
_speed = 70
_cleared_on_connect = False

_ble_loop: asyncio.AbstractEventLoop | None = None
_ble_queue: asyncio.Queue | None = None
_ble_client = None


def set_state(**kwargs):
    with _lock:
        state.update(kwargs)
        state["updated"] = time.time()
        if "link" in kwargs:
            state["ble"] = kwargs["link"] in ("usb", "ble")


def snapshot():
    with _lock:
        return dict(state)


def reset_map():
    with _lock:
        state["path"] = [[0.0, 0.0]]
        state["cliffs"] = []
        state["odo_mm"] = 0.0
        state["steps"] = 0
        state["cliff"] = None
        state["pose"] = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        state["nav"] = "idle"
        state["updated"] = time.time()


def _append_path(x: float, y: float):
    path = state["path"]
    if path and abs(path[-1][0] - x) < 2.0 and abs(path[-1][1] - y) < 2.0:
        return
    path.append([round(x, 1), round(y, 1)])
    if len(path) > 3000:
        del path[: len(path) - 3000]


def ingest_line(text: str):
    if not text:
        return
    text = text.strip()

    if text == "go":
        with _lock:
            state["steps"] = int(state.get("steps", 0)) + 1
            pose = state.get("pose") or {}
            _append_path(float(pose.get("x", 0)), float(pose.get("y", 0)))
            state["updated"] = time.time()
        return

    if text.startswith("STATE "):
        with _lock:
            state["nav"] = text.split(None, 1)[1].strip()
            state["auto"] = state["nav"] not in ("idle",)
            state["updated"] = time.time()
        return

    if text.startswith("PHASE "):
        with _lock:
            state["phase"] = text.split(None, 1)[1].strip()
            state["updated"] = time.time()
        return

    if text in ("CAMERA_ON", "SCAN_START", "MAP_START", "MAP_DONE", "HOME_START", "SCAN_SKIPPED", "SCAN_DONE", "SERP_DONE"):
        with _lock:
            state["event"] = text
            if text == "CAMERA_ON":
                state["phase"] = "scan"
            elif text == "MAP_START":
                state["phase"] = "map"
            elif text == "HOME_START":
                state["phase"] = "home"
            elif text in ("SCAN_SKIPPED", "SCAN_DONE", "SERP_DONE", "MAP_DONE"):
                state["auto"] = False
                if text == "SCAN_SKIPPED":
                    state["phase"] = "idle"
            state["updated"] = time.time()
        print(f"[robot] {text}")
        return

    if text.startswith("POSE "):
        parts = text.split()
        try:
            x, y, yaw = float(parts[1]), float(parts[2]), float(parts[3])
        except (IndexError, ValueError):
            return
        with _lock:
            state["pose"] = {"x": x, "y": y, "yaw": yaw}
            _append_path(x, y)
            state["updated"] = time.time()
        return

    if text.startswith("ODO"):
        parts = text.split()
        try:
            odo = float(parts[1])
        except (IndexError, ValueError):
            return
        with _lock:
            state["odo_mm"] = odo
            state["updated"] = time.time()
        return

    if text.startswith("CLIFF ") and not text.startswith("CLIFF,"):
        parts = text.split()
        if len(parts) >= 2:
            side = parts[1]
            try:
                odo = float(parts[2]) if len(parts) >= 3 else float(state.get("odo_mm", 0))
            except ValueError:
                odo = float(state.get("odo_mm", 0))
            with _lock:
                pose = state.get("pose") or {"x": odo, "y": 0.0}
                x = float(pose.get("x", odo))
                y = float(pose.get("y", 0.0))
                state["odo_mm"] = odo
                state["cliff"] = side
                _append_path(x, y)
                state["cliffs"].append(
                    {"side": side, "x_mm": round(x, 1), "y_mm": round(y, 1), "odo_mm": round(odo, 1)}
                )
                if len(state["cliffs"]) > 300:
                    del state["cliffs"][: len(state["cliffs"]) - 300]
                state["updated"] = time.time()
        return

    if text.startswith("E,"):
        with _lock:
            state["telem"] = text
            p = text.split(",")
            cliff_side = None
            odo = None
            auto = False
            nav = state.get("nav", "idle")
            for i in range(6, len(p)):
                if p[i] == "CLIFF" and i + 1 < len(p):
                    cliff_side = p[i + 1]
                if p[i] == "ODO" and i + 1 < len(p):
                    try:
                        odo = float(p[i + 1])
                    except ValueError:
                        pass
                if p[i] == "NAV" and i + 1 < len(p):
                    nav = p[i + 1]
                if p[i] == "AUTO":
                    auto = True
            if odo is not None:
                state["odo_mm"] = odo
            state["nav"] = nav
            state["auto"] = auto or nav not in ("idle",)
            state["cliff"] = cliff_side
            state["updated"] = time.time()


def list_candidate_ports():
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    ports = []
    for p in list_ports.comports():
        dev = p.device or ""
        desc = f"{p.description} {p.manufacturer} {p.product}".lower()
        hwid = (p.hwid or "").upper()
        # ESP32-S3 built-in JTAG/serial (303A:1001) is NOT the robot UART — skip.
        if "303A:1001" in hwid or "jtag" in desc:
            continue
        if (
            "wchusbserial" in dev
            or "usbmodem" in dev
            or "usbserial" in dev.lower()
            or "wch" in desc
            or "ch340" in desc
            or "ch343" in desc
            or "cp210" in desc
            or "esp32" in desc
            or "1A86:" in hwid  # WCH CH343
        ):
            ports.append(dev)
    # Prefer WCH/CH343 data cable over anything else.
    def rank(d: str) -> tuple:
        dl = d.lower()
        if "wchusbserial" in dl:
            return (0, d)
        if "usbserial" in dl:
            return (1, d)
        if "usbmodem" in dl:
            return (2, d)
        return (3, d)

    ports.sort(key=rank)
    return ports


def open_serial():
    global _ser, _cleared_on_connect
    import serial

    for port in list_candidate_ports():
        try:
            # Hold DTR/RTS low — otherwise macOS pyserial resets ESP32-S3 on open
            # and we miss boot/telem (or land in download mode).
            s = serial.Serial()
            s.port = port
            s.baudrate = BAUD
            s.timeout = 0.2
            s.dtr = False
            s.rts = False
            s.open()
            time.sleep(0.4)
            s.reset_input_buffer()
            with _ser_lock:
                if _ser and _ser.is_open:
                    try:
                        _ser.close()
                    except Exception:
                        pass
                _ser = s
            set_state(
                link="usb",
                scanning=False,
                error="",
                name=f"USB {port.split('/')[-1]}",
            )
            print(f"[usb] подключено {port}")
            _cleared_on_connect = False
            return True
        except Exception as e:
            print(f"[usb] {port}: {e}")
    return False


def close_serial():
    global _ser
    with _ser_lock:
        if _ser:
            try:
                _ser.close()
            except Exception:
                pass
            _ser = None


def write_cmd(cmd: str) -> bool:
    cmd = cmd.strip()
    if not cmd:
        return False

    global _speed
    if cmd[0] in "Vv" and len(cmd) > 1:
        try:
            _speed = max(30, min(90, int(cmd[1:])))
        except ValueError:
            pass

    if cmd.upper()[:1] in ("G", "Z"):
        reset_map()

    with _ser_lock:
        s = _ser
        if s and s.is_open:
            try:
                s.write((cmd + "\n").encode("utf-8"))
                s.flush()
                print(f"[usb] CMD {cmd}")
                return True
            except Exception as e:
                print(f"[usb] write fail: {e}")
                close_serial()

    if _ble_loop and _ble_queue and snapshot().get("link") == "ble":
        try:
            fut = asyncio.run_coroutine_threadsafe(_ble_queue.put(cmd), _ble_loop)
            fut.result(timeout=2.0)
            print(f"[ble] CMD {cmd}")
            return True
        except Exception as e:
            print(f"[ble] queue fail: {e}")
            return False
    return False


def usb_loop():
    global _cleared_on_connect
    buf = b""
    while not _stop.is_set():
        st = snapshot()
        with _ser_lock:
            s = _ser

        if s is None or not s.is_open:
            if st.get("link") == "ble":
                if open_serial():
                    print("[usb] кабель появился — переключаюсь с BLE")
                else:
                    time.sleep(3)
                continue

            set_state(scanning=True, error="ищу USB/BLE…")
            if open_serial():
                buf = b""
                continue
            time.sleep(0.8)
            continue

        if not _cleared_on_connect:
            _cleared_on_connect = True
            try:
                with _ser_lock:
                    if _ser and _ser.is_open:
                        _ser.write(b"C\n")
                        _ser.flush()
                print("[usb] сброс края (C) после коннекта")
            except Exception:
                pass

        try:
            chunk = s.read(512)
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode("utf-8", "replace").strip()
                    if not text:
                        continue
                    ingest_line(text)
                    if text.startswith(("E,", "ODO", "CLIFF", "POSE", "STATE", "go")):
                        set_state(link="usb", scanning=False, error="")
                    elif "ESP32_ROBOT" in text or "ready" in text:
                        set_state(link="usb", scanning=False, error="")
            else:
                time.sleep(0.02)
        except Exception as e:
            print(f"[usb] read fail: {e}")
            close_serial()
            if snapshot().get("link") == "usb":
                set_state(link="none", scanning=True, error=str(e))
            time.sleep(0.5)


async def _ble_send(client, cmd: str):
    await client.write_gatt_char(CMD_UUID, cmd.encode("utf-8"), response=True)


async def ble_loop():
    global _ble_queue, _ble_client
    from bleak import BleakClient, BleakScanner

    _ble_queue = asyncio.Queue()

    def on_telem(_sender, data: bytearray):
        try:
            text = bytes(data).decode("utf-8", "replace").strip()
        except Exception:
            return
        if snapshot().get("link") == "usb":
            return
        ingest_line(text)
        if text.startswith(("E,", "ODO", "CLIFF", "POSE", "STATE", "go")):
            set_state(link="ble", scanning=False, error="")

    while not _stop.is_set():
        if snapshot().get("link") == "usb" or list_candidate_ports():
            await asyncio.sleep(2)
            continue

        set_state(scanning=True, error="скан BLE…")
        print(f"[ble] ищу {DEVICE_NAME}…")
        try:
            device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=8.0)
            if device is None:
                device = await BleakScanner.find_device_by_filter(
                    lambda d, adv: (
                        (d.name == DEVICE_NAME)
                        or (adv.local_name == DEVICE_NAME)
                        or SERVICE.lower()
                        in [str(u).lower() for u in (adv.service_uuids or [])]
                    ),
                    timeout=6.0,
                )
            if device is None:
                set_state(link="none", scanning=True, error="ESP32_ROBOT не в эфире")
                await asyncio.sleep(2)
                continue

            print(f"[ble] найден {device.name} [{device.address}]")
            set_state(scanning=False, error="BLE connect…", name=device.name or DEVICE_NAME)

            async with BleakClient(device, timeout=20.0) as client:
                _ble_client = client
                if not client.is_connected:
                    raise RuntimeError("GATT не поднялся")
                await client.start_notify(TELEM_UUID, on_telem)
                set_state(link="ble", scanning=False, error="", name=device.name or DEVICE_NAME)
                print("[ble] подключено")
                await _ble_send(client, "C")
                await _ble_send(client, "V55")

                while client.is_connected and not _stop.is_set():
                    if snapshot().get("link") == "usb":
                        print("[ble] USB перехватил — отключаюсь")
                        break
                    try:
                        cmd = await asyncio.wait_for(_ble_queue.get(), timeout=1.0)
                    except asyncio.TimeoutError:
                        continue
                    if cmd:
                        await _ble_send(client, cmd)

        except asyncio.CancelledError:
            raise
        except Exception as e:
            print(f"[ble] обрыв: {e}")
            if snapshot().get("link") == "ble":
                set_state(link="none", scanning=True, error=str(e))
            await asyncio.sleep(1.5)
        finally:
            _ble_client = None


def ble_thread_main():
    global _ble_loop
    _ble_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(_ble_loop)
    try:
        _ble_loop.run_until_complete(ble_loop())
    except Exception as e:
        print(f"[ble] thread dead: {e}")


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def log_message(self, fmt, *args):
        if args and "/api/" in str(args[0]):
            return
        super().log_message(fmt, *args)

    def _json(self, code: int, obj):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/api/status":
            self._json(200, snapshot())
            return
        if path in ("/", "/index.html"):
            self.path = "/index.html"
        return super().do_GET()

    def do_POST(self):
        path = urlparse(self.path).path
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length else b"{}"
        try:
            data = json.loads(raw.decode("utf-8") or "{}")
        except json.JSONDecodeError:
            data = {}

        if path == "/api/cmd":
            cmd = str(data.get("cmd", "")).strip()
            if not cmd:
                self._json(400, {"ok": False, "error": "пустая команда"})
                return
            if not snapshot().get("ble"):
                self._json(503, {"ok": False, "error": "робот не на связи"})
                return
            ok = write_cmd(cmd)
            self._json(200 if ok else 503, {"ok": ok})
            return

        if path == "/api/map/reset":
            reset_map()
            write_cmd("Z")
            self._json(200, {"ok": True})
            return

        self._json(404, {"ok": False, "error": "not found"})


def main():
    reset_map()
    threading.Thread(target=usb_loop, daemon=True, name="usb").start()
    threading.Thread(target=ble_thread_main, daemon=True, name="ble").start()
    httpd = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"[http] пульт → http://{HOST}:{PORT}/")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        _stop.set()
        print("\nстоп")


if __name__ == "__main__":
    main()
