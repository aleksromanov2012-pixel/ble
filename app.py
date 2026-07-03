#!/usr/bin/env python3
"""Game-style fullscreen GUI controller for the ESP32_ROBOT (BLE).

Hold WASD or the arrow keys (or click/hold the on-screen D-pad) to drive the
robot, exactly like driving in a video game: press to move, release to stop.

Everything is expressed as a differential drive: the app computes a signed
speed for the left and right side and sends "M<left>,<right>" to the robot.

Run:  python app.py   (or:  .venv/bin/python app.py)   -  ESC quits.
Requires: pygame-ce (or pygame) and bleak  ->  pip install -r requirements.txt
"""

import asyncio
import math
import os
import threading
import time

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import pygame
from pygame import gfxdraw
from bleak import BleakClient, BleakError, BleakScanner

DEVICE_NAME = "ESP32_ROBOT"
SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab"
CHAR_UUID = "abcdefab-1234-5678-1234-abcdefabcdef"
TELEM_UUID = "fedcbaab-1234-5678-1234-abcdefabcdef"
SCAN_TIMEOUT = 12.0
SCAN_ATTEMPTS = 4
CONNECT_ATTEMPTS = 3
CONNECT_TIMEOUT = 25.0
RECONNECT_DELAY = 2.0
GATT_SETTLE_DELAY = 0.4

KEEPALIVE_INTERVAL = 0.2
SEND_MIN_INTERVAL = 0.05

SPEED_MIN, SPEED_MAX = 30, 255
SPEED_STEP, SPEED_DEFAULT = 15, 150
ACCEL_RATE = 520.0
STEER_GAIN = 0.6
PIVOT_INNER = 0.15

ENCODER_IDS = ("lv", "ln", "rn", "rv")
ENCODER_LABELS = {"lv": "LV", "ln": "LN", "rn": "RN", "rv": "RV"}
ENCODER_META = {
    "lv": {"name": "Left Front", "side": "left", "row": 0, "col": 0},
    "ln": {"name": "Left Rear", "side": "left", "row": 1, "col": 0},
    "rn": {"name": "Right Front", "side": "right", "row": 0, "col": 1},
    "rv": {"name": "Right Rear", "side": "right", "row": 1, "col": 1},
}


def normalize_uuid(value):
    return str(value).lower().replace("-", "")


def uuid_equal(left, right):
    return normalize_uuid(left) == normalize_uuid(right)


async def wait_for_gatt_services(client, timeout=5.0):
    """Bleak 3.x: service discovery runs during connect(); poll until ready."""
    deadline = time.monotonic() + timeout
    last_err = None
    while time.monotonic() < deadline:
        try:
            return client.services
        except BleakError as exc:
            last_err = exc
            await asyncio.sleep(0.1)
    raise RuntimeError(f"GATT service discovery timed out: {last_err}")


def parse_encoder_telemetry(line):
    """Parse 'E,ms,lv,ln,rn,rv' from robot.ino."""
    parts = line.strip().split(",")
    if len(parts) != 6 or parts[0] != "E":
        return None
    try:
        return {
            "ms": int(parts[1]),
            "lv": int(parts[2]),
            "ln": int(parts[3]),
            "rn": int(parts[4]),
            "rv": int(parts[5]),
        }
    except (TypeError, ValueError):
        return None


class BLEController:
    DISCONNECTED = "disconnected"
    SCANNING = "scanning"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ERROR = "error"

    def __init__(self):
        self._loop = asyncio.new_event_loop()
        self._client = None
        self._lock = threading.Lock()
        self._busy = False
        self._shutdown_requested = False
        self._reconnect_task = None
        self._poll_task = None
        self._telem_char = None
        self.state = self.DISCONNECTED
        self.detail = "Not connected"
        self.encoders = None
        self._enc_time = None
        self.telem_ok = False
        self.telem_detail = "not subscribed"
        self._notify_count = 0
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    @property
    def connected(self):
        return self.state == self.CONNECTED

    def connect(self):
        if self._shutdown_requested or self._busy or self.connected:
            return
        self._cancel_reconnect()
        self._busy = True
        asyncio.run_coroutine_threadsafe(self._connect(), self._loop)

    def send(self, cmd):
        if self.connected:
            asyncio.run_coroutine_threadsafe(self._send(cmd), self._loop)

    def get_encoders(self):
        with self._lock:
            telem_ok = self.telem_ok
            detail = self.telem_detail
            if self.encoders is None or self._enc_time is None:
                return None, None, telem_ok, detail
            return dict(self.encoders), time.time() - self._enc_time, telem_ok, detail

    def shutdown(self):
        self._shutdown_requested = True
        self._cancel_reconnect()
        fut = asyncio.run_coroutine_threadsafe(self._shutdown(), self._loop)
        try:
            fut.result(timeout=5.0)
        except Exception:
            pass
        self._loop.call_soon_threadsafe(self._loop.stop)

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _set(self, state, detail):
        with self._lock:
            self.state = state
            self.detail = detail

    def _cancel_reconnect(self):
        task = self._reconnect_task
        if task is not None and not task.done():
            task.cancel()
        self._reconnect_task = None

    def _schedule_reconnect(self):
        if self._shutdown_requested or self._busy or self.connected:
            return
        self._cancel_reconnect()
        self._reconnect_task = asyncio.run_coroutine_threadsafe(
            self._auto_reconnect(), self._loop
        )

    async def _auto_reconnect(self):
        try:
            await asyncio.sleep(RECONNECT_DELAY)
            if self._shutdown_requested or self.connected or self._busy:
                return
            self._busy = True
            await self._connect()
        except asyncio.CancelledError:
            pass
        except Exception:
            self._busy = False

    def _ingest_encoder_bytes(self, raw):
        if isinstance(raw, bytearray):
            raw = bytes(raw)
        elif not isinstance(raw, (bytes, bytearray)):
            raw = bytes(raw)
        text = raw.decode("utf-8", errors="ignore").strip("\x00\r\n ")
        parsed = parse_encoder_telemetry(text)
        if parsed is None:
            return False
        with self._lock:
            self.encoders = parsed
            self._enc_time = time.time()
            self.telem_ok = True
            self.telem_detail = "live"
            self._notify_count += 1
        return True

    async def _find_telem_char(self, client):
        for service in client.services:
            for char in service.characteristics:
                if uuid_equal(char.uuid, TELEM_UUID):
                    return char
        return None

    async def _read_encoder_once(self, client):
        char = self._telem_char or await self._find_telem_char(client)
        if char is None:
            return False
        raw = await client.read_gatt_char(char)
        return self._ingest_encoder_bytes(raw)

    async def _wait_for_encoder_data(self, timeout=4.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self.encoders is not None:
                    return True
            await asyncio.sleep(0.05)
        return False

    def _cancel_poll_task(self):
        task = self._poll_task
        if task is not None and not task.done():
            task.cancel()
        self._poll_task = None

    async def _encoder_poll_loop(self):
        try:
            while not self._shutdown_requested:
                client = self._client
                if client is None or not client.is_connected:
                    break
                try:
                    await self._read_encoder_once(client)
                except Exception:  # noqa: BLE001
                    pass
                await asyncio.sleep(0.25)
        except asyncio.CancelledError:
            pass

    async def _subscribe_encoders(self, client):
        self._telem_char = await self._find_telem_char(client)
        if self._telem_char is None:
            with self._lock:
                self.telem_ok = False
                self.telem_detail = "telemetry char not found"
            return False

        props = self._telem_char.properties
        if "notify" not in props and "indicate" not in props:
            with self._lock:
                self.telem_ok = False
                self.telem_detail = f"bad props: {props}"
            return False

        try:
            await client.start_notify(self._telem_char, self._on_encoder)
        except Exception as e:  # noqa: BLE001
            with self._lock:
                self.telem_ok = False
                self.telem_detail = f"notify failed: {e}"
            return False

        try:
            await self._read_encoder_once(client)
        except Exception as e:  # noqa: BLE001
            with self._lock:
                self.telem_detail = f"notify ok, read pending ({e})"

        if not await self._wait_for_encoder_data(timeout=4.0):
            try:
                await self._read_encoder_once(client)
            except Exception as e:  # noqa: BLE001
                with self._lock:
                    self.telem_ok = False
                    self.telem_detail = f"no encoder data ({e})"
                return False
            if not await self._wait_for_encoder_data(timeout=1.0):
                with self._lock:
                    self.telem_ok = False
                    self.telem_detail = "no encoder packets"
                return False

        with self._lock:
            self.telem_ok = True
            self.telem_detail = "live"
        return True

    async def _connect(self):
        client = None
        try:
            device = await self._find_device()
            if device is None:
                self._set(self.DISCONNECTED, "Robot not found - retrying...")
                self._schedule_reconnect()
                return

            last_error = None
            for attempt in range(1, CONNECT_ATTEMPTS + 1):
                if self._shutdown_requested:
                    return
                self._set(
                    self.CONNECTING,
                    f"Connecting {device.address} ({attempt}/{CONNECT_ATTEMPTS})...",
                )
                client = BleakClient(
                    device,
                    disconnected_callback=self._on_disconnect,
                    timeout=CONNECT_TIMEOUT,
                    services=[SERVICE_UUID],
                )
                try:
                    await client.connect()
                    if not client.is_connected:
                        raise RuntimeError("connect returned but link is down")
                    await asyncio.sleep(GATT_SETTLE_DELAY)
                    await wait_for_gatt_services(client)
                    if not await self._subscribe_encoders(client):
                        raise RuntimeError(self.telem_detail)
                    await client.write_gatt_char(CHAR_UUID, b"M0,0", response=False)
                    self._client = client
                    self._cancel_poll_task()
                    self._poll_task = asyncio.create_task(self._encoder_poll_loop())
                    self._set(self.CONNECTED, f"Connected to {device.address}")
                    return
                except Exception as e:  # noqa: BLE001
                    last_error = e
                    try:
                        await client.disconnect()
                    except Exception:  # noqa: BLE001
                        pass
                    client = None
                    if attempt < CONNECT_ATTEMPTS:
                        await asyncio.sleep(1.5 * attempt)

            self._client = None
            self._set(self.ERROR, f"Connect failed: {last_error}")
            self._schedule_reconnect()
        except Exception as e:  # noqa: BLE001
            self._client = None
            if client is not None:
                try:
                    await client.disconnect()
                except Exception:  # noqa: BLE001
                    pass
            self._set(self.ERROR, f"Connect failed: {e}")
            self._schedule_reconnect()
        finally:
            self._busy = False

    async def _find_device(self):
        best = None
        best_rssi = -127
        for attempt in range(1, SCAN_ATTEMPTS + 1):
            if self._shutdown_requested:
                return None
            self._set(self.SCANNING, f"Scanning ({attempt}/{SCAN_ATTEMPTS})...")
            found = await BleakScanner.discover(timeout=SCAN_TIMEOUT, return_adv=True)
            by_name = None
            name_rssi = -127
            by_service = None
            service_rssi = -127
            for device, adv in found.values():
                name = device.name or adv.local_name
                rssi = adv.rssi if adv.rssi is not None else -127
                if name == DEVICE_NAME and rssi > name_rssi:
                    by_name = device
                    name_rssi = rssi
                uuids = [u.lower() for u in (adv.service_uuids or [])]
                if SERVICE_UUID.lower() in uuids and rssi > service_rssi:
                    by_service = device
                    service_rssi = rssi
            if by_name is not None:
                return by_name
            candidate = by_service
            if candidate is not None and service_rssi > best_rssi:
                best = candidate
                best_rssi = service_rssi
            if attempt < SCAN_ATTEMPTS:
                await asyncio.sleep(1.0)
        return best

    async def _send(self, cmd):
        client = self._client
        if client is None:
            return
        try:
            await client.write_gatt_char(CHAR_UUID, cmd.encode(), response=False)
        except Exception as e:  # noqa: BLE001
            self._set(self.ERROR, f"Write failed: {e}")

    def _on_encoder(self, _sender, data):
        self._ingest_encoder_bytes(data)

    def _on_disconnect(self, _client):
        self._cancel_poll_task()
        self._client = None
        self._telem_char = None
        with self._lock:
            self.encoders = None
            self._enc_time = None
            self.telem_ok = False
            self.telem_detail = "disconnected"
            self._notify_count = 0
        if self._shutdown_requested:
            return
        if self.state != self.ERROR:
            self._set(self.DISCONNECTED, "Disconnected - reconnecting...")
            self._schedule_reconnect()

    async def _shutdown(self):
        self._cancel_reconnect()
        self._cancel_poll_task()
        current = asyncio.current_task()
        pending = [t for t in asyncio.all_tasks(self._loop) if t is not current]
        for t in pending:
            t.cancel()
        if pending:
            await asyncio.gather(*pending, return_exceptions=True)
        client = self._client
        if client is not None:
            try:
                await client.write_gatt_char(CHAR_UUID, b"M0,0", response=False)
                if self._telem_char is not None:
                    try:
                        await client.stop_notify(self._telem_char)
                    except Exception:  # noqa: BLE001
                        pass
                await client.disconnect()
            except Exception:  # noqa: BLE001
                pass
        self._client = None
        self._telem_char = None
        with self._lock:
            self.encoders = None
            self._enc_time = None
            self.telem_ok = False
            self.telem_detail = "shutdown"
            self._notify_count = 0


BG_TOP = (13, 15, 24)
BG_BOT = (20, 24, 40)
PANEL = (28, 32, 48)
PANEL_HI = (40, 46, 68)
PANEL_LINE = (52, 60, 86)
TEXT = (233, 238, 250)
MUTED = (132, 142, 168)
ACCENT = (84, 198, 255)
ACCENT2 = (150, 130, 255)
GREEN = (74, 222, 150)
AMBER = (245, 196, 84)
RED = (244, 100, 96)
STOP_COL = (244, 118, 110)

FPS = 60

BUTTONS = {
    "F": {"label": "FORWARD", "cell": (1, 0), "glyph": "up"},
    "L": {"label": "LEFT", "cell": (0, 1), "glyph": "left"},
    "S": {"label": "STOP", "cell": (1, 1), "glyph": "stop"},
    "R": {"label": "RIGHT", "cell": (2, 1), "glyph": "right"},
    "B": {"label": "BACK", "cell": (1, 2), "glyph": "down"},
}

KEY_TO_DIR = {
    pygame.K_w: "F", pygame.K_UP: "F",
    pygame.K_s: "B", pygame.K_DOWN: "B",
    pygame.K_a: "L", pygame.K_LEFT: "L",
    pygame.K_d: "R", pygame.K_RIGHT: "R",
}

MODE_TANK, MODE_STEER = 1, 2


def lerp(a, b, t):
    return a + (b - a) * t


def mix(c1, c2, t):
    t = max(0.0, min(1.0, t))
    return tuple(int(lerp(c1[i], c2[i], t)) for i in range(3))


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def approach(cur, target, step):
    if cur < target:
        return min(cur + step, target)
    if cur > target:
        return max(cur - step, target)
    return cur


def aa_circle(surf, x, y, r, color):
    x, y, r = int(x), int(y), int(r)
    if r <= 0:
        return
    gfxdraw.filled_circle(surf, x, y, r, color)
    gfxdraw.aacircle(surf, x, y, r, color)


def aa_polygon(surf, pts, color):
    ipts = [(int(p[0]), int(p[1])) for p in pts]
    gfxdraw.filled_polygon(surf, ipts, color)
    gfxdraw.aapolygon(surf, ipts, color)


class RobotApp:
    def __init__(self):
        pygame.init()
        self._init_display()
        self.clock = pygame.time.Clock()
        self.u = self.H / 1000.0
        self._build_assets()

        self.ble = BLEController()
        self.ble.connect()

        self.held_keys = []
        self.mouse_dir = None
        self.mode = MODE_TANK
        self.max_speed = SPEED_DEFAULT
        self.smooth = True

        self.cur_l = 0.0
        self.cur_r = 0.0
        self.out_l = 0
        self.out_r = 0
        self.last_sent = None
        self.last_send_t = 0.0

        self.glow = {bid: 0.0 for bid in BUTTONS}
        self.cmd_label = "IDLE"
        self._enc_prev = {mid: None for mid in ENCODER_IDS}
        self._enc_pulse = {mid: 0.0 for mid in ENCODER_IDS}

        self.button_rects = {}
        self.mode_rects = {}
        self.speed_rect = None
        self.smooth_rect = None
        self.running = True

    def _init_display(self):
        pygame.display.set_caption("ESP32 Robot - Game Controller")
        try:
            desktop = pygame.display.get_desktop_sizes()[0]
        except Exception:
            desktop = (1280, 800)

        self.window = None
        try:
            self.window = pygame.Window(
                "ESP32 Robot - Game Controller",
                size=desktop,
                fullscreen_desktop=True,
                allow_high_dpi=True,
            )
            self.screen = self.window.get_surface()
        except Exception:
            self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN | pygame.SCALED)

        self.W, self.H = self.screen.get_size()
        if self.window is not None and self.window.size[0]:
            self.mouse_scale = self.W / self.window.size[0]
        else:
            self.mouse_scale = 1.0

    def _present(self):
        if self.window is not None:
            self.window.flip()
        else:
            pygame.display.flip()

    def _build_assets(self):
        def font(px, bold=False):
            px = max(8, int(px))
            for name in ("Helvetica Neue", "Helvetica", "Arial", "DejaVu Sans"):
                if pygame.font.match_font(name):
                    return pygame.font.SysFont(name, px, bold=bold)
            return pygame.font.Font(None, px)

        u = self.u
        self.f_title = font(44 * u, bold=True)
        self.f_sub = font(16 * u)
        self.f_mode = font(21 * u, bold=True)
        self.f_status = font(19 * u)
        self.f_cmd = font(34 * u, bold=True)
        self.f_label = font(14 * u, bold=True)
        self.f_val = font(20 * u, bold=True)
        self.f_help_k = font(17 * u, bold=True)
        self.f_help_d = font(17 * u)
        self.f_enc_title = font(13 * u, bold=True)
        self.f_enc_name = font(11 * u)
        self.f_enc_value = font(30 * u, bold=True)
        self.f_enc_unit = font(10 * u)
        self._layout = {}
        self.bg = self._make_background()
        self.glow_layer = self._make_radial_glow()

    def _make_background(self):
        surf = pygame.Surface((self.W, self.H))
        for y in range(self.H):
            surf.fill(mix(BG_TOP, BG_BOT, y / self.H), rect=(0, y, self.W, 1))
        return surf

    def _make_radial_glow(self):
        d = int(min(self.W, self.H) * 0.85)
        surf = pygame.Surface((d, d), pygame.SRCALPHA)
        cx = cy = d // 2
        for i in range(60, 0, -1):
            t = i / 60
            r = int(cx * t)
            alpha = int(42 * (1 - t) ** 2)
            gfxdraw.filled_circle(surf, cx, cy, r, (*ACCENT, alpha))
        return surf

    def _mode_accent(self):
        return ACCENT if self.mode == MODE_TANK else ACCENT2

    def run(self):
        while self.running:
            dt = self.clock.tick(FPS) / 1000.0
            self.handle_events()
            self.update(dt)
            self.draw()
            self._present()
        self.ble.shutdown()
        pygame.quit()

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                self._on_keydown(event.key)
            elif event.type == pygame.KEYUP:
                if event.key in KEY_TO_DIR:
                    d = KEY_TO_DIR[event.key]
                    if d in self.held_keys:
                        self.held_keys.remove(d)
            elif event.type == pygame.MOUSEWHEEL:
                self._change_speed(SPEED_STEP * (1 if event.y > 0 else -1))
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                self._on_click(self._scaled_pos(event.pos))
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                self.mouse_dir = None

    def _on_keydown(self, key):
        if key == pygame.K_ESCAPE:
            self.running = False
        elif key == pygame.K_SPACE:
            self.held_keys.clear()
        elif key in (pygame.K_TAB, pygame.K_m):
            self.mode = MODE_STEER if self.mode == MODE_TANK else MODE_TANK
        elif key == pygame.K_g:
            self.smooth = not self.smooth
        elif key in (pygame.K_MINUS, pygame.K_LEFTBRACKET, pygame.K_KP_MINUS):
            self._change_speed(-SPEED_STEP)
        elif key in (pygame.K_EQUALS, pygame.K_RIGHTBRACKET, pygame.K_KP_PLUS, pygame.K_PLUS):
            self._change_speed(SPEED_STEP)
        elif key in (pygame.K_c, pygame.K_RETURN):
            self.ble.connect()
        elif key in KEY_TO_DIR:
            d = KEY_TO_DIR[key]
            if d not in self.held_keys:
                self.held_keys.append(d)

    def _on_click(self, pos):
        for m, rect in self.mode_rects.items():
            if rect.collidepoint(pos):
                self.mode = m
                return
        if self.smooth_rect and self.smooth_rect.collidepoint(pos):
            self.smooth = not self.smooth
            return
        if self.speed_rect and self.speed_rect.collidepoint(pos):
            frac = (pos[0] - self.speed_rect.left) / self.speed_rect.width
            self.max_speed = int(round(
                clamp(SPEED_MIN + frac * (SPEED_MAX - SPEED_MIN), SPEED_MIN, SPEED_MAX)))
            return
        self.mouse_dir = self._hit_button(pos)
        if self.mouse_dir == "S":
            self.held_keys.clear()

    def _change_speed(self, delta):
        self.max_speed = int(clamp(self.max_speed + delta, SPEED_MIN, SPEED_MAX))

    def _scaled_pos(self, pos):
        return (pos[0] * self.mouse_scale, pos[1] * self.mouse_scale)

    def _hit_button(self, pos):
        for bid, rect in self.button_rects.items():
            if rect.collidepoint(pos):
                return bid
        return None

    def _active_list(self):
        active = list(self.held_keys)
        if self.mouse_dir and self.mouse_dir != "S":
            active.append(self.mouse_dir)
        return active

    def _base_turn(self):
        base = turn = None
        for d in self._active_list():
            if d in ("F", "B"):
                base = d
            elif d in ("L", "R"):
                turn = d
        return base, turn

    def _targets(self):
        v = self.max_speed
        base, turn = self._base_turn()

        if self.mode == MODE_TANK:
            active = self._active_list()
            a = active[-1] if active else None
            mapping = {"F": (v, v), "B": (-v, -v), "L": (-v, v), "R": (v, -v)}
            l, r = mapping.get(a, (0, 0))
            lit = {a} if a else {"S"}
            label = BUTTONS[a]["label"] if a else "IDLE"
            return l, r, lit, label

        drive = v if base == "F" else -v if base == "B" else 0
        steer = v if turn == "R" else -v if turn == "L" else 0
        if drive == 0 and steer != 0:
            inner = int(v * PIVOT_INNER)
            l, r = (v, inner) if steer > 0 else (inner, v)
        else:
            t = steer * STEER_GAIN
            l = clamp(drive + t, -v, v)
            r = clamp(drive - t, -v, v)

        lit = set()
        if base:
            lit.add(base)
        if turn:
            lit.add(turn)
        if not lit:
            lit = {"S"}
        if base and turn:
            label = f"{BUTTONS[base]['label']} + {BUTTONS[turn]['label']}"
        elif base:
            label = BUTTONS[base]["label"]
        elif turn:
            label = BUTTONS[turn]["label"] + " ARC"
        else:
            label = "IDLE"
        return l, r, lit, label

    def _driving(self):
        return bool(self._active_list())

    def update(self, dt):
        now = time.time()
        driving = self._driving()

        if driving:
            tl, tr, lit, label = self._targets()
        else:
            lit, label = {"S"}, "IDLE"
            tl, tr = 0, 0

        # Smooth ramp only while driving; release key -> instant M0,0
        if self.smooth and driving:
            step = ACCEL_RATE * dt
            self.cur_l = approach(self.cur_l, tl, step)
            self.cur_r = approach(self.cur_r, tr, step)
        else:
            self.cur_l, self.cur_r = float(tl), float(tr)

        self.cmd_label = label if self.ble.connected else "NO LINK"
        self.out_l = int(round(self.cur_l))
        self.out_r = int(round(self.cur_r))

        pair = (self.out_l, self.out_r)
        elapsed = now - self.last_send_t
        changed = pair != self.last_sent
        if self.ble.connected:
            if not driving:
                should_send = changed or elapsed >= 0.03
            else:
                should_send = (
                    (changed and elapsed >= SEND_MIN_INTERVAL)
                    or elapsed >= KEEPALIVE_INTERVAL
                )
            if should_send:
                self.ble.send(f"M{self.out_l},{self.out_r}")
                self.last_sent = pair
                self.last_send_t = now
        elif not self.ble.connected:
            self.last_sent = None

        k = min(1.0, 13.0 * dt)
        for bid in BUTTONS:
            target = 1.0 if bid in lit else 0.0
            self.glow[bid] += (target - self.glow[bid]) * k

        enc, _, telem_ok, _ = self.ble.get_encoders()
        if enc is not None and telem_ok:
            for mid in ENCODER_IDS:
                prev = self._enc_prev[mid]
                if prev is not None and enc[mid] != prev:
                    self._enc_pulse[mid] = 1.0
                self._enc_prev[mid] = enc[mid]
        pulse_decay = min(1.0, dt * 4.5)
        for mid in ENCODER_IDS:
            self._enc_pulse[mid] = max(0.0, self._enc_pulse[mid] - pulse_decay)

    def _compute_layout(self):
        u = self.u
        H = self.H
        gap = int(12 * u)
        bottom_pad = int(14 * u)

        line_h = int(28 * u)
        help_h = line_h * 2 + int(46 * u)
        help_top = H - bottom_pad - help_h

        enc_header = int(30 * u)
        enc_row_h = int(74 * u)
        enc_footer = int(14 * u)
        enc_panel_h = enc_header + enc_row_h + enc_footer
        enc_top = help_top - gap - enc_panel_h

        telem_span = int(88 * u)
        telem_cy = enc_top - gap - telem_span + int(18 * u)

        cell = int(min(self.W, self.H) * 0.135)
        dpad_half = int(cell * 1.65)
        dpad_cy = int(H * 0.46)
        dpad_bottom = dpad_cy + dpad_half

        min_enc_top = dpad_bottom + gap
        if enc_top < min_enc_top:
            enc_top = min_enc_top
            telem_cy = enc_top - gap - telem_span + int(18 * u)

        if enc_top + enc_panel_h > help_top - gap:
            enc_row_h = max(int(58 * u), help_top - gap - enc_top - enc_header - enc_footer)
            enc_panel_h = enc_header + enc_row_h + enc_footer

        return {
            "dpad_cy": dpad_cy,
            "telem_cy": telem_cy,
            "enc_top": enc_top,
            "enc_header": enc_header,
            "enc_row_h": enc_row_h,
            "enc_footer": enc_footer,
            "help_top": help_top,
            "help_h": help_h,
            "line_h": line_h,
        }

    def draw(self):
        self.screen.blit(self.bg, (0, 0))
        cx = self.W // 2
        H = self.H
        layout = self._compute_layout()
        self._layout = layout
        self._draw_header(cx, int(H * 0.035))
        self._draw_mode_toggle(cx, int(H * 0.135))
        self._draw_status(cx, int(H * 0.198))
        self._draw_settings(cx, int(H * 0.262))
        self._draw_dpad(cx, layout["dpad_cy"])
        self._draw_telemetry(cx, layout["telem_cy"])
        self._draw_encoders(cx, layout["enc_top"])
        self._draw_help(cx, layout["help_top"])

    def _text(self, text, font, color, **anchor):
        img = font.render(text, True, color)
        rect = img.get_rect(**anchor)
        self.screen.blit(img, rect)
        return rect

    def _round_rect(self, rect, color, radius):
        pygame.draw.rect(self.screen, color, rect, border_radius=int(radius))

    def _round_border(self, rect, color, radius, width):
        pygame.draw.rect(self.screen, color, rect, width=int(width),
                         border_radius=int(radius))

    def _draw_header(self, cx, y):
        self._text("ROBOT CONTROL", self.f_title, TEXT, midtop=(cx, y))
        self._text("ESP32  \u00b7  Bluetooth LE  \u00b7  differential drive",
                   self.f_sub, MUTED, midtop=(cx, y + int(52 * self.u)))

    def _draw_mode_toggle(self, cx, cy):
        seg_w, seg_h, gap = int(210 * self.u), int(48 * self.u), int(8 * self.u)
        total = seg_w * 2 + gap
        x0 = cx - total // 2
        container = pygame.Rect(x0 - gap, cy - seg_h // 2 - gap, total + gap * 2, seg_h + gap * 2)
        self._round_rect(container, PANEL, seg_h)
        self.mode_rects = {}
        for i, (m, label, col) in enumerate(
            [(MODE_TANK, "TANK MODE", ACCENT), (MODE_STEER, "STEER MODE", ACCENT2)]
        ):
            rect = pygame.Rect(x0 + i * (seg_w + gap), cy - seg_h // 2, seg_w, seg_h)
            self.mode_rects[m] = rect
            active = self.mode == m
            if active:
                self._round_rect(rect, mix(PANEL, col, 0.30), seg_h // 2)
                self._round_border(rect, col, seg_h // 2, max(2, int(2 * self.u)))
            self._text(label, self.f_mode, TEXT if active else MUTED, center=rect.center)

    def _draw_status(self, cx, cy):
        st = self.ble.state
        color = {
            BLEController.CONNECTED: GREEN,
            BLEController.SCANNING: AMBER,
            BLEController.CONNECTING: AMBER,
            BLEController.ERROR: RED,
            BLEController.DISCONNECTED: MUTED,
        }.get(st, MUTED)
        pill = pygame.Rect(cx - int(220 * self.u), cy - int(22 * self.u),
                           int(440 * self.u), int(44 * self.u))
        self._round_rect(pill, PANEL, pill.height // 2)
        pulse = 0.5 + 0.5 * math.sin(time.time() * 4)
        dot_col = color
        if st in (BLEController.SCANNING, BLEController.CONNECTING):
            dot_col = mix(PANEL, color, 0.35 + 0.65 * pulse)
        dot_x = pill.left + int(26 * self.u)
        aa_circle(self.screen, dot_x, pill.centery, int(7 * self.u), dot_col)
        self._text(self.ble.detail, self.f_status, TEXT,
                   midleft=(dot_x + int(20 * self.u), pill.centery))

    def _draw_settings(self, cx, cy):
        u = self.u
        sw, sh, tw, gap = int(330 * u), int(46 * u), int(260 * u), int(22 * u)
        total = sw + gap + tw
        x0 = cx - total // 2
        box = pygame.Rect(x0, cy - sh // 2, sw, sh)
        self._round_rect(box, PANEL, int(12 * u))
        self._text("SPEED", self.f_label, MUTED, midleft=(box.left + int(16 * u), box.centery))
        track = pygame.Rect(box.left + int(96 * u), box.centery - int(7 * u),
                            sw - int(160 * u), int(14 * u))
        self.speed_rect = track
        self._round_rect(track, mix(PANEL, BG_TOP, 0.5), int(7 * u))
        frac = (self.max_speed - SPEED_MIN) / (SPEED_MAX - SPEED_MIN)
        fill = pygame.Rect(track.left, track.top, int(track.width * frac), track.height)
        self._round_rect(fill, ACCENT, int(7 * u))
        self._text(f"{int(self.max_speed / SPEED_MAX * 100)}%", self.f_val, TEXT,
                   midright=(box.right - int(14 * u), box.centery))
        tog = pygame.Rect(x0 + sw + gap, cy - sh // 2, tw, sh)
        self.smooth_rect = tog
        col = GREEN if self.smooth else MUTED
        self._round_rect(tog, mix(PANEL, col, 0.16 if self.smooth else 0.0), int(12 * u))
        self._round_border(tog, col if self.smooth else PANEL_LINE, int(12 * u),
                           max(2, int(2 * u)))
        self._text("SMOOTH ACCEL", self.f_label, TEXT if self.smooth else MUTED,
                   midleft=(tog.left + int(16 * u), tog.centery))

    def _draw_dpad(self, cx, cy):
        cell = int(min(self.W, self.H) * 0.135)
        gap = int(cell * 0.10)
        grid = cell * 3 + gap * 2
        ox, oy = cx - grid // 2, cy - grid // 2
        gl = self.glow_layer
        self.screen.blit(gl, (cx - gl.get_width() // 2, cy - gl.get_height() // 2))
        self.button_rects = {}
        for bid, b in BUTTONS.items():
            gx, gy = b["cell"]
            rect = pygame.Rect(ox + gx * (cell + gap), oy + gy * (cell + gap), cell, cell)
            self.button_rects[bid] = rect
            self._draw_button(bid, rect)

    def _draw_button(self, bid, rect):
        b = BUTTONS[bid]
        g = self.glow[bid]
        accent = STOP_COL if bid == "S" else self._mode_accent()
        radius = int(rect.width * 0.18)
        if g > 0.02:
            pad = int(22 * self.u)
            halo = pygame.Surface((rect.w + pad * 2, rect.h + pad * 2), pygame.SRCALPHA)
            pygame.draw.rect(halo, (*accent, int(95 * g)), halo.get_rect(),
                             border_radius=radius + pad)
            self.screen.blit(halo, (rect.x - pad, rect.y - pad))
        base = mix(PANEL, PANEL_HI, 0.5)
        self._round_rect(rect, mix(base, accent, 0.5 * g), radius)
        self._round_border(rect, mix(PANEL_LINE, accent, g), radius, max(2, int(2 * self.u)))
        glyph_col = mix(MUTED, (255, 255, 255), 0.35 + 0.65 * g)
        self._draw_glyph(b["glyph"], rect, glyph_col, g)
        self._text(b["label"], self.f_label, mix(MUTED, TEXT, 0.25 + 0.75 * g),
                   midbottom=(rect.centerx, rect.bottom - int(16 * self.u)))

    def _draw_glyph(self, glyph, rect, color, g):
        cx = rect.centerx
        cy = rect.centery - int(rect.height * 0.06)
        s = rect.width * (0.17 + 0.012 * g)
        if glyph == "stop":
            side = int(rect.width * 0.26)
            r = pygame.Rect(0, 0, side, side)
            r.center = (cx, cy)
            self._round_rect(r, color, side * 0.22)
            return
        if glyph == "up":
            pts = [(cx, cy - s), (cx - s, cy + s * 0.72), (cx + s, cy + s * 0.72)]
        elif glyph == "down":
            pts = [(cx, cy + s), (cx - s, cy - s * 0.72), (cx + s, cy - s * 0.72)]
        elif glyph == "left":
            pts = [(cx - s, cy), (cx + s * 0.72, cy - s), (cx + s * 0.72, cy + s)]
        else:
            pts = [(cx + s, cy), (cx - s * 0.72, cy - s), (cx - s * 0.72, cy + s)]
        aa_polygon(self.screen, pts, color)

    def _draw_telemetry(self, cx, cy):
        active = self.cmd_label not in ("IDLE", "NO LINK")
        col = self._mode_accent() if active else MUTED
        self._text("CURRENT COMMAND", self.f_label, MUTED, midbottom=(cx, cy - int(6 * self.u)))
        self._text(self.cmd_label, self.f_cmd, col, midtop=(cx, cy))
        bar_w, bar_h = int(min(self.W, self.H) * 0.34), int(16 * self.u)
        y = cy + int(50 * self.u)
        self._signed_bar(cx, y, bar_w, bar_h, self.out_l, "L")
        self._signed_bar(cx, y + int(28 * self.u), bar_w, bar_h, self.out_r, "R")

    def _encoder_status(self, enc, age, telem_ok, telem_detail):
        if not self.ble.connected:
            return "OFFLINE", MUTED, telem_detail or "not connected"
        if enc is None or not telem_ok:
            return "WAIT", AMBER, telem_detail or "waiting..."
        if age is None or age > 1.0:
            return "STALE", AMBER, f"last update {age:.1f}s ago" if age else "stale"
        return "LIVE", GREEN, "20 Hz"

    def _draw_encoder_card(self, rect, mid, value, live, side_col, pulse):
        u = self.u
        meta = ENCODER_META[mid]
        radius = int(14 * u)
        glow = 0.18 + 0.42 * pulse
        fill = mix(PANEL, side_col, glow if live else 0.05)
        self._round_rect(rect, fill, radius)
        self._round_border(rect, mix(PANEL_LINE, side_col, 0.35 + 0.45 * pulse), radius, max(1, int(u)))

        stripe_h = max(3, int(4 * u))
        stripe = pygame.Rect(rect.left + int(10 * u), rect.top + int(10 * u),
                             rect.width - int(20 * u), stripe_h)
        self._round_rect(stripe, mix(side_col, (255, 255, 255), 0.12), stripe_h)

        self._text(ENCODER_LABELS[mid], self.f_enc_title, side_col,
                   midtop=(rect.centerx, rect.top + int(18 * u)))
        self._text(meta["name"], self.f_enc_name, MUTED,
                   midtop=(rect.centerx, rect.top + int(34 * u)))

        if value is None:
            val_text = "---"
            val_col = MUTED
        else:
            val_text = f"{value:+d}"
            if value > 0:
                val_col = mix(TEXT, GREEN, 0.35 + 0.35 * pulse)
            elif value < 0:
                val_col = mix(TEXT, AMBER, 0.35 + 0.35 * pulse)
            else:
                val_col = TEXT

        self._text(val_text, self.f_enc_value, val_col, midtop=(rect.centerx, rect.top + int(46 * u)))
        self._text("ticks", self.f_enc_unit, MUTED, midbottom=(rect.centerx, rect.bottom - int(8 * u)))

    def _draw_encoders(self, cx, y):
        enc, age, telem_ok, telem_detail = self.ble.get_encoders()
        u = self.u
        layout = self._layout or self._compute_layout()
        panel_w = int(min(self.W, self.H) * 0.82)
        header_h = layout.get("enc_header", int(30 * u))
        card_h = layout.get("enc_row_h", int(74 * u))
        footer_h = layout.get("enc_footer", int(14 * u))
        panel_h = header_h + card_h + footer_h
        panel = pygame.Rect(cx - panel_w // 2, y, panel_w, panel_h)
        self._round_rect(panel, mix(PANEL, BG_TOP, 0.35), int(16 * u))
        self._round_border(panel, PANEL_LINE, int(16 * u), max(1, int(u)))

        status, status_col, status_hint = self._encoder_status(enc, age, telem_ok, telem_detail)
        self._text("WHEEL ENCODERS", self.f_label, ACCENT,
                   midleft=(panel.left + int(16 * u), panel.top + int(16 * u)))

        badge_w = int(74 * u)
        badge = pygame.Rect(panel.right - badge_w - int(14 * u), panel.top + int(7 * u),
                            badge_w, int(22 * u))
        pulse = 0.55 + 0.45 * math.sin(time.time() * 5) if status == "LIVE" else 0.0
        badge_col = mix(PANEL, status_col, 0.22 + 0.18 * pulse)
        self._round_rect(badge, badge_col, badge.height // 2)
        self._round_border(badge, status_col, badge.height // 2, max(1, int(u)))
        dot_x = badge.left + int(12 * u)
        aa_circle(self.screen, dot_x, badge.centery, int(4 * u), status_col if status != "OFFLINE" else MUTED)
        self._text(status, self.f_enc_title, status_col, midleft=(dot_x + int(10 * u), badge.centery))

        grid_top = panel.top + header_h
        inner_w = panel.width - int(32 * u)
        inner_x = panel.left + int(16 * u)
        card_gap = int(10 * u)
        card_w = (inner_w - card_gap * 3) // 4

        for i, mid in enumerate(ENCODER_IDS):
            rect = pygame.Rect(
                inner_x + i * (card_w + card_gap),
                grid_top,
                card_w,
                card_h,
            )
            side_col = ACCENT if ENCODER_META[mid]["side"] == "left" else ACCENT2
            value = None if enc is None else enc[mid]
            live = value is not None and telem_ok and (age is None or age <= 1.0)
            self._draw_encoder_card(rect, mid, value, live, side_col, self._enc_pulse[mid])

        hint = status_hint if enc is None or not telem_ok or (age is not None and age > 1.0) else "LV · LN · RN · RV"
        self._text(hint, self.f_enc_name, MUTED, midbottom=(panel.centerx, panel.bottom - int(5 * u)))

    def _signed_bar(self, cx, y, w, h, value, label):
        track = pygame.Rect(cx - w // 2, y, w, h)
        self._round_rect(track, PANEL, h // 2)
        mid = track.centerx
        pygame.draw.line(self.screen, PANEL_LINE, (mid, track.top),
                         (mid, track.bottom), max(1, int(self.u)))
        frac = clamp(value / SPEED_MAX, -1.0, 1.0)
        fw = int(abs(frac) * (w // 2))
        if fw > 1:
            col = GREEN if value >= 0 else AMBER
            fr = pygame.Rect(mid, track.top, fw, h) if value >= 0 \
                else pygame.Rect(mid - fw, track.top, fw, h)
            self._round_rect(fr, col, h // 2)
        self._text(label, self.f_label, MUTED,
                   midright=(track.left - int(12 * self.u), track.centery))
        self._text(f"{value:+d}", self.f_label, TEXT,
                   midleft=(track.right + int(12 * self.u), track.centery))

    def _draw_help(self, cx, panel_top):
        rows = [
            ("W / \u2191", "Forward"), ("S / \u2193", "Backward"),
            ("A / \u2190", "Turn left"), ("D / \u2192", "Turn right"),
            ("Space", "Brake / Stop"),
            ("Tab / M", "TANK / STEER"), ("G", "Smooth accel"),
            ("- / =", "Speed"), ("C / Enter", "Reconnect"), ("Esc", "Quit"),
        ]
        layout = self._layout or self._compute_layout()
        line_h = layout.get("line_h", int(28 * self.u))
        cols, cell_w = 5, int(self.W * 0.17)
        total_w = cell_w * cols
        x0 = cx - total_w // 2 + int(30 * self.u)
        panel = pygame.Rect(0, 0, total_w + int(50 * self.u), layout.get("help_h", line_h * 2 + int(46 * self.u)))
        panel.midtop = (cx, panel_top)
        self._round_rect(panel, PANEL, int(18 * self.u))
        self._round_border(panel, PANEL_LINE, int(18 * self.u), max(1, int(self.u)))
        self._text("CONTROLS", self.f_label, ACCENT, midtop=(cx, panel.top + int(10 * self.u)))
        start_y = panel.top + int(38 * self.u)
        for i, (keys, desc) in enumerate(rows):
            x = x0 + (i % cols) * cell_w
            yy = start_y + (i // cols) * line_h
            kr = self._text(keys, self.f_help_k, TEXT, topleft=(x, yy))
            self._text(desc, self.f_help_d, MUTED,
                       topleft=(x + max(int(78 * self.u), kr.width + int(12 * self.u)), yy))


def main():
    RobotApp().run()


if __name__ == "__main__":
    main()
