#!/usr/bin/env python3
"""Пока BLE молчит: USB ESP ↔ metalinspector.ru.

  INSPECTOR_ROBOT_TOKEN=... python3 control/site_usb_bridge.py

Читает команды пульта с сайта и пишет в Serial.
Строки go/SCAN_START с ESP → POST /api/robot/go (Pi снимает кадр).
"""
from __future__ import annotations

import glob
import os
import sys
import time

import httpx
import serial

SERVER = os.environ.get("INSPECTOR_SERVER_URL", "http://127.0.0.1:8712").rstrip("/")
TOKEN = os.environ.get("INSPECTOR_ROBOT_TOKEN", "a896062ed82a5b33a9d8ff5c163056f9")
BAUD = 115200


def headers():
    return {"X-Robot-Token": TOKEN}


def open_port():
    # Serial прошивки — нативный USB CDC (usbmodem); CH340 (wchusbserial) команды не получает
    ports = glob.glob("/dev/cu.usbmodem*") or glob.glob("/dev/cu.wchusbserial*")
    if not ports:
        raise RuntimeError("нет USB ESP")
    ser = serial.Serial(ports[0], BAUD, timeout=0.05)
    print("USB", ports[0], flush=True)
    return ser


def main():
    if not TOKEN:
        sys.exit("нет токена")
    ser = open_port()
    buf = ""
    print("bridge →", SERVER, flush=True)
    while True:
        try:
            r = httpx.get(
                f"{SERVER}/api/robot/poll",
                headers=headers(),
                params={"wait": 8},
                timeout=25,
            )
            r.raise_for_status()
            data = r.json()
        except Exception as e:
            print("poll", e, flush=True)
            time.sleep(1)
            continue

        for cmd in data.get("cmds") or []:
            if cmd.startswith("ATTACH:"):
                print("attach", cmd, flush=True)
                continue
            try:
                ser.write((cmd.strip() + "\n").encode())
                print("→ESP", cmd, flush=True)
            except Exception as e:
                print("write", e, flush=True)
                try:
                    ser.close()
                except Exception:
                    pass
                ser = open_port()

        # drain serial
        try:
            chunk = ser.read(ser.in_waiting or 1).decode("utf-8", "replace")
        except Exception:
            chunk = ""
            try:
                ser.close()
            except Exception:
                pass
            time.sleep(0.5)
            ser = open_port()
        if chunk:
            buf += chunk
            *lines, buf = buf.replace("\r", "\n").split("\n")
            for line in lines:
                low = line.strip().lower()
                if not low:
                    continue
                if low in ("go", "scan_start", "camera_on") or low.startswith("go"):
                    try:
                        httpx.post(
                            f"{SERVER}/api/robot/go",
                            headers=headers(),
                            json={"line": low},
                            timeout=10,
                        )
                        print("go→site", low, flush=True)
                    except Exception as e:
                        print("go post", e, flush=True)
                if low.startswith("e,"):
                    try:
                        httpx.post(
                            f"{SERVER}/api/robot/telem",
                            headers=headers(),
                            json={"telem": line.strip()[:200], "ble_ok": True},
                            timeout=5,
                        )
                    except Exception:
                        pass


if __name__ == "__main__":
    main()
