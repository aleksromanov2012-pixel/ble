#!/usr/bin/env python3
"""BLE client for the ESP32_ROBOT.

Scans for the robot (by name, with a service-UUID fallback), connects, and
forwards single-character drive commands over a BLE write characteristic.

Commands: F (forward), B (backward), L (left), R (right), S (stop), Q (quit).

Requires: pip install bleak
"""

import asyncio
import sys

from bleak import BleakScanner, BleakClient

DEVICE_NAME = "ESP32_ROBOT"
SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab"
CHAR_UUID = "abcdefab-1234-5678-1234-abcdefabcdef"

VALID_COMMANDS = {"F", "B", "L", "R", "S"}
SCAN_TIMEOUT = 10.0


async def find_device():
    """Return a matching BLE device, or None.

    Matches by advertised name first, then falls back to the service UUID
    (handy when the name is None in the scan results).
    """
    print(f"Scanning for '{DEVICE_NAME}' ({SCAN_TIMEOUT:.0f}s)...")
    # return_adv=True gives us (device, advertisement_data) pairs so we can
    # inspect advertised service UUIDs for the fallback match.
    found = await BleakScanner.discover(timeout=SCAN_TIMEOUT, return_adv=True)

    by_name = None
    by_service = None
    for device, adv in found.values():
        name = device.name or adv.local_name
        if name == DEVICE_NAME:
            by_name = device
            break
        uuids = [u.lower() for u in (adv.service_uuids or [])]
        if SERVICE_UUID.lower() in uuids:
            by_service = device

    return by_name or by_service


def read_command():
    """Blocking prompt; returns an uppercase command string (may be empty)."""
    try:
        return input("F/B/L/R/S (Q to quit): ").strip().upper()
    except EOFError:
        return "Q"


async def run_client(device):
    async with BleakClient(device, services=[SERVICE_UUID]) as client:
        print(f"Connected to {device.address}")
        try:
            while True:
                if not client.is_connected:
                    print("Connection lost.")
                    return
                # input() blocks, so run it in a thread to keep the event loop
                # responsive (lets disconnects/KeyboardInterrupt be handled).
                cmd = await asyncio.to_thread(read_command)

                if cmd == "Q":
                    print("Quitting: sending stop.")
                    await safe_write(client, "S")
                    return
                if cmd == "":
                    continue
                if cmd not in VALID_COMMANDS:
                    print(f"Unknown command '{cmd}'. Use F/B/L/R/S or Q.")
                    continue

                await safe_write(client, cmd)
        except KeyboardInterrupt:
            print("\nInterrupted: sending stop.")
            await safe_write(client, "S")


async def safe_write(client, cmd):
    """Write a command, ignoring failures if the link just dropped."""
    try:
        await client.write_gatt_char(CHAR_UUID, cmd.encode(), response=False)
    except Exception as e:  # noqa: BLE001 - best-effort on a flaky link
        print(f"Write failed: {e}")


async def main():
    device = await find_device()
    if device is None:
        print(f"'{DEVICE_NAME}' NOT FOUND. Is the robot powered on and advertising?")
        return 1

    try:
        await run_client(device)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as e:  # noqa: BLE001
        print(f"Connection error: {e}")
        return 1
    print("Disconnected.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(asyncio.run(main()) or 0)
    except KeyboardInterrupt:
        pass
