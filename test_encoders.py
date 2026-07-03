#!/usr/bin/env python3
"""Repeated BLE encoder telemetry test using the same BLEController as app.py."""

import argparse
import sys
import time

from app import BLEController


def wait_for_encoders(ble, timeout=30.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        enc, age, telem_ok, detail = ble.get_encoders()
        if ble.connected and enc is not None and telem_ok:
            return enc, age, detail
        time.sleep(0.2)
    enc, age, telem_ok, detail = ble.get_encoders()
    return enc, age, detail


def run_cycle(cycle, connect_timeout=35.0):
    ble = BLEController()
    try:
        ble.connect()
        enc, age, detail = wait_for_encoders(ble, timeout=connect_timeout)
        ok = ble.connected and enc is not None
        status = "PASS" if ok else "FAIL"
        print(
            f"cycle {cycle:02d}: {status} "
            f"state={ble.state} telem={ble.telem_ok} detail={detail!r} "
            f"enc={enc} age={age} notifies={ble._notify_count}"
        )
        return ok
    finally:
        ble.shutdown()
        time.sleep(1.0)


def main():
    parser = argparse.ArgumentParser(description="Stress-test encoder telemetry.")
    parser.add_argument("-n", "--cycles", type=int, default=20, help="Number of connect cycles")
    parser.add_argument("--min-pass", type=int, default=15, help="Minimum passing cycles")
    args = parser.parse_args()

    passed = 0
    for i in range(1, args.cycles + 1):
        if run_cycle(i):
            passed += 1

    print(f"\nSUMMARY: {passed}/{args.cycles} cycles passed")
    return 0 if passed >= args.min_pass else 1


if __name__ == "__main__":
    sys.exit(main())
