#!/usr/bin/env python3
"""Read PID + encoder telemetry from ESP32 over USB Serial.

The firmware in robot/robot.ino streams CSV lines at 20 Hz:
  T,ms,lv_cnt,ln_cnt,rn_cnt,rv_cnt,
  lv_vel,ln_vel,rn_vel,rv_vel,
  lv_tgt,ln_tgt,rn_tgt,rv_tgt,
  lv_pwm,ln_pwm,rn_pwm,rv_pwm,
  lv_err,ln_err,rn_err,rv_err

Usage:
  python serial_monitor.py                  # auto-detect port
  python serial_monitor.py /dev/cu.usbserial-XXXX
"""

import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Install pyserial: pip install pyserial")
    sys.exit(1)

BAUD = 115200
FIELDS = [
    "ms",
    "lv_cnt", "ln_cnt", "rn_cnt", "rv_cnt",
    "lv_vel", "ln_vel", "rn_vel", "rv_vel",
    "lv_tgt", "ln_tgt", "rn_tgt", "rv_tgt",
    "lv_pwm", "ln_pwm", "rn_pwm", "rv_pwm",
    "lv_err", "ln_err", "rn_err", "rv_err",
]


def find_port():
    for p in list_ports.comports():
        desc = (p.description or "").lower()
        if any(k in desc for k in ("usb", "serial", "uart", "cp210", "ch340", "esp32")):
            return p.device
    ports = [p.device for p in list_ports.comports()]
    return ports[0] if ports else None


def parse_line(line):
    parts = line.strip().split(",")
    if len(parts) != len(FIELDS) + 1 or parts[0] != "T":
        return None
    try:
        return dict(zip(FIELDS, parts[1:]))
    except ValueError:
        return None


def fmt_row(data):
    enc = "enc LV={lv_cnt} LN={ln_cnt} RN={rn_cnt} RV={rv_cnt}".format(**data)
    vel = "vel LV={lv_vel} LN={ln_vel} RN={rn_vel} RV={rv_vel}".format(**data)
    pwm = "pwm LV={lv_pwm} LN={ln_pwm} RN={rn_pwm} RV={rv_pwm}".format(**data)
    return f"[{data['ms']} ms] {enc} | {vel} | {pwm}"


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if not port:
        print("No serial port found. Pass the port explicitly.")
        return 1

    print(f"Opening {port} @ {BAUD} baud...")
    with serial.Serial(port, BAUD, timeout=1) as ser:
        time.sleep(2)  # wait for ESP32 reset after USB open
        ser.reset_input_buffer()
        print("Listening for telemetry (Ctrl+C to quit)\n")

        while True:
            raw = ser.readline().decode("utf-8", errors="ignore")
            if not raw:
                continue
            if raw.startswith("T,"):
                data = parse_line(raw)
                if data:
                    print(fmt_row(data))
            elif raw.strip():
                print(f"LOG: {raw.rstrip()}")

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main() or 0)
    except KeyboardInterrupt:
        print("\nStopped.")
