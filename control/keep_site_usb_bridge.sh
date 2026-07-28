#!/bin/bash
set -u
cd /Users/VladislavOzerin/metalinspector
export INSPECTOR_ROBOT_TOKEN=a896062ed82a5b33a9d8ff5c163056f9
export INSPECTOR_SERVER_URL=http://127.0.0.1:8712
PY=/Library/Frameworks/Python.framework/Versions/3.11/bin/python3
[ -x "$PY" ] || PY=python3
LOG=/tmp/site_usb_bridge.log
LOCKDIR=/tmp/site_usb_bridge.lockdir

if ! mkdir "$LOCKDIR" 2>/dev/null; then
  exit 0
fi
trap 'rm -rf "$LOCKDIR"' EXIT

while true; do
  if pgrep -f 'control/site_usb_bridge.py' >/dev/null 2>&1; then sleep 5; continue; fi
  if ! ls /dev/cu.usbmodem* >/dev/null 2>&1 && ! ls /dev/cu.wchusbserial* >/dev/null 2>&1; then
    echo "$(date '+%H:%M:%S') wait USB" >>"$LOG"; sleep 3; continue
  fi
  if ! curl -sf -o /dev/null --max-time 2 http://127.0.0.1:8712/app/; then
    echo "$(date '+%H:%M:%S') wait server" >>"$LOG"; sleep 2; continue
  fi
  echo "$(date '+%H:%M:%S') start bridge → http://127.0.0.1:8712" >>"$LOG"
  INSPECTOR_SERVER_URL=http://127.0.0.1:8712 INSPECTOR_ROBOT_TOKEN=$INSPECTOR_ROBOT_TOKEN \
    "$PY" control/site_usb_bridge.py >>"$LOG" 2>&1 || true
  echo "$(date '+%H:%M:%S') bridge exit $?" >>"$LOG"
  sleep 2
done
