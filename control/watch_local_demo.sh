#!/bin/bash
cd /Users/VladislavOzerin/metalinspector
PY=/Library/Frameworks/Python.framework/Versions/3.11/bin/python3
export INSPECTOR_AUTH=0
export INSPECTOR_ROBOT_TOKEN=a896062ed82a5b33a9d8ff5c163056f9
export INSPECTOR_SERVER_URL=http://127.0.0.1:8712
while true; do
  if ! curl -sf -o /dev/null --max-time 2 http://127.0.0.1:8712/app/; then
    pgrep -f 'uvicorn inspector.server.app' >/dev/null || \
      nohup "$PY" -m uvicorn inspector.server.app:app --host 127.0.0.1 --port 8712 >>/tmp/mi_local.log 2>&1 &
  fi
  if ! pgrep -f 'control/site_usb_bridge.py' >/dev/null; then
    if ls /dev/cu.usbmodem* >/dev/null 2>&1 || ls /dev/cu.wchusbserial* >/dev/null 2>&1; then
      if curl -sf -o /dev/null --max-time 2 http://127.0.0.1:8712/app/; then
        nohup env INSPECTOR_SERVER_URL=http://127.0.0.1:8712 INSPECTOR_ROBOT_TOKEN=$INSPECTOR_ROBOT_TOKEN \
          "$PY" control/site_usb_bridge.py >>/tmp/site_usb_bridge.log 2>&1 &
      fi
    fi
  fi
  sleep 5
done
