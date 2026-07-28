#!/bin/bash
# macOS: без flock. Один keeper через mkdir-lock.
set -u
cd /Users/VladislavOzerin/metalinspector
export INSPECTOR_AUTH=0
export INSPECTOR_ROBOT_TOKEN=a896062ed82a5b33a9d8ff5c163056f9
PY=/Library/Frameworks/Python.framework/Versions/3.11/bin/python3
[ -x "$PY" ] || PY=python3
LOG=/tmp/mi_local.log
PIDF=/tmp/mi_local.pid
LOCKDIR=/tmp/mi_local.lockdir

if ! mkdir "$LOCKDIR" 2>/dev/null; then
  # уже есть keeper
  exit 0
fi
trap 'rm -rf "$LOCKDIR"' EXIT

alive() { curl -sf -o /dev/null --max-time 2 http://127.0.0.1:8712/app/; }

while true; do
  if alive; then sleep 5; continue; fi
  if [[ -f "$PIDF" ]]; then
    old=$(cat "$PIDF" 2>/dev/null || true)
    [[ -n "${old:-}" ]] && kill "$old" 2>/dev/null || true
    sleep 1
  fi
  echo "$(date '+%H:%M:%S') start uvicorn" >>"$LOG"
  "$PY" -m uvicorn inspector.server.app:app --host 127.0.0.1 --port 8712 >>"$LOG" 2>&1 &
  echo $! >"$PIDF"
  for i in $(seq 1 20); do alive && break; sleep 0.5; done
  while kill -0 "$(cat "$PIDF" 2>/dev/null)" 2>/dev/null; do sleep 5; done
  echo "$(date '+%H:%M:%S') uvicorn exit" >>"$LOG"
  sleep 1
done
