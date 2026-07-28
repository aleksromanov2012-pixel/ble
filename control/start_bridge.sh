#!/bin/bash
# Пульт http://127.0.0.1:8765/ — по умолчанию BLE. Без pkill -f (убивал процесс).
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOG=/tmp/ble-bridge.log
PIDFILE=/tmp/ble-bridge.pid
export ROBOT_LINK="${ROBOT_LINK:-ble}"

PY="$ROOT/.venv/bin/python"
[ -x "$PY" ] || PY="$(command -v python3)"

alive() {
  curl -sS -m 1 -o /dev/null http://127.0.0.1:8765/api/status 2>/dev/null
}

pid_alive() {
  local p
  p=$(cat "$PIDFILE" 2>/dev/null || true)
  [ -n "${p:-}" ] && kill -0 "$p" 2>/dev/null
}

stop_old() {
  local p
  p=$(cat "$PIDFILE" 2>/dev/null || true)
  if [ -n "${p:-}" ]; then
    kill "$p" 2>/dev/null || true
    sleep 0.3
    kill -9 "$p" 2>/dev/null || true
  fi
  # Only the python that runs bridge.py — not parent shells
  for p in $(pgrep -f "/Users/xxx/BLE/control/bridge.py" 2>/dev/null || true); do
    kill "$p" 2>/dev/null || true
  done
  if lsof -nP -iTCP:8765 -sTCP:LISTEN >/dev/null 2>&1; then
    kill $(lsof -t -nP -iTCP:8765 -sTCP:LISTEN) 2>/dev/null || true
  fi
  sleep 0.3
  rm -f "$PIDFILE"
}

if alive && pid_alive; then
  echo "уже жив → http://127.0.0.1:8765/  (ROBOT_LINK=$ROBOT_LINK)"
  open "http://127.0.0.1:8765/" 2>/dev/null || true
  exit 0
fi

if alive; then
  # HTTP up but pidfile stale — leave it
  echo "порт 8765 отвечает → http://127.0.0.1:8765/"
  open "http://127.0.0.1:8765/" 2>/dev/null || true
  exit 0
fi

stop_old
cd "$ROOT"
echo "[$(date '+%F %T')] start ROBOT_LINK=$ROBOT_LINK $PY" >>"$LOG"
# setsid/nohup so Cursor session end does not kill BLE bridge
if command -v setsid >/dev/null 2>&1; then
  setsid env ROBOT_LINK="$ROBOT_LINK" "$PY" -u "$ROOT/control/bridge.py" >>"$LOG" 2>&1 </dev/null &
else
  nohup env ROBOT_LINK="$ROBOT_LINK" "$PY" -u "$ROOT/control/bridge.py" >>"$LOG" 2>&1 </dev/null &
fi
echo $! >"$PIDFILE"

for _ in 1 2 3 4 5 6 7 8 9 10 11 12; do
  sleep 0.4
  if alive; then
    echo "пульт → http://127.0.0.1:8765/  (канал $ROBOT_LINK)"
    open "http://127.0.0.1:8765/" 2>/dev/null || true
    exit 0
  fi
done

echo "не поднялся — $LOG:"
tail -50 "$LOG"
exit 1
