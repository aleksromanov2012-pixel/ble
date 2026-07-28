#!/bin/bash
# Пульт МеталИнспектор — по умолчанию ВСЁ через BLE (змейка + телеметрия)
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
LOG=/tmp/ble-bridge.log
PIDFILE=/tmp/ble-bridge.pid

# ble | usb | auto
export ROBOT_LINK="${ROBOT_LINK:-ble}"

alive() {
  pgrep -f 'control/bridge.py' >/dev/null 2>&1 && \
    curl -sS -m 1 -o /dev/null http://127.0.0.1:8765/ 2>/dev/null
}

if alive; then
  echo "уже запущен → http://127.0.0.1:8765/  (ROBOT_LINK=$ROBOT_LINK)"
  open "http://127.0.0.1:8765/" 2>/dev/null || true
  exit 0
fi

if lsof -nP -iTCP:8765 -sTCP:LISTEN >/dev/null 2>&1; then
  kill $(lsof -t -nP -iTCP:8765 -sTCP:LISTEN) 2>/dev/null || true
  sleep 0.5
fi

PY="$(command -v python3)"
if [ -x "$ROOT/.venv/bin/python" ]; then
  PY="$ROOT/.venv/bin/python"
fi

echo "[start] ROBOT_LINK=$ROBOT_LINK  python=$PY" | tee -a "$LOG"
nohup env ROBOT_LINK="$ROBOT_LINK" "$PY" -u "$ROOT/control/bridge.py" >>"$LOG" 2>&1 &
echo $! >"$PIDFILE"
sleep 1.5

if alive; then
  echo "пульт → http://127.0.0.1:8765/  (канал $ROBOT_LINK)"
  open "http://127.0.0.1:8765/" 2>/dev/null || true
else
  echo "не поднялся — $LOG:"
  tail -40 "$LOG"
  exit 1
fi
