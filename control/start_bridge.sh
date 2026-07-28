#!/bin/bash
# Стабильный запуск пульта (не умирает с терминалом Cursor)
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
LOG=/tmp/ble-bridge.log
PIDFILE=/tmp/ble-bridge.pid

alive() {
  pgrep -f 'control/bridge.py' >/dev/null 2>&1 && \
    curl -sS -m 1 -o /dev/null http://127.0.0.1:8765/ 2>/dev/null
}

if alive; then
  echo "уже запущен → http://127.0.0.1:8765/"
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

nohup "$PY" -u "$ROOT/control/bridge.py" >>"$LOG" 2>&1 &
echo $! >"$PIDFILE"
sleep 1.2

if alive; then
  echo "пульт → http://127.0.0.1:8765/"
  open "http://127.0.0.1:8765/" 2>/dev/null || true
else
  echo "не поднялся — $LOG:"
  tail -40 "$LOG"
  exit 1
fi
