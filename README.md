# ESP32 BLE Robot

A 4-motor robot (2 dual motor drivers) controlled over BLE. The ESP32 runs a
NimBLE GATT server; a Python script (using [bleak](https://github.com/hbldh/bleak))
connects and sends single-character drive commands.

## Files

- `robot.ino` — ESP32 Arduino sketch (NimBLE-Arduino 2.x, ESP32 core 3.x).
- `client.py` — Python BLE client (bleak).
- `requirements.txt` — Python dependencies.

## Wiring assumptions

- **Motor driver type:** two-PWM-input per motor (in/in mode), e.g. **DRV8833**
  or **TB6612FNG**. Each motor uses a DIR pin and a PWM pin, and *both* are
  driven with `analogWrite()`:
  - forward: `DIR = 0`, `PWM = speed`
  - reverse: `DIR = speed`, `PWM = 0`
  - stop: `DIR = 0`, `PWM = 0`
  > If your driver is a PH/EN style (one direction pin + one enable PWM), the
  > control logic would differ — this sketch assumes in/in mode.
- **Pin map** (ESP32 GPIO numbers):

  | Motor          | DIR | PWM |
  |----------------|-----|-----|
  | Driver1 Motor1 | 10  | 11  |
  | Driver1 Motor2 | 37  | 36  |
  | Driver2 Motor1 | 2   | 1   |
  | Driver2 Motor2 | 7   | 6   |

- `speedVal` defaults to `150` (range 0–255; `analogWrite` is 8-bit on ESP32).
- Verify these GPIOs are valid/output-capable on your specific ESP32 module.

## Required libraries

- **Arduino:** [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) **2.x**
  (Arduino IDE → Library Manager → "NimBLE-Arduino"). Build for the ESP32
  Arduino core **3.x**.
- **Python:** `pip install bleak` (or `pip install -r requirements.txt`).

## Flash the ESP32

1. Open `robot.ino` in the Arduino IDE.
2. Install the **esp32** board package and select your board.
3. Install **NimBLE-Arduino 2.x** via Library Manager.
4. Upload. Open Serial Monitor at **115200** baud to see logs
   (`ESP32_ROBOT advertising...`, connection events, received commands).

## Run the GUI controller (recommended)

A fullscreen, game-style desktop app: hold **WASD** / **arrow keys** (or click &
hold the on-screen D-pad) to drive, release to stop — just like driving in a
game. Rendered at native (Retina) resolution with anti-aliasing, so it stays
crisp instead of pixelated.

```bash
pip install -r requirements.txt
python3 app.py
```

- It auto-scans and connects to `ESP32_ROBOT` on launch.
- **W/↑** forward, **S/↓** back, **A/←** left, **D/→** right.
- **Space** = brake/stop, **C / Enter** = reconnect, **Esc** = quit.
- **Tab / M** (or the on-screen switch) toggles the driving mode.
- The latest pressed key wins, and the active command is resent periodically so
  a dropped BLE packet can't leave the robot stuck.

### Driving modes & options (each toggles independently)

- **TANK mode** (`Tab` / `M`) — left/right spin the robot in place (true tank
  turn). The original behavior.
- **STEER mode** (`Tab` / `M`) — left/right *curve* the robot smoothly while it
  drives, like a car. This is real proportional steering: the app computes a
  different speed for each side and sends it to the robot.
- **Max speed** (`-`/`=`, `[`/`]`, mouse wheel, or drag the on-screen bar) —
  caps how fast the wheels run (30–255).
- **Smooth accel** (`G` or the on-screen switch) — when on, side speeds *ramp*
  toward their target for smooth starts/stops instead of snapping.

The app shows live per-side wheel output (L / R) at the bottom.

This needs the updated firmware (see protocol below). Re-flash `robot.ino`.

## BLE command protocol

The robot accepts ASCII commands written to the characteristic:

- **`M<left>,<right>`** — differential drive. `left`/`right` are signed integers
  in `-255..255` (positive = forward, negative = reverse) applied to each side.
  Examples: `M150,150` forward, `M-150,150` spin left, `M150,60` arc right,
  `M0,0` stop. This is what `app.py` uses for steering, variable speed and
  smooth ramping.
- **`F` / `B` / `L` / `R` / `S`** — legacy single-character commands (forward,
  back, left-spin, right-spin, stop) at a fixed speed. Used by `client.py`.

Both are handled by `robot/robot.ino`, so re-flash it after updating.

## Run the simple console client

```bash
pip install -r requirements.txt
python3 client.py
```

The client scans for `ESP32_ROBOT` (falling back to the service UUID if the
advertised name is missing), connects, then prompts for commands:

```
F/B/L/R/S (Q to quit):
```

- `F` forward, `B` backward, `L` turn left, `R` turn right, `S` stop.
- Lowercase is accepted; unknown commands are rejected.
- `Q` (or Ctrl+C) sends a stop and disconnects cleanly.

## Safety behavior

- Motors are stopped at startup before BLE init.
- On BLE disconnect the ESP32 stops all motors and restarts advertising, so the
  robot won't keep driving if the client drops, and you can reconnect.
- Unknown commands map to stop.
