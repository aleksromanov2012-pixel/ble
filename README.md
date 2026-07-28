# ESP32 BLE Robot — МеталИнспектор (ездa)

Рабочий стек езды из дампа `metalins` (камера на Pi сейчас offline — режим **DRIVE_ONLY**).

## Что использовать

| Компонент | Путь | Назначение |
|-----------|------|------------|
| **Прошивка** | `firmware/metalinspector_robot/` | PI-прямолинейность, trim, ToF-край, змейка `G` |
| **Пульт** | `control/bridge.py` + `index.html` | USB/BLE → http://127.0.0.1:8765/ |
| Flash | `scripts/flash_robot.sh` | arduino-cli → ESP32-S3 |
| Legacy GUI | `app.py` | pygame WASD (команды `M`/`F`… совместимы) |
| Legacy sketch | `robot/robot.ino` | **не прошивать** — старая простая езда без PI/ToF |

## Быстрый старт (только езда)

1. Прошей ESP32-S3 (нужен data-USB):
   ```bash
   bash scripts/flash_robot.sh
   ```
2. Запусти пульт:
   ```bash
   pip install -r requirements.txt
   bash control/start_bridge.sh
   ```
3. Браузер: **http://127.0.0.1:8765/**  
   Без кабеля — BLE `ESP32_ROBOT`. С кабелем — USB приоритетнее.

## Управление

- **W/A/S/D** или кнопки — ручная езда  
- **G** — авто-змейка (сначала план `Pширина,высота` мм с пульта)  
- **Space** — стоп  
- **C** — сброс latch края  
- Слайдер скорости → `V30`…`V90`

## Железо (LOCKED)

Моторы DIR/PWM и энкодеры — см. `.cursor/rules/robot-hardware-pins.mdc`.  
В прошивке LN инвертирован; trim влево +52. ToF VL53L0X на I2C (SDA8/SCL9, XSHUT 12–15).

## Камера

Сломана на Pi — в прошивке `DRIVE_ONLY=1`: после покрытия нет `CAMERA_ON` / второго SCAN-прохода. События `go` можно игнорировать.
