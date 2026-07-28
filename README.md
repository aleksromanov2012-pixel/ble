# ESP32 BLE Robot — МеталИнспектор (ездa)

Рабочий стек езды (камера на Pi offline — **DRIVE_ONLY**).

## Что использовать

| Компонент | Путь | Назначение |
|-----------|------|------------|
| **Прошивка** | `firmware/metalinspector_robot/` | змейка `G`, ToF-край (только авто), WASD без края |
| **Пульт** | `control/bridge.py` + `index.html` | BLE → http://127.0.0.1:8765/ |
| Flash | `scripts/flash_robot.sh` | arduino-cli → ESP32-S3 |

## Быстрый старт

```bash
bash scripts/flash_robot.sh          # data-USB
bash control/start_bridge.sh         # ROBOT_LINK=ble по умолчанию
# → http://127.0.0.1:8765/
```

Канал по умолчанию **BLE**. USB-кабель можно оставить для питания/прошивки — пульт serial не перехватывает при `ROBOT_LINK=ble`.

## Управление

- **WASD** / стрелки — ручная езда, **края ToF выкл** (свободная езда)
- **Space** — стоп; **− / =** или слайдер — скорость **60…255** (дефолт **255**)
- **G** — авто-змейка (край снова ВКЛ): назад → 90 → сдвиг → 90…
- **C** — сброс latch; **X0/X1** — край вручную off/on

## Железо (LOCKED)

Пины — `.cursor/rules/robot-hardware-pins.mdc`. LN инвертирован. Trim сейчас **L+0 / R+110** (против увода влево). ToF VL53L0X I2C SDA8/SCL9.
