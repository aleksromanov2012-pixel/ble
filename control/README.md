# Пульт робота — канал BLE

## Запуск

```bash
cd /Users/xxx/BLE
bash control/start_bridge.sh
```

→ **http://127.0.0.1:8765/**  
`ROBOT_LINK=ble` (по умолчанию): команды и телеметрия только по Bluetooth.

## Управление

| Клавиша / кнопка | Действие |
|---|---|
| WASD / стрелки | Ручная езда, **без края ToF** |
| Space | Стоп |
| − / = / слайдер | Скорость **60…255** (макс по умолчанию) |
| Авто (G) | Змейка; край ToF снова **ВКЛ** |
| C | Сброс latch края |
| X0 / X1 | Край off / on |

## Змейка

План размера мм → **Авто (G)** → `P…` затем `G` по BLE.  
Чётная полоса: назад → направо → сдвиг → направо. Нечётная: налево.

## Прошивка

```bash
bash scripts/flash_robot.sh
# firmware/metalinspector_robot — не robot/robot.ino
```

Пины LOCKED. LN invert. Trim L+0 / R+110. `DRIVE_ONLY=1` (без камеры).
