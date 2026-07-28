# Пульт робота МеталИнспектор (режим езды, камера offline)

## Запуск

```bash
cd /Users/xxx/BLE
pip install -r requirements.txt   # bleak + pyserial
bash control/start_bridge.sh
```

Открыть: http://127.0.0.1:8765/

- Есть USB-кабель → канал USB  
- Нет кабеля → автопоиск BLE `ESP32_ROBOT`

## Управление

| Кнопка / клавиша | Команда |
|---|---|
| Авто + карта | `G` — змейка по плану `Pwidth,height` |
| Вперёд / ↑ W | `F` — прямо + PI по энкодерам + стоп по краю |
| Назад / ↓ S | `B` |
| Влево / ← A | `L` |
| Вправо / → D | `R` |
| СТОП / Space | `S` — short-brake |
| Сброс края | `C` |
| Защита off | `X0` (только отладка), `X1` — снова ON |

Скорость: слайдер шлёт `V30`…`V90` (авто capped ~75).

## Прошивка (обязательно эта, не старый `robot/robot.ino`)

```bash
bash scripts/flash_robot.sh
# или:
arduino-cli compile -b esp32:esp32:esp32s3 firmware/metalinspector_robot
arduino-cli upload -p /dev/cu.wchusbserial* -b esp32:esp32:esp32s3 firmware/metalinspector_robot
```

Библиотеки Arduino: **NimBLE-Arduino 2.x**, **VL53L0X** (Pololu).

Пины моторов/энкодеров — LOCKED (см. `.cursor/rules/robot-hardware-pins.mdc`).
LN инвертирован в прошивке. Trim влево +52 PWM против увода вправо.
`DRIVE_ONLY=1`: после покрытия нет второго прохода с камерой.
