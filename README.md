How to build PlatformIO based project
=====================================

1. [Install PlatformIO Core](https://docs.platformio.org/page/core.html)
2. Download [development platform with examples](https://github.com/platformio/platform-ststm32/archive/develop.zip)
3. Extract ZIP archive
4. Run these commands:

```shell
# Change directory to example
$ cd platform-ststm32/examples/stm32cube-hal-blink

# Build project
$ pio run

# Upload firmware
$ pio run --target upload

# Build specific environment
$ pio run -e nucleo_f401re

# Upload firmware for the specific environment
$ pio run -e nucleo_f401re --target upload

# Clean build files
$ pio run --target clean
```

## Настройка радиомодуля NRF24L01

Для настройки адреса радиомодуля отредактируйте файл `src/USER_DEFINES/radio_settings.h`:

```c
/**
 * @brief Адрес радиомодуля для связи между передатчиком и приемником
 * @note  Этот адрес должен быть одинаковым на передатчике и приемнике
 * @note  Изменяйте этот адрес для настройки уникальной связи
 */
#define RADIO_ADDRESS {0x12, 0x34, 0x56, 0x78, 0x9A}
```

**Важно:** Тот же адрес должен быть установлен в приемнике в файле `nrf_test/src/radio_settings.h`

### Функции радиомодуля:
- **Передача педалей:** Работает только от внутренней батареи
- **Режим привязки:** Удерживайте обе педали 3+ секунд для входа в режим привязки
- **Индикация ошибок:** Красное мигание при ошибке передачи, зеленое при успехе
