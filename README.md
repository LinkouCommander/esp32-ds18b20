# ESP32 DS18B20 Library
This is a lightweight DS18B20 driver for ESP32, implemented in C with ESP-IDF.  
It provides both the **One-Wire protocol implementation** and a **DS18B20 user interface** in a single library.

## Notes
- Tested in **PlatformIO** with **ESP-IDF framework**  
- Supports **multiple sensors on a single GPIO pin**  
- Supports **multiple pins simultaneously**  
- **Parasite power mode is not supported**

---

## Installation

### Option 1: PlatformIO Registry
Add the following to your `platformio.ini`:
```ini
lib_deps = LinkouCommander/esp32-ds18b20
```

### Option 2: Local Library
Clone ds18b20.c and ds18b20.h into your lib folder:
```lua
|-- lib
|   |-- ds18b20
|       |- ds18b20.c
|       |- ds18b20.h
```

---

## Usage
### Initialization
```c
// Initialize device with GPIO pin, call at startup
void ds18b20_init(ds18b20_dev_t* dev, gpio_num_t pin);
```

### Device Search
```c
// Reset the search state
void ds18b20_search_reset(ds18b20_dev_t* dev);
// Search for devices
long ds18b20_search(ds18b20_dev_t* dev, uint8_t *newAddr);
```

### Temperature Reading
```c
// Request all sensors to update temperature values.
// Call this before reading temperatures.
bool ds18b20_requestTemperatures(ds18b20_dev_t* dev);

// Read current temperature
float ds18b20_getTempC(ds18b20_dev_t* dev, uint8_t* addr);
float ds18b20_getTempF(ds18b20_dev_t* dev, uint8_t* addr);
```

---

## References
- Based on: 
  - [feelfreelinux/ds18b20](https://github.com/feelfreelinux/ds18b20)  
  - [esp-idf-lib/ds18x20](https://github.com/esp-idf-lib/ds18x20)
- Datasheet: [DS18B20 PDF](https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf)  
- Arduino libraries:  
  - `<OneWire.h>`  
  - `<DallasTemperature.h>`  