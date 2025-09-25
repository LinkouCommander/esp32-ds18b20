/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "ds18b20.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

// OneWire commands
#define SEARCHROM       0xF0  // 
#define CONVERTT     	0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define SKIPROM			0xCC  // Command to address all devices on the bus
#define SELECTROM	    0x55  // Command to address all devices on the bus
#define COPYSCRATCH     0x48  // Copy scratchpad to EEPROM
#define READSCRATCH     0xBE  // Read from scratchpad
#define WRITESCRATCH    0x4E  // Write to scratchpad
#define RECALLSCRATCH   0xB8  // Recall from EEPROM to scratchpad
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power
#define ALARMSEARCH     0xEC  // Query bus for devices with an alarm condition
// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP  3
#define CONFIGURATION   4
#define INTERNAL_BYTE   5
#define COUNT_REMAIN    6
#define COUNT_PER_C     7
#define SCRATCHPAD_CRC  8
// DSROM FIELDS
#define DSROM_FAMILY    0
#define DSROM_CRC       7
// Device resolution
#define TEMP_9_BIT  0x1F //  9 bit
#define TEMP_10_BIT 0x3F // 10 bit
#define TEMP_11_BIT 0x5F // 11 bit
#define TEMP_12_BIT 0x7F // 12 bit
// Constants for device models
#define DS18S20MODEL 0x10  // also DS1820
#define DS18B20MODEL 0x28  // also MAX31820
#define DS1822MODEL  0x22
#define DS1825MODEL  0x3B  // also MAX31850
#define DS28EA00MODEL 0x42

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#define PORT_ENTER_CRITICAL portENTER_CRITICAL(&mux)
#define PORT_EXIT_CRITICAL portEXIT_CRITICAL(&mux)

static uint8_t _crc8(const uint8_t *addr, uint8_t len);
static int32_t ds18b20_calculateTemperature(uint8_t* scratchPad);
// static const char *TAG = "DS18B20";

// Initialization
// since the pull mode is set to GPIO_FLOATING, an external pull-up resistor is required
static void _init(gpio_num_t pin) {
    gpio_reset_pin(pin);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT_OD);
    // gpio_set_pull_mode(pin, GPIO_FLOATING);
}

// Sends reset pulse
static bool _reset(gpio_num_t pin) {
    bool presence;
    uint8_t retries = 125;

    gpio_set_level(pin, 1);
    // wait until the wire is high... just in case
    do {
        if (--retries == 0) return false;
        esp_rom_delay_us(5);
    } while ( gpio_get_level(pin) == 0);

	gpio_set_level(pin, 0);
	esp_rom_delay_us(480);
	PORT_ENTER_CRITICAL;
	gpio_set_level(pin, 1);
	esp_rom_delay_us(70);
    if(gpio_get_level(pin) == 0) presence = true;
    PORT_EXIT_CRITICAL;
	esp_rom_delay_us(410);

    // ESP_LOGI(TAG, "presence state: %d", presence);

	return presence;
}
// Sends one bit to bus
static void _write_bit(gpio_num_t pin, int bit){
	if (bit) {
        PORT_ENTER_CRITICAL;
		gpio_set_level(pin,0);
		esp_rom_delay_us(6);
		gpio_set_level(pin,1);
        PORT_EXIT_CRITICAL;
		esp_rom_delay_us(64);
	} else {
        PORT_ENTER_CRITICAL;
		gpio_set_level(pin,0);
		esp_rom_delay_us(65);
        gpio_set_level(pin, 1);
        PORT_EXIT_CRITICAL;
        esp_rom_delay_us(5);
	}
}
// Sends one byte to bus
static void _write(gpio_num_t pin, const uint8_t byte){
    for(int i = 0; i < 8; i++) {
        int bit = (byte >> i) & 0x01;
        _write_bit(pin, bit);
    }
}
// Reads one bit from bus
static int _read_bit(gpio_num_t pin) {
	int r = 0;
	PORT_ENTER_CRITICAL;
	gpio_set_level(pin, 0);
	esp_rom_delay_us(3);
	gpio_set_level(pin, 1);
	esp_rom_delay_us(10);
	r = gpio_get_level(pin);
	PORT_EXIT_CRITICAL;
    esp_rom_delay_us(57);
	return r;
}
// Reads one byte from bus
static uint8_t _read(gpio_num_t pin) {
    uint8_t byte = 0;
    for(int i = 0; i < 8; i++) {
        int bit = _read_bit(pin);
        byte |= (bit << i);
    }
    return byte;
}

// Do a ROM select (Select device with address (ROM))
static void _select(gpio_num_t pin, const uint8_t* addr) {
    _write(pin, SELECTROM); // Choose ROM
    for (int i = 0; i < 8; i++) {
        _write(pin, addr[i]);
    }
}
// Do a ROM skip
static void _skip(gpio_num_t pin) {
    _write(pin, SKIPROM);
}
// reset the search state
static void _search_reset(gpio_num_t pin, dev_search_t* search, bool in_search) {
	// reset the search state
	search->LastDiscrepancy = 0;
	search->LastDeviceFlag = false;
	search->LastFamilyDiscrepancy = 0;

    if(!in_search) {
        for (int i = 0; i < 8; i++) {
            search->ROM_NO[i] = 0;
        }
    }
}
// --- Replaced by the one from the Dallas Semiconductor web site ---
//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search
static long _search(uint8_t *newAddr, gpio_num_t pin, dev_search_t* search) {
	uint8_t id_bit_number = 1;
	uint8_t last_zero = 0;
	int id_bit, cmp_id_bit;
	int search_direction;

    uint8_t byte_index = 0;
    unsigned char bit_mask = 1;

    // No presence is detected
    if (!_reset(pin)) {
        _search_reset(pin, search, true);
        return -ERESET;
    }

    // All devices are found
    if(search->LastDeviceFlag) {
        _search_reset(pin, search, true);
        return 0;
    }

    _write(pin, SEARCHROM);

    // loop to do the search
    do {
        // read a bit and its complement
        id_bit = _read_bit(pin);
        cmp_id_bit = _read_bit(pin);

        // ESP_LOGI(TAG, "id_bit: %d, cmp_id_bit: %d", id_bit, cmp_id_bit);

        // No devices respond in search
        if ((id_bit == 1) && (cmp_id_bit == 1)) {
            _search_reset(pin, search, true);
            return -ENF;
        }
        // There are both 0s and 1s in the current bit position. This is a discrepancy.
        else if ((id_bit == 0) && (cmp_id_bit == 0)) {
            // Decide which bit (0 or 1) to take during the ROM search:
            // 1. If the bit is before the latest discrepancy → keep same path
            // 2. If new branch (discrepancy) not reached yet → choose 0
            // 3. If revisit last discrepancy → choose 1
            if(id_bit_number < search->LastDiscrepancy) {
                search_direction = (search->ROM_NO[byte_index] & bit_mask) ? 1 : 0;
            }
            else if(id_bit_number > search->LastDiscrepancy) {
                search_direction = 0;
            }
            else {
                search_direction = 1;
            }

            // record the latest discrepancy by last_zero;
            if(search_direction == 0) {
                last_zero = id_bit_number;
                if(last_zero < 9) {
                    search->LastFamilyDiscrepancy = last_zero;
                }
            }
        }
        // There are only 0s or 1s in the bit of the participating ROM numbers
        else {
            search_direction = id_bit; // bit write value for search
        }

        // set or clear the bit in the ROM byte byte_index
        // with mask bit_mask
        if (search_direction == 1)
            search->ROM_NO[byte_index] |= bit_mask;
        else
            search->ROM_NO[byte_index] &= ~bit_mask;

        // master sends search direction to slave
        _write_bit(pin, search_direction);
        // ESP_LOGI(TAG, "search_direction: %d", search_direction);

        // maintain bit tracker
        id_bit_number++;
        bit_mask <<= 1;

        // reset bit_mask after finish lookup a byte
        if(bit_mask == 0) {
            byte_index++;
            bit_mask = 1;
        }
    } while (id_bit_number <= 64);  // loop until through all ROM bytes 0-7

    // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
    search->LastDiscrepancy = last_zero;
    if (search->LastDiscrepancy == 0) {
        search->LastDeviceFlag = true;
    }

    if(_crc8(search->ROM_NO, 7) != search->ROM_NO[7]) {
        _search_reset(pin, search, true);
        return -EINVLD;
    }

    for (int i = 0; i < 8; i++){
        newAddr[i] = search->ROM_NO[i];
    }
	return 1;
}

// Dow-CRC using polynomial X^8 + X^5 + X^4 + X^0
// Tiny 2x16 entry CRC table created by Arjen Lentz
// See http://lentz.com.au/blog/calculating-crc-with-a-tiny-32-entry-lookup-table
static const uint8_t dscrc2x16_table[] = {
	0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
	0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
	0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
	0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74
};
// Compute a Dallas Semiconductor 8 bit CRC. These show up in the ROM
// and the registers.  (Use tiny 2x16 entry CRC table)
static uint8_t _crc8(const uint8_t *addr, uint8_t len){
	uint8_t crc = 0;
	while (len--) {
		crc = *addr++ ^ crc;  // just re-using crc as intermediate
		crc = pgm_read_byte(dscrc2x16_table + (crc & 0x0f)) ^
		pgm_read_byte(dscrc2x16_table + 16 + ((crc >> 4) & 0x0f));
	}
	return crc;
}

void ds18b20_init(ds18b20_dev_t* dev, gpio_num_t pin) {
    _init(pin);
    dev->pin = pin;
    dev->parasite = false;
    dev->bitResolution = 0;
}
void ds18b20_search_reset(ds18b20_dev_t* dev) {
    _search_reset(dev->pin, &dev->search, false);
}
long ds18b20_search(ds18b20_dev_t* dev, uint8_t *newAddr) {
    return _search(newAddr, dev->pin, &dev->search);
}

bool ds18b20_isParasitePowerMode(ds18b20_dev_t* dev) {
    return dev->parasite;
}
bool ds18b20_validFamily(const uint8_t* addr) {
    switch (addr[0]) {
        // case DS18S20MODEL:
        case DS18B20MODEL:
        // case DS1822MODEL:
        // case DS1825MODEL:
        // case DS28EA00MODEL:
            return true;
        default:
            return false;
    }
}
bool ds18b20_isConnected(ds18b20_dev_t* dev, const uint8_t* addr, uint8_t *scratchPad) {
	bool b = ds18b20_readScratchPad(dev, addr, scratchPad);
	return b 
        && !ds18b20_isAllZeros(scratchPad) 
        && (_crc8(scratchPad, 8) == scratchPad[SCRATCHPAD_CRC]);
}
bool ds18b20_isConversionComplete(ds18b20_dev_t* dev) {
    gpio_num_t pin = dev->pin;
	int b = _read_bit(pin);
	return (b == 1);
}
bool ds18b20_isAllZeros(const uint8_t * const scratchPad) {
	for (size_t i = 0; i < 9; i++) {
		if (scratchPad[i] != 0) {
			return false;
		}
	}
	return true;
}

uint8_t ds18b20_getResolution(ds18b20_dev_t* dev, const uint8_t* addr) {
    if (addr[0] == DS18S20MODEL) return 12;
    
    ScratchPad scratchPad;
    if (ds18b20_isConnected(dev, addr, scratchPad)) {
        if (addr[0] == DS1825MODEL && scratchPad[CONFIGURATION] & 0x80) {
            return 12;
        }
        
        switch (scratchPad[CONFIGURATION]) {
            case TEMP_12_BIT: return 12;
            case TEMP_11_BIT: return 11;
            case TEMP_10_BIT: return 10;
            case TEMP_9_BIT: return 9;
        }
    }
    return 0;
}

// Read all registers in a simple loop
// byte 0: temperature LSB
// byte 1: temperature MSB
// byte 2: high alarm temp
// byte 3: low alarm temp
// byte 4: DS18B20 & DS1822: configuration register
// byte 5: internal use & crc
// byte 6: DS18B20 & DS1822: store for crc
// byte 7: DS18B20 & DS1822: store for crc
// byte 8: SCRATCHPAD_CRC
bool ds18b20_readScratchPad(ds18b20_dev_t* dev, const uint8_t* addr, uint8_t* scratchPad) {
	// send the reset command and fail fast
    gpio_num_t pin = dev->pin;
    if(!_reset(pin)) return false;

    _select(pin, addr);
    _write(pin, READSCRATCH);

	for (uint8_t i = 0; i < 9; i++) {
		scratchPad[i] = _read(pin);
	}

    return _reset(pin);
}
void ds18b20_writeScratchPad(ds18b20_dev_t* dev, const uint8_t* addr, const uint8_t *scratchPad) {
    gpio_num_t pin = dev->pin;
    _reset(pin);
    _select(pin, addr);
    _write(pin, WRITESCRATCH);
    _write(pin, scratchPad[HIGH_ALARM_TEMP]);
    _write(pin, scratchPad[LOW_ALARM_TEMP]);
    _reset(pin);
}
bool ds18b20_readPowerSupply(ds18b20_dev_t* dev, const uint8_t* addr) {
    gpio_num_t pin = dev->pin;
    bool parasiteMode = false;
    _reset(pin);
    if (addr == NULL) {
        _skip(pin);
    } else {
        _select(pin, addr);
    }
    
    _write(pin, READPOWERSUPPLY);
    if (_read_bit(pin) == 0) {
        parasiteMode = true;
    }
    _reset(pin);
    return parasiteMode;
}

bool ds18b20_requestTemperatures(ds18b20_dev_t* dev) {
    return ds18b20_requestTemperaturesByAddress(dev, NULL);
}
bool ds18b20_requestTemperaturesByAddress(ds18b20_dev_t* dev, uint8_t* addr) {
    gpio_num_t pin = dev->pin;
    if(!_reset(pin)) return false;

    if(addr == NULL) _skip(pin);
    else _select(pin, addr);

    _write(pin, CONVERTT);

    vTaskDelay(pdMS_TO_TICKS(MAX_CONVERSION_TIMEOUT));
    if(!ds18b20_isConversionComplete(dev)) return false;
    return true;
}
int32_t ds18b20_getTemp(ds18b20_dev_t* dev, uint8_t* addr) {
    ScratchPad scratchPad;
    uint8_t retries = 5;

    while(retries-- > 0) {
        if(ds18b20_isConnected(dev, addr, scratchPad)) {
            return ds18b20_calculateTemperature(scratchPad);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return DEVICE_DISCONNECTED_RAW;
}
float ds18b20_getTempC(ds18b20_dev_t* dev, uint8_t* addr) {
    int32_t raw = ds18b20_getTemp(dev, addr);
    if(raw <= DEVICE_DISCONNECTED_RAW) return DEVICE_DISCONNECTED_C;
    return (float)raw * 0.0078125f;
}
float ds18b20_getTempF(ds18b20_dev_t* dev, uint8_t* addr) {
	float tempC = ds18b20_getTempC(dev, addr);
    if(tempC <= DEVICE_DISCONNECTED_C) return DEVICE_DISCONNECTED_F;
    return tempC * 1.8f + 32.0f;
}
// reads scratchpad and returns fixed-point temperature, scaling factor 2^-7
static int32_t ds18b20_calculateTemperature(uint8_t* scratchPad) {
	int16_t fpTemperature = (((int16_t) scratchPad[TEMP_MSB]) << 11) | (((int16_t) scratchPad[TEMP_LSB]) << 3);
	return fpTemperature;
}