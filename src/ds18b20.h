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
#include "driver/gpio.h"
#include <stdbool.h>

#ifndef DS18B20_H_  
#define DS18B20_H_

#define noInterrupts() portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;taskENTER_CRITICAL(&mux)
#define interrupts() taskEXIT_CRITICAL(&mux)

#define DEVICE_DISCONNECTED_C -127
#define DEVICE_DISCONNECTED_F -196.6
#define DEVICE_DISCONNECTED_RAW -7040
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define pgm_read_byte(addr)   (*(const unsigned char *)(addr))

// Configuration Constants
#define MAX_INITIALIZATION_RETRIES 3
#define MAX_CONVERSION_TIMEOUT     750

// Bug code
#define EINVLD  10
#define ERESET  11
#define ENF     12
#define ELDF    13

typedef uint8_t DeviceAddress[8];
typedef uint8_t ScratchPad[9];

typedef struct dev_search {
    uint8_t LastDiscrepancy;
    bool LastDeviceFlag;
    uint8_t LastFamilyDiscrepancy;
    unsigned char ROM_NO[8];
} dev_search_t;

typedef struct ds18b20_dev {
    gpio_num_t pin;
    bool parasite;
    uint8_t bitResolution;
    struct dev_search search;
} ds18b20_dev_t;

/* *INDENT-OFF* */
#ifdef __cplusplus
    extern "C" {
#endif
/* *INDENT-ON* */

#define ds18b20_send ds18b20_write
#define ds18b20_send_byte ds18b20_write_byte
#define ds18b20_RST_PULSE ds18b20_reset

void ds18b20_init(ds18b20_dev_t* dev, gpio_num_t pin);
void ds18b20_search_reset(ds18b20_dev_t* dev);
long ds18b20_search(ds18b20_dev_t* dev, uint8_t *newAddr);

bool ds18b20_isParasitePowerMode(ds18b20_dev_t* dev);
bool ds18b20_validFamily(const uint8_t* addr);
bool ds18b20_isConnected(ds18b20_dev_t* dev, const uint8_t* deviceAddress, uint8_t *scratchPad);
bool ds18b20_isConversionComplete(ds18b20_dev_t* dev);
bool ds18b20_isAllZeros(const uint8_t * const scratchPad);

uint8_t ds18b20_getResolution(ds18b20_dev_t* dev, const uint8_t* addr);

bool ds18b20_readScratchPad(ds18b20_dev_t* dev, const uint8_t* addr, uint8_t* scratchPad);
void ds18b20_writeScratchPad(ds18b20_dev_t* dev, const uint8_t* addr, const uint8_t* scratchPad);
bool ds18b20_readPowerSupply(ds18b20_dev_t* dev, const uint8_t* addr);

bool ds18b20_requestTemperatures(ds18b20_dev_t* dev);
bool ds18b20_requestTemperaturesByAddress(ds18b20_dev_t* dev, uint8_t* addr);
int32_t ds18b20_getTemp(ds18b20_dev_t* dev, uint8_t* addr);
float ds18b20_getTempC(ds18b20_dev_t* dev, uint8_t* addr);
float ds18b20_getTempF(ds18b20_dev_t* dev, uint8_t* addr);

/* *INDENT-OFF* */
#ifdef __cplusplus
    }
#endif
/* *INDENT-ON* */

#endif