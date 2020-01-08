/*
    Copyright 2017 fishpepper <AT> gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http:// www.gnu.org/licenses/>.

   author: fishpepper <AT> gmail.com
*/

#include "storage.h"
#include "hal_storage.h"
#include "debug.h"
#include "wdt.h"
#include "delay.h"
#include "led.h"
#ifdef REDPINE_PROTOCOL
#include "redpine.h"
#else
#include "frsky.h"
#endif
#include "hal_defines.h"


// run time copy of persistant storage data:
EXTERNAL_MEMORY STORAGE_DESC storage;

void storage_init(void) {
    uint8_t i;

    debug("storage: init\n"); debug_flush();

    // init hal storage
    hal_storage_init();

    // reload data from flash
    storage_read_from_flash();

    debug("storage: loaded hoptable[] = ");
    for (i = 0; i < MAX_HOPTABLE_SIZE; i++) {
        debug_put_hex8(storage.hop_table[i]);
        debug_putc(' ');
        debug_flush();
    }
    debug_put_newline();
}

static const uint8_t storage_default_hoptable[] =
      {0x0,0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8,0x9,0xA,0xB,0xC,0xD,0xE,0xF,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2A,0x2B,0x2C,0x2D,0x2E,0x2F,0x30,0x31};

void storage_read_from_flash(void) {
    uint8_t *storage_ptr;
    uint16_t len;
    uint8_t i;

    debug("storage: reading\n"); debug_flush();
    storage_ptr = (uint8_t*)&storage;
    len = sizeof(storage);

    hal_storage_read(storage_ptr, len);

#ifdef FRSKY_USE_FIXED_ID
    // allow override for testing
    if (1) {
#else  // FRSKY_USE_FIXED_ID
    // only init with defaults when no valid storage id was found
    if (storage.version != STORAGE_VERSION_ID) {
#endif  // FRSKY_USE_FIXED_ID
        debug("storage: init with defaults (txid 0x1668)\n");

        storage.version = STORAGE_VERSION_ID;

        // hard coded config for debugging:
        storage.txid[0] = 0x16;
        storage.txid[1] = 0x68;
        storage.freq_offset = DEFAULT_FSCAL_VALUE;

        for (i = 0; i < MAX_HOPTABLE_SIZE; i++) {
            storage.hop_table[i] = storage_default_hoptable[i];
        }

        // store settings
        storage_write_to_flash();
    }
}

void storage_write_to_flash(void) {
    uint8_t *storage_ptr;
    uint16_t len;

    debug("storage: writing\n"); debug_flush();
    storage.version = STORAGE_VERSION_ID;

    storage_ptr = (uint8_t*)&storage;
    len = sizeof(storage);

    // execute flash write:
    hal_storage_write(storage_ptr, len);
}

