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

#include "hal_storage.h"
#include "debug.h"
#include "eeprom.h"

//Using virtual eeprom in flash
uint16_t VirtAddVarTab[NumbOfVar];

void hal_storage_init(void) {
	for (int i = 0; i < NumbOfVar; i++) {
		VirtAddVarTab[i] = i;
	}
	/* Unlock the Flash Program Erase controller */
	FLASH_Unlock();

	/* EEPROM Init */
	EE_Init();
}

void hal_storage_write(uint16_t *buffer, uint16_t len) {
	for (int i = 0; i < len; i++) {
		EE_WriteVariable(i, buffer[i]);
	}
}


void hal_storage_read(uint16_t *storage_ptr, uint16_t len) {
	for (int i = 0; i < len; i++) {
		EE_ReadVariable(i, storage_ptr+i);
	}
}


