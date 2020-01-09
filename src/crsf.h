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

#ifndef CRSF_H_
#define CRSF_H_
#include <stdint.h>

#ifdef CRSF_ENABLED

#define CRSF_DATA_LEN 25
extern EXTERNAL_MEMORY uint8_t sbus_data[SBUS_DATA_LEN];

#else

#endif  // CRSF_ENABLED

#endif  // CRSF_H_
