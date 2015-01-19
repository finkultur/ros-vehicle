/*
  Copyright 2012-2014 Benjamin Vedder benjamin@vedder.se
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

#include "utils.hpp"

/*
  Calculates a checksum of the given data.
  From bldc-tool/packetinterface.cpp.
*/
unsigned short crc16(const unsigned char *buf, unsigned int len) {
  unsigned int i;
  unsigned short cksum = 0;
  for (i = 0; i < len; i++) {
    cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8); 
  }
  return cksum;
}

/*
  Returns an int16_t from the buffer, increasing the index by 2 bytes.
  Code from bldc/buffer.c.
*/
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index) {
  int16_t res = ((uint16_t) buffer[*index]) << 8 | 
          ((uint16_t) buffer[*index + 1]);
  *index += 2;
  return res;
}

/*
  Returns an int32_t from the buffer, increasing the index by 4 bytes.
  Code from bldc/buffer.c.
*/
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) {
  int32_t res = ((uint32_t) buffer[*index]) << 24 |
          ((uint32_t) buffer[*index + 1]) << 16 |
          ((uint32_t) buffer[*index + 2]) << 8 | 
          ((uint32_t) buffer[*index + 3]);
  *index += 4;
  return res;
}

