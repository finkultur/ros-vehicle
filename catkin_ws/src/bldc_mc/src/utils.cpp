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

