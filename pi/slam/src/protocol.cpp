#include "slam/protocol.h"

namespace slam {

bool readRecord(FILE *f, Record &r) {
  uint8_t hdr[5];
  if (fread(hdr, 1, 5, f) != 5) return false;
  r.type = char(hdr[0]);
  const uint32_t len = uint32_t(hdr[1]) | uint32_t(hdr[2]) << 8 | uint32_t(hdr[3]) << 16 | uint32_t(hdr[4]) << 24;
  if (len > 64u * 1024 * 1024) return false;  // hỏng luồng
  r.data.resize(len);
  return len == 0 || fread(r.data.data(), 1, len, f) == len;
}

void writeRecord(FILE *f, char type, const void *data, uint32_t len) {
  const uint8_t hdr[5] = {uint8_t(type), uint8_t(len), uint8_t(len >> 8), uint8_t(len >> 16), uint8_t(len >> 24)};
  fwrite(hdr, 1, 5, f);
  if (len) fwrite(data, 1, len, f);
}

}  // namespace slam
