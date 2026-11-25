#pragma once
// Viết JSON tối giản (đầu ra stdout của slam_node), không cần thư viện ngoài.
#include <cmath>
#include <cstdio>
#include <string>

namespace slam {

inline std::string jstr(const std::string &s) {
  std::string o = "\"";
  for (unsigned char c : s) {
    switch (c) {
      case '"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      case '\r': o += "\\r"; break;
      case '\t': o += "\\t"; break;
      default:
        if (c < 0x20) {
          char b[8];
          std::snprintf(b, sizeof b, "\\u%04x", c);
          o += b;
        } else {
          o += char(c);
        }
    }
  }
  return o + "\"";
}

inline std::string jnum(double v, int prec = 3) {
  if (!std::isfinite(v)) return "null";
  char b[48];
  std::snprintf(b, sizeof b, "%.*f", prec, v);
  return b;
}

inline std::string jbool(bool b) { return b ? "true" : "false"; }

}  // namespace slam
