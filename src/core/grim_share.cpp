#include "grim_share.h"
#include <cstdlib>
#include <vector>

// stb_image_write is compiled here for its zlib compressor; stb_image (and its
// decompressor) is compiled into the executable by definitive_background.cpp.
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>
#include <stb_image.h>

namespace {
constexpr const char *kPrefix = "VSGRIM1:";
constexpr const char *kAlphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
// Codes come from strangers: never inflate past this (the largest genomes seen are ~130 KB).
constexpr int kMaxJsonBytes = 4 * 1024 * 1024;

std::string base64url(const unsigned char *p, int n) {
  std::string out;
  out.reserve(static_cast<size_t>(n) * 4 / 3 + 4);
  for (int i = 0; i < n; i += 3) {
    const unsigned v = (p[i] << 16) | (i + 1 < n ? p[i + 1] << 8 : 0) | (i + 2 < n ? p[i + 2] : 0);
    out += kAlphabet[(v >> 18) & 63];
    out += kAlphabet[(v >> 12) & 63];
    if (i + 1 < n) out += kAlphabet[(v >> 6) & 63];
    if (i + 2 < n) out += kAlphabet[v & 63];
  }
  return out;
}

bool unbase64url(const std::string &s, std::vector<unsigned char> &out) {
  unsigned acc = 0;
  int bits = 0;
  for (char c : s) {
    int v = -1;
    if (c >= 'A' && c <= 'Z') v = c - 'A';
    else if (c >= 'a' && c <= 'z') v = c - 'a' + 26;
    else if (c >= '0' && c <= '9') v = c - '0' + 52;
    else if (c == '-' || c == '+') v = 62; // standard base64 too
    else if (c == '_' || c == '/') v = 63;
    else if (c == '=' || c == ' ' || c == '\n' || c == '\r' || c == '\t') continue;
    if (v < 0) return false;
    acc = (acc << 6) | static_cast<unsigned>(v);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      out.push_back(static_cast<unsigned char>((acc >> bits) & 0xFFu));
    }
  }
  return true;
}
} // namespace

std::string grim_share_code(const GrimGenome &genome) {
  std::string json = grim_genome_serialize(genome);
  int len = 0;
  unsigned char *z = stbi_zlib_compress(reinterpret_cast<unsigned char *>(json.data()),
                                        static_cast<int>(json.size()), &len, 9);
  if (z == nullptr) {
    return json; // still pasteable: the parser takes plain JSON
  }
  std::string code = kPrefix + base64url(z, len);
  STBIW_FREE(z);
  return code;
}

bool grim_share_parse(const std::string &text, GrimGenome &out, std::string &err) {
  const size_t start = text.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    err = "the clipboard is empty";
    return false;
  }
  if (text[start] == '{') {
    return grim_genome_parse(text.substr(start), out, err);
  }
  const std::string prefix = kPrefix;
  if (text.compare(start, prefix.size(), prefix) != 0) {
    err = "not a VibeStation corruption code";
    return false;
  }
  std::vector<unsigned char> z;
  if (!unbase64url(text.substr(start + prefix.size()), z) || z.empty()) {
    err = "the code is damaged (bad characters)";
    return false;
  }
  std::vector<char> json(kMaxJsonBytes);
  const int n = stbi_zlib_decode_buffer(json.data(), kMaxJsonBytes, reinterpret_cast<const char *>(z.data()),
                                        static_cast<int>(z.size()));
  if (n <= 0) {
    err = "the code is damaged or incomplete";
    return false;
  }
  return grim_genome_parse(std::string(json.data(), static_cast<size_t>(n)), out, err);
}
