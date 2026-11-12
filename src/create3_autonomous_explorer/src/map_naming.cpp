#include "create3_autonomous_explorer/map_naming.hpp"

#include <algorithm>
#include <array>
#include <cstdio>

namespace create3_autonomous_explorer
{

namespace
{

[[nodiscard]] constexpr bool is_alnum(char c)
{
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9');
}

}  // namespace

std::string sanitize_map_name(std::string_view raw)
{
  std::string out;
  out.reserve(std::min(raw.size(), kMaxMapNameLength));
  bool has_alnum = false;
  for (const char c : raw) {
    if (out.size() >= kMaxMapNameLength) {
      break;
    }
    if (is_alnum(c)) {
      has_alnum = true;
      out.push_back(c);
    } else if (c == '_' || c == '-') {
      out.push_back(c);
    } else {
      out.push_back('_');
    }
  }
  if (!has_alnum) {
    out.clear();
  }
  return out;
}

std::string make_timestamp_name(const std::tm & time)
{
  std::array<char, 32> buf{};
  const std::size_t n = std::strftime(buf.data(), buf.size(), "map_%Y%m%d_%H%M%S", &time);
  return std::string(buf.data(), n);
}

MapPaths compose_map_paths(const std::filesystem::path & dir, std::string_view name)
{
  MapPaths paths;
  paths.base = (dir / std::filesystem::path(std::string(name))).string();
  paths.yaml = paths.base + ".yaml";
  paths.pgm = paths.base + ".pgm";
  paths.posegraph = paths.base + ".posegraph";
  paths.data = paths.base + ".data";
  return paths;
}

}  // namespace create3_autonomous_explorer
