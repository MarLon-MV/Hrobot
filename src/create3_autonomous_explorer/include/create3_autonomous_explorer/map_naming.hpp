#ifndef CREATE3_AUTONOMOUS_EXPLORER__MAP_NAMING_HPP_
#define CREATE3_AUTONOMOUS_EXPLORER__MAP_NAMING_HPP_

#include <cstddef>
#include <ctime>
#include <filesystem>
#include <string>
#include <string_view>

namespace create3_autonomous_explorer
{

/// Maximum length (bytes) of a sanitized map name.
inline constexpr std::size_t kMaxMapNameLength = 64;

/// Files produced for one saved map (all share the same base path).
struct MapPaths
{
  std::string base;       ///< <dir>/<name>, no extension (what the service returns as map_path).
  std::string yaml;       ///< <base>.yaml      (nav2 map_saver)
  std::string pgm;        ///< <base>.pgm       (nav2 map_saver, default image format)
  std::string posegraph;  ///< <base>.posegraph (slam_toolbox serialization)
  std::string data;       ///< <base>.data      (slam_toolbox serialization)
};

/// Makes a user-provided name safe to use as a file name.
///  - Only [A-Za-z0-9_-] survive; every other byte (including '.', '/', '\\', spaces and each
///    byte of multi-byte UTF-8 sequences) becomes '_'. Hence ".." and path separators cannot occur.
///  - The result is truncated to `kMaxMapNameLength`.
///  - Design: if the result has no alphanumeric character (empty input, only symbols, ...) an
///    EMPTY string is returned, meaning "no usable name"; the caller falls back to
///    `make_timestamp_name`.
[[nodiscard]] std::string sanitize_map_name(std::string_view raw);

/// Deterministic `map_YYYYmmdd_HHMMSS` from an injected broken-down time (no clock access).
[[nodiscard]] std::string make_timestamp_name(const std::tm & time);

/// Composes the output paths for `name` (already sanitized) inside `dir`. Pure string work:
/// performs no filesystem I/O.
[[nodiscard]] MapPaths compose_map_paths(const std::filesystem::path & dir, std::string_view name);

}  // namespace create3_autonomous_explorer

#endif  // CREATE3_AUTONOMOUS_EXPLORER__MAP_NAMING_HPP_
