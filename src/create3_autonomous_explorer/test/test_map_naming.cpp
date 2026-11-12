#include <gtest/gtest.h>

#include <ctime>
#include <string>

#include "create3_autonomous_explorer/map_naming.hpp"

using create3_autonomous_explorer::compose_map_paths;
using create3_autonomous_explorer::kMaxMapNameLength;
using create3_autonomous_explorer::make_timestamp_name;
using create3_autonomous_explorer::sanitize_map_name;

TEST(SanitizeMapName, KeepsValidName)
{
  EXPECT_EQ(sanitize_map_name("sala1"), "sala1");
  EXPECT_EQ(sanitize_map_name("Planta-2_ala_B"), "Planta-2_ala_B");
}

TEST(SanitizeMapName, ReplacesSpaces)
{
  EXPECT_EQ(sanitize_map_name("mi mapa 1"), "mi_mapa_1");
}

TEST(SanitizeMapName, NeutralizesParentDirectory)
{
  const std::string r = sanitize_map_name("../../etc/passwd");
  EXPECT_TRUE(r.find("..") == std::string::npos);
  EXPECT_TRUE(r.find('/') == std::string::npos);
  EXPECT_EQ(r, "______etc_passwd");
  EXPECT_EQ(sanitize_map_name(".."), "");
}

TEST(SanitizeMapName, ReplacesSeparators)
{
  EXPECT_EQ(sanitize_map_name("a/b"), "a_b");
  EXPECT_EQ(sanitize_map_name("a\\b"), "a_b");
  EXPECT_EQ(sanitize_map_name("/abs/path"), "_abs_path");
}

TEST(SanitizeMapName, ReplacesDots)
{
  EXPECT_EQ(sanitize_map_name("map.v2.yaml"), "map_v2_yaml");
}

TEST(SanitizeMapName, UnicodeBytesBecomeUnderscores)
{
  const std::string r = sanitize_map_name("sal\xC3\xB3n");  // "salón" in UTF-8
  EXPECT_EQ(r, "sal__n");
  for (const char c : r) {
    EXPECT_TRUE(static_cast<unsigned char>(c) < 0x80);
  }
}

TEST(SanitizeMapName, EmptyInputGivesEmpty)
{
  EXPECT_EQ(sanitize_map_name(""), "");
}

TEST(SanitizeMapName, OnlySymbolsGivesEmpty)
{
  EXPECT_EQ(sanitize_map_name("!!!@@@###"), "");
  EXPECT_EQ(sanitize_map_name("___---"), "");
  EXPECT_EQ(sanitize_map_name("   "), "");
}

TEST(SanitizeMapName, VeryLongNameIsTruncated)
{
  const std::string raw(10000, 'a');
  const std::string r = sanitize_map_name(raw);
  EXPECT_EQ(r.size(), kMaxMapNameLength);
}

TEST(SanitizeMapName, LongSymbolPrefixThenAlnumBeyondLimitGivesEmpty)
{
  // The only alphanumeric characters lie past the truncation limit: nothing usable remains.
  const std::string raw = std::string(kMaxMapNameLength + 5, '!') + "abc";
  EXPECT_EQ(sanitize_map_name(raw), "");
}

TEST(SanitizeMapName, EmbeddedNulIsReplaced)
{
  const std::string raw("a\0b", 3);
  EXPECT_EQ(sanitize_map_name(raw), "a_b");
}

TEST(MakeTimestampName, FormatsDeterministically)
{
  std::tm t{};
  t.tm_year = 2025 - 1900;
  t.tm_mon = 0;
  t.tm_mday = 5;
  t.tm_hour = 3;
  t.tm_min = 7;
  t.tm_sec = 9;
  EXPECT_EQ(make_timestamp_name(t), "map_20250105_030709");
  EXPECT_EQ(make_timestamp_name(t), make_timestamp_name(t));
}

TEST(MakeTimestampName, EndOfYear)
{
  std::tm t{};
  t.tm_year = 1999 - 1900;
  t.tm_mon = 11;
  t.tm_mday = 31;
  t.tm_hour = 23;
  t.tm_min = 59;
  t.tm_sec = 59;
  EXPECT_EQ(make_timestamp_name(t), "map_19991231_235959");
}

TEST(ComposeMapPaths, BuildsAllPaths)
{
  const auto p = compose_map_paths("/home/u/.create3_maps", "sala1");
  EXPECT_EQ(p.base, "/home/u/.create3_maps/sala1");
  EXPECT_EQ(p.yaml, "/home/u/.create3_maps/sala1.yaml");
  EXPECT_EQ(p.pgm, "/home/u/.create3_maps/sala1.pgm");
  EXPECT_EQ(p.posegraph, "/home/u/.create3_maps/sala1.posegraph");
  EXPECT_EQ(p.data, "/home/u/.create3_maps/sala1.data");
}

TEST(ComposeMapPaths, TrailingSlashInDir)
{
  const auto p = compose_map_paths("/maps/", "m");
  EXPECT_EQ(p.base, "/maps/m");
}

TEST(ComposeMapPaths, SanitizedNameStaysInsideDir)
{
  const auto p = compose_map_paths("/maps", sanitize_map_name("../x"));
  EXPECT_EQ(p.base, "/maps/___x");
}
