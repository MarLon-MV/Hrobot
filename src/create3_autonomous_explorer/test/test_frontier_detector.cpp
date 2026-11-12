#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

#include "create3_autonomous_explorer/frontier_detector.hpp"

using create3_autonomous_explorer::Frontier;
using create3_autonomous_explorer::FrontierDetector;
using create3_autonomous_explorer::GridMap;
using create3_autonomous_explorer::Point2D;

namespace
{

constexpr std::int8_t kUnknown = -1;
constexpr std::int8_t kFree = 0;
constexpr std::int8_t kOccupied = 100;

GridMap make_map(int width, int height, std::int8_t fill)
{
  GridMap m;
  m.width = width;
  m.height = height;
  m.resolution = 1.0;
  m.origin = Point2D{0.0, 0.0};
  m.data.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), fill);
  return m;
}

void set_cell(GridMap & m, int col, int row, std::int8_t v)
{
  m.data[static_cast<std::size_t>(row) * static_cast<std::size_t>(m.width) +
         static_cast<std::size_t>(col)] = v;
}

/// Free columns [0, free_cols), unknown elsewhere, for the given rows.
GridMap make_half_free_map(int width, int height, int free_cols)
{
  GridMap m = make_map(width, height, kUnknown);
  for (int r = 0; r < height; ++r) {
    for (int c = 0; c < free_cols; ++c) {
      set_cell(m, c, r, kFree);
    }
  }
  return m;
}

}  // namespace

TEST(FrontierDetector, EmptyMapYieldsNoFrontiers)
{
  const FrontierDetector detector(1);
  const GridMap m;
  EXPECT_TRUE(detector.detect(m, Point2D{0.0, 0.0}).empty());
}

TEST(FrontierDetector, InconsistentDataSizeYieldsNoFrontiers)
{
  const FrontierDetector detector(1);
  GridMap m = make_map(5, 5, kFree);
  m.data.resize(10);
  EXPECT_TRUE(detector.detect(m, Point2D{0.0, 0.0}).empty());
}

TEST(FrontierDetector, AllUnknownYieldsNoFrontiers)
{
  const FrontierDetector detector(1);
  EXPECT_TRUE(detector.detect(make_map(8, 8, kUnknown), Point2D{0.0, 0.0}).empty());
}

TEST(FrontierDetector, AllFreeYieldsNoFrontiers)
{
  const FrontierDetector detector(1);
  EXPECT_TRUE(detector.detect(make_map(8, 8, kFree), Point2D{0.0, 0.0}).empty());
}

TEST(FrontierDetector, FreeAndOccupiedWithoutUnknownYieldsNoFrontiers)
{
  const FrontierDetector detector(1);
  GridMap m = make_map(8, 8, kFree);
  for (int r = 0; r < 8; ++r) {
    set_cell(m, 4, r, kOccupied);
  }
  EXPECT_TRUE(detector.detect(m, Point2D{0.0, 0.0}).empty());
}

TEST(FrontierDetector, SingleFrontierSizeCentroidAndCells)
{
  const FrontierDetector detector(3);
  // Free cols 0-1, unknown cols 2-4: frontier is column 1 (5 cells).
  const auto frontiers = detector.detect(make_half_free_map(5, 5, 2), Point2D{0.0, 0.0});
  ASSERT_EQ(frontiers.size(), 1u);
  EXPECT_EQ(frontiers[0].size, 5u);
  EXPECT_EQ(frontiers[0].cells.size(), 5u);
  EXPECT_NEAR(frontiers[0].centroid.x, 1.5, 1e-9);
  EXPECT_NEAR(frontiers[0].centroid.y, 2.5, 1e-9);
}

TEST(FrontierDetector, CentroidUsesResolutionAndOrigin)
{
  const FrontierDetector detector(3);
  GridMap m = make_half_free_map(5, 5, 2);
  m.resolution = 0.5;
  m.origin = Point2D{-10.0, 4.0};
  const auto frontiers = detector.detect(m, Point2D{0.0, 0.0});
  ASSERT_EQ(frontiers.size(), 1u);
  EXPECT_NEAR(frontiers[0].centroid.x, -10.0 + 1.5 * 0.5, 1e-9);
  EXPECT_NEAR(frontiers[0].centroid.y, 4.0 + 2.5 * 0.5, 1e-9);
}

TEST(FrontierDetector, TwoSeparateFrontiers)
{
  const FrontierDetector detector(3);
  GridMap m = make_half_free_map(5, 11, 2);
  set_cell(m, 1, 5, kOccupied);  // splits column 1 into rows 0-4 and 6-10
  const auto frontiers = detector.detect(m, Point2D{0.0, 0.0});
  ASSERT_EQ(frontiers.size(), 2u);
  EXPECT_EQ(frontiers[0].size, 5u);
  EXPECT_EQ(frontiers[1].size, 5u);
  EXPECT_NEAR(frontiers[0].centroid.y, 2.5, 1e-9);
  EXPECT_NEAR(frontiers[1].centroid.y, 8.5, 1e-9);
}

TEST(FrontierDetector, DiagonalCellsAreConnected)
{
  const FrontierDetector detector(3);
  // Free staircase over unknown: free cells (0,0),(1,1),(2,2) are only
  // diagonally adjacent, all border unknown cells -> one frontier of 3.
  GridMap m = make_map(3, 3, kUnknown);
  set_cell(m, 0, 0, kFree);
  set_cell(m, 1, 1, kFree);
  set_cell(m, 2, 2, kFree);
  const auto frontiers = detector.detect(m, Point2D{0.0, 0.0});
  ASSERT_EQ(frontiers.size(), 1u);
  EXPECT_EQ(frontiers[0].size, 3u);
}

TEST(FrontierDetector, IsolatedUnknownCellProducesRingFrontier)
{
  // Single unknown cell inside a free area: the 8 free neighbours form a ring.
  GridMap m = make_map(7, 7, kFree);
  set_cell(m, 3, 3, kUnknown);
  const auto found = FrontierDetector(3).detect(m, Point2D{0.0, 0.0});
  ASSERT_EQ(found.size(), 1u);
  EXPECT_EQ(found[0].size, 8u);
  EXPECT_NEAR(found[0].centroid.x, 3.5, 1e-9);
  EXPECT_NEAR(found[0].centroid.y, 3.5, 1e-9);
  // Too small once the minimum size exceeds the ring.
  EXPECT_TRUE(FrontierDetector(9).detect(m, Point2D{0.0, 0.0}).empty());
}

TEST(FrontierDetector, UnknownBehindOccupiedIsNotFrontier)
{
  const FrontierDetector detector(1);
  GridMap m = make_map(5, 5, kUnknown);
  for (int r = 0; r < 5; ++r) {
    set_cell(m, 0, r, kFree);
    set_cell(m, 1, r, kOccupied);  // wall separates free from unknown
  }
  // Column 0 free cells touch only free/occupied cells.
  EXPECT_TRUE(detector.detect(m, Point2D{0.0, 0.0}).empty());
}

TEST(FrontierDetector, MinFrontierSizeFiltersSmallGroups)
{
  GridMap m = make_half_free_map(5, 11, 2);
  set_cell(m, 1, 5, kOccupied);
  set_cell(m, 1, 6, kOccupied);
  set_cell(m, 1, 7, kOccupied);
  set_cell(m, 1, 8, kOccupied);  // second group shrinks to rows 9-10 (2 cells)
  EXPECT_EQ(FrontierDetector(2).detect(m, Point2D{0.0, 0.0}).size(), 2u);
  const auto only_big = FrontierDetector(3).detect(m, Point2D{0.0, 0.0});
  ASSERT_EQ(only_big.size(), 1u);
  EXPECT_EQ(only_big[0].size, 5u);
  EXPECT_TRUE(FrontierDetector(6).detect(m, Point2D{0.0, 0.0}).empty());
}

TEST(FrontierDetector, MinFrontierSizeSetterAndGetter)
{
  FrontierDetector detector(3);
  EXPECT_EQ(detector.min_frontier_size(), 3u);
  detector.set_min_frontier_size(7);
  EXPECT_EQ(detector.min_frontier_size(), 7u);
}

TEST(FrontierDetector, FrontiersOrderedByDistanceToRobot)
{
  const FrontierDetector detector(3);
  GridMap m = make_half_free_map(5, 11, 2);
  set_cell(m, 1, 5, kOccupied);
  const auto near_bottom = detector.detect(m, Point2D{0.0, 0.0});
  ASSERT_EQ(near_bottom.size(), 2u);
  EXPECT_NEAR(near_bottom[0].centroid.y, 2.5, 1e-9);
  const auto near_top = detector.detect(m, Point2D{0.0, 10.0});
  ASSERT_EQ(near_top.size(), 2u);
  EXPECT_NEAR(near_top[0].centroid.y, 8.5, 1e-9);
}

TEST(FrontierDetector, RobotPositionOutsideMapDoesNotCrash)
{
  const FrontierDetector detector(3);
  const auto frontiers = detector.detect(make_half_free_map(5, 5, 2), Point2D{-100.0, 999.0});
  EXPECT_EQ(frontiers.size(), 1u);
}
