#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "create3_autonomous_explorer/goal_evaluator.hpp"

using create3_autonomous_explorer::Frontier;
using create3_autonomous_explorer::GoalEvaluator;
using create3_autonomous_explorer::GoalWeights;
using create3_autonomous_explorer::Point2D;

namespace
{

constexpr double kPi = 3.14159265358979323846;

Frontier make_frontier(double x, double y, std::size_t size)
{
  Frontier f;
  f.centroid = Point2D{x, y};
  f.size = size;
  return f;
}

GoalWeights only(double size, double distance, double angle)
{
  GoalWeights w;
  w.size = size;
  w.distance = distance;
  w.angle = angle;
  return w;
}

}  // namespace

TEST(GoalEvaluator, ScoreMatchesFormula)
{
  const GoalEvaluator ev(only(1.0, 0.5, 2.0));
  const Frontier f = make_frontier(3.0, 4.0, 10);
  const double expected = 10.0 - 0.5 * 5.0 - 2.0 * std::atan2(4.0, 3.0);
  EXPECT_NEAR(ev.score(f, 0.0, 0.0, 0.0), expected, 1e-9);
}

TEST(GoalEvaluator, ScoreAtRobotPositionIsFiniteWithZeroAngleError)
{
  const GoalEvaluator ev(only(1.0, 1.0, 1.0));
  const Frontier f = make_frontier(2.0, 2.0, 4);
  const double s = ev.score(f, 2.0, 2.0, 1.2);
  EXPECT_TRUE(std::isfinite(s));
  EXPECT_NEAR(s, 4.0, 1e-9);
}

TEST(GoalEvaluator, AngleErrorWrapsAroundPi)
{
  const GoalEvaluator ev(only(0.0, 0.0, 1.0));
  const Frontier f = make_frontier(1.0, 0.0, 1);
  // yaw just below 2*pi is equivalent to a small negative yaw.
  EXPECT_NEAR(ev.score(f, 0.0, 0.0, 2.0 * kPi - 0.1), -0.1, 1e-9);
  EXPECT_NEAR(ev.score(f, 0.0, 0.0, -2.0 * kPi + 0.1), -0.1, 1e-9);
}

TEST(GoalEvaluator, SelectBestOnEmptyReturnsNullopt)
{
  const GoalEvaluator ev;
  EXPECT_FALSE(ev.select_best({}, 0.0, 0.0, 0.0).has_value());
}

TEST(GoalEvaluator, SelectBestPrefersLargerFrontier)
{
  const GoalEvaluator ev(only(1.0, 0.0, 0.0));
  const std::vector<Frontier> fs = {make_frontier(1.0, 0.0, 5), make_frontier(2.0, 0.0, 20),
    make_frontier(3.0, 0.0, 10)};
  const auto best = ev.select_best(fs, 0.0, 0.0, 0.0);
  ASSERT_TRUE(best.has_value());
  EXPECT_EQ(best->size, 20u);
}

TEST(GoalEvaluator, SelectBestPrefersCloserFrontier)
{
  const GoalEvaluator ev(only(0.0, 1.0, 0.0));
  const std::vector<Frontier> fs = {make_frontier(10.0, 0.0, 5), make_frontier(2.0, 0.0, 5),
    make_frontier(0.0, 6.0, 5)};
  const auto best = ev.select_best(fs, 0.0, 0.0, 0.0);
  ASSERT_TRUE(best.has_value());
  EXPECT_NEAR(best->centroid.x, 2.0, 1e-9);
}

TEST(GoalEvaluator, SelectBestPrefersFrontierAheadOfRobot)
{
  const GoalEvaluator ev(only(0.0, 0.0, 1.0));
  // Same distance; robot faces +x.
  const std::vector<Frontier> fs = {make_frontier(-2.0, 0.0, 5), make_frontier(0.0, 2.0, 5),
    make_frontier(2.0, 0.0, 5)};
  const auto best = ev.select_best(fs, 0.0, 0.0, 0.0);
  ASSERT_TRUE(best.has_value());
  EXPECT_NEAR(best->centroid.x, 2.0, 1e-9);
  // Robot faces -x: the opposite one wins.
  const auto best2 = ev.select_best(fs, 0.0, 0.0, kPi);
  ASSERT_TRUE(best2.has_value());
  EXPECT_NEAR(best2->centroid.x, -2.0, 1e-9);
}

TEST(GoalEvaluator, SelectBestFillsScore)
{
  const GoalEvaluator ev(only(1.0, 0.0, 0.0));
  const auto best = ev.select_best({make_frontier(1.0, 1.0, 7)}, 0.0, 0.0, 0.0);
  ASSERT_TRUE(best.has_value());
  EXPECT_NEAR(best->score, 7.0, 1e-9);
}

TEST(GoalEvaluator, TieReturnsFirstFrontier)
{
  const GoalEvaluator ev(only(1.0, 0.0, 0.0));
  const std::vector<Frontier> fs = {make_frontier(5.0, 5.0, 10), make_frontier(-5.0, 1.0, 10),
    make_frontier(0.0, 9.0, 10)};
  const auto best = ev.select_best(fs, 0.0, 0.0, 0.0);
  ASSERT_TRUE(best.has_value());
  EXPECT_NEAR(best->centroid.x, 5.0, 1e-9);
  EXPECT_NEAR(best->centroid.y, 5.0, 1e-9);
}

TEST(GoalEvaluator, BlacklistRadiusBehaviour)
{
  GoalEvaluator ev(GoalWeights{}, 0.5);
  EXPECT_DOUBLE_EQ(ev.blacklist_radius(), 0.5);
  EXPECT_FALSE(ev.is_blacklisted(Point2D{1.0, 1.0}));
  ev.add_to_blacklist(Point2D{1.0, 1.0});
  EXPECT_TRUE(ev.is_blacklisted(Point2D{1.0, 1.0}));
  EXPECT_TRUE(ev.is_blacklisted(Point2D{1.3, 1.0}));
  EXPECT_TRUE(ev.is_blacklisted(Point2D{1.5, 1.0}));  // boundary is inclusive
  EXPECT_FALSE(ev.is_blacklisted(Point2D{1.6, 1.0}));
  EXPECT_FALSE(ev.is_blacklisted(Point2D{3.0, 3.0}));
}

TEST(GoalEvaluator, ClearBlacklist)
{
  GoalEvaluator ev(GoalWeights{}, 0.5);
  ev.add_to_blacklist(Point2D{1.0, 1.0});
  ev.add_to_blacklist(Point2D{5.0, 5.0});
  ev.clear_blacklist();
  EXPECT_FALSE(ev.is_blacklisted(Point2D{1.0, 1.0}));
  EXPECT_FALSE(ev.is_blacklisted(Point2D{5.0, 5.0}));
}

TEST(GoalEvaluator, SelectBestSkipsBlacklistedFrontier)
{
  GoalEvaluator ev(only(1.0, 0.0, 0.0), 0.5);
  const std::vector<Frontier> fs = {make_frontier(1.0, 0.0, 20), make_frontier(5.0, 0.0, 10)};
  ev.add_to_blacklist(Point2D{1.2, 0.0});
  const auto best = ev.select_best(fs, 0.0, 0.0, 0.0);
  ASSERT_TRUE(best.has_value());
  EXPECT_NEAR(best->centroid.x, 5.0, 1e-9);
}

TEST(GoalEvaluator, AllBlacklistedReturnsNullopt)
{
  GoalEvaluator ev(GoalWeights{}, 0.5);
  const std::vector<Frontier> fs = {make_frontier(1.0, 0.0, 20), make_frontier(5.0, 0.0, 10)};
  ev.add_to_blacklist(Point2D{1.0, 0.0});
  ev.add_to_blacklist(Point2D{5.0, 0.1});
  EXPECT_FALSE(ev.select_best(fs, 0.0, 0.0, 0.0).has_value());
}

TEST(GoalEvaluator, ConfiguredWeightsAreExposed)
{
  const GoalEvaluator ev(only(2.0, 3.0, 4.0), 1.0);
  EXPECT_DOUBLE_EQ(ev.weights().size, 2.0);
  EXPECT_DOUBLE_EQ(ev.weights().distance, 3.0);
  EXPECT_DOUBLE_EQ(ev.weights().angle, 4.0);
}
