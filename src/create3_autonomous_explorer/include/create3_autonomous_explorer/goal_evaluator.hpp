#ifndef CREATE3_AUTONOMOUS_EXPLORER__GOAL_EVALUATOR_HPP_
#define CREATE3_AUTONOMOUS_EXPLORER__GOAL_EVALUATOR_HPP_

#include <optional>
#include <vector>

#include "create3_autonomous_explorer/types.hpp"

namespace create3_autonomous_explorer
{

struct GoalWeights
{
  double size{1.0};      ///< Reward per frontier cell.
  double distance{1.0};  ///< Penalty per meter to the centroid.
  double angle{1.0};     ///< Penalty per radian of heading error, in [0, pi].
};

/// Scores frontiers and selects the best goal, skipping blacklisted ones.
///   score = w.size * frontier.size
///         - w.distance * dist(robot, centroid)
///         - w.angle * |wrap(atan2(dy, dx) - yaw)|
/// Higher is better. A centroid at the robot position has angle error 0.
class GoalEvaluator
{
public:
  explicit GoalEvaluator(const GoalWeights & weights = GoalWeights{},
                         double blacklist_radius = 0.5);

  double score(const Frontier & frontier, double robot_x, double robot_y,
               double robot_yaw) const;

  /// Best non-blacklisted frontier (its `score` field filled in), or nullopt if
  /// none. On equal scores the earliest frontier in `frontiers` wins.
  std::optional<Frontier> select_best(const std::vector<Frontier> & frontiers,
                                      double robot_x, double robot_y,
                                      double robot_yaw) const;

  void add_to_blacklist(const Point2D & point);
  /// True if `point` is within `blacklist_radius` (inclusive) of a blacklisted point.
  bool is_blacklisted(const Point2D & point) const;
  void clear_blacklist();

  const GoalWeights & weights() const;
  double blacklist_radius() const;

private:
  GoalWeights weights_;
  double blacklist_radius_;
  std::vector<Point2D> blacklist_;
};

}  // namespace create3_autonomous_explorer

#endif  // CREATE3_AUTONOMOUS_EXPLORER__GOAL_EVALUATOR_HPP_
