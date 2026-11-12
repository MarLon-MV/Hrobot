#include "create3_autonomous_explorer/goal_evaluator.hpp"

#include <cmath>

namespace create3_autonomous_explorer
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

double wrap_angle(double a)
{
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a < 0.0) {
    a += 2.0 * kPi;
  }
  return a - kPi;
}
}  // namespace

GoalEvaluator::GoalEvaluator(const GoalWeights & weights, double blacklist_radius)
: weights_(weights), blacklist_radius_(blacklist_radius) {}

double GoalEvaluator::score(
  const Frontier & frontier, double robot_x, double robot_y, double robot_yaw) const
{
  const double dx = frontier.centroid.x - robot_x;
  const double dy = frontier.centroid.y - robot_y;
  const double dist = std::hypot(dx, dy);
  double angle_error = 0.0;
  if (dist > 0.0) {
    angle_error = std::fabs(wrap_angle(std::atan2(dy, dx) - robot_yaw));
  }
  return weights_.size * static_cast<double>(frontier.size) - weights_.distance * dist -
         weights_.angle * angle_error;
}

std::optional<Frontier> GoalEvaluator::select_best(
  const std::vector<Frontier> & frontiers, double robot_x, double robot_y,
  double robot_yaw) const
{
  std::optional<Frontier> best;
  for (const auto & f : frontiers) {
    if (is_blacklisted(f.centroid)) {
      continue;
    }
    const double s = score(f, robot_x, robot_y, robot_yaw);
    if (!best.has_value() || s > best->score) {
      best = f;
      best->score = s;
    }
  }
  return best;
}

void GoalEvaluator::add_to_blacklist(const Point2D & point) {blacklist_.push_back(point);}

bool GoalEvaluator::is_blacklisted(const Point2D & point) const
{
  for (const auto & b : blacklist_) {
    if (std::hypot(point.x - b.x, point.y - b.y) <= blacklist_radius_) {
      return true;
    }
  }
  return false;
}

void GoalEvaluator::clear_blacklist() {blacklist_.clear();}

const GoalWeights & GoalEvaluator::weights() const {return weights_;}

double GoalEvaluator::blacklist_radius() const {return blacklist_radius_;}

}  // namespace create3_autonomous_explorer
