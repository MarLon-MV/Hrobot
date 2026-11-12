#ifndef CREATE3_AUTONOMOUS_EXPLORER__TYPES_HPP_
#define CREATE3_AUTONOMOUS_EXPLORER__TYPES_HPP_

#include <cstddef>
#include <vector>

namespace create3_autonomous_explorer
{

/// 2D point in the map frame, in meters.
struct Point2D
{
  double x{0.0};
  double y{0.0};
};

/// Group of 8-connected frontier cells.
struct Frontier
{
  Point2D centroid{};          ///< Mean of the cell centers (map frame, meters).
  std::size_t size{0};         ///< Number of cells in the frontier.
  double score{0.0};           ///< Filled by GoalEvaluator (higher is better).
  std::vector<Point2D> cells{};  ///< Cell centers (map frame, meters).
};

enum class ExplorerState
{
  Idle,
  WaitingForMap,
  Detecting,
  Navigating,
  Recovering,
  Finished,
  Failed
};

}  // namespace create3_autonomous_explorer

#endif  // CREATE3_AUTONOMOUS_EXPLORER__TYPES_HPP_
