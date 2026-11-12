#ifndef CREATE3_AUTONOMOUS_EXPLORER__FRONTIER_DETECTOR_HPP_
#define CREATE3_AUTONOMOUS_EXPLORER__FRONTIER_DETECTOR_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "create3_autonomous_explorer/types.hpp"

namespace create3_autonomous_explorer
{

/// Plain occupancy grid. Row-major: index = row * width + col.
/// Values: -1 unknown, 0 free, 100 occupied (nav_msgs/OccupancyGrid convention).
/// `origin` is the world position of the lower-left corner of cell (0, 0);
/// the center of cell (col, row) is origin + ((col + 0.5) * res, (row + 0.5) * res).
struct GridMap
{
  int width{0};
  int height{0};
  double resolution{0.05};
  Point2D origin{};
  std::vector<std::int8_t> data{};
};

/// Frontier detection by BFS. A frontier cell is a free cell (value 0) with at
/// least one unknown (-1) cell among its 8 neighbours. Frontier cells are grouped
/// by 8-connectivity; groups smaller than `min_frontier_size` are discarded.
class FrontierDetector
{
public:
  explicit FrontierDetector(std::size_t min_frontier_size = 3);

  /// Returns the frontiers of `map`, ordered by ascending distance between the
  /// frontier centroid and `robot_position` (stable for equal distances).
  /// Invalid maps (non-positive size, data.size() != width*height) yield {}.
  /// `Frontier::score` is left at 0.
  [[nodiscard]] std::vector<Frontier> detect(const GridMap & map, const Point2D & robot_position) const;

  [[nodiscard]] std::size_t min_frontier_size() const;
  void set_min_frontier_size(std::size_t min_frontier_size);

private:
  std::size_t min_frontier_size_;
};

}  // namespace create3_autonomous_explorer

#endif  // CREATE3_AUTONOMOUS_EXPLORER__FRONTIER_DETECTOR_HPP_
