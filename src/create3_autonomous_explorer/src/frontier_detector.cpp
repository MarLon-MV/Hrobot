#include "create3_autonomous_explorer/frontier_detector.hpp"

#include <algorithm>
#include <cstddef>
#include <queue>
#include <utility>

namespace create3_autonomous_explorer
{

namespace
{

constexpr std::int8_t kUnknown = -1;
constexpr std::int8_t kFree = 0;

[[nodiscard]] inline std::size_t idx(int col, int row, int width)
{
  return static_cast<std::size_t>(row) * static_cast<std::size_t>(width) +
         static_cast<std::size_t>(col);
}

[[nodiscard]] bool is_frontier_cell(const GridMap & map, int col, int row)
{
  if (map.data[idx(col, row, map.width)] != kFree) {
    return false;
  }
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      if (dr == 0 && dc == 0) {
        continue;
      }
      const int c = col + dc;
      const int r = row + dr;
      if (c < 0 || r < 0 || c >= map.width || r >= map.height) {
        continue;
      }
      if (map.data[idx(c, r, map.width)] == kUnknown) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace

FrontierDetector::FrontierDetector(std::size_t min_frontier_size)
: min_frontier_size_(min_frontier_size) {}

std::size_t FrontierDetector::min_frontier_size() const {return min_frontier_size_;}

void FrontierDetector::set_min_frontier_size(std::size_t min_frontier_size)
{
  min_frontier_size_ = min_frontier_size;
}

std::vector<Frontier> FrontierDetector::detect(
  const GridMap & map, const Point2D & robot_position) const
{
  std::vector<Frontier> result;
  if (map.width <= 0 || map.height <= 0 ||
    map.data.size() != static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height))
  {
    return result;
  }

  std::vector<bool> is_frontier(map.data.size(), false);
  for (int r = 0; r < map.height; ++r) {
    for (int c = 0; c < map.width; ++c) {
      is_frontier[idx(c, r, map.width)] = is_frontier_cell(map, c, r);
    }
  }

  std::vector<bool> visited(map.data.size(), false);
  for (int r = 0; r < map.height; ++r) {
    for (int c = 0; c < map.width; ++c) {
      const std::size_t start = idx(c, r, map.width);
      if (!is_frontier[start] || visited[start]) {
        continue;
      }
      Frontier frontier;
      double sum_x = 0.0;
      double sum_y = 0.0;
      std::queue<std::pair<int, int>> queue;
      queue.emplace(c, r);
      visited[start] = true;
      while (!queue.empty()) {
        const auto [cc, cr] = queue.front();
        queue.pop();
        const Point2D center{
          map.origin.x + (cc + 0.5) * map.resolution,
          map.origin.y + (cr + 0.5) * map.resolution};
        frontier.cells.push_back(center);
        sum_x += center.x;
        sum_y += center.y;
        for (int dr = -1; dr <= 1; ++dr) {
          for (int dc = -1; dc <= 1; ++dc) {
            const int nc = cc + dc;
            const int nr = cr + dr;
            if ((dr == 0 && dc == 0) || nc < 0 || nr < 0 || nc >= map.width ||
              nr >= map.height)
            {
              continue;
            }
            const std::size_t n = idx(nc, nr, map.width);
            if (is_frontier[n] && !visited[n]) {
              visited[n] = true;
              queue.emplace(nc, nr);
            }
          }
        }
      }
      frontier.size = frontier.cells.size();
      if (frontier.size < min_frontier_size_) {
        continue;
      }
      const double n = static_cast<double>(frontier.size);
      frontier.centroid = Point2D{sum_x / n, sum_y / n};
      result.push_back(std::move(frontier));
    }
  }

  const auto dist2 = [&robot_position](const Frontier & f) {
      const double dx = f.centroid.x - robot_position.x;
      const double dy = f.centroid.y - robot_position.y;
      return dx * dx + dy * dy;
    };
  std::stable_sort(
    result.begin(), result.end(),
    [&dist2](const Frontier & a, const Frontier & b) {return dist2(a) < dist2(b);});
  return result;
}

}  // namespace create3_autonomous_explorer
