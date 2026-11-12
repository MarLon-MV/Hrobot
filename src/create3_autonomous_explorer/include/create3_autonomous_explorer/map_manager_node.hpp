#ifndef CREATE3_AUTONOMOUS_EXPLORER__MAP_MANAGER_NODE_HPP_
#define CREATE3_AUTONOMOUS_EXPLORER__MAP_MANAGER_NODE_HPP_

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>

#include "create3_bot_interfaces/srv/save_exploration_map.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "slam_toolbox/srv/serialize_pose_graph.hpp"

namespace create3_autonomous_explorer
{

/// Saves the current map on request: occupancy grid (nav2 map_saver: .yaml + .pgm) and the
/// slam_toolbox pose graph (.posegraph + .data) so the map can be extended later.
///
/// Threading / blocking pattern (use with MultiThreadedExecutor, >= 2 threads):
///  - `service_group_` (MutuallyExclusive): the `save_exploration_map` service. Requests are
///    therefore serialized, and the handler is allowed to block while waiting for the clients.
///  - `client_group_`  (Reentrant): the SaveMap / SerializePoseGraph clients. Their responses
///    are dispatched in this other group, so another executor thread completes the future
///    while the service thread waits on it with `std::future::wait_for` (bounded by
///    `service_timeout_sec`). Waiting on a future whose response callback lives in the same
///    mutually-exclusive group (or in a single-threaded executor) would deadlock; that is why
///    the groups are separate.
/// All parameters are immutable after construction; there is no other shared mutable state.
class MapManagerNode : public rclcpp::Node
{
public:
  using SaveExplorationMap = create3_bot_interfaces::srv::SaveExplorationMap;
  using SaveMap = nav2_msgs::srv::SaveMap;
  using SerializePoseGraph = slam_toolbox::srv::SerializePoseGraph;

  explicit MapManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void handle_save(
    const std::shared_ptr<SaveExplorationMap::Request> request,
    std::shared_ptr<SaveExplorationMap::Response> response);
  /// Does the work; may throw (the caller converts any exception into a failure response).
  void do_save(const SaveExplorationMap::Request & request, SaveExplorationMap::Response & response);

  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp::Service<SaveExplorationMap>::SharedPtr save_service_;
  rclcpp::Client<SaveMap>::SharedPtr map_saver_client_;
  rclcpp::Client<SerializePoseGraph>::SharedPtr serialize_client_;

  // --- Parameters (immutable after construction) ---
  std::filesystem::path maps_dir_;
  std::chrono::milliseconds service_timeout_{5000};
  std::string map_topic_;
  std::string image_format_;
  std::string map_mode_;
  float occupied_thresh_{0.65F};
  float free_thresh_{0.25F};
};

}  // namespace create3_autonomous_explorer

#endif  // CREATE3_AUTONOMOUS_EXPLORER__MAP_MANAGER_NODE_HPP_
