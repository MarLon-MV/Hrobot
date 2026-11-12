#ifndef CREATE3_AUTONOMOUS_EXPLORER__EXPLORER_NODE_HPP_
#define CREATE3_AUTONOMOUS_EXPLORER__EXPLORER_NODE_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "create3_autonomous_explorer/frontier_detector.hpp"
#include "create3_autonomous_explorer/goal_evaluator.hpp"
#include "create3_autonomous_explorer/types.hpp"
#include "create3_bot_interfaces/action/explore_map.hpp"
#include "create3_bot_interfaces/srv/save_exploration_map.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace create3_autonomous_explorer
{

/// ROS 2 node that drives frontier-based exploration.
///
/// Threading model (use with MultiThreadedExecutor):
///  - `map_group_`    (MutuallyExclusive): /map subscription.
///  - `action_group_` (Reentrant): ExploreMap server, NavigateToPose client,
///                                 SaveExplorationMap client.
///  - `timer_group_`  (MutuallyExclusive): control loop timer.
/// All mutable shared state lives behind `mutex_`. ROS calls (publish, send
/// goal, cancel, complete action) are never made while holding `mutex_`.
class ExplorerNode : public rclcpp::Node
{
public:
  using ExploreMap = create3_bot_interfaces::action::ExploreMap;
  using GoalHandleExploreMap = rclcpp_action::ServerGoalHandle<ExploreMap>;
  using SaveMapSrv = create3_bot_interfaces::srv::SaveExplorationMap;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit ExplorerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct Pose2D
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  enum class FinishStatus
  {
    Success,    ///< No reachable frontiers left.
    Timeout,    ///< max_duration_sec exceeded.
    Cancelled,  ///< Cancelled by the action client.
    Failure     ///< Too many recovery attempts, etc.
  };

  /// Log message queued while `mutex_` is held; emitted by `flush_logs()` after releasing it.
  struct LogEntry
  {
    bool warn{false};
    int throttle_ms{0};  ///< > 0: throttled warning.
    std::string text;
  };

  /// Work decided under the lock and executed after releasing it.
  struct TickActions
  {
    std::shared_ptr<GoalHandleExploreMap> goal_handle;
    std::vector<std::shared_ptr<NavGoalHandle>> cancel_nav;
    std::optional<Frontier> send_goal;
    double send_goal_yaw{0.0};
    std::uint64_t send_goal_generation{0};
    std::optional<geometry_msgs::msg::Twist> cmd_vel;
    bool send_save_request{false};
    bool publish_markers{false};
    std::vector<Frontier> marker_frontiers;
    std::optional<Frontier> marker_goal;
    bool clear_markers{false};
    std::shared_ptr<ExploreMap::Feedback> feedback;
    bool complete{false};
    FinishStatus status{FinishStatus::Success};
    std::shared_ptr<ExploreMap::Result> result;
  };

  // --- Action server callbacks ---
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExploreMap::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandleExploreMap> goal_handle);
  void handle_accepted(std::shared_ptr<GoalHandleExploreMap> goal_handle);

  // --- Map / pose ---
  void map_callback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  [[nodiscard]] bool lookup_pose(Pose2D & pose) const;

  // --- Navigation client callbacks (generation guards against stale callbacks) ---
  void nav_goal_response(std::uint64_t generation, std::shared_ptr<NavGoalHandle> handle);
  void nav_feedback(
    std::uint64_t generation,
    const std::shared_ptr<const NavigateToPose::Feedback> & feedback);
  void nav_result(std::uint64_t generation, const NavGoalHandle::WrappedResult & result);
  void handle_nav_result_locked(
    std::uint64_t generation, const NavGoalHandle::WrappedResult & result, double now_s);

  // --- Control loop ---
  void control_loop();
  /// Phase 1 (lock held). Returns a map snapshot if frontier detection is needed.
  [[nodiscard]] std::shared_ptr<const GridMap> plan_locked(
    bool pose_ok, const Pose2D & pose, bool nav_ready, double now_s, TickActions & act);
  /// Phase 3 (lock held). Applies detection results and may pick a new goal.
  void apply_detection_locked(
    std::vector<Frontier> frontiers, const Pose2D & pose, bool nav_ready, double now_s,
    TickActions & act);
  void execute(TickActions & act);

  // --- Helpers (lock held) ---
  void begin_finish_locked(FinishStatus status, const std::string & message, double now_s);
  void retire_nav_locked();
  void update_feedback_locked(TickActions & act) const;
  /// Queues a log message (lock held). Never logs directly while the mutex is taken.
  void queue_log_locked(bool warn, std::string text, int throttle_ms = 0);

  // --- Helpers (no lock) ---
  void send_nav_goal(const Frontier & frontier, double yaw, std::uint64_t generation);
  void send_save_request();
  /// Emits queued log messages. Takes the lock only to swap the queue, logs outside it.
  void flush_logs();
  /// Cancels a Nav2 goal; swallows errors from already-terminal/unknown goals.
  void cancel_nav_goal_safe(const std::shared_ptr<NavGoalHandle> & handle);
  void publish_markers(
    const std::vector<Frontier> & frontiers, const std::optional<Frontier> & current);
  void publish_clear_markers();
  [[nodiscard]] double now_sec() const;
  [[nodiscard]] static const char * state_to_string(ExplorerState state);

  // --- ROS interfaces ---
  rclcpp::CallbackGroup::SharedPtr map_group_;
  rclcpp::CallbackGroup::SharedPtr action_group_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp_action::Server<ExploreMap>::SharedPtr action_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Client<SaveMapSrv>::SharedPtr save_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // --- Parameters (immutable after construction, readable without lock) ---
  std::string map_frame_;
  std::string base_frame_;
  double free_threshold_{25.0};
  double stuck_timeout_sec_{15.0};
  double progress_epsilon_{0.1};
  double recovery_rotation_sec_{4.0};
  double recovery_angular_speed_{0.5};
  int max_recovery_attempts_{3};
  double detection_period_sec_{1.0};
  bool save_map_on_finish_{true};
  std::string map_name_;
  double save_timeout_sec_{30.0};

  // --- Shared state, guarded by mutex_ ---
  mutable std::mutex mutex_;
  std::shared_ptr<const GridMap> map_;
  ExplorerState state_{ExplorerState::Idle};
  FrontierDetector detector_;  // immutable after construction
  GoalEvaluator evaluator_;
  std::shared_ptr<GoalHandleExploreMap> goal_handle_;
  bool busy_{false};  // true from handle_goal acceptance until completion
  bool cancel_requested_{false};
  double max_duration_sec_{0.0};
  double start_time_{0.0};
  std::uint32_t frontiers_explored_{0};
  double distance_traveled_{0.0};
  std::optional<Pose2D> last_pose_;
  std::vector<Frontier> frontiers_;
  double last_detection_time_{0.0};
  std::optional<Frontier> current_goal_;

  std::uint64_t nav_generation_{0};
  bool nav_pending_{false};   // goal sent, no response yet
  bool nav_accepted_{false};  // goal accepted and running
  std::shared_ptr<NavGoalHandle> nav_handle_;
  std::vector<std::shared_ptr<NavGoalHandle>> to_cancel_;
  double best_distance_remaining_{0.0};
  double last_progress_time_{0.0};

  int recovery_attempts_{0};
  double recovery_end_time_{0.0};
  bool stop_requested_{false};

  bool finishing_{false};
  FinishStatus finish_status_{FinishStatus::Success};
  bool save_request_needed_{false};
  std::uint64_t save_session_{0};
  bool save_pending_{false};
  double save_deadline_{0.0};
  std::string map_path_;
  std::string posegraph_path_;
  std::vector<LogEntry> pending_logs_;
};

}  // namespace create3_autonomous_explorer

#endif  // CREATE3_AUTONOMOUS_EXPLORER__EXPLORER_NODE_HPP_
