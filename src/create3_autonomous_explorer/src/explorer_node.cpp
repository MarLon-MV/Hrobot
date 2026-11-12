#include "create3_autonomous_explorer/explorer_node.hpp"

#include <algorithm>
#include <exception>
#include <mutex>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace create3_autonomous_explorer
{

using std::placeholders::_1;
using std::placeholders::_2;

namespace
{

/// Yaw from a quaternion (z-axis rotation); avoids a tf2_geometry_msgs dependency.
[[nodiscard]] double yaw_from_quaternion(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

[[nodiscard]] visualization_msgs::msg::Marker make_marker(
  int id, const std::string & ns, int type, double x, double y, double z, double scale)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";  // overwritten by caller
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x = x;
  m.pose.position.y = y;
  m.pose.position.z = z;
  m.pose.orientation.w = 1.0;
  m.scale.x = scale;
  m.scale.y = scale;
  m.scale.z = scale;
  return m;
}

}  // namespace

ExplorerNode::ExplorerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("explorer_node", options)
{
  // ---- Parameters ----
  const int min_size = declare_parameter<int>("min_frontier_size", 3);
  GoalWeights weights;
  weights.size = declare_parameter<double>("weight_size", 1.0);
  weights.distance = declare_parameter<double>("weight_distance", 1.0);
  weights.angle = declare_parameter<double>("weight_angle", 1.0);
  const double blacklist_radius = declare_parameter<double>("blacklist_radius", 0.5);
  double control_rate_hz = declare_parameter<double>("control_rate_hz", 10.0);
  stuck_timeout_sec_ = declare_parameter<double>("stuck_timeout_sec", 15.0);
  progress_epsilon_ = declare_parameter<double>("progress_epsilon", 0.1);
  recovery_rotation_sec_ = declare_parameter<double>("recovery_rotation_sec", 4.0);
  recovery_angular_speed_ = declare_parameter<double>("recovery_angular_speed", 0.5);
  max_recovery_attempts_ = declare_parameter<int>("max_recovery_attempts", 3);
  detection_period_sec_ = declare_parameter<double>("detection_period_sec", 1.0);
  free_threshold_ = declare_parameter<double>("free_threshold", 25.0);
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  const std::string map_topic = declare_parameter<std::string>("map_topic", "/map");
  const std::string frontiers_topic = declare_parameter<std::string>("frontiers_topic", "/frontiers");
  const std::string cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
  const std::string nav_action = declare_parameter<std::string>("navigate_action", "navigate_to_pose");
  // Optional map saving. Empty service name disables it; map_path stays empty then.
  const std::string save_service =
    declare_parameter<std::string>("save_map_service", "save_exploration_map");
  save_map_on_finish_ = declare_parameter<bool>("save_map_on_finish", true) && !save_service.empty();
  map_name_ = declare_parameter<std::string>("map_name", "exploration_map");
  save_timeout_sec_ = declare_parameter<double>("save_timeout_sec", 30.0);

  if (!(control_rate_hz > 0.0)) {
    RCLCPP_WARN(get_logger(), "control_rate_hz must be > 0; using 10.0");
    control_rate_hz = 10.0;
  }
  if (min_size < 1) {
    RCLCPP_WARN(get_logger(), "min_frontier_size must be >= 1; using 1");
  }
  detector_ = FrontierDetector(static_cast<std::size_t>(std::max(min_size, 1)));
  evaluator_ = GoalEvaluator(weights, std::max(blacklist_radius, 0.0));
  max_recovery_attempts_ = std::max(max_recovery_attempts_, 0);

  // ---- Callback groups ----
  map_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  timer_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // ---- TF2 ----
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // ---- Pub / Sub ----
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = map_group_;
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic, rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&ExplorerNode::map_callback, this, _1), sub_options);

  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(frontiers_topic, 10);
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

  // ---- Action client / service client ----
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, nav_action, action_group_);
  if (save_map_on_finish_) {
    save_client_ = create_client<SaveMapSrv>(save_service, rclcpp::ServicesQoS(), action_group_);
  }

  // ---- Action server ----
  action_server_ = rclcpp_action::create_server<ExploreMap>(
    this, "explore_map",
    std::bind(&ExplorerNode::handle_goal, this, _1, _2),
    std::bind(&ExplorerNode::handle_cancel, this, _1),
    std::bind(&ExplorerNode::handle_accepted, this, _1),
    rcl_action_server_get_default_options(), action_group_);

  // ---- Control loop ----
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / control_rate_hz));
  timer_ = create_wall_timer(period, std::bind(&ExplorerNode::control_loop, this), timer_group_);

  RCLCPP_INFO(get_logger(), "explorer_node ready (action: explore_map)");
}

// ---------------------------------------------------------------------------
// Action server
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse ExplorerNode::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const ExploreMap::Goal> goal)
{
  if (!goal->start_exploration) {
    RCLCPP_WARN(get_logger(), "Goal rejected: start_exploration is false");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!std::isfinite(goal->max_duration_sec) || goal->max_duration_sec < 0.0F) {
    RCLCPP_WARN(get_logger(), "Goal rejected: invalid max_duration_sec");
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool already_busy = false;
  {
    std::scoped_lock lock(mutex_);
    already_busy = busy_;
    if (!already_busy) {
      busy_ = true;  // reserve until handle_accepted / completion
    }
  }
  if (already_busy) {
    RCLCPP_WARN(get_logger(), "Goal rejected: an exploration is already running");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ExplorerNode::handle_cancel(
  std::shared_ptr<GoalHandleExploreMap> goal_handle)
{
  std::scoped_lock lock(mutex_);
  if (goal_handle != goal_handle_ || finishing_) {
    return rclcpp_action::CancelResponse::REJECT;
  }
  cancel_requested_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ExplorerNode::handle_accepted(std::shared_ptr<GoalHandleExploreMap> goal_handle)
{
  const double now_s = now_sec();
  double max_duration = 0.0;
  {
    std::scoped_lock lock(mutex_);
    goal_handle_ = goal_handle;
    busy_ = true;
    cancel_requested_ = false;
    finishing_ = false;
    stop_requested_ = false;
    save_request_needed_ = false;
    save_pending_ = false;
    map_path_.clear();
    posegraph_path_.clear();
    ++save_session_;
    max_duration_sec_ = static_cast<double>(goal_handle->get_goal()->max_duration_sec);
    start_time_ = now_s;
    frontiers_explored_ = 0;
    distance_traveled_ = 0.0;
    last_pose_.reset();
    frontiers_.clear();
    last_detection_time_ = 0.0;
    current_goal_.reset();
    nav_pending_ = false;
    nav_accepted_ = false;
    nav_handle_.reset();
    recovery_attempts_ = 0;
    evaluator_.clear_blacklist();
    state_ = map_ ? ExplorerState::Detecting : ExplorerState::WaitingForMap;
    max_duration = max_duration_sec_;
  }
  RCLCPP_INFO(get_logger(), "Exploration started (max_duration_sec=%.1f)", max_duration);
}

// ---------------------------------------------------------------------------
// Map and pose
// ---------------------------------------------------------------------------

void ExplorerNode::map_callback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  // ROS -> GridMap conversion happens only here. Built outside the lock.
  auto grid = std::make_shared<GridMap>();
  grid->width = static_cast<int>(msg->info.width);
  grid->height = static_cast<int>(msg->info.height);
  grid->resolution = static_cast<double>(msg->info.resolution);
  grid->origin = Point2D{msg->info.origin.position.x, msg->info.origin.position.y};
  grid->data.reserve(msg->data.size());
  for (const auto v : msg->data) {
    if (v < 0) {
      grid->data.push_back(-1);
    } else if (static_cast<double>(v) <= free_threshold_) {
      grid->data.push_back(0);
    } else {
      grid->data.push_back(100);
    }
  }
  const auto & q = msg->info.origin.orientation;
  if (std::abs(yaw_from_quaternion(q.x, q.y, q.z, q.w)) > 1e-3) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000, "Map origin is rotated; rotation is ignored");
  }

  std::scoped_lock lock(mutex_);
  map_ = std::move(grid);
}

bool ExplorerNode::lookup_pose(Pose2D & pose) const
{
  try {
    const auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    const auto & q = tf.transform.rotation;
    pose.x = tf.transform.translation.x;
    pose.y = tf.transform.translation.y;
    pose.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "TF %s -> %s unavailable: %s", map_frame_.c_str(),
      base_frame_.c_str(), ex.what());
    return false;
  }
}

// ---------------------------------------------------------------------------
// Navigation client callbacks
// ---------------------------------------------------------------------------

void ExplorerNode::nav_goal_response(std::uint64_t generation, std::shared_ptr<NavGoalHandle> handle)
{
  const double now_s = now_sec();
  bool handled = false;
  {
    std::scoped_lock lock(mutex_);
    if (generation == nav_generation_ && nav_pending_) {
      nav_pending_ = false;
      if (handle) {
        nav_accepted_ = true;
        nav_handle_ = handle;
        best_distance_remaining_ = std::numeric_limits<double>::infinity();
        last_progress_time_ = now_s;
      } else {
        queue_log_locked(true, "NavigateToPose goal rejected; blacklisting frontier");
        if (current_goal_) {
          evaluator_.add_to_blacklist(current_goal_->centroid);
        }
        current_goal_.reset();
        ++recovery_attempts_;
        if (recovery_attempts_ > max_recovery_attempts_) {
          begin_finish_locked(FinishStatus::Failure, "too many failed navigation goals", now_s);
        } else if (!finishing_) {
          state_ = ExplorerState::Detecting;
        }
      }
      handled = true;
    }
  }
  flush_logs();
  if (handled) {
    return;
  }
  // Stale response (goal was retired meanwhile): cancel it if it was accepted.
  cancel_nav_goal_safe(handle);
}

void ExplorerNode::nav_feedback(
  std::uint64_t generation, const std::shared_ptr<const NavigateToPose::Feedback> & feedback)
{
  const double now_s = now_sec();
  std::scoped_lock lock(mutex_);
  if (generation != nav_generation_ || !nav_accepted_) {
    return;
  }
  const double remaining = static_cast<double>(feedback->distance_remaining);
  if (remaining < best_distance_remaining_ - progress_epsilon_) {
    best_distance_remaining_ = remaining;
    last_progress_time_ = now_s;
  }
}

void ExplorerNode::nav_result(std::uint64_t generation, const NavGoalHandle::WrappedResult & result)
{
  {
    const double now_s = now_sec();
    std::scoped_lock lock(mutex_);
    handle_nav_result_locked(generation, result, now_s);
  }
  flush_logs();
}

void ExplorerNode::handle_nav_result_locked(
  std::uint64_t generation, const NavGoalHandle::WrappedResult & result, double now_s)
{
  if (generation != nav_generation_) {
    return;  // stale: retired by us (cancel, stuck, finish)
  }
  nav_accepted_ = false;
  nav_pending_ = false;
  nav_handle_.reset();
  if (finishing_ || !goal_handle_) {
    return;
  }
  const auto goal = current_goal_;
  current_goal_.reset();
  if (goal) {
    // Always blacklist the visited/failed point to avoid re-selecting it.
    evaluator_.add_to_blacklist(goal->centroid);
  }
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      ++frontiers_explored_;
      recovery_attempts_ = 0;
      state_ = ExplorerState::Detecting;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      queue_log_locked(true, "NavigateToPose was canceled externally");
      state_ = ExplorerState::Detecting;
      break;
    case rclcpp_action::ResultCode::ABORTED:
    case rclcpp_action::ResultCode::UNKNOWN:
    default:
      queue_log_locked(true, "NavigateToPose aborted/unknown; blacklisting frontier");
      ++recovery_attempts_;
      if (recovery_attempts_ > max_recovery_attempts_) {
        begin_finish_locked(FinishStatus::Failure, "too many failed navigation goals", now_s);
      } else {
        state_ = ExplorerState::Detecting;
      }
      break;
  }
}

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------

void ExplorerNode::control_loop()
{
  Pose2D pose;
  const bool pose_ok = lookup_pose(pose);  // thread-safe tf buffer, no lock needed
  const bool nav_ready = nav_client_->action_server_is_ready();
  const double now_s = now_sec();

  TickActions act;
  std::shared_ptr<const GridMap> detect_map;
  {
    std::scoped_lock lock(mutex_);
    if (!goal_handle_) {
      return;
    }
    detect_map = plan_locked(pose_ok, pose, nav_ready, now_s, act);
  }
  execute(act);

  if (!detect_map) {
    return;
  }

  // Heavy work (BFS) without holding the lock. detector_ is immutable.
  std::vector<Frontier> frontiers = detector_.detect(*detect_map, Point2D{pose.x, pose.y});

  TickActions act2;
  {
    std::scoped_lock lock(mutex_);
    // The session may have ended or changed state while detecting.
    if (!goal_handle_ || goal_handle_ != act.goal_handle || finishing_ ||
      (state_ != ExplorerState::Detecting && state_ != ExplorerState::Navigating))
    {
      return;
    }
    act2.goal_handle = goal_handle_;
    apply_detection_locked(std::move(frontiers), pose, nav_ready, now_s, act2);
  }
  execute(act2);
}

std::shared_ptr<const GridMap> ExplorerNode::plan_locked(
  bool pose_ok, const Pose2D & pose, bool nav_ready, double now_s, TickActions & act)
{
  act.goal_handle = goal_handle_;

  auto drain = [this, &act]() {
      act.cancel_nav = std::move(to_cancel_);
      to_cancel_.clear();
      if (stop_requested_) {
        act.cmd_vel = geometry_msgs::msg::Twist();
        stop_requested_ = false;
      }
      if (save_request_needed_) {
        act.send_save_request = true;
        save_request_needed_ = false;
      }
    };

  // 1. Finishing: wait for the optional map save, then complete the action.
  if (finishing_) {
    drain();
    if (!save_pending_ || now_s >= save_deadline_) {
      act.complete = true;
      act.status = finish_status_;
      act.result = std::make_shared<ExploreMap::Result>();
      act.result->success = (finish_status_ == FinishStatus::Success);
      act.result->frontiers_explored = frontiers_explored_;
      act.result->map_path = map_path_;
      act.clear_markers = true;
      goal_handle_.reset();
      busy_ = false;
      state_ = ExplorerState::Idle;
      frontiers_.clear();
      current_goal_.reset();
    }
    return nullptr;
  }

  // 2. Cancel / timeout.
  if (cancel_requested_) {
    begin_finish_locked(FinishStatus::Cancelled, "exploration cancelled", now_s);
    drain();
    return nullptr;
  }
  if (max_duration_sec_ > 0.0 && now_s - start_time_ >= max_duration_sec_) {
    begin_finish_locked(FinishStatus::Timeout, "max_duration_sec exceeded", now_s);
    drain();
    return nullptr;
  }

  // 3. State machine.
  std::shared_ptr<const GridMap> detect_map;
  if (state_ == ExplorerState::WaitingForMap) {
    if (map_) {
      state_ = ExplorerState::Detecting;
    }
  }

  if (state_ == ExplorerState::Recovering) {
    if (now_s >= recovery_end_time_) {
      act.cmd_vel = geometry_msgs::msg::Twist();  // stop
      if (current_goal_) {
        evaluator_.add_to_blacklist(current_goal_->centroid);
      }
      current_goal_.reset();
      state_ = ExplorerState::Detecting;
    } else {
      geometry_msgs::msg::Twist twist;
      twist.angular.z = recovery_angular_speed_;
      act.cmd_vel = twist;
    }
  } else if (pose_ok) {
    if (last_pose_) {
      distance_traveled_ += std::hypot(pose.x - last_pose_->x, pose.y - last_pose_->y);
    }
    last_pose_ = pose;

    if (state_ == ExplorerState::Navigating) {
      if ((nav_pending_ || nav_accepted_) && now_s - last_progress_time_ > stuck_timeout_sec_) {
        queue_log_locked(true, "Navigation stuck (no progress); starting rotation recovery");
        ++recovery_attempts_;
        retire_nav_locked();
        if (recovery_attempts_ > max_recovery_attempts_) {
          begin_finish_locked(FinishStatus::Failure, "stuck too many times", now_s);
        } else {
          state_ = ExplorerState::Recovering;
          recovery_end_time_ = now_s + recovery_rotation_sec_;
        }
      } else if (now_s - last_detection_time_ >= detection_period_sec_ && map_) {
        detect_map = map_;
      }
    } else if (
      state_ == ExplorerState::Detecting && map_ &&
      (nav_ready || now_s - last_detection_time_ >= detection_period_sec_))
    {
      // Without Nav2 ready we only wait: do not rerun the BFS at control-loop rate.
      detect_map = map_;
    }
  }
  // Without TF (pose_ok == false) the node just waits; stuck logic needs a pose.

  drain();
  update_feedback_locked(act);
  return detect_map;
}

void ExplorerNode::apply_detection_locked(
  std::vector<Frontier> frontiers, const Pose2D & pose, bool nav_ready, double now_s,
  TickActions & act)
{
  frontiers_ = std::move(frontiers);
  last_detection_time_ = now_s;
  for (auto & f : frontiers_) {
    f.score = evaluator_.score(f, pose.x, pose.y, pose.yaw);  // for markers
  }

  if (state_ == ExplorerState::Detecting) {
    const auto best = evaluator_.select_best(frontiers_, pose.x, pose.y, pose.yaw);
    if (!best) {
      begin_finish_locked(FinishStatus::Success, "no frontiers left", now_s);
      act.cancel_nav = std::move(to_cancel_);
      to_cancel_.clear();
      if (save_request_needed_) {
        act.send_save_request = true;
        save_request_needed_ = false;
      }
      act.publish_markers = true;
      return;
    }
    if (nav_ready) {
      current_goal_ = best;
      state_ = ExplorerState::Navigating;
      nav_generation_++;
      nav_pending_ = true;
      nav_accepted_ = false;
      nav_handle_.reset();
      last_progress_time_ = now_s;
      best_distance_remaining_ = std::numeric_limits<double>::infinity();
      act.send_goal = best;
      act.send_goal_yaw = std::atan2(best->centroid.y - pose.y, best->centroid.x - pose.x);
      act.send_goal_generation = nav_generation_;
    } else {
      queue_log_locked(true, "navigate_to_pose action server not available yet", 5000);
    }
  }

  act.publish_markers = true;
  act.marker_frontiers = frontiers_;
  act.marker_goal = current_goal_;
  update_feedback_locked(act);
}

void ExplorerNode::update_feedback_locked(TickActions & act) const
{
  auto fb = std::make_shared<ExploreMap::Feedback>();
  fb->current_state = state_to_string(state_);
  std::uint32_t remaining = 0;
  for (const auto & f : frontiers_) {
    if (!evaluator_.is_blacklisted(f.centroid)) {
      ++remaining;
    }
  }
  fb->remaining_frontiers = remaining;
  fb->distance_traveled = static_cast<float>(distance_traveled_);
  act.feedback = std::move(fb);
}

void ExplorerNode::execute(TickActions & act)
{
  flush_logs();  // messages queued by plan_locked / apply_detection_locked
  for (const auto & h : act.cancel_nav) {
    cancel_nav_goal_safe(h);
  }
  if (act.cmd_vel) {
    cmd_vel_pub_->publish(*act.cmd_vel);
  }
  if (act.send_save_request) {
    send_save_request();
  }
  if (act.send_goal) {
    send_nav_goal(*act.send_goal, act.send_goal_yaw, act.send_goal_generation);
  }
  if (act.publish_markers) {
    publish_markers(act.marker_frontiers, act.marker_goal);
  }
  if (act.clear_markers) {
    publish_clear_markers();
  }
  if (act.feedback && act.goal_handle && !act.complete && act.goal_handle->is_active()) {
    act.goal_handle->publish_feedback(act.feedback);
  }
  if (act.complete && act.goal_handle && act.goal_handle->is_active()) {
    // Result semantics:
    //  Success   -> succeed (success=true)
    //  Timeout   -> succeed (success=false, partial map)
    //  Cancelled -> canceled
    //  Failure   -> abort
    // canceled() is only valid from CANCELING; if the cancel request was accepted but the
    // goal has not transitioned yet, report abort instead of throwing.
    try {
      if (act.goal_handle->is_canceling()) {
        act.goal_handle->canceled(act.result);
      } else if (act.status == FinishStatus::Cancelled || act.status == FinishStatus::Failure) {
        act.goal_handle->abort(act.result);
      } else {
        act.goal_handle->succeed(act.result);
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Could not finalize ExploreMap goal: %s", e.what());
    }
    RCLCPP_INFO(
      get_logger(), "Exploration finished (explored=%u, map_path='%s')",
      static_cast<unsigned>(act.result->frontiers_explored), act.result->map_path.c_str());
  }
}

void ExplorerNode::cancel_nav_goal_safe(const std::shared_ptr<NavGoalHandle> & handle)
{
  if (!handle) {
    return;
  }
  try {
    nav_client_->async_cancel_goal(handle);
  } catch (const std::exception & e) {
    // The goal may already be terminal / unknown to the client; nothing left to cancel.
    RCLCPP_WARN(get_logger(), "Could not cancel NavigateToPose goal: %s", e.what());
  }
}

// ---------------------------------------------------------------------------
// Helpers (lock held)
// ---------------------------------------------------------------------------

void ExplorerNode::retire_nav_locked()
{
  ++nav_generation_;  // invalidates pending callbacks of the previous goal
  if (nav_handle_) {
    to_cancel_.push_back(nav_handle_);
  }
  nav_handle_.reset();
  nav_pending_ = false;
  nav_accepted_ = false;
}

void ExplorerNode::begin_finish_locked(
  FinishStatus status, const std::string & message, double now_s)
{
  if (finishing_) {
    return;
  }
  queue_log_locked(false, "Finishing exploration: " + message);
  retire_nav_locked();
  current_goal_.reset();
  finishing_ = true;
  finish_status_ = status;
  state_ = (status == FinishStatus::Failure) ? ExplorerState::Failed : ExplorerState::Finished;
  stop_requested_ = true;
  save_pending_ = save_map_on_finish_ && save_client_ && map_ != nullptr;
  save_request_needed_ = save_pending_;
  save_deadline_ = now_s + save_timeout_sec_;
}

void ExplorerNode::queue_log_locked(bool warn, std::string text, int throttle_ms)
{
  pending_logs_.push_back(LogEntry{warn, throttle_ms, std::move(text)});
}

// ---------------------------------------------------------------------------
// Helpers (no lock)
// ---------------------------------------------------------------------------

void ExplorerNode::flush_logs()
{
  std::vector<LogEntry> logs;
  {
    std::scoped_lock lock(mutex_);
    logs.swap(pending_logs_);
  }
  for (const auto & entry : logs) {
    if (!entry.warn) {
      RCLCPP_INFO(get_logger(), "%s", entry.text.c_str());
    } else if (entry.throttle_ms > 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), entry.throttle_ms, "%s", entry.text.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", entry.text.c_str());
    }
  }
}

void ExplorerNode::send_nav_goal(const Frontier & frontier, double yaw, std::uint64_t generation)
{
  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = map_frame_;
  goal.pose.header.stamp = now();
  goal.pose.pose.position.x = frontier.centroid.x;
  goal.pose.pose.position.y = frontier.centroid.y;
  goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
  goal.pose.pose.orientation.w = std::cos(yaw / 2.0);

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback =
    [this, generation](std::shared_ptr<NavGoalHandle> handle) {
      nav_goal_response(generation, std::move(handle));
    };
  options.feedback_callback =
    [this, generation](
    std::shared_ptr<NavGoalHandle>, const std::shared_ptr<const NavigateToPose::Feedback> fb) {
      nav_feedback(generation, fb);
    };
  options.result_callback =
    [this, generation](const NavGoalHandle::WrappedResult & result) {
      nav_result(generation, result);
    };
  nav_client_->async_send_goal(goal, options);
}

void ExplorerNode::send_save_request()
{
  if (!save_client_ || !save_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Save-map service not available; map_path will be empty");
    {
      std::scoped_lock lock(mutex_);
      save_pending_ = false;
    }
    return;
  }
  auto request = std::make_shared<SaveMapSrv::Request>();
  request->map_name = map_name_;
  std::uint64_t session = 0;
  {
    std::scoped_lock lock(mutex_);
    session = save_session_;
  }
  save_client_->async_send_request(
    request, [this, session](rclcpp::Client<SaveMapSrv>::SharedFuture future) {
      const auto response = future.get();
      bool failed = false;
      {
        std::scoped_lock lock(mutex_);
        if (session != save_session_) {
          return;  // late response from a previous session
        }
        if (response->success) {
          map_path_ = response->map_path;
          posegraph_path_ = response->posegraph_path;
        } else {
          failed = true;
        }
        save_pending_ = false;
      }
      if (failed) {
        RCLCPP_WARN(get_logger(), "Save map failed: %s", response->message.c_str());
      } else {
        RCLCPP_INFO(
          get_logger(), "Map saved (map_path='%s', posegraph_path='%s')",
          response->map_path.c_str(), response->posegraph_path.c_str());
      }
    });
}

void ExplorerNode::publish_markers(
  const std::vector<Frontier> & frontiers, const std::optional<Frontier> & current)
{
  visualization_msgs::msg::MarkerArray array;
  const auto stamp = now();

  visualization_msgs::msg::Marker del;
  del.header.frame_id = map_frame_;
  del.header.stamp = stamp;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(del);

  double min_score = std::numeric_limits<double>::infinity();
  double max_score = -std::numeric_limits<double>::infinity();
  for (const auto & f : frontiers) {
    min_score = std::min(min_score, f.score);
    max_score = std::max(max_score, f.score);
  }
  const double span = max_score - min_score;

  int id = 0;
  for (const auto & f : frontiers) {
    // Scale by size, clamped to [0.15, 0.6] m.
    const double scale = std::clamp(0.1 + 0.02 * static_cast<double>(f.size), 0.15, 0.6);
    // Color by score: red (low) -> green (high); yellow if all scores are equal.
    const double t = (span > 1e-9) ? (f.score - min_score) / span : 0.5;

    auto sphere = make_marker(
      id++, "frontiers", visualization_msgs::msg::Marker::SPHERE, f.centroid.x, f.centroid.y, 0.1,
      scale);
    sphere.header.frame_id = map_frame_;
    sphere.header.stamp = stamp;
    sphere.color.r = static_cast<float>(1.0 - t);
    sphere.color.g = static_cast<float>(t);
    sphere.color.b = 0.0F;
    sphere.color.a = 0.8F;
    array.markers.push_back(sphere);

    char buf[64];
    std::snprintf(buf, sizeof(buf), "size=%zu score=%.1f", f.size, f.score);
    auto text = make_marker(
      id++, "frontier_text", visualization_msgs::msg::Marker::TEXT_VIEW_FACING, f.centroid.x,
      f.centroid.y, 0.1 + scale + 0.15, 0.2);
    text.header.frame_id = map_frame_;
    text.header.stamp = stamp;
    text.text = buf;
    text.color.r = 1.0F;
    text.color.g = 1.0F;
    text.color.b = 1.0F;
    text.color.a = 1.0F;
    array.markers.push_back(text);
  }

  if (current) {
    auto goal = make_marker(
      id++, "current_goal", visualization_msgs::msg::Marker::CYLINDER, current->centroid.x,
      current->centroid.y, 0.25, 0.4);
    goal.scale.z = 0.5;
    goal.header.frame_id = map_frame_;
    goal.header.stamp = stamp;
    goal.color.r = 0.0F;
    goal.color.g = 0.4F;
    goal.color.b = 1.0F;
    goal.color.a = 0.9F;
    array.markers.push_back(goal);
  }
  markers_pub_->publish(array);
}

void ExplorerNode::publish_clear_markers()
{
  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker del;
  del.header.frame_id = map_frame_;
  del.header.stamp = now();
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(del);
  markers_pub_->publish(array);
}

double ExplorerNode::now_sec() const
{
  return get_clock()->now().seconds();
}

const char * ExplorerNode::state_to_string(ExplorerState state)
{
  switch (state) {
    case ExplorerState::Idle: return "Idle";
    case ExplorerState::WaitingForMap: return "WaitingForMap";
    case ExplorerState::Detecting: return "Detecting";
    case ExplorerState::Navigating: return "Navigating";
    case ExplorerState::Recovering: return "Recovering";
    case ExplorerState::Finished: return "Finished";
    case ExplorerState::Failed: return "Failed";
  }
  return "Unknown";
}

}  // namespace create3_autonomous_explorer
