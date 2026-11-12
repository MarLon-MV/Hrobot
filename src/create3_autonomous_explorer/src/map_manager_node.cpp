#include "create3_autonomous_explorer/map_manager_node.hpp"

#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <functional>
#include <future>
#include <system_error>
#include <utility>

#include "create3_autonomous_explorer/map_naming.hpp"

namespace create3_autonomous_explorer
{

using std::placeholders::_1;
using std::placeholders::_2;

namespace
{

/// Upper bound for service_timeout_sec (avoids overflow converting to milliseconds).
constexpr double kMaxServiceTimeoutSec = 600.0;

/// Expands a leading "~" (alone or followed by '/') using $HOME. Left untouched if HOME is unset.
[[nodiscard]] std::filesystem::path expand_home(const std::string & raw)
{
  if (raw.empty() || raw.front() != '~') {
    return std::filesystem::path(raw);
  }
  if (raw.size() > 1 && raw[1] != '/') {
    return std::filesystem::path(raw);  // "~user" is not supported
  }
  const char * home = std::getenv("HOME");
  if (home == nullptr || *home == '\0') {
    return std::filesystem::path(raw);
  }
  return std::filesystem::path(home) / std::filesystem::path(raw.substr(raw.size() > 1 ? 2 : 1));
}

/// Calls a service and waits (bounded) for the response without touching the executor.
/// Returns nullptr and fills `error` when unavailable, timed out or failed.
template<typename Srv>
[[nodiscard]] std::shared_ptr<typename Srv::Response> call_service(
  const typename rclcpp::Client<Srv>::SharedPtr & client,
  typename rclcpp::Client<Srv>::SharedRequest request, std::chrono::milliseconds timeout,
  const std::string & label, std::string & error)
{
  if (!client->wait_for_service(timeout)) {
    error = label + " service not available";
    return nullptr;
  }
  auto pending = client->async_send_request(std::move(request));
  if (pending.future.wait_for(timeout) != std::future_status::ready) {
    client->remove_pending_request(pending.request_id);
    error = label + " service timed out";
    return nullptr;
  }
  auto response = pending.future.get();
  if (!response) {
    error = label + " service returned no response";
  }
  return response;
}

}  // namespace

MapManagerNode::MapManagerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("map_manager_node", options)
{
  const std::string maps_dir = declare_parameter<std::string>("maps_dir", "~/.create3_maps");
  const std::string saver_service =
    declare_parameter<std::string>("map_saver_service", "map_saver/save_map");
  const std::string serialize_service =
    declare_parameter<std::string>("serialize_service", "slam_toolbox/serialize_map");
  double timeout_sec = declare_parameter<double>("service_timeout_sec", 5.0);
  const double occupied = declare_parameter<double>("occupied_thresh", 0.65);
  const double free_thresh = declare_parameter<double>("free_thresh", 0.25);
  image_format_ = declare_parameter<std::string>("image_format", "pgm");
  map_mode_ = declare_parameter<std::string>("map_mode", "trinary");
  map_topic_ = declare_parameter<std::string>("map_topic", "map");
  const std::string save_service =
    declare_parameter<std::string>("save_service_name", "save_exploration_map");

  if (!(timeout_sec > 0.0)) {
    RCLCPP_WARN(get_logger(), "service_timeout_sec must be > 0; using 5.0");
    timeout_sec = 5.0;
  }
  timeout_sec = std::min(timeout_sec, kMaxServiceTimeoutSec);
  service_timeout_ = std::chrono::milliseconds(static_cast<long long>(timeout_sec * 1000.0));
  occupied_thresh_ = static_cast<float>(occupied);
  free_thresh_ = static_cast<float>(free_thresh);

  maps_dir_ = expand_home(maps_dir);
  std::error_code ec;
  std::filesystem::create_directories(maps_dir_, ec);
  if (ec) {
    RCLCPP_WARN(
      get_logger(), "Could not create maps_dir '%s': %s (will retry on each request)",
      maps_dir_.string().c_str(), ec.message().c_str());
  }

  service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  map_saver_client_ =
    create_client<SaveMap>(saver_service, rclcpp::ServicesQoS(), client_group_);
  serialize_client_ =
    create_client<SerializePoseGraph>(serialize_service, rclcpp::ServicesQoS(), client_group_);
  save_service_ = create_service<SaveExplorationMap>(
    save_service, std::bind(&MapManagerNode::handle_save, this, _1, _2),
    rclcpp::ServicesQoS(), service_group_);

  RCLCPP_INFO(
    get_logger(), "map_manager_node ready (service: %s, maps_dir: %s)", save_service.c_str(),
    maps_dir_.string().c_str());
}

void MapManagerNode::handle_save(
  const std::shared_ptr<SaveExplorationMap::Request> request,
  std::shared_ptr<SaveExplorationMap::Response> response)
{
  try {
    do_save(*request, *response);
  } catch (const std::exception & e) {
    response->success = false;
    response->map_path.clear();
    response->posegraph_path.clear();
    response->message = std::string("unexpected error: ") + e.what();
  } catch (...) {
    response->success = false;
    response->map_path.clear();
    response->posegraph_path.clear();
    response->message = "unexpected unknown error";
  }
  if (response->success) {
    RCLCPP_INFO(get_logger(), "Save map: %s", response->message.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Save map failed: %s", response->message.c_str());
  }
}

void MapManagerNode::do_save(
  const SaveExplorationMap::Request & request, SaveExplorationMap::Response & response)
{
  response.success = false;
  response.map_path.clear();
  response.posegraph_path.clear();

  // 1. Name: sanitize; fall back to a timestamp built from the current wall-clock time.
  std::string name = sanitize_map_name(request.map_name);
  if (name.empty()) {
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    name = make_timestamp_name(local);
  }

  // 2. Output directory.
  std::error_code ec;
  std::filesystem::create_directories(maps_dir_, ec);
  if (ec) {
    response.message =
      "cannot create maps directory '" + maps_dir_.string() + "': " + ec.message();
    return;
  }
  const MapPaths paths = compose_map_paths(maps_dir_, name);

  // 3. Occupancy grid (mandatory).
  std::string error;
  auto save_req = std::make_shared<SaveMap::Request>();
  save_req->map_topic = map_topic_;
  save_req->map_url = paths.base;
  save_req->image_format = image_format_;
  save_req->map_mode = map_mode_;
  save_req->free_thresh = free_thresh_;
  save_req->occupied_thresh = occupied_thresh_;
  const auto save_res =
    call_service<SaveMap>(map_saver_client_, save_req, service_timeout_, "map_saver", error);
  if (!save_res) {
    response.message = "could not save the map: " + error;
    return;
  }
  if (!save_res->result) {
    response.message = "map_saver reported failure saving '" + paths.base + "'";
    return;
  }
  response.map_path = paths.base;

  // 4. Pose graph (optional: needed only to extend the map later).
  auto ser_req = std::make_shared<SerializePoseGraph::Request>();
  ser_req->filename = paths.base;
  error.clear();
  const auto ser_res = call_service<SerializePoseGraph>(
    serialize_client_, ser_req, service_timeout_, "slam_toolbox serialize_map", error);
  if (ser_res && ser_res->result == SerializePoseGraph::Response::RESULT_SUCCESS) {
    response.posegraph_path = paths.base;
    response.success = true;
    response.message = "map saved to '" + paths.base + "' (.yaml/.pgm + .posegraph/.data)";
    return;
  }
  if (ser_res) {
    error = "slam_toolbox serialize_map reported failure";
  }
  response.success = true;
  response.message = "map saved to '" + paths.base + "' (.yaml/.pgm) but the pose graph was not " +
    "saved (" + error + "); this map cannot be extended later";
}

}  // namespace create3_autonomous_explorer
