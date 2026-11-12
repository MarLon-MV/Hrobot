#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace create3_autonomous_explorer
{

namespace
{

/// Smallest accepted publisher queue depth.
constexpr int kMinOutputDepth = 1;

}  // namespace

/// Republishes the Create 3 odometry with a QoS that Nav2 can subscribe to.
///
/// The Create 3 (`robot_state`) publishes `odom` as BEST_EFFORT, depth 1, while Nav2's
/// controller_server and bt_navigator subscribe RELIABLE: a RELIABLE subscriber never matches a
/// BEST_EFFORT publisher, so Nav2 would get no odometry. This node subscribes best effort
/// (`rclcpp::SensorDataQoS`) and republishes the very same message as RELIABLE.
/// Topic names are relative, so they resolve under the node namespace (e.g. /r2/odom ->
/// /r2/odom_reliable). The callback is the only user of `publisher_`, which is created in the
/// constructor and never modified afterwards (no shared mutable state, no locking needed).
class OdomRelayNode : public rclcpp::Node
{
public:
  using Odometry = nav_msgs::msg::Odometry;

  explicit OdomRelayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("odom_relay_node", options)
  {
    const std::string input_topic = declare_parameter<std::string>("input_topic", "odom");
    const std::string output_topic =
      declare_parameter<std::string>("output_topic", "odom_reliable");
    int output_depth = declare_parameter<int>("output_depth", 10);
    if (output_depth < kMinOutputDepth) {
      RCLCPP_WARN(get_logger(), "output_depth must be >= 1; using 1");
      output_depth = kMinOutputDepth;
    }

    publisher_ = create_publisher<Odometry>(
      output_topic, rclcpp::QoS(static_cast<std::size_t>(output_depth)).reliable());
    subscription_ = create_subscription<Odometry>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&OdomRelayNode::on_odom, this, std::placeholders::_1));
  }

private:
  /// Takes ownership of the received message and hands it to the publisher (no copy).
  void on_odom(Odometry::UniquePtr msg) const { publisher_->publish(std::move(msg)); }

  rclcpp::Publisher<Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<Odometry>::SharedPtr subscription_;
};

}  // namespace create3_autonomous_explorer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<create3_autonomous_explorer::OdomRelayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
