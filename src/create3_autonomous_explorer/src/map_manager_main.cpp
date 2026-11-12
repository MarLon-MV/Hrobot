#include <cstddef>
#include <memory>

#include "create3_autonomous_explorer/map_manager_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<create3_autonomous_explorer::MapManagerNode>();
  // At least 2 threads are required: one blocks in the service handler while another
  // dispatches the clients' responses (see MapManagerNode).
  constexpr std::size_t kThreads = 4;
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), kThreads);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
