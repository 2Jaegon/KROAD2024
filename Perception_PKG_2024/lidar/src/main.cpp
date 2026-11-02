#include "LiDAR_big_static.cpp"
#include "LiDAR_small_static.cpp"
#include "LiDAR_parallel_parking.cpp"
#include "LiDAR_tunnel.cpp"
#include "LiDAR_uturn.cpp"


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto mission1 = std::make_shared<LiDAR_big_static>();
  auto mission2 = std::make_shared<LiDAR_small_static>();
  auto mission3 = std::make_shared<LiDAR_parallel_parking>();
  auto mission4 = std::make_shared<LiDAR_tunnel>();
  auto mission5 = std::make_shared<LiDAR_uturn>();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mission1);
  executor.add_node(mission2);
  executor.add_node(mission3);
  executor.add_node(mission4);
  executor.add_node(mission5);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
