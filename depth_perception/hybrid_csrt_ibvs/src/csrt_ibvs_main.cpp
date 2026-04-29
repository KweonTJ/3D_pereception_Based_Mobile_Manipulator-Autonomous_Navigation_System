#include "hybrid_csrt_ibvs/csrt_ibvs_node.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hybrid_csrt_ibvs::CsrtIbvsNode>());
  rclcpp::shutdown();
  return 0;
}
