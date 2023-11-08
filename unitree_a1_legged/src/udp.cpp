#include "unitree_a1_legged/unitree_legged_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<unitree_a1_legged::UnitreeLeggedNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}