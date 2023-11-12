#include "unitree_a1_joystick/unitree_a1_joystick_node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<unitree_a1_legged::UnitreeJoystickNode>(options));
  rclcpp::shutdown();

  return 0;
}