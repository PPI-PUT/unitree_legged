#include <unordered_map>
#include <vector>
#include <cstring>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "unitree_a1_driver/unitree_a1_driver.hpp"
#include "unitree_a1_legged_msgs/msg/low_cmd.hpp"
#include "unitree_a1_legged_msgs/msg/low_state.hpp"
#include "unitree_a1_legged_msgs/msg/motor_cmd.hpp"
#include "unitree_a1_legged_msgs/msg/motor_state.hpp"
#include "unitree_a1_legged_msgs/msg/quadruped_state.hpp"
#include "unitree_a1_legged_msgs/msg/quadruped_cmd.hpp"
#include "unitree_a1_legged_msgs/msg/foot_force_state.hpp"
#include "unitree_a1_legged_msgs/msg/joint_command.hpp"

namespace unitree_a1_driver
{

/**
     * @brief Class for converting between UNITREE_LEGGED_SDK and ROS2 messages.
     */
class UNITREE_A1_LEGGED_PUBLIC Converter
{
public:
  /**
       * @brief Converts the UNITREE_LEGGED_SDK::IMU to a ROS2 message.
       */
  static sensor_msgs::msg::Imu stateToMsg(const IMU & state);
  /**
       * @brief Converts the UNITREE_LEGGED_SDK::LowState to a ROS2 message.
       */
  static unitree_a1_legged_msgs::msg::LowState stateToMsg(const LowState & state);
  /**
       * @brief Converts the UNITREE_LEGGED_SDK::MotorState to a QuadrupedState ROS2 message.
       */
  static unitree_a1_legged_msgs::msg::QuadrupedState stateToMsg(const MotorState (& state)[20]);
  /**
       * @brief Converts the UNITREE_LEGGED_SDK::MotorState to a ROS2 message.
       */
  static unitree_a1_legged_msgs::msg::MotorState stateToMsg(const MotorState & state);
  /**
       * @brief Converts the UNITREE_LEGGED_SDK::FootForceState to a ROS2 message.
       */
  static unitree_a1_legged_msgs::msg::FootForceState stateToMsg(const int16_t state[4]);
  /**
       * @brief Converts from LowCmd ROS2 message to UNITREE_LEGGED_SDK::LowCmd.
       */
  static void msgToCmd(const unitree_a1_legged_msgs::msg::LowCmd::SharedPtr msg, LowCmd & cmd);
  static void msgToCmd(
    const unitree_a1_legged_msgs::msg::JointCommand::SharedPtr msg,
    LowCmd & cmd);
  /**
       * @brief Converts from QuadrupedCmd ROS2 message to UNITREE_LEGGED_SDK::LowCmd.
       */
  static void msgToCmd(const unitree_a1_legged_msgs::msg::QuadrupedCmd msg, LowCmd & cmd);
  /**
       * @brief Converts from MotorCmd ROS2 message to UNITREE_LEGGED_SDK::MotorCmd.
       */
  static MotorCmd msgToCmd(const unitree_a1_legged_msgs::msg::MotorCmd & msg);
  /**
       * @brief Map for joint names.
       */
  static const std::unordered_map<std::string, int> jointIndexMap;
  /**
       * @brief Returns a vector of joint names.
       */
  static std::vector<std::string> getJointNames();
  /**
       * @brief Returns the number of joints.
       */
  static size_t getJointCount();
  /**
       * @brief Get the Joint State Msg object
       */
  static sensor_msgs::msg::JointState getJointStateMsg(const LowState & state);
  /**
       * @brief Get the Joint Command Msg object
       */
  static unitree_a1_legged_msgs::msg::JointCommand getJointCommandMsg();
  static sensor_msgs::msg::Joy stateToMsg(const uint8_t (& state)[40]);
  static void getWrenchMsg(
    const int16_t state[4],
    geometry_msgs::msg::WrenchStamped & front_right,
    geometry_msgs::msg::WrenchStamped & front_left,
    geometry_msgs::msg::WrenchStamped & rear_right,
    geometry_msgs::msg::WrenchStamped & rear_left);
};
} // namespace unitree_a1_driver