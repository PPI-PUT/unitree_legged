#include "rclcpp/rclcpp.hpp"
#include "unitree_a1_legged_msgs/msg/low_cmd.hpp"
#include "unitree_a1_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/a1_const.h"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#
using namespace UNITREE_LEGGED_SDK;

class ExampleTorque : public rclcpp::Node
{
public:
    ExampleTorque() : Node("example_torque_low")
    {
        state_ = this->create_subscription<unitree_a1_legged_msgs::msg::LowState>("unitree_lowlevel/state", 1, std::bind(&ExampleTorque::stateCallback, this, std::placeholders::_1));
        cmd_ = this->create_publisher<unitree_a1_legged_msgs::msg::LowCmd>("unitree_lowlevel/command", 1);
        pub_ = this->create_publisher<std_msgs::msg::Float64>("example_torque", 1);
    }
private:
    int motiontime_ = 0;
    const double amplitude_ = 0.5;
    const double frequency_ = 5.0; // Adjust the frequency as needed
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
    rclcpp::Subscription<unitree_a1_legged_msgs::msg::LowState>::SharedPtr state_;
    rclcpp::Publisher<unitree_a1_legged_msgs::msg::LowCmd>::SharedPtr cmd_;
    void stateCallback(unitree_a1_legged_msgs::msg::LowState::SharedPtr msg)
    {
        double value = amplitude_ * sin(2.0 * M_PI * frequency_ * motiontime_ / 1000.0);
        motiontime_++;
        unitree_a1_legged_msgs::msg::LowCmd cmd;
        cmd.mode = 0x0A; // motor switch to servo (PMSM) mode
        cmd.motor_cmd.front_right.hip.q = a1_Hip_min;
        cmd.motor_cmd.front_right.hip.dq = 0.0;
        cmd.motor_cmd.front_right.hip.tau = 0.0;
        cmd.motor_cmd.front_right.hip.kp = 10.0;
        cmd.motor_cmd.front_right.hip.kd = 1.0;
        cmd.motor_cmd.front_right.calf.q = a1_Calf_min;
        cmd.motor_cmd.front_right.calf.dq = 0.0;
        cmd.motor_cmd.front_right.calf.tau = 0.0;
        cmd.motor_cmd.front_right.calf.kp = 10.0;
        cmd.motor_cmd.front_right.calf.kd = 1.0;
        cmd.motor_cmd.front_right.thigh.q = PosStopF;
        cmd.motor_cmd.front_right.thigh.dq = VelStopF;
        cmd.motor_cmd.front_right.thigh.tau = value;
        if (msg->motor_state.front_right.thigh.q >= a1_Thigh_max - 0.5)
        {
            return;
        }
        if (msg->motor_state.front_right.thigh.q <= a1_Thigh_min + 0.5)
        {
            return;
        }
        cmd_ -> publish(cmd);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExampleTorque>());
    rclcpp::shutdown();
    return 0;
}