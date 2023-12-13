#include "rclcpp/rclcpp.hpp"
#include "unitree_a1_legged_msgs/msg/low_cmd.hpp"
#include "unitree_a1_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/quadruped.h"

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos * (1 - rate) + targetPos * rate;
    return p;
}

class FixedStand : public rclcpp::Node
{
public:
    FixedStand() : Node("fixed_stand_low")
    {
        state_ = this->create_subscription<unitree_a1_legged_msgs::msg::LowState>("unitree_lowlevel/state", 1, std::bind(&FixedStand::stateCallback, this, std::placeholders::_1));
        cmd_ = this->create_publisher<unitree_a1_legged_msgs::msg::LowCmd>("unitree_lowlevel/command", 1);
    }
    rclcpp::Publisher<unitree_a1_legged_msgs::msg::LowCmd>::SharedPtr cmd_;

private:
    // init timer
    void stateCallback(unitree_a1_legged_msgs::msg::LowState::SharedPtr msg)
    {
        if (init_)
        {
            lastPos_[0] = msg->motor_state.front_right.hip.q;
            lastPos_[1] = msg->motor_state.front_right.thigh.q;
            lastPos_[2] = msg->motor_state.front_right.calf.q;
            lastPos_[3] = msg->motor_state.front_left.hip.q;
            lastPos_[4] = msg->motor_state.front_left.thigh.q;
            lastPos_[5] = msg->motor_state.front_left.calf.q;
            lastPos_[6] = msg->motor_state.rear_right.hip.q;
            lastPos_[7] = msg->motor_state.rear_right.thigh.q;
            lastPos_[8] = msg->motor_state.rear_right.calf.q;
            lastPos_[9] = msg->motor_state.rear_left.hip.q;
            lastPos_[10] = msg->motor_state.rear_left.thigh.q;
            lastPos_[11] = msg->motor_state.rear_left.calf.q;
            init_ = false;
        }
        this->initParam(cmd_msg_);
        if (motiontime_ < steps_)
        {
            percent_ = static_cast<float>(motiontime_) / steps_;
            for (size_t i = 0; i < targetPos_.size(); i++)
            {
                targetPos_[i] = jointLinearInterpolation(lastPos_[i], standPos_[i], percent_);
            }
            RCLCPP_INFO(this->get_logger(), "percent: %f, targetPos: %f", percent_, targetPos_[0]);

            motiontime_++;
            
            cmd_msg_.motor_cmd.front_right.hip.q = targetPos_[0];
            cmd_msg_.motor_cmd.front_right.thigh.q = targetPos_[1];
            cmd_msg_.motor_cmd.front_right.calf.q = targetPos_[2];
            cmd_msg_.motor_cmd.front_left.hip.q = targetPos_[3];
            cmd_msg_.motor_cmd.front_left.thigh.q = targetPos_[4];
            cmd_msg_.motor_cmd.front_left.calf.q = targetPos_[5];
            cmd_msg_.motor_cmd.rear_right.hip.q = targetPos_[6];
            cmd_msg_.motor_cmd.rear_right.thigh.q = targetPos_[7];
            cmd_msg_.motor_cmd.rear_right.calf.q = targetPos_[8];
            cmd_msg_.motor_cmd.rear_left.hip.q = targetPos_[9];
            cmd_msg_.motor_cmd.rear_left.thigh.q = targetPos_[10];
            cmd_msg_.motor_cmd.rear_left.calf.q = targetPos_[11];
        }
        cmd_->publish(cmd_msg_);

        return;
    }
    void initParam(unitree_a1_legged_msgs::msg::LowCmd &cmd_msg)
    {
        cmd_msg.mode = 0x0A;
        cmd_msg.motor_cmd.front_right.hip.mode = 0x0A;
        cmd_msg.motor_cmd.front_right.hip.kp = 70.0;
        cmd_msg.motor_cmd.front_right.hip.kd = 3.0;
        cmd_msg.motor_cmd.front_left.hip.mode = 0x0A;
        cmd_msg.motor_cmd.front_left.hip.kp = 70.0;
        cmd_msg.motor_cmd.front_left.hip.kd = 3.0;
        cmd_msg.motor_cmd.rear_right.hip.mode = 0x0A;
        cmd_msg.motor_cmd.rear_right.hip.kp = 70.0;
        cmd_msg.motor_cmd.rear_right.hip.kd = 3.0;
        cmd_msg.motor_cmd.rear_left.hip.mode = 0x0A;
        cmd_msg.motor_cmd.rear_left.hip.kp = 70.0;
        cmd_msg.motor_cmd.rear_left.hip.kd = 3.0;

        cmd_msg.motor_cmd.front_right.thigh.mode = 0x0A;
        cmd_msg.motor_cmd.front_right.thigh.kp = 180.0;
        cmd_msg.motor_cmd.front_right.thigh.kd = 8.0;
        cmd_msg.motor_cmd.front_left.thigh.mode = 0x0A;
        cmd_msg.motor_cmd.front_left.thigh.kp = 180.0;
        cmd_msg.motor_cmd.front_left.thigh.kd = 8.0;
        cmd_msg.motor_cmd.rear_right.thigh.mode = 0x0A;
        cmd_msg.motor_cmd.rear_right.thigh.kp = 180.0;
        cmd_msg.motor_cmd.rear_right.thigh.kd = 8.0;
        cmd_msg.motor_cmd.rear_left.thigh.mode = 0x0A;
        cmd_msg.motor_cmd.rear_left.thigh.kp = 180.0;
        cmd_msg.motor_cmd.rear_left.thigh.kd = 8.0;

        cmd_msg.motor_cmd.front_right.calf.mode = 0x0A;
        cmd_msg.motor_cmd.front_right.calf.kp = 300.0;
        cmd_msg.motor_cmd.front_right.calf.kd = 15.0;
        cmd_msg.motor_cmd.front_left.calf.mode = 0x0A;
        cmd_msg.motor_cmd.front_left.calf.kp = 300.0;
        cmd_msg.motor_cmd.front_left.calf.kd = 15.0;
        cmd_msg.motor_cmd.rear_right.calf.mode = 0x0A;
        cmd_msg.motor_cmd.rear_right.calf.kp = 300.0;
        cmd_msg.motor_cmd.rear_right.calf.kd = 15.0;
        cmd_msg.motor_cmd.rear_left.calf.mode = 0x0A;
        cmd_msg.motor_cmd.rear_left.calf.kp = 300.0;
        cmd_msg.motor_cmd.rear_left.calf.kd = 15.0;
    }
    rclcpp::Subscription<unitree_a1_legged_msgs::msg::LowState>::SharedPtr state_;
    std::vector<float> targetPos_ = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                                     0.0, 0.67, -1.3, 0.0, 0.67, -1.3};
    std::vector<float> standPos_ = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                                     0.0, 0.67, -1.3, 0.0, 0.67, -1.3};
    std::vector<float> lastPos_ = std::vector<float>(targetPos_.size(), 0.0);
    unitree_a1_legged_msgs::msg::LowCmd cmd_msg_;
    float steps_ = 2000.0;
    float percent_ = 0.0;
    int motiontime_ = 1;
    bool init_ = true;
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FixedStand>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}