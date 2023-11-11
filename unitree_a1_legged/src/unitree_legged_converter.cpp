// Copyright 2023 Mobile Robots Laboratory at Poznan University of Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "unitree_a1_legged/unitree_legged_converter.hpp"

using namespace UNITREE_LEGGED_SDK;

namespace unitree_a1_legged
{
    sensor_msgs::msg::Imu Converter::stateToMsg(const IMU &state)
    {
        sensor_msgs::msg::Imu msg;
        msg.header.frame_id = "imu_link";
        msg.orientation.x = state.quaternion[0];
        msg.orientation.y = state.quaternion[1];
        msg.orientation.z = state.quaternion[2];
        msg.orientation.w = state.quaternion[3];
        msg.angular_velocity.x = state.gyroscope[0];
        msg.angular_velocity.y = state.gyroscope[0];
        msg.angular_velocity.z = state.gyroscope[0];
        msg.linear_acceleration.x = state.accelerometer[0];
        msg.linear_acceleration.y = state.accelerometer[1];
        msg.linear_acceleration.z = state.accelerometer[2];
        return msg;
    }
    unitree_a1_legged_msgs::msg::LowState Converter::stateToMsg(const LowState &state)
    {
        unitree_a1_legged_msgs::msg::LowState msg;
        msg.a1.level_flag = state.levelFlag;
        msg.a1.comm_version = state.commVersion;
        msg.a1.robot_id = state.robotID;
        msg.a1.sn = state.SN;
        msg.a1.band_width = state.bandWidth;
        msg.imu = Converter::stateToMsg(state.imu);
        msg.motor_state = Converter::stateToMsg(state.motorState);
        msg.foot_force = Converter::stateToMsg(state.footForce);
        msg.foot_force_est = Converter::stateToMsg(state.footForceEst);
        msg.tick = state.tick;
        for (int i = 0; i < 40; i++)
        {
            msg.wireless_remote[i] = state.wirelessRemote[i];
        }
        msg.reserve = state.reserve;
        msg.crc = state.crc;
        return msg;
    }
    unitree_a1_legged_msgs::msg::QuadrupedState Converter::stateToMsg(const MotorState (&state)[20])
    {
        unitree_a1_legged_msgs::msg::QuadrupedState msg;
        msg.front_left.hip = Converter::stateToMsg(state[FR_0]);
        msg.front_left.thigh = Converter::stateToMsg(state[FR_1]);
        msg.front_left.calf = Converter::stateToMsg(state[FR_2]);
        msg.front_left.hip = Converter::stateToMsg(state[FL_0]);
        msg.front_left.thigh = Converter::stateToMsg(state[FL_1]);
        msg.front_left.calf = Converter::stateToMsg(state[FL_2]);
        msg.rear_right.hip = Converter::stateToMsg(state[RR_0]);
        msg.rear_right.thigh = Converter::stateToMsg(state[RR_1]);
        msg.rear_right.calf = Converter::stateToMsg(state[RR_2]);
        msg.rear_left.hip = Converter::stateToMsg(state[RL_0]);
        msg.rear_left.thigh = Converter::stateToMsg(state[RL_1]);
        msg.rear_left.calf = Converter::stateToMsg(state[RL_2]);
        return msg;
    }
    unitree_a1_legged_msgs::msg::MotorState Converter::stateToMsg(const MotorState &state)
    {
        unitree_a1_legged_msgs::msg::MotorState msg;
        msg.mode = state.mode;
        msg.q = state.q;
        msg.dq = state.dq;
        msg.ddq = state.ddq;
        msg.tau_est = state.tauEst;
        msg.q_raw = state.q_raw;
        msg.dq_raw = state.dq_raw;
        msg.ddq_raw = state.ddq_raw;
        msg.temperature = state.temperature;
        msg.reserve[0] = state.reserve[0];
        msg.reserve[1] = state.reserve[1];
        return msg;
    }
    unitree_a1_legged_msgs::msg::FootForceState Converter::stateToMsg(const int16_t state[4])
    {
        unitree_a1_legged_msgs::msg::FootForceState msg;
        msg.front_left = state[FR_];
        msg.front_right = state[FL_];
        msg.rear_right = state[RR_];
        msg.rear_left = state[RL_];
        return msg;
    }
    void Converter::msgToCmd(const unitree_a1_legged_msgs::msg::LowCmd::SharedPtr msg, LowCmd &cmd)
    {
        // cmd.levelFlag = msg->a1.level_flag;
        // cmd.commVersion = msg->a1.comm_version;
        // cmd.robotID = msg->a1.robot_id;
        // cmd.SN = msg->a1.sn;
        // cmd.bandWidth = msg->a1.band_width;
        for (const auto &[key, value] : Converter::jointIndexMap)
        {
            cmd.motorCmd[value].mode = msg->mode;
        }
        Converter::msgToCmd(msg->motor_cmd, cmd);
        // for (int i = 0; i < 4; i++)
        // {
        //     cmd.led[i].r = msg->led[i].r;
        //     cmd.led[i].g = msg->led[i].g;
        //     cmd.led[i].b = msg->led[i].b;
        // }
        // cmd.reserve = msg->reserve;
        // cmd.crc = msg->crc;
    }
    void Converter::msgToCmd(const unitree_a1_legged_msgs::msg::JointCommand::SharedPtr msg, LowCmd &cmd)
    {
        // check size of msg
        if (msg->joint.name.size() != Converter::getJointCount() ||
            msg->joint.position.size() != Converter::getJointCount() ||
            msg->joint.velocity.size() != Converter::getJointCount() ||
            msg->joint.effort.size() != Converter::getJointCount())
        {
            return;
        }
        for (const auto &[key, value] : Converter::jointIndexMap)
        {
            cmd.motorCmd[value].mode = msg->mode;
            cmd.motorCmd[value].Kp = msg->kp;
            cmd.motorCmd[value].Kd = msg->kd;
            cmd.motorCmd[value].q = msg->joint.position[value];
            cmd.motorCmd[value].dq = msg->joint.velocity[value];
            cmd.motorCmd[value].tau = msg->joint.effort[value];
        }
    }
    void Converter::msgToCmd(const unitree_a1_legged_msgs::msg::QuadrupedCmd msg, LowCmd &cmd)
    {
        cmd.motorCmd[FR_0] = Converter::msgToCmd(msg.front_right.hip);
        cmd.motorCmd[FR_1] = Converter::msgToCmd(msg.front_right.thigh);
        cmd.motorCmd[FR_2] = Converter::msgToCmd(msg.front_right.calf);
        cmd.motorCmd[FL_0] = Converter::msgToCmd(msg.front_left.hip);
        cmd.motorCmd[FL_1] = Converter::msgToCmd(msg.front_left.thigh);
        cmd.motorCmd[FL_2] = Converter::msgToCmd(msg.front_left.calf);
        cmd.motorCmd[RR_0] = Converter::msgToCmd(msg.rear_right.hip);
        cmd.motorCmd[RR_1] = Converter::msgToCmd(msg.rear_right.thigh);
        cmd.motorCmd[RR_2] = Converter::msgToCmd(msg.rear_right.calf);
        cmd.motorCmd[RL_0] = Converter::msgToCmd(msg.rear_left.hip);
        cmd.motorCmd[RL_1] = Converter::msgToCmd(msg.rear_left.thigh);
        cmd.motorCmd[RL_2] = Converter::msgToCmd(msg.rear_left.calf);
    }
    MotorCmd Converter::msgToCmd(const unitree_a1_legged_msgs::msg::MotorCmd &msg)
    {
        UNITREE_LEGGED_SDK::MotorCmd cmd;
        cmd.mode = msg.mode;
        cmd.q = msg.q;
        cmd.dq = msg.dq;
        cmd.tau = msg.tau;
        cmd.Kp = msg.kp;
        cmd.Kd = msg.kd;
        cmd.reserve[0] = msg.reserve[0];
        cmd.reserve[1] = msg.reserve[1];
        cmd.reserve[2] = msg.reserve[2];
        return cmd;
    }
    const std::unordered_map<std::string, int> Converter::jointIndexMap = {
        {"FR_hip_joint", FR_0},
        {"FR_thigh_joint", FR_1},
        {"FR_calf_joint", FR_2},
        {"FL_hip_joint", FL_0},
        {"FL_thigh_joint", FL_1},
        {"FL_calf_joint", FL_2},
        {"RR_hip_joint", RR_0},
        {"RR_thigh_joint", RR_1},
        {"RR_calf_joint", RR_2},
        {"RL_hip_joint", RL_0},
        {"RL_thigh_joint", RL_1},
        {"RL_calf_joint", RL_2}};

    std::vector<std::string> Converter::getJointNames()
    {
        std::vector<std::string> names;
        for (const auto &pair : Converter::jointIndexMap)
        {
            names.push_back(pair.first);
        }
        return names;
    }
    size_t Converter::getJointCount()
    {
        return Converter::jointIndexMap.size();
    }
    sensor_msgs::msg::JointState Converter::getJointStateMsg(const LowState &state)
    {
        sensor_msgs::msg::JointState msg;
        msg.name = Converter::getJointNames();
        msg.position.resize(Converter::getJointCount());
        msg.velocity.resize(Converter::getJointCount());
        msg.effort.resize(Converter::getJointCount());
        for (const auto &[key, value] : Converter::jointIndexMap)
        {
            msg.position[value] = state.motorState[value].q;
            msg.velocity[value] = state.motorState[value].dq;
            msg.effort[value] = state.motorState[value].tauEst;
        }
        return msg;
    }
    unitree_a1_legged_msgs::msg::JointCommand Converter::getJointCommandMsg()
    {
        unitree_a1_legged_msgs::msg::JointCommand msg;
        sensor_msgs::msg::JointState joint_msg;
        joint_msg.name = Converter::getJointNames();
        joint_msg.position.resize(Converter::getJointCount());
        joint_msg.velocity.resize(Converter::getJointCount());
        joint_msg.effort.resize(Converter::getJointCount());
        msg.joint = joint_msg;
        return msg;
    }

} // namespace unitree_a1_legged