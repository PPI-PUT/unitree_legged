// Copyright 2023 Maciej Krupka
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "unitree_a1_legged/unitree_a1_legged.hpp"

#include <iostream>

namespace unitree_a1_legged
{

    UnitreeLegged::UnitreeLegged()
        : low_udp_(LOWLEVEL), safe_(LeggedType::A1)
    {
        low_udp_.InitCmdData(low_cmd_);
    }
    UDP UnitreeLegged::getLowUdp()
    {
        return low_udp_;
    }
    LowCmd UnitreeLegged::getLowCmd()
    {
        return low_cmd_;
    }
    LowState UnitreeLegged::getLowState()
    {
        return low_state_;
    }
    void UnitreeLegged::setMotorMode(uint8_t mode)
    {
        for (size_t i = 0; i < sizeof(low_cmd_.motorCmd) / sizeof(low_cmd_.motorCmd[0]); i++)
        {
            low_cmd_.motorCmd[i].mode = mode;
        }
        this->sendLowCmd(low_cmd_);
    }
    void UnitreeLegged::sendLowCmd(LowCmd &cmd)
    {
        low_udp_.SetSend(cmd);
        low_udp_.Send();
    }
    void UnitreeLegged::sendProtectLowCmd(LowCmd &cmd, const int input_factor)
    {
        safe_.PowerProtect(cmd, low_state_, input_factor);
        low_udp_.SetSend(cmd);
        low_udp_.Send();
    }
    void UnitreeLegged::recvLowState()
    {
        low_udp_.Recv();
        low_udp_.GetRecv(low_state_);
    }

}  // namespace unitree_a1_legged