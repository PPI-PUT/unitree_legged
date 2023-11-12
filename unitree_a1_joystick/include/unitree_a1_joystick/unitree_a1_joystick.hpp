#include <sensor_msgs/msg/joy.hpp>
#include <algorithm>
namespace unitree_a1_legged
{
    class UnitreeJoystick
    {
    public:
        UnitreeJoystick(const sensor_msgs::msg::Joy &j) : j_(j) {}
        float linear_x() { return lx(); }
        float linear_y() { return ly(); }
        float angular_z() { return rx(); }

    private:
        const sensor_msgs::msg::Joy j_;

        float lx() { return j_.axes.at(0); }
        float rx() { return j_.axes.at(1); }
        float ry() { return j_.axes.at(2); }
        float ly() { return j_.axes.at(3); }
        float L2_axes() { return j_.axes.at(4); }

        bool R1() { return j_.buttons.at(0); }
        bool L1() { return j_.buttons.at(1); }
        bool start() { return j_.buttons.at(2); }
        bool select() { return j_.buttons.at(3); }
        bool R2() { return j_.buttons.at(4); }
        bool L2() { return j_.buttons.at(5); }
        bool F1() { return j_.buttons.at(6); }
        bool F2() { return j_.buttons.at(7); }
        bool A() { return j_.buttons.at(8); }
        bool B() { return j_.buttons.at(9); }
        bool X() { return j_.buttons.at(10); }
        bool Y() { return j_.buttons.at(11); }
        bool up() { return j_.buttons.at(12); }
        bool right() { return j_.buttons.at(13); }
        bool down() { return j_.buttons.at(14); }
        bool left() { return j_.buttons.at(15); }
    };
} // namespace unitree_a1_legged