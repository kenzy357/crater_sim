//  gamepad_node.cpp
//
//  Subscribes : /joy          (sensor_msgs/msg/Joy)
//  Publishes  : /cmd_vel      (geometry_msgs/msg/Twist)
//
//  Mapping:
//    Twist.linear.x  = forward velocity  vx  [m/s]
//    Twist.linear.y  = strafe  velocity  vy  [m/s]
//    Twist.angular.z = yaw rate          omega [rad/s]
//
//  Topic names, axis indices, limits, deadzone and inversion flags
//  are all ROS2 parameters — edit config/rover.yaml to change them.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <string>

class GamepadNode : public rclcpp::Node
{
public:
    GamepadNode() : Node("gamepad_node")
    {
        // ── Declare parameters (values come from rover.yaml) ──────────────────
        declare_parameter("joy_topic",     "/joy");
        declare_parameter("cmd_vel_topic", "/cmd_vel");

        declare_parameter("axis_lx",   0);
        declare_parameter("axis_ly",   1);
        declare_parameter("axis_rx",   2);

        declare_parameter("deadzone",  0.08);
        declare_parameter("vmax",      2.0);
        declare_parameter("wmax",      0.5);

        declare_parameter("invert_ly", true);
        declare_parameter("invert_rx", true);

        // ── Read parameters ───────────────────────────────────────────────────
        joy_topic_     = get_parameter("joy_topic").as_string();
        cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();

        axis_lx_   = get_parameter("axis_lx").as_int();
        axis_ly_   = get_parameter("axis_ly").as_int();
        axis_rx_   = get_parameter("axis_rx").as_int();

        deadzone_  = get_parameter("deadzone").as_double();
        vmax_      = get_parameter("vmax").as_double();
        wmax_      = get_parameter("wmax").as_double();

        invert_ly_ = get_parameter("invert_ly").as_bool();
        invert_rx_ = get_parameter("invert_rx").as_bool();

        // ── Subscriber & publisher ────────────────────────────────────────────
        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            joy_topic_, 10,
            std::bind(&GamepadNode::joy_callback, this, std::placeholders::_1));

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, 10);

        RCLCPP_INFO(get_logger(), "gamepad_node started");
        RCLCPP_INFO(get_logger(), "  joy    → %s", joy_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  cmd_vel← %s", cmd_vel_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  axes: LX=%d  LY=%d  RX=%d",
                    axis_lx_, axis_ly_, axis_rx_);
        RCLCPP_INFO(get_logger(), "  vmax=%.2f  wmax=%.2f  deadzone=%.3f",
                    vmax_, wmax_, deadzone_);
    }

private:
    // ── Helpers ───────────────────────────────────────────────────────────────

    // Applies deadzone and returns a value in [-1, 1]
    double apply_deadzone(double raw) const {
        if (std::abs(raw) < deadzone_) return 0.0;
        // Rescale so output starts from 0 right at the deadzone edge
        double sign = (raw > 0.0) ? 1.0 : -1.0;
        return sign * (std::abs(raw) - deadzone_) / (1.0 - deadzone_);
    }

    // Safe axis read — returns 0 if index is out of range
    double get_axis(const sensor_msgs::msg::Joy::SharedPtr& msg, int idx) const {
        if (idx < 0 || idx >= static_cast<int>(msg->axes.size())) {
            RCLCPP_WARN_ONCE(get_logger(),
                "Axis index %d out of range (controller has %zu axes)",
                idx, msg->axes.size());
            return 0.0;
        }
        return static_cast<double>(msg->axes[idx]);
    }

    // ── Joy callback ──────────────────────────────────────────────────────────
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        double lx = apply_deadzone(get_axis(msg, axis_lx_));
        double ly = apply_deadzone(get_axis(msg, axis_ly_));
        double rx = apply_deadzone(get_axis(msg, axis_rx_));

        if (invert_ly_) ly = -ly;
        if (invert_rx_) rx = -rx;

        geometry_msgs::msg::Twist twist;
        twist.linear.x  = vmax_ * ly;   // forward
        twist.linear.y  = vmax_ * lx;   // strafe
        twist.angular.z = wmax_ * rx;   // yaw

        cmd_pub_->publish(twist);

        RCLCPP_DEBUG(get_logger(),
            "vx=%.3f  vy=%.3f  w=%.3f",
            twist.linear.x, twist.linear.y, twist.angular.z);
    }

    // ── Members ───────────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr    joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   cmd_pub_;

    std::string joy_topic_;
    std::string cmd_vel_topic_;

    int    axis_lx_, axis_ly_, axis_rx_;
    double deadzone_, vmax_, wmax_;
    bool   invert_ly_, invert_rx_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GamepadNode>());
    rclcpp::shutdown();
    return 0;
}
