//  kinematics_node.cpp
//
//  Subscribes : /cmd_vel        (geometry_msgs/msg/Twist)
//  Publishes  : /wheel_commands (std_msgs/msg/Float64MultiArray)
//
//  Float64MultiArray layout (8 values, index order: FL=0 FR=1 RL=2 RR=3):
//    [0..3] delta  — steering angle per wheel [rad]
//    [4..7] omega  — spin speed    per wheel [rad/s]  (negative = reverse)
//       index:  [0]    [1]    [2]    [3]    [4]    [5]    [6]    [7]
//              delta  delta  delta  delta  omega  omega  omega  omega
//                FL     FR     RL     RR     FL     FR     RL     RR    
//
//  Wheel order:
//    FL = front-left   FR = front-right
//    RL = rear-left    RR = rear-right
//
//  Rover geometry (L, W, R) and all other parameters come from rover.yaml.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <array>
#include <cmath>
#include <string>

static constexpr double PI = 3.141592653589793238462643383279502884;

class KinematicsNode : public rclcpp::Node
{
public:
    KinematicsNode() : Node("kinematics_node")
    {
        // ── Declare parameters ────────────────────────────────────────────────
        declare_parameter("cmd_vel_topic",   "/cmd_vel");
        declare_parameter("wheel_cmd_topic", "/wheel_commands");

        declare_parameter("L",               1.5);
        declare_parameter("W",               1.0);
        declare_parameter("R",               0.15);
        declare_parameter("speed_threshold", 0.05);
        declare_parameter("publish_rate",    100.0);

        // ── Read parameters ───────────────────────────────────────────────────
        cmd_vel_topic_   = get_parameter("cmd_vel_topic").as_string();
        wheel_cmd_topic_ = get_parameter("wheel_cmd_topic").as_string();

        L_               = get_parameter("L").as_double();
        W_               = get_parameter("W").as_double();
        R_               = get_parameter("R").as_double();
        speed_threshold_ = get_parameter("speed_threshold").as_double();

        // Initialise previous delta to 0 (wheels pointing forward)
        prev_delta_.fill(0.0); //##############################################################################################

        // ── Subscriber & publisher ────────────────────────────────────────────
        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, 10,
            std::bind(&KinematicsNode::cmd_callback, this, std::placeholders::_1));

        wheel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            wheel_cmd_topic_, 10);

        RCLCPP_INFO(get_logger(), "kinematics_node started");
        RCLCPP_INFO(get_logger(), "  cmd_vel     → %s", cmd_vel_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  wheel_cmds  ← %s", wheel_cmd_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  L=%.3f m  W=%.3f m  R=%.3f m",
                    L_, W_, R_);
    }

private:
    // ── 4WS Inverse Kinematics ────────────────────────────────────────────────
    //
    //  Wheel positions in body frame (X=right, Y=forward):

    //    FL = (L/2,  W/2)    
    //    FR = (L/2,  -W/2)
    //    RL = (-L/2, W/2)    
    //    RR = (-L/2, -W/2)
    //
    //  Velocity at each wheel contact point:
    //    vix = vx - omega * y_wheel
    //    viy = vy + omega * x_wheel
    //
    //  Steering angle:   delta = atan2(viy, vix)
    //  Spin speed:       omega_wheel = |v| / R
    //
    //  If delta < 0: flip spin direction and add PI instead of steering 180°

    struct WheelCmd {
        double delta;   // steering angle [rad]
        double omega;   // spin speed     [rad/s]
    };

    std::array<WheelCmd, 4> compute_kinematics(double vx, double vy, double omega)
    {
        // Wheel positions [x, y] in body frame
        const double wx[4] = { L_/2.0,  L_/2.0, -L_/2.0,  -L_/2.0 };
        const double wy[4] = {  W_/2.0,  -W_/2.0, W_/2.0, -W_/2.0 };

        std::array<WheelCmd, 4> out{};

        for (int i = 0; i < 4; ++i) {
            const double vix = vx - omega * wy[i];
            const double viy = vy + omega * wx[i];

            double speed = std::sqrt(vix*vix + viy*viy);
            double ww    = speed / R_;
            double delta = prev_delta_[i];   // hold angle when nearly stopped

            if (speed > speed_threshold_) {
                delta = std::atan2(viy, vix);  // in (-pi,pi]

                // Flip spin direction instead of rotating steering 180°
                if (std::abs(delta) > PI/2.0) {
                    ww    = -ww;

                    if (delta>0){
                        delta -= PI; // in [-pi/2, pi/2]
                    } else {
                        delta += PI; // in [-pi/2, pi/2]
                    }
                    
                }
            } else {
                ww = 0.0;
            }

            out[i].delta = delta;
            out[i].omega = ww;
        }

        return out;
    }

    // ── cmd_vel callback ──────────────────────────────────────────────────────
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const double vx    = msg->linear.x;
        const double vy    = msg->linear.y;
        const double omega = msg->angular.z;

        auto wheels = compute_kinematics(vx, vy, omega);

        // Update persistent steering angles for next iteration
        for (int i = 0; i < 4; ++i)
            prev_delta_[i] = wheels[i].delta;

        // ── Pack into Float64MultiArray ───────────────────────────────────────
        //   [0..3] = delta (FL FR RL RR)
        //   [4..7] = omega (FL FR RL RR)
        std_msgs::msg::Float64MultiArray wheel_msg;
        wheel_msg.data.resize(8);

        for (int i = 0; i < 4; ++i) {
            wheel_msg.data[i]     = wheels[i].delta;
            wheel_msg.data[i + 4] = wheels[i].omega;
        }

        // Label the layout so subscribers know what each index means
        std_msgs::msg::MultiArrayDimension dim;
        dim.label  = "delta_FL delta_FR delta_RL delta_RR omega_FL omega_FR omega_RL omega_RR";
        dim.size   = 8;
        dim.stride = 8;
        wheel_msg.layout.dim.push_back(dim);

        wheel_pub_->publish(wheel_msg);

        // Debug log — only print when rover is moving
        if (vx != 0.0 || vy != 0.0 || omega != 0.0) {
            const char* labels[] = { "FL", "FR", "RL", "RR" };
            for (int i = 0; i < 4; ++i) {
                RCLCPP_DEBUG(get_logger(),
                    "%s  delta=%.2f deg  omega=%.3f rad/s",
                    labels[i],
                    wheels[i].delta * 180.0 / PI,
                    wheels[i].omega);
            }
        }
    }

    // ── Members ───────────────────────────────────────────────────────────────
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr         cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr      wheel_pub_;

    std::string cmd_vel_topic_;
    std::string wheel_cmd_topic_;

    double L_, W_, R_;
    double speed_threshold_;

    std::array<double, 4> prev_delta_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
