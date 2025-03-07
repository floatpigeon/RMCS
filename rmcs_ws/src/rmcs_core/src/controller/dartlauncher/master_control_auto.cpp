#include <eigen3/Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <switch.hpp>

namespace rmcs_core::controller::dartlauncher {

class DartAutoGuide
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartAutoGuide()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        limit_velocity = get_parameter("limit_velocity").as_double();
    }

    void update() override {}

private:
    rclcpp::Logger logger_;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    double limit_velocity;

    bool angle_control_enable_ = false;
    bool friction_enable_      = false;
    bool filling_enable_       = false;

    rmcs_msgs::Switch switch_left_  = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch switch_right_ = rmcs_msgs::Switch::UNKNOWN;

    InputInterface<cv::Point> input_target_position_;
    InputInterface<rmcs_msgs::Switch> input_switch_left_;
    InputInterface<rmcs_msgs::Switch> input_switch_right_;
    InputInterface<Eigen::Vector2d> input_joystick_left_;
    InputInterface<Eigen::Vector2d> input_joystick_right_;

    OutputInterface<bool> output_angle_control_enable_;
    OutputInterface<Eigen::Vector2d> output_angle_control_;
    OutputInterface<bool> output_friction_enable_;
    OutputInterface<double> output_dart_launch_velocity_;
    OutputInterface<bool> output_dart_filling_enable_;
};

} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::DartAutoGuide, rmcs_executor::Component)

/*
镖架制导的主控模块，包含目标选择、自主火控
发送误差值给AngleControl，协调控制角度控制和飞镖填装
未来需要与射表接洽
*/