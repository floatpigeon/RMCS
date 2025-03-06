#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <switch.hpp>

namespace rmcs_core::controller::dartlauncher {

class DartManualControl
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartManualControl()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        limit_velocity = get_parameter("limit_velocity").as_double();

        register_input("/remote/switch/left", input_switch_left_, false);
        register_input("/remote/switch/right", input_switch_right_, false);
        register_input("/remote/joystick/left", input_joystick_left_, false);
        register_input("/remote/joystick/right", input_joystick_right_, false);

        register_output("/dart/master_control/friction_command", output_friction_enable_, false);
        register_output("/dart/master_control/friction_control_velocity", output_dart_launch_velocity_, nan);
        register_output("/dart/master_control/angle_command", output_angle_control_enable_, false);
        register_output("/dart/master_control/angle_control_vector", output_angle_control_, Eigen::Vector2d::Zero());
        register_output("/dart/master_control/filling_command", output_dart_filling_enable_, false);
    }

    void update() override {
        update_remote_control_commands();
        *output_friction_enable_      = friction_enable_;
        *output_angle_control_enable_ = angle_control_enable_;

        update_output_control_values();
    }

private:
    void update_remote_control_commands() {
        using namespace rmcs_msgs;
        switch_left_  = *input_switch_left_;
        switch_right_ = *input_switch_right_;

        angle_control_enable_ = false;
        friction_enable_      = false;
        filling_enable_       = false;

        if (switch_left_ == Switch::UP && switch_right_ == Switch::UP) {
            angle_control_enable_ = true;
        }

        if (switch_right_ == Switch::MIDDLE || switch_right_ == Switch::UP) {
            friction_enable_ = true;
        }

        if (switch_left_ == Switch::UP || switch_right_ == Switch::MIDDLE) {
            filling_enable_ = true;
        }
    }

    void update_output_control_values() {
        double pitch_control_input_ = 30.0 * input_joystick_right_->x();
        double yaw_control_input_   = 30.0 * input_joystick_right_->y();

        output_angle_control_->x() = std::max(-limit_velocity, std::min(limit_velocity, yaw_control_input_));
        output_angle_control_->y() = std::max(-limit_velocity, std::min(limit_velocity, pitch_control_input_));

        *output_dart_launch_velocity_ = nan;
    }

    rclcpp::Logger logger_;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    double limit_velocity;

    bool angle_control_enable_ = false;
    bool friction_enable_      = false;
    bool filling_enable_       = false;

    rmcs_msgs::Switch switch_left_  = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch switch_right_ = rmcs_msgs::Switch::UNKNOWN;

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
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::DartManualControl, rmcs_executor::Component)

/*
双下停止
右中开摩擦轮
右中左中填装复位，此时左拨上再回中连续打两发镖
双上手动控制角度启用，操作右摇杆控制
*/