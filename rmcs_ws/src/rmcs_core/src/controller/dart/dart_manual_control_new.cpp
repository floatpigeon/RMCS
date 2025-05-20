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

        register_input("/remote/switch/left", input_switch_left_, false);
        register_input("/remote/switch/right", input_switch_right_, false);
        register_input("/remote/joystick/left", input_joystick_left_, false);
        register_input("/remote/joystick/right", input_joystick_right_, false);

        register_output("/dart/control_command/friction_enable", friction_enable_command_, false);
        register_output("/dart/control_command/filling_start", dart_filling_start_command_, false);
        register_output("/dart/control_command/pitch_control", pitch_control_command_, nan);
        register_output("/dart/yaw_angle/control_velocity", yaw_control_velocity_, nan);
    }

    void update() override {
        update_remote_control_commands();
        *friction_enable_command_    = friction_enable_;
        *dart_filling_start_command_ = filling_enable_;
    }

private:
    void update_remote_control_commands() {
        using namespace rmcs_msgs;
        switch_left_  = *input_switch_left_;
        switch_right_ = *input_switch_right_;

        friction_enable_ = false;
        filling_enable_  = false;

        if (switch_left_ == Switch::UP && switch_right_ == Switch::DOWN) {
            // 左上右下，角度控制
            *yaw_control_velocity_  = 30 * input_joystick_left_->y();
            *pitch_control_command_ = input_joystick_right_->x();
        } else {
            *yaw_control_velocity_  = nan;
            *pitch_control_command_ = 0;
        }

        if (switch_left_ != Switch::DOWN && switch_right_ == Switch::UP) {
            friction_enable_ = true;
        }

        if (switch_left_ == Switch::UP && friction_enable_) {
            filling_enable_ = true;
        }
    }

    rclcpp::Logger logger_;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    bool friction_enable_ = false;
    bool filling_enable_  = false;

    rmcs_msgs::Switch switch_left_  = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch switch_right_ = rmcs_msgs::Switch::UNKNOWN;

    InputInterface<rmcs_msgs::Switch> input_switch_left_;
    InputInterface<rmcs_msgs::Switch> input_switch_right_;
    InputInterface<Eigen::Vector2d> input_joystick_left_;
    InputInterface<Eigen::Vector2d> input_joystick_right_;

    OutputInterface<double> yaw_control_velocity_;
    OutputInterface<double> pitch_control_command_;
    OutputInterface<bool> friction_enable_command_;
    OutputInterface<bool> dart_filling_start_command_;
};
} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::DartManualControl, rmcs_executor::Component)
