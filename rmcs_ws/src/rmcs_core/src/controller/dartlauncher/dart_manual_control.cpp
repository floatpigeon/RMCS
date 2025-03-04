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

        register_input("/dart/conveyor/velocity", input_conveyor_velocity_);
        register_output("/dart/conveyor/control_velocity", output_conveyor_control_velocity_, nan);

        register_output("/dart/master_control/friction_command", output_friction_enable_, false);
        register_output("/dart/master_control/friction_control_velocity", output_dart_launch_velocity_, nan);
        register_output("/dart/master_control/angle_command", output_angle_control_enable_, false);
        register_output(
            "/dart/master_control/angle_control_vector", output_angle_control_vector_, Eigen::Vector2d::Zero());
    }

    void update() override {
        update_remote_control_commands();
        *output_friction_enable_      = friction_enable_;
        *output_angle_control_enable_ = angle_control_enable_;

        update_output_control_values();

        if (filling_enable_) {
            dart_filling_control();
        }
    }

private:
    void update_remote_control_commands() {
        using namespace rmcs_msgs;
        switch_left_  = *input_switch_left_;
        switch_right_ = *input_switch_right_;

        if ((switch_left_ == Switch::DOWN && switch_right_ == Switch::DOWN) || switch_left_ == Switch::UNKNOWN
            || switch_right_ == Switch::UNKNOWN) {
            angle_control_enable_ = false;
            friction_enable_      = false;
            filling_enable_       = false;
        }

        if (switch_left_ == Switch::UP && switch_right_ == Switch::UP) {
            angle_control_enable_ = true;
        } else {
            angle_control_enable_ = false;
        }

        if (switch_right_ == Switch::MIDDLE || switch_right_ == Switch::UP) {
            friction_enable_ = true;
        } else {
            friction_enable_ = false;
        }

        if (switch_right_ == Switch::MIDDLE) {
            if (switch_left_ == Switch::UP) {
                filling_enable_ = true;
            }
        } else {
            filling_enable_     = false;
            conveyor_direction_ = -1;
        }
    }

    void update_output_control_values() {
        double pitch_control_input_ = 30.0 * input_joystick_right_->x();
        double yaw_control_input_   = 30.0 * input_joystick_right_->y();

        output_angle_control_vector_->x() = std::max(-limit_velocity, std::min(limit_velocity, yaw_control_input_));
        output_angle_control_vector_->y() = std::max(-limit_velocity, std::min(limit_velocity, pitch_control_input_));

        *output_dart_launch_velocity_ = nan;
    }

    void dart_filling_control() {
        if (conveyor_is_stable_ && *input_conveyor_velocity_ == 0) {
            if (conveyor_direction_ > 0) {
                launch_count_++;
            }
            conveyor_is_stable_ = false;
            conveyor_direction_ = -1 * conveyor_direction_;
        }

        if (abs(*input_conveyor_velocity_) >= 10.0) {
            conveyor_is_stable_ = true;
        }

        if (launch_count_ == 2) {
            filling_enable_ = false;
        }

        *output_conveyor_control_velocity_ = filling_enable_ ? 100 * conveyor_direction_ : nan;
    }

    rclcpp::Logger logger_;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    double limit_velocity;

    int conveyor_direction_  = -1;
    int launch_count_        = 0;
    bool conveyor_is_stable_ = false;

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
    OutputInterface<Eigen::Vector2d> output_angle_control_vector_;
    OutputInterface<bool> output_friction_enable_;
    OutputInterface<double> output_dart_launch_velocity_;

    InputInterface<double> input_conveyor_velocity_;
    OutputInterface<double> output_conveyor_control_velocity_;
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