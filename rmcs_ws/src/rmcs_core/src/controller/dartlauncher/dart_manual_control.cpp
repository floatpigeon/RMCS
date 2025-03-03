/*
TODO: auto_filling()
*/
#include <eigen3/Eigen/Dense>
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
        , logger_(get_logger()) {}

    void update() override {
        update_controller_status();
        *output_friction_enable_      = friction_enable_;
        *output_angle_control_enable_ = angle_control_enable_;

        update_angle_control_data();
        if (filling_enable_) {
            auto_filling();
        }
    }

private:
    void update_controller_status() {
        using namespace rmcs_msgs;
        switch_left_  = *input_switch_left_;
        switch_right_ = *input_switch_right_;

        angle_control_enable_ = false;
        friction_enable_      = false;
        filling_enable_       = false;

        if ((switch_left_ == Switch::DOWN && switch_right_ == Switch::DOWN) || switch_left_ == Switch::UNKNOWN
            || switch_right_ == Switch::UNKNOWN) {
            return;
        }
        if (switch_right_ != Switch::DOWN && switch_right_ != Switch::UNKNOWN) {
            friction_enable_ = true;
        }
        if (switch_right_ == Switch::MIDDLE && (switch_left_ == Switch::MIDDLE || switch_left_ == Switch::UP)) {
            filling_enable_ = true;
        }
        if (switch_left_ == Switch::UP && switch_right_ == Switch::UP) {
            angle_control_enable_ = true;
        }
    }

    void update_angle_control_data() {
        double pitch_control_input_ = 30.0 * joystick_right_->x();
        double yaw_control_input_   = 30.0 * joystick_right_->y();

        angle_control_vector_->x() =
            std::max(-angle_velocity_limit_, std::min(angle_velocity_limit_, yaw_control_input_));
        angle_control_vector_->y() =
            std::max(-angle_velocity_limit_, std::min(angle_velocity_limit_, pitch_control_input_));
    }

    void auto_filling() {}

    rclcpp::Logger logger_;
    static constexpr double nan  = std::numeric_limits<double>::quiet_NaN();
    double angle_velocity_limit_ = 30.00;

    bool angle_control_enable_ = false;
    bool friction_enable_      = false;
    bool filling_enable_       = false;

    rmcs_msgs::Switch switch_left_  = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch switch_right_ = rmcs_msgs::Switch::UNKNOWN;

    InputInterface<rmcs_msgs::Switch> input_switch_left_;
    InputInterface<rmcs_msgs::Switch> input_switch_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<Eigen::Vector2d> joystick_right_;

    OutputInterface<Eigen::Vector2d> angle_control_vector_;
    OutputInterface<double> output_inital_launch_velocity_;
    OutputInterface<bool> output_friction_enable_;
    OutputInterface<bool> output_angle_control_enable_;
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