/*
TODO: calibration_angles()
*/
#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::dartlauncher {

class AngleControl
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AngleControl()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        register_input("/dart/control_command/angle_control_enable", angle_control_enable_);
        register_input("/dart/auto_guide/angle_control_vector", angle_control_vector_);

        register_output("/dart/yaw_angle/control_velocity", yaw_control_velocity_, nan);
        register_output("/dart/pitch_angle/control_velocity", pitch_control_velocity_, nan);
    }

    void update() override {
        if (!*angle_control_enable_) {
            reset_all_controls();
        } else {
            *yaw_control_velocity_   = angle_control_vector_->x();
            *pitch_control_velocity_ = angle_control_vector_->y();
        }
    }

private:
    void reset_all_controls() {
        *yaw_control_velocity_   = nan;
        *pitch_control_velocity_ = nan;
    }

    void calibration_angles() {}                           // to be improved in the future

    rclcpp::Logger logger_;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    InputInterface<bool> angle_control_enable_;            // from dart_auto_guide or dart_manual_control
    InputInterface<Eigen::Vector2d> angle_control_vector_; // from dart_auto_guide or dart_manual_control

    OutputInterface<double> yaw_control_velocity_;
    OutputInterface<double> pitch_control_velocity_;
};
} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::AngleControl, rmcs_executor::Component)
