#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
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
        register_input("/dart/master_control/angle_command", angle_control_enable_);
        register_input("/dart/master_control/angle_control_vector", angle_control_vector_);

        register_output("/dart/yaw_angle/control_velocity", yaw_control_velocity_, nan);
        register_output("/dart/pitch_angle/control_velocity", pitch_control_velocity_, nan);
        register_output("/dart/pitch/angle", pitch_angle_);

        register_input("/dart/device/imu_data", imu_data_);
    }

    void update() override {
        calibration_angles();

        if (!*angle_control_enable_) {
            *yaw_control_velocity_   = nan;
            *pitch_control_velocity_ = nan;
        } else {
            *yaw_control_velocity_   = angle_control_vector_->x();
            *pitch_control_velocity_ = angle_control_vector_->y();
        }
    }

private:
    void calibration_angles() {
        auto dart_imu_pose             = imu_data_->normalized();
        Eigen::Matrix3d rotationMatrix = dart_imu_pose.toRotationMatrix();
        Eigen::Vector3d rpy_angles     = rotationMatrix.eulerAngles(2, 0, 1);
        double pitch                   = 180.0 + rpy_angles[1] * 180.0 / M_PI;
        RCLCPP_INFO(logger_, "current_pitch:%lf", pitch);
    }

    rclcpp::Logger logger_;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    InputInterface<bool> angle_control_enable_;            // from dart_auto_guide or dart_manual_control
    InputInterface<Eigen::Vector2d> angle_control_vector_; // from dart_auto_guide or dart_manual_control
    InputInterface<Eigen::Quaterniond> imu_data_;

    OutputInterface<double> yaw_control_velocity_;
    OutputInterface<double> pitch_control_velocity_;
    OutputInterface<double> pitch_angle_;
};
} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::AngleControl, rmcs_executor::Component)
