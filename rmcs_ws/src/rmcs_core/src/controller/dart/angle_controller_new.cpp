#include <algorithm>
#include <boost/graph/graph_traits.hpp>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/type_support_decl.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::dartlauncher {

class AngleControl
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AngleControl()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        pitch_init_angle_  = get_parameter("pitch_init_angle").as_double();
        pitch_lower_limit_ = get_parameter("pitch_lower_limit").as_double();
        pitch_upper_limit_ = get_parameter("pitch_upper_limit").as_double();

        register_input("/dart/device/imu_data", imu_data_);
        register_input("/dart/master_control/angle_control_vector", angle_control_vector_);

        register_output("/dart/pitch_angle/setpoint", pitch_angle_setpoint_);
        register_output("/dart/pitch_angle/current_angle", pitch_angle_current_value_, nan);
        register_output("/dart/yaw_angle/control_velocity", yaw_angle_control_velocity_);

        launch_time_ = std::chrono::steady_clock::now();
    }

    void update() override {
        if (!imu_data_stable_) {
            auto delta_time_ =
                std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - launch_time_);

            if (delta_time_.count() > 60) {
                RCLCPP_INFO(logger_, "imu_ready");
                *pitch_angle_setpoint_ = pitch_init_angle_;
                imu_data_stable_       = true;
            } else {
                return;
            }
        }

        *pitch_angle_current_value_ = calc_pitch_angle();
        RCLCPP_INFO(
            logger_, "current_pitch:%lf,setpoint:%lf", *pitch_angle_current_value_, *pitch_angle_setpoint_);

        double pitch_expected_angle = *pitch_angle_setpoint_ + 0.0005 * angle_control_vector_->y();
        *pitch_angle_setpoint_ = std::max(pitch_lower_limit_, std::min(pitch_expected_angle, pitch_upper_limit_));

        *yaw_angle_control_velocity_ = 30 * angle_control_vector_->x();
    }

private:
    double calc_pitch_angle() {
        auto dart_imu_pose             = imu_data_->normalized();
        Eigen::Matrix3d rotationMatrix = dart_imu_pose.toRotationMatrix();
        Eigen::Vector3d rpy_angles     = rotationMatrix.eulerAngles(2, 0, 1);
        double pitch                   = 180.0 + rpy_angles[1] * 180.0 / M_PI;
        return pitch;
    }

    rclcpp::Logger logger_;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    bool imu_data_stable_ = false;
    std::chrono::steady_clock::time_point launch_time_;
    InputInterface<Eigen::Quaterniond> imu_data_;

    InputInterface<Eigen::Vector2d> angle_control_vector_;
    OutputInterface<double> pitch_angle_setpoint_;
    OutputInterface<double> pitch_angle_current_value_;
    OutputInterface<double> yaw_angle_control_velocity_;

    double pitch_init_angle_;
    double pitch_upper_limit_;
    double pitch_lower_limit_;
};
} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::AngleControl, rmcs_executor::Component)
