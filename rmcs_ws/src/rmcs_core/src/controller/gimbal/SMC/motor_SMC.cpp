#include "controller/filter/lowpass_filter.hpp"
#include <deque>
#include <eigen3/Eigen/Dense>
#include <numbers>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <switch.hpp>

namespace rmcs_core::controller::gimbal {

class MotorSMC
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MotorSMC()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

        register_input("/SMC/test_motor/angle", current_angle_);
        register_input("/SMC/test_motor/velocity", current_velocity_);

        register_output("/SMC/test_motor/target_angle", target_angle_);
        register_output("/SMC/test_motor/target_velocity", target_velocity_);
        register_output("/SMC/test_motor/target_acceleration", target_acceleration_);
    }

    void update() override {
        velocity_window_.push_back(*target_velocity_);
        if (velocity_window_.size() > 10) {
            velocity_window_.pop_front();
        }

        position_window_.push_back(*target_angle_);
        if (position_window_.size() > 10) {
            position_window_.pop_front();
        }

        if ((*switch_left_ == rmcs_msgs::Switch::DOWN && *switch_right_ == rmcs_msgs::Switch::DOWN)
            || (*switch_left_ == rmcs_msgs::Switch::UNKNOWN && *switch_right_ == rmcs_msgs::Switch::UNKNOWN)) {
            init();
        } else {
            // calc_target_value();
            position_response_simulation();
        }
    }

private:
    void init() {
        angle_integral_       = *current_angle_;
        *target_acceleration_ = 0;
        *target_velocity_     = 0;
    }

    void position_response_simulation() {
        *target_angle_ = std::numbers::pi + joystick_left_->y() * std::numbers::pi * 0.75;

        if (position_window_.size() < 2) {
            *target_velocity_ = nan;
        }
        *target_velocity_ = least_square_calculator(position_window_);

        if (velocity_window_.size() < 2) {
            *target_acceleration_ = nan;
        }
        *target_acceleration_ = least_square_calculator(velocity_window_);
    }

    void calc_target_value() {
        // velocity
        *target_velocity_ = joystick_left_->y() * joystick_sensitivity_;

        // angle
        angle_integral_ = angle_integral_ + *target_velocity_ * 0.001;

        if (angle_integral_ > 2 * std::numbers::pi)
            angle_integral_ -= 2 * std::numbers::pi;
        else if (angle_integral_ < 0)
            angle_integral_ += 2 * std::numbers::pi;

        *target_angle_ = angle_integral_;

        // acceleration
        if (velocity_window_.size() < 2) {
            *target_acceleration_ = nan;
        }
        *target_acceleration_ = least_square_calculator(velocity_window_);
    }

    double least_square_calculator(const std::deque<double>& angular_velocities) {
        int window_size = static_cast<int>(velocity_window_.size());

        double sum_x         = 0.0;
        double sum_y         = 0.0;
        double sum_xy        = 0.0;
        double sum_x_squared = 0.0;

        for (int i = 0; i < window_size; ++i) {
            double x_val = static_cast<double>(i);
            double y_val = angular_velocities[i];

            sum_x += x_val;
            sum_y += y_val;
            sum_xy += x_val * y_val;
            sum_x_squared += x_val * x_val;
        }

        double denominator = window_size * sum_x_squared - sum_x * sum_x;
        if (denominator == 0.0) {
            return 0.0;
        }

        double m_prime              = (window_size * sum_xy - sum_x * sum_y) / denominator;
        double angular_acceleration = m_prime * 0.001;

        return angular_acceleration;
    }

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    InputInterface<double> current_angle_;
    InputInterface<double> current_velocity_;

    double angle_integral_       = 0;
    double joystick_sensitivity_ = 10;

    std::deque<double> velocity_window_;
    std::deque<double> position_window_;

    OutputInterface<double> target_angle_;
    OutputInterface<double> target_velocity_;
    OutputInterface<double> target_acceleration_;

    rmcs_core::controller::filter::LowPassFilter joystick_input_fliter_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::MotorSMC, rmcs_executor::Component)