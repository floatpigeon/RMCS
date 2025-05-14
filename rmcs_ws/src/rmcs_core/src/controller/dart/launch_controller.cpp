#include <cstdlib>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::dartlauncher {

class LaunchControl
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    LaunchControl()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        auto first_friction_parameter = get_parameter("first_velocity");
        if (first_friction_parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            first_friction_working_velocity_.make_and_bind_directly(first_friction_parameter.as_double());
        } else {
            register_input(first_friction_parameter.as_string(), first_friction_working_velocity_);
        }

        auto second_friction_parameter = get_parameter("second_velocity");
        if (second_friction_parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            second_friction_working_velocity_.make_and_bind_directly(second_friction_parameter.as_double());
        } else {
            register_input(second_friction_parameter.as_string(), second_friction_working_velocity_);
        }

        register_output("/dart/first_friction/control_velocity", first_friction_control_velocity_, nan);
        register_output("/dart/second_friction/control_velocity", second_friction_control_velocity_, nan);
        register_input("/dart/first_right_friction/velocity", first_friction_current_velocity_, false);
        register_input("/dart/master_control/friction_command", friction_enable_flag_);

        register_input("/dart/master_control/filling_command", filling_start_flag_);
        register_input("/dart/conveyor/velocity", conveyor_current_velocity_, false);
        register_output("/dart/conveyor/control_velocity", conveyor_control_velocity_, nan);
    }

    void update() override {
        if (*friction_enable_flag_) {
            *first_friction_control_velocity_  = *first_friction_working_velocity_;
            *second_friction_control_velocity_ = *second_friction_working_velocity_;
        } else {
            dart_fill_working_ = false;
            if (abs(*first_friction_current_velocity_) <= 5 || stop_count_ > 1000) {
                *first_friction_control_velocity_  = nan;
                *second_friction_control_velocity_ = nan;
            } else {
                *first_friction_control_velocity_  = 0.0;
                *second_friction_control_velocity_ = 0.0;
                stop_count_++;
            }
        }

        stop_count_ = 0;

        if (dart_fill_working_) {
            dart_filling_control();
        } else {
            conveyor_working_direction_ = -3;
            *conveyor_control_velocity_ = nan;
            dart_launch_count_          = 0;

            if (*filling_start_flag_ == true) {
                dart_fill_working_ = true;
            }
        }
    }

private:
    void dart_filling_control() {
        if (conveyor_velocity_stable_flag_ && *conveyor_current_velocity_ == 0.0) {
            if (conveyor_working_direction_ < 0) {
                dart_launch_count_++;
            }
            conveyor_velocity_stable_flag_ = false;
            conveyor_working_direction_    = -1 * conveyor_working_direction_ - 2.2;
        }

        if (abs(*conveyor_current_velocity_) > 5.0) {
            conveyor_velocity_stable_flag_ = true;
        }

        if (dart_launch_count_ == 2) {
            dart_fill_working_ = false;
        }

        *conveyor_control_velocity_ = dart_fill_working_ ? 200 * conveyor_working_direction_ : nan;
    }
    rclcpp::Logger logger_;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    InputInterface<bool> friction_enable_flag_;
    InputInterface<double> first_friction_working_velocity_, second_friction_working_velocity_;
    InputInterface<double> first_friction_current_velocity_;
    OutputInterface<double> first_friction_control_velocity_, second_friction_control_velocity_;

    bool conveyor_velocity_stable_flag_ = false;
    bool dart_fill_working_             = false;
    int dart_launch_count_              = 0;
    double conveyor_working_direction_  = -1.0;
    InputInterface<bool> filling_start_flag_;
    InputInterface<double> conveyor_current_velocity_;
    OutputInterface<double> conveyor_control_velocity_;
    int stop_count_ = 0;
};
} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::LaunchControl, rmcs_executor::Component)
