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
        first_friction_default_velocity_  = get_parameter("first_default_velicity").as_double();
        second_friction_default_velocity_ = get_parameter("second_default_velocity").as_double();

        register_input("/dart/master_control/friction_command", friction_enable_);
        register_input("/dart/master_control/friction_control_velocity", input_dart_launch_velocity_);

        register_output("/dart/first_friction/control_velocity", first_friction_working_velocity_, nan);
        register_output("/dart/second_friction/control_velocity", second_friction_working_velocity_, nan);
    }

    void update() override {
        if (*friction_enable_ == false) {
            reset_all_controls();
        } else {
            update_friction_velocitys();
        }
    }

private:
    void reset_all_controls() {
        *first_friction_working_velocity_  = nan;
        *second_friction_working_velocity_ = nan;
    }

    void update_friction_velocitys() {
        if (*input_dart_launch_velocity_ == nan) {
            *first_friction_working_velocity_  = first_friction_default_velocity_;
            *second_friction_working_velocity_ = second_friction_default_velocity_;
        } else {
            // 需要测试实际初速和摩擦轮转速的换算关系，当前先按理想情况计算
            double launch_rotation_speed       = *input_dart_launch_velocity_ / 0.05;
            *second_friction_working_velocity_ = launch_rotation_speed;
            *first_friction_working_velocity_  = launch_rotation_speed + 200;
        }
    }

    rclcpp::Logger logger_;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    double first_friction_default_velocity_;
    double second_friction_default_velocity_;

    InputInterface<bool> friction_enable_;                    // from dart_auto_guide or dart_manual_control
    InputInterface<double> input_dart_launch_velocity_;       // from dart_auto_guide or dart_manual_control,unit: m/s

    OutputInterface<double> first_friction_working_velocity_; // close to filling direction called first
    OutputInterface<double> second_friction_working_velocity_;
};

} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::LaunchControl, rmcs_executor::Component)