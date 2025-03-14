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
        first_friction_default_velocity_  = get_parameter("first_default_velicity").as_double();
        second_friction_default_velocity_ = get_parameter("second_default_velocity").as_double();

        register_input("/dart/master_control/friction_command", input_command_friction_enable_);
        register_input("/dart/master_control/friction_control_velocity", input_dart_launch_velocity_);
        register_output("/dart/first_friction/control_velocity", output_first_friction_velocity_, nan);
        register_output("/dart/second_friction/control_velocity", output_second_friction_velocity_, nan);

        register_input("/dart/master_control/filling_command", input_command_dart_filling_enable_);
        register_input("/dart/conveyor/velocity", input_conveyor_velocity_, false);
        register_output("/dart/conveyor/control_velocity", output_conveyor_control_velocity_, nan);
    }

    void update() override {
        if (*input_command_friction_enable_ == false) {
            filling_enable_                   = false;
            *output_first_friction_velocity_  = nan;
            *output_second_friction_velocity_ = nan;
        } else {
            update_friction_velocitys();
        }

        if (filling_enable_ == true) {
            dart_filling_control();
        } else if (filling_enable_ == false) {
            *output_conveyor_control_velocity_ = nan;

            if (*input_command_dart_filling_enable_ == true) {
                filling_enable_ = true;
            }
        }
    }

private:
    void update_friction_velocitys() {
        // if (*input_dart_launch_velocity_ == nan) {
        //     *output_first_friction_velocity_  = first_friction_default_velocity_;
        //     *output_second_friction_velocity_ = second_friction_default_velocity_;
        // } else {
        //     // 需要测试实际初速和摩擦轮转速的换算关系，当前先按理想情况计算
        //     double launch_rotation_speed      = *input_dart_launch_velocity_ / 0.05;
        //     *output_second_friction_velocity_ = launch_rotation_speed;
        //     *output_first_friction_velocity_  = launch_rotation_speed + 200; // 200这个值具体待测
        // }

        // *output_first_friction_velocity_  = first_friction_default_velocity_;
        // *output_second_friction_velocity_ = second_friction_default_velocity_;

        *output_first_friction_velocity_  = nan;
        *output_second_friction_velocity_ = nan;
    }

    void dart_filling_control() {
        if (conveyor_velocity_stable_ && *input_conveyor_velocity_ == 0.0) {
            if (conveyor_working_direction_ < 0) {
                dart_launch_count_++;
            }
            conveyor_velocity_stable_   = false;
            conveyor_working_direction_ = -1 * conveyor_working_direction_ - 0.5;
        }

        if (abs(*input_conveyor_velocity_) >= 10.0) {
            conveyor_velocity_stable_ = true;
        }

        if (dart_launch_count_ == 2) {
            filling_enable_ = false;
        }

        *output_conveyor_control_velocity_ = filling_enable_ ? 100 * conveyor_working_direction_ : nan;
    }

    rclcpp::Logger logger_;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    double first_friction_default_velocity_;
    double second_friction_default_velocity_;

    InputInterface<bool> input_command_friction_enable_;     // from dart_auto_guide or dart_manual_control
    InputInterface<double> input_dart_launch_velocity_;      // from dart_auto_guide or dart_manual_control,unit: m/s

    OutputInterface<double> output_first_friction_velocity_; // close to filling direction called first
    OutputInterface<double> output_second_friction_velocity_;

    bool conveyor_velocity_stable_     = false;
    bool filling_enable_               = false;
    int dart_launch_count_             = 0;
    double conveyor_working_direction_ = 0.0;
    InputInterface<bool> input_command_dart_filling_enable_; // from dart_auto_guide or dart_manual_control
    InputInterface<double> input_conveyor_velocity_;         // from dart_auto_guide or dart_manual_control
    OutputInterface<double> output_conveyor_control_velocity_;
};

} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::LaunchControl, rmcs_executor::Component)