#include <chrono>
#include <cmath>
#include <cstddef>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace rmcs_core::controller::dart {

class ControllerDataPublisher
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ControllerDataPublisher()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/dart/first_left_friction/velocity", input_friction_velocity_[0]);
        register_input("/dart/first_right_friction/velocity", input_friction_velocity_[1]);
        register_input("/dart/second_left_friction/velocity", input_friction_velocity_[2]);
        register_input("/dart/second_right_friction/velocity", input_friction_velocity_[3]);

        register_input("/dart/first_left_friction/temperature", input_friction_temperature_[0]);
        register_input("/dart/first_right_friction/temperature", input_friction_temperature_[1]);
        register_input("/dart/second_left_friction/temperature", input_friction_temperature_[2]);
        register_input("/dart/second_right_friction/temperature", input_friction_temperature_[3]);

        register_input("/dart/first_left_friction/temperature", input_friction_torque_[0]);
        register_input("/dart/first_right_friction/temperature", input_friction_torque_[1]);
        register_input("/dart/second_left_friction/temperature", input_friction_torque_[2]);
        register_input("/dart/second_right_friction/temperature", input_friction_torque_[3]);

        register_input("/dart/pitch_left/velocity", input_pitch_data_[0]);
        register_input("/dart/pitch_right/velocity", input_pitch_data_[1]);

        pub_temperature_[0] = this->create_publisher<std_msgs::msg::String>("first_left_friction_temp", 10);
        pub_temperature_[1] = this->create_publisher<std_msgs::msg::String>("first_right_friction_temp", 10);
        pub_temperature_[2] = this->create_publisher<std_msgs::msg::String>("second_left_friction_temp", 10);
        pub_temperature_[3] = this->create_publisher<std_msgs::msg::String>("second_right_friction_temp", 10);

        pub_velocity_[0] = this->create_publisher<std_msgs::msg::String>("first_left_friction_velocity", 10);
        pub_velocity_[1] = this->create_publisher<std_msgs::msg::String>("first_right_friction_velocity", 10);
        pub_velocity_[2] = this->create_publisher<std_msgs::msg::String>("second_left_friction_velocity", 10);
        pub_velocity_[3] = this->create_publisher<std_msgs::msg::String>("second_right_friction_velocity", 10);

        pub_torque_[0] = this->create_publisher<std_msgs::msg::String>("first_left_friction_torque", 10);
        pub_torque_[1] = this->create_publisher<std_msgs::msg::String>("first_right_friction_torque", 10);
        pub_torque_[2] = this->create_publisher<std_msgs::msg::String>("second_left_friction_torque", 10);
        pub_torque_[3] = this->create_publisher<std_msgs::msg::String>("second_right_friction_torque", 10);

        pub_pitch_data_[0] = this->create_publisher<std_msgs::msg::String>("pitch_left_velocity", 10);
        pub_pitch_data_[1] = this->create_publisher<std_msgs::msg::String>("pitch_right_velocity", 10);
        pub_pitch_data_[2] = this->create_publisher<std_msgs::msg::String>("pitch_left_position", 10);
        pub_pitch_data_[3] = this->create_publisher<std_msgs::msg::String>("pitch_right_position", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), [this]() { this->publish_message(); });
    }

    void update() override {
        for (size_t i = 0; i < 4; i++) {
            msg_friction_temperature_[i].data = std::to_string(*input_friction_temperature_[i]);
            msg_friction_velocity_[i].data    = std::to_string(*input_friction_velocity_[i]);
            msg_friction_torque_[i].data      = std::to_string(*input_friction_torque_[i]);
        }

        msg_pitch_data_[0].data = std::to_string(*input_pitch_data_[0]);
        msg_pitch_data_[1].data = std::to_string(*input_pitch_data_[1]);
        pitch_velocity_integral_[0] += *input_pitch_data_[0];
        pitch_velocity_integral_[1] += *input_pitch_data_[1];
        msg_pitch_data_[2].data = std::to_string(pitch_velocity_integral_[0]);
        msg_pitch_data_[3].data = std::to_string(pitch_velocity_integral_[1]);
    }

private:
    void publish_message() {
        for (size_t i = 0; i < 4; i++) {
            pub_temperature_[i]->publish(msg_friction_temperature_[i]);
            pub_velocity_[i]->publish(msg_friction_velocity_[i]);
            pub_torque_[i]->publish(msg_friction_torque_[i]);
            pub_pitch_data_[i]->publish(msg_pitch_data_[i]);
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_velocity_[4];
    InputInterface<double> input_friction_velocity_[4];
    std_msgs::msg::String msg_friction_velocity_[4];

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_temperature_[4];
    InputInterface<double> input_friction_temperature_[4];
    std_msgs::msg::String msg_friction_temperature_[4];

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_torque_[4];
    InputInterface<double> input_friction_torque_[4];
    std_msgs::msg::String msg_friction_torque_[4];

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_pitch_data_[4];
    InputInterface<double> input_pitch_data_[2];
    double pitch_velocity_integral_[2] = {0, 0};
    std_msgs::msg::String msg_pitch_data_[4];
};
} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::ControllerDataPublisher, rmcs_executor::Component)
