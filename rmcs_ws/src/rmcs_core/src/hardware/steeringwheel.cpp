#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "librmcs/client/cboard.hpp"
#include <cstdint>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/detail/int32__struct.hpp>

namespace rmcs_core::hardware {

class SteeringWheel
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::client::CBoard {
public:
    SteeringWheel()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::client::CBoard{static_cast<int>(get_parameter("usb_pid").as_int())}
        , logger_(get_logger())
        , steeringwheel_command_(
              create_partner_component<SteeringWheelCommand>(get_component_name() + "_command", *this))
        , dr16_{*this}
        , chassis_wheel_motors_(
              {*this, *steeringwheel_command_, "/chassis/left_front_wheel"},
              {*this, *steeringwheel_command_, "/chassis/left_back_wheel"},
              {*this, *steeringwheel_command_, "/chassis/right_back_wheel"},
              {*this, *steeringwheel_command_, "/chassis/right_front_wheel"})
        , chassis_steer_motors_(
              {*this, *steeringwheel_command_, "/chassis/left_front_steer"},
              {*this, *steeringwheel_command_, "/chassis/left_back_steer"},
              {*this, *steeringwheel_command_, "/chassis/right_back_steer"},
              {*this, *steeringwheel_command_, "/chassis/right_front_steer"})
        , transmit_buffer_(*this, 32)
        , event_thread_([this]() { handle_events(); }) {

        for (auto& motor : chassis_wheel_motors_) {
            motor.configure(device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                                .set_reversed()
                                .set_reduction_ratio(19.)
                                .enable_multi_turn_angle());
        }

        chassis_steer_motors_[0].configure(
            device::DjiMotor::Config{device::DjiMotor::Type::GM6020}
                .set_encoder_zero_point(static_cast<int>(get_parameter("left_front_zero_point").as_int()))
                .enable_multi_turn_angle());
        chassis_steer_motors_[1].configure(
            device::DjiMotor::Config{device::DjiMotor::Type::GM6020}
                .set_encoder_zero_point(static_cast<int>(get_parameter("left_back_zero_point").as_int()))
                .enable_multi_turn_angle());
        chassis_steer_motors_[2].configure(
            device::DjiMotor::Config{device::DjiMotor::Type::GM6020}
                .set_encoder_zero_point(static_cast<int>(get_parameter("right_back_zero_point").as_int()))
                .enable_multi_turn_angle());
        chassis_steer_motors_[3].configure(
            device::DjiMotor::Config{device::DjiMotor::Type::GM6020}
                .set_encoder_zero_point(static_cast<int>(get_parameter("right_front_zero_point").as_int()))
                .enable_multi_turn_angle());

        // TODO: measure wheel distance
        constexpr double wheel_distance_x = 0.00, wheel_distance_y = 0.00;

        using namespace rmcs_description;
        tf_->set_transform<BaseLink, LeftFrontWheelLink>(
            Eigen::Translation3d{wheel_distance_x / 2, wheel_distance_y / 2, 0});
        tf_->set_transform<BaseLink, LeftBackWheelLink>(
            Eigen::Translation3d{-wheel_distance_x / 2, wheel_distance_y / 2, 0});
        tf_->set_transform<BaseLink, RightBackWheelLink>(
            Eigen::Translation3d{-wheel_distance_x / 2, -wheel_distance_y / 2, 0});
        tf_->set_transform<BaseLink, RightFrontWheelLink>(
            Eigen::Translation3d{wheel_distance_x / 2, -wheel_distance_y / 2, 0});
    }

    ~SteeringWheel() {
        stop_handling_events();
        event_thread_.join();
    }

    void update() override {
        update_motors();
        dr16_.update_status();
    }

    void command_update() {
        uint16_t can_commands[4];

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = chassis_wheel_motors_[0].generate_command();
        can_commands[1] = chassis_wheel_motors_[1].generate_command();
        can_commands[2] = chassis_steer_motors_[0].generate_command();
        can_commands[3] = chassis_steer_motors_[1].generate_command();
        transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can2_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = chassis_wheel_motors_[2].generate_command();
        can_commands[1] = chassis_wheel_motors_[3].generate_command();
        can_commands[2] = chassis_steer_motors_[2].generate_command();
        can_commands[3] = chassis_steer_motors_[3].generate_command();
        transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        transmit_buffer_.trigger_transmission();
    }

private:
    void update_motors() {
        for (auto& motor : chassis_wheel_motors_)
            motor.update_status();
        for (auto& motor : chassis_steer_motors_)
            motor.update_status();
    }

    void calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            logger_, "[chassis_steer_motors calibration] New left font offset: %d",
            chassis_steer_motors_[0].calibrate_zero_point());
        RCLCPP_INFO(
            logger_, "[chassis_steer_motors calibration] New left back offset: %d",
            chassis_steer_motors_[1].calibrate_zero_point());
        RCLCPP_INFO(
            logger_, "[chassis_steer_motors calibration] New right back offset: %d",
            chassis_steer_motors_[2].calibrate_zero_point());
        RCLCPP_INFO(
            logger_, "[chassis_steer_motors calibration] New right font offset: %d",
            chassis_steer_motors_[3].calibrate_zero_point());
    }

protected:
    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {

        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;
        if (can_id == 0x201) {
            auto& motor = chassis_wheel_motors_[0];
            motor.store_status(can_data);
        } else if (can_id == 0x202) {
            auto& motor = chassis_wheel_motors_[1];
            motor.store_status(can_data);
        } else if (can_id == 0x203) {
            auto& motor = chassis_steer_motors_[0];
            motor.store_status(can_data);
        } else if (can_id == 0x204) {
            auto& motor = chassis_steer_motors_[1];
            motor.store_status(can_data);
        }
    }

    void can2_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {

        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;
        if (can_id == 0x201) {
            auto& motor = chassis_wheel_motors_[2];
            motor.store_status(can_data);
        } else if (can_id == 0x202) {
            auto& motor = chassis_wheel_motors_[3];
            motor.store_status(can_data);
        } else if (can_id == 0x203) {
            auto& motor = chassis_steer_motors_[2];
            motor.store_status(can_data);
        } else if (can_id == 0x204) {
            auto& motor = chassis_steer_motors_[3];
            motor.store_status(can_data);
        }
    }

    void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
        dr16_.store_status(uart_data, uart_data_length);
    }

private:
    rclcpp::Logger logger_;

    class SteeringWheelCommand : public rmcs_executor::Component {
    public:
        explicit SteeringWheelCommand(SteeringWheel& steeringwheel)
            : steeringwheel_(steeringwheel) {}

        void update() override { steeringwheel_.command_update(); }

        SteeringWheel& steeringwheel_;
    };
    std::shared_ptr<SteeringWheelCommand> steeringwheel_command_;

    device::Dr16 dr16_;
    OutputInterface<rmcs_description::Tf> tf_;

    device::DjiMotor chassis_wheel_motors_[4];
    device::DjiMotor chassis_steer_motors_[4];

    librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
    std::thread event_thread_;
};
} // namespace rmcs_core::hardware
