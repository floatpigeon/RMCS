#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "librmcs/client/cboard.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
namespace rmcs_core::hardware {

class SMCHardware
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::client::CBoard {
public:
    SMCHardware()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::client::CBoard{static_cast<int>(get_parameter("usb_pid").as_int())}
        , logger_(get_logger())
        , smc_command_(create_partner_component<SMCCommand>(get_component_name() + "_command", *this))
        , dr16_{*this}
        , test_motor_(*this, *smc_command_, "/SMC/test_motor")
        , transmit_buffer_(*this, 32)
        , event_thread_([this]() { handle_events(); }) {
        RCLCPP_INFO(logger_, "test");

        test_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::GM6020}.set_encoder_zero_point(
                static_cast<int>(get_parameter("zero_point").as_int())));
    }

    ~SMCHardware() override {
        stop_handling_events();
        event_thread_.join();
    }

    void update() override {
        test_motor_.update_status();
        dr16_.update_status();
    }

    void command_update() {
        uint16_t can_commands[4];

        can_commands[0] = test_motor_.generate_command();
        // can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        transmit_buffer_.trigger_transmission();
    }

protected:
    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;

        if (can_id == 0x205) {
            auto& motor = test_motor_;
            motor.store_status(can_data);
        }
    }

    void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
        dr16_.store_status(uart_data, uart_data_length);
    }

private:
    rclcpp::Logger logger_;

    class SMCCommand : public rmcs_executor::Component {
    public:
        explicit SMCCommand(SMCHardware& infantry)
            : smc_(infantry) {}

        void update() override { smc_.command_update(); }

        SMCHardware& smc_;
    };
    std::shared_ptr<SMCCommand> smc_command_;

    device::Dr16 dr16_;
    device::DjiMotor test_motor_;

    librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
    std::thread event_thread_;
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::SMCHardware, rmcs_executor::Component)