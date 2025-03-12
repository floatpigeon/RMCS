// #include <rclcpp/logger.hpp>
// #include <rclcpp/node.hpp>
// #include <rmcs_executor/component.hpp>
// namespace rmcs_core::controller::dartlauncher {

// class MsgProcess
//     : public rmcs_executor::Component
//     , public rclcpp::Node {
// public:
//     MsgProcess()
//         : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
//         , logger_(get_logger()) {}

//     void update() override {}

// private:
//     rclcpp::Logger logger_;
// };

// } // namespace rmcs_core::controller::dartlauncher

// #include <pluginlib/class_list_macros.hpp>
// PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::MsgProcess, rmcs_executor::Component)