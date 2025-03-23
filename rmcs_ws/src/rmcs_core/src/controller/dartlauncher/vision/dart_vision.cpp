// #include "controller/dartlauncher/vision/image_process.hpp"
// #include <hikcamera/image_capturer.hpp>
// #include <opencv2/core/mat.hpp>
// #include <opencv2/opencv.hpp>
// #include <rclcpp/logger.hpp>
// #include <rclcpp/node.hpp>
// #include <rmcs_executor/component.hpp>
// namespace rmcs_core::controller::dartlauncher {

// class DartVision
//     : public rmcs_executor::Component
//     , public rclcpp::Node {
// public:
//     DartVision()
//         : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
//         , logger_(get_logger()) {}

//     void update() override {}

// private:
//     void image_capture() {}

//     rclcpp::Logger logger_;
// };

// } // namespace rmcs_core::controller::dartlauncher

// #include <pluginlib/class_list_macros.hpp>
// PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::DartVision, rmcs_executor::Component)