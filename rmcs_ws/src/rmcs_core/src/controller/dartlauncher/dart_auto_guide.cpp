#include <eigen3/Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <vector>

namespace rmcs_core::controller::dartlauncher {

class DartAutoGuide
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartAutoGuide()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {}

    void update() override {}

private:
    void target_selector() {}
    rclcpp::Logger logger_;

    InputInterface<std::vector<cv::Point>> input_possible_target_points_; // from vision_process
    OutputInterface<Eigen::Vector2d> output_error_vector_;

    // Status of each controller
    bool angle_control_enable_ = false;
    bool friction_enable_      = false;
    bool filling_enable_       = false;
};

} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::DartAutoGuide, rmcs_executor::Component)

/*
镖架制导的主控模块，包含目标选择、自主火控
发送误差值给AngleControl，协调控制角度控制和飞镖填装
未来需要与射表接洽
*/