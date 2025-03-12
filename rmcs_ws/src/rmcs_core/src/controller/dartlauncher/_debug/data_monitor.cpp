#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
namespace rmcs_core::controller::dartlauncher {

class DataMonitor
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DataMonitor()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {}

    void update() override { fps_calc(update_last_time_point_, true); }

private:
    rclcpp::Logger logger_;

    double fps_calc(std::chrono::steady_clock::time_point& last_time_point, bool log = false) {
        auto time_point_now = std::chrono::steady_clock::now();
        long delta_time =
            std::chrono::duration_cast<std::chrono::microseconds>(time_point_now - last_time_point).count();
        last_time_point = time_point_now;

        double fps = 1000000.00 / static_cast<double>(delta_time);
        if (log && fps <= 800) {
            RCLCPP_INFO(logger_, "fps = %8.3lf", fps);
        }

        return fps;
    }

    std::chrono::steady_clock::time_point update_last_time_point_;
};

} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::DataMonitor, rmcs_executor::Component)