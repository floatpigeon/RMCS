#include <chrono>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <thread>
namespace rmcs_core::controller::dartlauncher {

class CameraDataMonitor
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    CameraDataMonitor()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        // register_input("camera/display_image", input_camera_image_);
    }

    void update() override { fps_calc(update_last_time_point_, true); }

private:
    rclcpp::Logger logger_;

    double fps_calc(std::chrono::steady_clock::time_point& last_time_point, bool log = false) {
        auto time_point_now = std::chrono::steady_clock::now();
        long delta_time =
            std::chrono::duration_cast<std::chrono::microseconds>(time_point_now - last_time_point).count();
        last_time_point = time_point_now;

        double fps = 1000000.00 / static_cast<double>(delta_time);
        if (log) {
            RCLCPP_INFO(logger_, "fps = %8.3lf", fps);
        }
        return fps;
    }

    void image_displayer() {
        while (true) {
            cv::Mat display;
            {
                std::lock_guard<std::mutex> lock(display_mtx_);
                if (!camera_latest_image_.empty()) {
                    display = camera_latest_image_;
                }
            }

            if (!display.empty()) {
                cv::Point yaw_center_top  = cv::Point(display.cols / 2, 0);
                cv::Point yaw_center_down = cv::Point(display.cols / 2, display.cols);
                cv::line(display, yaw_center_top, yaw_center_down, cv::Scalar(255, 0, 255), 1);
                cv::imshow("display", display);
                cv::waitKey(2);
            }
        }
    }

    std::chrono::steady_clock::time_point update_last_time_point_;
    std::chrono::steady_clock::time_point camera_last_time_point_;

    std::thread display_thread_;
    std::mutex display_mtx_;
    InputInterface<cv::Mat> input_camera_image_;
    cv::Mat camera_latest_image_;
};
} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::CameraDataMonitor, rmcs_executor::Component)