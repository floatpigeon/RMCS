#include "hikcamera/image_capturer.hpp"
#include <chrono>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <thread>

namespace rmcs_core::controller::dartlauncher {

class VisionProcess
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    VisionProcess()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        camera_profile_.invert_image  = get_parameter("invert_image").as_bool();
        camera_profile_.exposure_time = std::chrono::microseconds(get_parameter("exposure_time").as_int());
        camera_profile_.gain          = static_cast<float>(get_parameter("gain").as_double());
        image_capture_                = std::make_unique<hikcamera::ImageCapturer>(camera_profile_);
        camera_read_thread_           = std::thread(&VisionProcess::image_capture, this);

        register_output("/dart/camera_image", output_image_);
    }

    void update() override {
        {
            std::lock_guard<std::mutex> lock(camera_read_mtx_);
            *output_image_ = camera_latest_image_;
        }
    }

private:
    void image_capture() {
        while (true) {
            if (!camera_capture_enable_) {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
                continue;
            }

            cv::Mat reading = image_capture_->read();
            {
                std::lock_guard<std::mutex> lock(camera_read_mtx_);
                camera_latest_image_ = reading;
            }
            cv::imshow("display", reading);
            cv::waitKey(1);
        }
    }

    rclcpp::Logger logger_;
    std::thread camera_read_thread_;
    std::thread image_process_thread_;

    hikcamera::ImageCapturer::CameraProfile camera_profile_;
    std::unique_ptr<hikcamera::ImageCapturer> image_capture_;
    bool camera_capture_enable_ = true;
    std::mutex camera_read_mtx_;
    cv::Mat camera_latest_image_;

    OutputInterface<cv::Mat> output_image_;
};
} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::VisionProcess, rmcs_executor::Component)