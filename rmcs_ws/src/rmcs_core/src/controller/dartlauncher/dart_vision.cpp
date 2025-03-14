#include "controller/dartlauncher/vision/image_process.hpp"
#include "hikcamera/image_capturer.hpp"
#include <chrono>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <thread>
#include <vector>

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

        camera_read_thread_   = std::thread(&VisionProcess::image_capture, this);
        image_process_thread_ = std::thread(&VisionProcess::identify, this);

        register_output("/dart/vision/camera_image", output_latest_display_image_);
        register_output("/dart/vision/latest_processed_image_id", output_latest_processed_image_id_, -1);
        register_output("/dart/vision/possible_points", output_possible_target_points_, std::vector<cv::Point>());
    }

    void update() override {
        {
            std::lock_guard<std::mutex> lock(image_process_mtx_);
            *output_latest_display_image_   = display_image_buffer_;
            *output_possible_target_points_ = possible_points_buffer_;
        }
    }

private:
    void image_capture() {
        while (true) {
            if (!camera_capture_enable_) {
                latest_camera_image_id_ = -1;
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
                continue;
            }

            cv::Mat reading = image_capture_->read();
            {
                std::lock_guard<std::mutex> lock(camera_read_mtx_);
                latest_camera_image_buffer_ = reading;
                latest_camera_image_id_     = (latest_camera_image_id_ + 1) % 300;
            }
        }
    }

    void identify() {
        while (true) {
            cv::Mat display_image;
            int image_id;
            {
                std::lock_guard<std::mutex> lock(camera_read_mtx_);
                display_image = latest_camera_image_buffer_;
                image_id      = latest_camera_image_id_;
            }

            if (image_id == -1) {
                last_processed_image_id_ = -1;
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
                continue;
            }
            if (image_id == last_processed_image_id_) {
                std::this_thread::sleep_for(std::chrono::microseconds(10));
                continue;
            }

            auto preprocessed_image  = ImageProcess::preprocess(display_image, cv::COLOR_RGB2HLS);
            last_processed_image_id_ = image_id;
            auto possible_targets    = ImageProcess::first_filter(preprocessed_image, display_image, false);

            {
                std::lock_guard<std::mutex> lock(image_process_mtx_);
                display_image_buffer_      = display_image;
                possible_points_buffer_    = possible_targets;
                latest_processed_image_id_ = image_id;
            }
        }
    }

    rclcpp::Logger logger_;
    std::thread camera_read_thread_, image_process_thread_;
    std::mutex camera_read_mtx_, image_process_mtx_;

    int latest_processed_image_id_;
    cv::Mat display_image_buffer_;
    std::vector<cv::Point> possible_points_buffer_;
    OutputInterface<cv::Mat> output_latest_display_image_;
    OutputInterface<int> output_latest_processed_image_id_;
    OutputInterface<std::vector<cv::Point>> output_possible_target_points_;

    hikcamera::ImageCapturer::CameraProfile camera_profile_;
    std::unique_ptr<hikcamera::ImageCapturer> image_capture_;

    bool camera_capture_enable_ = true;
    cv::Mat latest_camera_image_buffer_;
    int latest_camera_image_id_  = -1;
    int last_processed_image_id_ = -1;
};
} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::VisionProcess, rmcs_executor::Component)