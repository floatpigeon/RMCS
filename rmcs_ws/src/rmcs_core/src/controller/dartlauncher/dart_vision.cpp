#include "controller/dartlauncher/image_process_methods.hpp"
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
        double camera_fps, identify_fps;
        {
            std::lock_guard<std::mutex> lock(camera_read_mtx_);
            camera_fps                         = camera_fps_;
            identify_fps                       = identify_fps_;
            *output_latest_processed_image_id_ = latest_processed_id_;
        }

        {
            std::lock_guard<std::mutex> lock(image_process_mtx_);
            *output_latest_display_image_   = display_image_buffer_;
            *output_possible_target_points_ = possible_points_buffer_;
        }

        double update_fps = fps_calc(update_last_time_point_, false);
        if (false) {
            RCLCPP_INFO(
                logger_, "fps:update:%8.3lf,camera:%8.3lf,identify:%8.3lf,image_id:%5d", update_fps, camera_fps,
                identify_fps, *output_latest_processed_image_id_);
        }
    }

private:
    void image_capture() {
        while (true) {
            if (!camera_capture_enable_) {
                latest_image_id_ = -1;
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
                continue;
            }

            cv::Mat reading = image_capture_->read();
            double fps      = fps_calc(camera_last_time_point_);
            {
                std::lock_guard<std::mutex> lock(camera_read_mtx_);
                camera_latest_image_ = reading;
                camera_fps_          = fps;
                latest_image_id_     = (latest_image_id_ + 1) % 300;
            }
        }
    }

    void identify() {
        while (true) {
            cv::Mat display_image;
            int image_id;
            {
                std::lock_guard<std::mutex> lock(camera_read_mtx_);
                display_image = camera_latest_image_;
                image_id      = latest_image_id_;
            }

            if (image_id == -1) {
                last_image_id_ = -1;
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
                continue;
            }
            if (image_id == last_image_id_) {
                std::this_thread::sleep_for(std::chrono::microseconds(10));
                continue;
            }

            auto preprocessed_image = ImageProcessMethods::preprocess(display_image, cv::COLOR_RGB2HLS);
            last_image_id_          = image_id;
            auto possible_targets   = ImageProcessMethods::first_filter(preprocessed_image, display_image, true);
            RCLCPP_INFO(logger_, "possible_target_num:%3zu", possible_targets.size());

            double fps = fps_calc(process_last_time_point_);
            {
                std::lock_guard<std::mutex> lock(image_process_mtx_);
                display_image_buffer_   = display_image;
                possible_points_buffer_ = possible_targets;
                identify_fps_           = fps;
                latest_processed_id_    = image_id;
            }
        }
    }

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

    rclcpp::Logger logger_;
    std::thread camera_read_thread_, image_process_thread_;
    std::mutex camera_read_mtx_, image_process_mtx_;

    int latest_processed_id_;
    cv::Mat display_image_buffer_;
    std::vector<cv::Point> possible_points_buffer_;
    OutputInterface<cv::Mat> output_latest_display_image_;
    OutputInterface<int> output_latest_processed_image_id_;
    OutputInterface<std::vector<cv::Point>> output_possible_target_points_;

    // image_capture resources
    hikcamera::ImageCapturer::CameraProfile camera_profile_;
    std::unique_ptr<hikcamera::ImageCapturer> image_capture_;
    bool camera_capture_enable_ = true;
    cv::Mat camera_latest_image_;
    int latest_image_id_ = -1;
    int last_image_id_   = -1;

    // fps_calc resources
    std::chrono::steady_clock::time_point update_last_time_point_;
    std::chrono::steady_clock::time_point camera_last_time_point_;
    std::chrono::steady_clock::time_point process_last_time_point_;
    double camera_fps_, identify_fps_;
};
} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::VisionProcess, rmcs_executor::Component)