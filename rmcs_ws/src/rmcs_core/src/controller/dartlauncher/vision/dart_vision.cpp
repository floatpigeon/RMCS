#include "controller/dartlauncher/vision/identifier.hpp"
#include <hikcamera/image_capturer.hpp>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <thread>
namespace rmcs_core::controller::dartlauncher {

class DartVision
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartVision()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        camera_profile_.invert_image  = get_parameter("invert_image").as_bool();
        camera_profile_.exposure_time = std::chrono::microseconds(get_parameter("exposure_time").as_int());
        camera_profile_.gain          = static_cast<float>(get_parameter("gain").as_double());
        image_capture_                = std::make_unique<hikcamera::ImageCapturer>(camera_profile_);

        register_output("/dart/vision/camera_image", output_camera_image_);
        // register_output("/dart/vision/display_image", output_display_image_);

        camera_thread_ = std::thread(&DartVision::image_capture, this);
    }

    void update() override {}

private:
    void image_capture() {
        while (true) {
            // if (!camera_capture_enable_) {
            //     std::this_thread::sleep_for(std::chrono::microseconds(1000));
            //     continue;
            // }

            cv::Mat read          = image_capture_->read();
            *output_camera_image_ = read;

            // identifier_.load(read);
            // if (switch_command) {
            //     identifier_.start_idenitfy();
            //     switch_command = false;
            // }
            // auto display = identifier_.get_display_image();
        }
    }
    bool switch_command = true;

    rclcpp::Logger logger_;

    std::thread camera_thread_;
    std::mutex camera_mtx_;
    bool camera_capture_enable_ = true;

    // GuideLightIdentifier identifier_;

    hikcamera::ImageCapturer::CameraProfile camera_profile_;
    std::unique_ptr<hikcamera::ImageCapturer> image_capture_;

    OutputInterface<cv::Mat> output_camera_image_;
    // OutputInterface<cv::Mat> output_display_image_;
};

} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::DartVision, rmcs_executor::Component)