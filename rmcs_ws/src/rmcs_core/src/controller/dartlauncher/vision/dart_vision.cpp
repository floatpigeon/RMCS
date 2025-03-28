#include "controller/dartlauncher/dart_resources.hpp"
#include "controller/dartlauncher/vision/image_process.hpp"
#include <hikcamera/image_capturer.hpp>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
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

        camera_thread_  = std::thread(&DartVision::image_capture, this);
        identify_thead_ = std::thread(&DartVision::guide_light_identify, this);

        // register_input("/dart/master_control/auto_guide_enable", auto_guide_enable_);
    }

    void update() override {}

private:
    void image_capture() {
        while (true) {
            if (!true) {
                {
                    std::lock_guard<std::mutex> lock(camera_image_mtx_);
                    latest_camera_image_buffer_.image = cv::Mat();
                    latest_camera_image_buffer_.id    = -1;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
                continue;
            }

            cv::Mat read = image_capture_->read();
            {
                std::lock_guard<std::mutex> lock(camera_image_mtx_);
                latest_camera_image_buffer_.image = read;
                latest_camera_image_buffer_.id    = (latest_camera_image_buffer_.id + 1) % 300;
            }
        }
    }

    void guide_light_identify() {
        while (true) {
            ImageData latest_image;
            {
                std::lock_guard<std::mutex> lock(camera_image_mtx_);
                latest_image = latest_camera_image_buffer_;
            }

            if (latest_image.id == -1 || latest_image.id == latest_processed_image_.id) {
                latest_processed_image_ = latest_image;
                continue;
            }

            cv::Mat preprocessed_image = ImageProcess::pre_process_beta(latest_image.image, cv::COLOR_RGB2HLS);
            ImageProcess::first_filter_beta(preprocessed_image, latest_image.image, true);
        }
    }

    rclcpp::Logger logger_;
    std::thread camera_thread_, identify_thead_;
    std::mutex camera_image_mtx_, identify_mtx_;

    ImageData latest_camera_image_buffer_;
    ImageData latest_processed_image_; // only in identify_thread now

    hikcamera::ImageCapturer::CameraProfile camera_profile_;
    std::unique_ptr<hikcamera::ImageCapturer> image_capture_;

    // InputInterface<bool> auto_guide_enable_;
};

} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::DartVision, rmcs_executor::Component)