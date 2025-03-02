#include "hikcamera/image_capturer.hpp"
#include <chrono>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
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
        camera_read_thread_           = std::thread(&VisionProcess::image_capture, this);

        image_process_thread_ = std::thread(&VisionProcess::identify, this);

        register_output("/dart/camera_image", output_image_);
    }

    void update() override {
        double camera_fps, identify_fps;
        int the_id;
        {
            std::lock_guard<std::mutex> lock(camera_read_mtx_);
            camera_fps   = camera_fps_;
            identify_fps = identify_fps_;
            the_id       = the_latest_processed_id_;
        }
        {
            std::lock_guard<std::mutex> lock(image_process_mtx_);
            *output_image_ = latest_display_image_buffer_;
        }
        double update_fps = fps_calc(update_last_time_point_, false);

        RCLCPP_INFO(
            logger_, "fps:update:%8.3lf,camera:%8.3lf,identify:%8.3lf,image_id:%5d", update_fps, camera_fps,
            identify_fps, the_id);
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
                latest_image_id_     = (latest_image_id_ + 1) % 100;
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
                last_processed_image_id_ = -1;
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
                continue;
            }
            if (image_id == last_processed_image_id_) {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                continue;
            }

            last_processed_image_id_   = image_id;
            cv::Mat preprocessed_image = preprocess(display_image, cv::COLOR_RGB2HLS);
            auto possible_targets      = first_filter(preprocessed_image, display_image);

            double fps = fps_calc(process_last_time_point_);
            {
                std::lock_guard<std::mutex> lock(image_process_mtx_);
                latest_display_image_buffer_   = display_image;
                possible_target_points_buffer_ = possible_targets;
                identify_fps_                  = fps;
                the_latest_processed_id_       = image_id;
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

    cv::Mat preprocess(const cv::Mat& input, int code) {
        cv::Mat process;
        cv::cvtColor(input, process, code);
        cv::Scalar lower_limit;
        cv::Scalar upper_limit;

        switch (code) {
        case cv::COLOR_RGB2HLS:
            lower_limit = cv::Scalar(50, 80, 128);
            upper_limit = cv::Scalar(70, 160, 255);
            break;

        case cv::COLOR_RGB2HSV:
            lower_limit = cv::Scalar(40, 50, 200);
            upper_limit = cv::Scalar(50, 255, 255);
            break;

        case cv::COLOR_RGB2BGR:
            lower_limit = cv::Scalar(0, 180, 0);
            upper_limit = cv::Scalar(80, 255, 80);
            break;

        default: RCLCPP_WARN(logger_, "VisionProcess::preprocess : invalid color_code"); break;
        }

        cv::inRange(input, lower_limit, upper_limit, process);
        static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(process, process, cv::MORPH_OPEN, kernel);
        cv::dilate(process, process, kernel);

        return process;
    }

    static std::vector<cv::Point> first_filter(cv::Mat& process, cv::Mat& display) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(process, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::Point> possible_targets;
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area <= 64) {
                continue;
            }

            double perimeter        = cv::arcLength(contour, true);
            cv::RotatedRect minRect = cv::minAreaRect(contour);
            cv::Point2f rectPoints[4];
            minRect.points(rectPoints);

            double a = sqrt(pow(rectPoints[1].x - rectPoints[0].x, 2) + pow(rectPoints[1].y - rectPoints[0].y, 2));
            double b = sqrt(pow(rectPoints[1].x - rectPoints[2].x, 2) + pow(rectPoints[1].y - rectPoints[2].y, 2));

            if (perimeter > 2 * (a + b) || a / b > 1.5 || b / a > 1.5) {
                continue;
            }
            for (int i = 0; i < 4; i++) {
                cv::line(display, rectPoints[i], rectPoints[(i + 1) % 4], cv::Scalar(255, 0, 255), 1);
            }
            possible_targets.emplace_back(minRect.center);
        }

        int rows      = display.rows;
        int half_cols = display.cols / 2;
        cv::line(display, cv::Point(half_cols, 0), cv::Point(half_cols, rows), cv::Scalar(255, 0, 255), 1);

        return possible_targets;
    }

    rclcpp::Logger logger_;
    std::thread camera_read_thread_, image_process_thread_;
    std::mutex camera_read_mtx_, image_process_mtx_;

    cv::Mat latest_display_image_buffer_;
    std::vector<cv::Point> possible_target_points_buffer_;
    OutputInterface<cv::Mat> output_image_;
    OutputInterface<std::vector<cv::Point>> output_possible_target_points_;

    // image_capture resources
    hikcamera::ImageCapturer::CameraProfile camera_profile_;
    std::unique_ptr<hikcamera::ImageCapturer> image_capture_;
    bool camera_capture_enable_ = true;
    cv::Mat camera_latest_image_;
    int latest_image_id_         = -1;
    int last_processed_image_id_ = -1;

    // fps_calc resources
    std::chrono::steady_clock::time_point update_last_time_point_;
    std::chrono::steady_clock::time_point camera_last_time_point_;
    std::chrono::steady_clock::time_point process_last_time_point_;
    double camera_fps_, identify_fps_;

    // debug resources
    int the_latest_processed_id_; //  To check if there is frame drop
};
} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::VisionProcess, rmcs_executor::Component)