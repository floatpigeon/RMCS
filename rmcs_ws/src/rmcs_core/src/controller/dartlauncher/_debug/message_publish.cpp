#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

namespace rmcs_core::controller::dart {

class MessagePublisher
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MessagePublisher()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        camera_enable_   = get_parameter("camera_enable").as_bool();
        cal_fps_enable_  = get_parameter("cal_fps_enable").as_bool();
        friction_enable_ = get_parameter("friction_enable").as_bool();

        if (friction_enable_) {
            register_input("/dart/second_left_friction/velocity", friction_lf_velocity_);
            register_input("/dart/first_left_friction/velocity", friction_lb_velocity_);
            register_input("/dart/first_right_friction/velocity", friction_rb_velocity_);
            register_input("/dart/second_right_friction/velocity", friction_rf_velocity_);

            publisher_1_ = this->create_publisher<std_msgs::msg::String>("msg_friction_lf_current_velocity_", 10);
            publisher_2_ = this->create_publisher<std_msgs::msg::String>("msg_friction_lb_current_velocity_", 10);
            publisher_3_ = this->create_publisher<std_msgs::msg::String>("msg_friction_rb_current_velocity_", 10);
            publisher_4_ = this->create_publisher<std_msgs::msg::String>("msg_friction_rf_current_velocity_", 10);
            timer_       = this->create_wall_timer(std::chrono::milliseconds(1), [this]() { this->publish_message(); });
        }

        if (camera_enable_) {
            register_input("/dart/vision/display_image", display_image_);
            display_thread_ = std::thread(&MessagePublisher::image_displayer, this);
        }
    }

    void update() override {
        if (friction_enable_) {
            msg_friction_lf_current_velocity_.data = std::to_string(*friction_lf_velocity_);
            msg_friction_lb_current_velocity_.data = std::to_string(*friction_lb_velocity_);
            msg_friction_rb_current_velocity_.data = std::to_string(*friction_rb_velocity_);
            msg_friction_rf_current_velocity_.data = std::to_string(*friction_rf_velocity_);
        }

        if (camera_enable_) {
            {
                std::lock_guard<std::mutex> lock(display_mutex_);
                lastest_image_ = *display_image_;
            }
        }
        if (camera_enable_ && cal_fps_enable_) {
            calc_fps();
        }
    }

private:
    void publish_message() {
        publisher_1_->publish(msg_friction_lf_current_velocity_);
        publisher_2_->publish(msg_friction_lb_current_velocity_);
        publisher_3_->publish(msg_friction_rb_current_velocity_);
        publisher_4_->publish(msg_friction_rf_current_velocity_);
    }

    void calc_fps() {
        auto time_now = std::chrono::steady_clock::now();

        long update_delta_time =
            std::chrono::duration_cast<std::chrono::microseconds>(time_now - update_last_time_point_).count();
        update_last_time_point_ = time_now;

        long update_fps = 1000000 / update_delta_time + 1;

        if (true) {
            RCLCPP_WARN(get_logger(), "update:%7.3ld", update_fps);
        }
    }
    std::chrono::steady_clock::time_point update_last_time_point_;

    void image_displayer() {
        while (!display_stop_flag_) {
            cv::Mat display;
            {
                std::lock_guard<std::mutex> lock(display_mutex_);
                if (!lastest_image_.empty()) {
                    display = lastest_image_;
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

    bool camera_enable_ = false, friction_enable_ = false;
    bool cal_fps_enable_ = false;
    InputInterface<cv::Mat> display_image_;
    std::thread display_thread_;
    std::atomic<bool> display_stop_flag_ = false;
    std::mutex display_mutex_;
    cv::Mat lastest_image_;

    InputInterface<double> friction_lf_velocity_;
    InputInterface<double> friction_lb_velocity_;
    InputInterface<double> friction_rb_velocity_;
    InputInterface<double> friction_rf_velocity_;

    std::thread publisher_thread_;
    std::atomic<bool> publisher_stop_flag_ = false;
    std::mutex publisher_mutex_;

    std_msgs::msg::String msg_friction_lf_current_velocity_;
    std_msgs::msg::String msg_friction_lb_current_velocity_;
    std_msgs::msg::String msg_friction_rb_current_velocity_;
    std_msgs::msg::String msg_friction_rf_current_velocity_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_1_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_2_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_3_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_4_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::MessagePublisher, rmcs_executor::Component)
