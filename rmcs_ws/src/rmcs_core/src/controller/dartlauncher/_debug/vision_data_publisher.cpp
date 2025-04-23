#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rmcs_executor/component.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rmcs_core::controller::dart {

class VisionDataPublisher
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    VisionDataPublisher()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        //
        register_input("/dart/vision/camera_image", input_camera_image_);
        // register_input("/dart/vidion/display_image", input_display_image_);

        camera_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/dart_vision/camera_image", 10);
        // display_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/dart_vision/display_image", 10);
    }

    void update() override {

        cv_bridge::CvImage cv_image(std_msgs::msg::Header(), "bgr8", *input_camera_image_);
        msg_camera_image_ = *cv_image.toImageMsg();
        camera_image_publisher_->publish(msg_camera_image_);

        // msg_display_image_.data = *input_display_image_;
        // display_image_publisher_->publish(msg_display_image_);
    }

private:
    InputInterface<cv::Mat> input_camera_image_, input_display_image_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_image_publisher_, display_image_publisher_;
    sensor_msgs::msg::Image msg_camera_image_, msg_display_image_;
};
} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::VisionDataPublisher, rmcs_executor::Component)
