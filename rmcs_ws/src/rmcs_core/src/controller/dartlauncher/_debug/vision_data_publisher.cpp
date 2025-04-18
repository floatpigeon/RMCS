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
        register_input("/dart/vidion/display_image", input_display_image_);
    }

    void update() override {
        msg_camera_image_.data  = *input_camera_image_;
        msg_display_image_.data = *input_display_image_;

        camera_image_publisher_->publish(msg_camera_image_);
        display_image_publisher_->publish(msg_display_image_);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;

    InputInterface<cv::Mat> input_camera_image_, input_display_image_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_image_publisher_, display_image_publisher_;
    sensor_msgs::msg::Image msg_camera_image_, msg_display_image_;
};
} // namespace rmcs_core::controller::dart