#include <cv_bridge/cv_bridge.hpp>
#include <hikcamera/image_capturer.hpp>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>

namespace rmcs_examples {

class GetCameraFrame
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GetCameraFrame()
        : Node(get_component_name(), rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        camera_profile_.invert_image = get_parameter("invert_image").as_bool();
        camera_profile_.exposure_time = std::chrono::microseconds(get_parameter("exposure_time").as_int());
        camera_profile_.gain = static_cast<float>(get_parameter("gain").as_double());

        image_type_ = get_parameter("image_type").as_string();

        image_capturer_ = std::make_unique<hikcamera::ImageCapturer>(camera_profile_);

        image_publisher_ =
            this->create_publisher<sensor_msgs::msg::Image>(get_parameter("image_topic_name").as_string(), 1000);

        camera_thread_ = std::thread(&GetCameraFrame::camera_frame_update, this);
    }

    void update() override {
        // 注意：相机帧率和rmcs更新频率不同，image_capturer_->read()这一句不应该写在这里，会阻塞
        // 所以自瞄有一个自己的线程
    }

private:
    void camera_frame_update() {
        while (true) {
            cv::Mat camera_frame = image_capturer_->read();

            cv::imshow("test", camera_frame);
            cv::waitKey(1);

            // 正常来说，我们不会在这里直接使用imshow()去显示图像
            // 不过调试的话，暂时没什么问题
            // 更好的做法是使用publisher发布图像信息，然后使用foxglove或者rviz显示出来，如下示例

            sensor_msgs::msg::Image message;
            cv_bridge::CvImage cv_image(std_msgs::msg::Header(), image_type_, camera_frame);
            message = *cv_image.toImageMsg();
            image_publisher_->publish(message);
        }
    }
    rclcpp::Logger logger_;
    hikcamera::ImageCapturer::CameraProfile camera_profile_;
    std::unique_ptr<hikcamera::ImageCapturer> image_capturer_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    std::string image_type_; // 因为彩色图和灰度图编码不一样，需要自行设置，一般是rgb8和mono8

    std::thread camera_thread_;
};

} // namespace rmcs_examples

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_examples::GetCameraFrame, rmcs_executor::Component)