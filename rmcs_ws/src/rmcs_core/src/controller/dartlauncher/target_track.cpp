#include "controller/dartlauncher/dart_resources.hpp"
#include <mutex>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <vector>

namespace rmcs_core::controller::dartlauncher {

class TargetTracker
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    TargetTracker()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {}

    void update() override {
        {
            std::lock_guard<std::mutex> lock(buffer_mtx_);
            buffer_possible_points_ = *input_possible_points_;
        }

        if (*dart_vision_guide_enable_ == true) {
            begin_sign_ = *input_latest_processed_image_id_ - 1;
            // identify_target() 异步，待完善
        } else {
            begin_sign_ = -1;
        }

        if (tracker_ready_) {
            target_tracking(the_target_);
        }
    }

private:
    void identify_target() {
        std::vector<cv::Point> possible_points;
        {
            std::lock_guard<std::mutex> lock(buffer_mtx_);
            possible_points = buffer_possible_points_;
        }

        for (const auto& point : possible_points) {
            int value         = point.x + point.y;
            bool is_new_point = true;

            for (auto& target : possible_targets_) {
                if (is_new_point == true && abs(target.latest_site.x + target.latest_site.y - value) <= 10.0) {
                    target.round_count++;
                    target.latest_site = point;
                    is_new_point       = false;
                }
            }

            if (is_new_point) {
                possible_targets_.emplace_back(point);
            }
        }

        if (*input_latest_processed_image_id_ == begin_sign_) {
            for (const auto& target : possible_targets_) {
                if (target.round_count > the_target_.round_count)
                    the_target_ = target;
            }
        }

        tracker_ready_ = true;
    }

    void target_tracking(TargetData& target) {
        int target_and_value = target.latest_site.x - target.latest_site.y;
        for (const auto& point : buffer_possible_points_) {

            auto move_direction = (point.x - target.first_site.x) * (point.x - target.latest_site.x);
            if (abs(point.x + point.y - target_and_value) <= 10.0 && move_direction > 0) {
                target.latest_site = point;
            }
        }
        // TODO
    }

    rclcpp::Logger logger_;
    int begin_sign_ = -1;

    std::mutex buffer_mtx_;
    std::vector<cv::Point> buffer_possible_points_;
    std::vector<TargetData> possible_targets_;
    TargetData the_target_;
    bool tracker_ready_ = false;

    InputInterface<std::vector<cv::Point>> input_possible_points_;
    InputInterface<int> input_latest_processed_image_id_;

    InputInterface<bool> dart_vision_guide_enable_;
    OutputInterface<bool> yaw_align_ready_;
};

} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::TargetTracker, rmcs_executor::Component)