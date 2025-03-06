#include "controller/dartlauncher/dart_resources.hpp"
#include <chrono>
#include <mutex>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <thread>
#include <vector>

namespace rmcs_core::controller::dartlauncher {

class TargetTracker
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    TargetTracker()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        register_input("/dart/vision/possible_points", input_possible_points_);
        register_input("/dart/vision/latest_processed_image_id", input_latest_processed_image_id_);
        register_input("/dart/master_control/vision_guide_command", input_dart_vision_guide_enable_, false);

        register_output("/dart/vision/latest_target_site", output_latest_target_site_, cv::Point(-1, -1));

        identify_thread_ = std::thread(&TargetTracker::identify_target, this);
    }

    void update() override {
        bool identify_enable, target_track_enable;
        if (begin_sign_ == -1 && *input_dart_vision_guide_enable_) {
            begin_sign_     = *input_latest_processed_image_id_ - 1;
            identify_enable = true;
        } else {
            begin_sign_ = -1;
            possible_targets_.clear();
            identify_enable     = false;
            target_track_enable = false;
        }

        {
            std::lock_guard<std::mutex> lock(buffer_mtx_);
            buffer_possible_points_ = *input_possible_points_;
            identify_work_flag_     = identify_enable;
            target_track_enable     = track_work_flag_;
        }

        if (target_track_enable) {
            target_tracking(the_target_);
            *output_latest_target_site_ = the_target_.latest_site;
        }
    }

private:
    void identify_target() {
        while (true) {
            bool identify_enable;
            std::vector<cv::Point> possible_points;
            {
                std::lock_guard<std::mutex> lock(buffer_mtx_);
                possible_points = buffer_possible_points_;
                identify_enable = identify_work_flag_;
            }

            if (!identify_enable || track_work_flag_) {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
                continue;
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

                {
                    std::lock_guard<std::mutex> lock(buffer_mtx_);
                    track_work_flag_ = true;
                    identify_enable  = false;
                }
            }
        }
    }

    void target_tracking(TargetData& target) {
        int target_and_value = target.latest_site.x - target.latest_site.y;
        for (const auto& point : buffer_possible_points_) {

            auto move_direction = (point.x - target.first_site.x) * (point.x - target.latest_site.x);
            if (abs(point.x + point.y - target_and_value) <= 10.0 && move_direction > 0) {
                target.latest_site = point;
                break;
            }
        }
        target.latest_site = cv::Point(-1, -1);
    }

    rclcpp::Logger logger_;
    int begin_sign_ = -1;

    std::mutex buffer_mtx_;
    std::thread identify_thread_;
    bool identify_work_flag_ = false;

    std::vector<cv::Point> buffer_possible_points_;
    std::vector<TargetData> possible_targets_;
    TargetData the_target_;
    bool track_work_flag_ = false;

    InputInterface<std::vector<cv::Point>> input_possible_points_;
    InputInterface<int> input_latest_processed_image_id_;
    InputInterface<bool> input_dart_vision_guide_enable_;

    OutputInterface<cv::Point> output_latest_target_site_;
};

} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::TargetTracker, rmcs_executor::Component)
