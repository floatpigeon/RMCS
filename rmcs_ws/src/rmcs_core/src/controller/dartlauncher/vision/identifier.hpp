#include <algorithm>
#include <cstddef>
#include <iostream>
#include <mutex>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <thread>
#include <vector>

namespace rmcs_core::controller::dartlauncher {

enum class LaunchStage {
    Disable,
    Detect,
    Track,
    Launch,
};

struct TargetData {
    cv::Point2d first_position;
    cv::Point2d latest_position;
    double max_move_dist;
    int catch_count;
    int miss_count;
};

class GuideLightIdentifier {
public:
    GuideLightIdentifier() { identify_thread_ = std::thread(&GuideLightIdentifier::idenitfy, this); }

    void load(const cv::Mat& src) {
        if (src.empty())
            return;

        std::lock_guard<std::mutex> lock(buffer_mtx_);
        latest_image_         = src;
        single_detect_enable_ = true;
    }

    void start_idenitfy() {
        std::lock_guard<std::mutex> lock(buffer_mtx_);
        launch_stage_ = LaunchStage::Detect;
    }

    cv::Mat get_display_image() {
        if (latest_display_image_.empty()) {
            cv::Mat empty(100, 100, CV_8UC3, cv::Scalar(0, 0, 0));
            return empty;
        }

        std::lock_guard<std::mutex> lock(buffer_mtx_);
        return latest_display_image_;
    }

    cv::Point get_target() { return target_initial_position; }

private:
    void idenitfy() {
        while (true) {
            if (launch_stage_ == LaunchStage::Disable) {
                detect_frame_count_ = 0;
                target_data_collection_.clear();
                continue;
            }

            bool enable_flag;
            {
                std::lock_guard<std::mutex> lock(buffer_mtx_);
                enable_flag = single_detect_enable_;
            }

            if (!enable_flag)
                continue;

            cv::Mat input;
            {
                std::lock_guard<std::mutex> lock(buffer_mtx_);
                input = latest_image_;
            }

            if (launch_stage_ == LaunchStage::Detect) {
                cv::Mat display_image;
                std::vector<cv::Point2d> possible_targets = image_process(input, cv::COLOR_RGB2HSV, display_image);
                single_detect_enable_                     = false;

                size_t confirmed_id = -1;
                if (detect_frame_count_ < 150) {
                    static_target_filter(possible_targets, target_data_collection_);
                    detect_frame_count_++;
                    continue;
                } else {
                    double max_score = 0;
                    if (target_data_collection_.empty()) {
                        // TODO:未识别到任何东西的特殊处理
                        // TODO:扩大筛选参数重新检测，做内录方便调出查看
                        launch_stage_       = LaunchStage::Detect;
                        detect_frame_count_ = 0;
                        std::cout << "no target,try again" << std::endl;

                        continue;
                    }

                    for (size_t i = 0; i < target_data_collection_.size(); ++i) {
                        double this_score = target_data_collection_[i].catch_count
                                          + 1000 / (target_data_collection_[i].max_move_dist + 1);

                        if (this_score > max_score) {
                            confirmed_id = i;
                            max_score    = this_score;
                        }
                    }
                }

                {
                    std::lock_guard<std::mutex> lock(buffer_mtx_);
                    target_initial_position = target_data_collection_[confirmed_id].latest_position;
                    launch_stage_           = LaunchStage::Track;
                    continue;
                }
            } else if (launch_stage_ == LaunchStage::Track) {
                // 跟踪器启动
                std::cout << "tracking" << std::endl;
            }
        }
    }

    static std::vector<cv::Point2d> image_process(const cv::Mat& input, int code, cv::Mat& output) {
        cv::Mat color_mask;
        output = input;
        cv::cvtColor(input, color_mask, code);
        cv::Scalar lower_limit;
        cv::Scalar upper_limit;

        switch (code) {
        case cv::COLOR_RGB2HLS:
            lower_limit = cv::Scalar(40, 20, 0);
            upper_limit = cv::Scalar(70, 128, 255);
            break;

        case cv::COLOR_RGB2HSV:
            lower_limit = cv::Scalar(90, 180, 200);
            upper_limit = cv::Scalar(100, 220, 255);
            break;

        default: break;
        }

        cv::Mat binary;
        cv::inRange(color_mask, lower_limit, upper_limit, binary);

        static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
        cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(binary, circles, cv::HOUGH_GRADIENT, 1.0, binary.rows / 10.0, 50, 30, 6, 100);

        std::vector<cv::Point2d> possible_targets;
        for (const auto& circle : circles) {
            cv::Point2d center(cvRound(circle[0]), cvRound(circle[1]));
            int radius = cvRound(circle[2]);
            cv::circle(output, center, radius, cv::Scalar(255, 0, 255), 3);
            possible_targets.emplace_back(center);
        }

        // cv::imshow("binary", binary);
        // cv::imshow("output", output);
        // cv::waitKey(1);
        return possible_targets;
    }

    static void static_target_filter(const std::vector<cv::Point2d>& points, std::vector<TargetData>& collection) {
        const int distance_threshold = 10;

        std::vector<bool> matched(points.size(), false);

        for (auto& collected : collection) {
            double min_dist = std::numeric_limits<double>::max();
            int point_id    = -1;

            for (size_t i = 0; i < points.size(); i++) {
                if (matched[i])
                    continue;

                const cv::Point2d& current_pt(points[i]);
                double dist = cv::norm(current_pt - collected.latest_position);

                if (dist < distance_threshold && dist < min_dist) {
                    min_dist = dist;
                    point_id = static_cast<int>(i);
                }
            }

            if (point_id != -1) {
                double this_move_dist     = cv::norm(collected.first_position - points[point_id]);
                collected.max_move_dist   = std::max(collected.max_move_dist, this_move_dist);
                collected.latest_position = points[point_id];
                collected.catch_count++;
                collected.miss_count = 0;
                matched[point_id]    = true;
            } else {
                collected.miss_count++;
            }
        }

        for (size_t i = 0; i < points.size(); i++) {
            if (!matched[i]) {
                TargetData td(points[i], points[i], 0, 1, 0);
                collection.emplace_back(td);
            }
        }

        collection.erase(
            std::remove_if(
                collection.begin(), collection.end(),
                [](const TargetData& target) { return (target.miss_count > 20); }),
            collection.end());

        // std::cout << "points:" << points.size() << ",collection:" << collection.size() << std::endl;
    }

    std::vector<TargetData> target_data_collection_;

    std::thread identify_thread_;
    std::mutex buffer_mtx_;

    cv::Mat latest_image_;
    bool single_detect_enable_ = false;
    LaunchStage launch_stage_  = LaunchStage::Disable;
    int detect_frame_count_    = -1;

    cv::Mat latest_display_image_;
    cv::Point target_initial_position;
};
} // namespace rmcs_core::controller::dartlauncher