#pragma once

#include <condition_variable>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>
namespace rmcs_core::controller::dartlauncher {

struct TargetData {
    cv::Point2d first_position;
    cv::Point2d latest_position;
    double max_move_dist;
    int catch_count;
    int miss_count;
};

class Identifier {
public:
    Identifier();
    ~Identifier();

    void start() {
        if (is_running_)
            return;

        std::lock_guard<std::mutex> lock(thread_mtx_);
        is_running_         = true;
        stop_command_       = false;
        result_ready_       = false;
        detect_frame_count_ = 0;
        identifier_thread_  = std::thread(&Identifier::update, this);
    }

    void stop() {
        if (!is_running_)
            return;
        {
            std::lock_guard<std::mutex> lock(thread_mtx_);
            stop_command_ = true;
            cv_.notify_all(); // 唤醒可能等待的线程
        }

        if (identifier_thread_.joinable()) {
            identifier_thread_.join();
        }
        is_running_ = false;
        processing_ = false;
    }

    void load(cv::Mat& image) {
        if (!is_running_)
            return;

        std::lock_guard<std::mutex> lock(camera_image_mtx_);
        laetst_camera_image_ = image.clone();
        cv_.notify_one();     // 通知有新数据到达
        processing_ = true;
    }

    cv::Point get_initial_position() { return target_initial_position_; }

    cv::Mat get_display_image() {
        std::lock_guard<std::mutex> lock(display_image_mtx_);
        return display_image_;
    }

    bool is_running() const { return is_running_; }
    bool result_ready() const { return result_ready_; }

private:
    void update() {
        while (true) {
            if (result_ready_) {
                stop();
            }
            cv::Mat input;
            {
                std::unique_lock<std::mutex> lock_1(thread_mtx_);
                cv_.wait(lock_1, [this]() { return !laetst_camera_image_.empty() || stop_command_; });

                if (stop_command_)
                    break;

                {
                    std::lock_guard<std::mutex> lock_2(camera_image_mtx_);
                    input = laetst_camera_image_;
                }
            }

            if (!processing_)
                continue;
            // process
            cv::Mat display;
            size_t confirmed_id = -1;
            if (detect_frame_count_ < 150) {
                std::vector<cv::Point2d> possible_targets = image_process(input, cv::COLOR_RGB2HSV, display);
                target_filter(possible_targets, target_data_collection_);
                detect_frame_count_++;
                processing_ = false;
                {
                    std::lock_guard<std::mutex> lock(display_image_mtx_);
                    display_image_ = display;
                }
                continue;
            } else {
                double max_score = 0;
                if (target_data_collection_.empty()) {
                    // TODO:未识别到任何东西时的处理，比如更换方法重新筛选或者扩大范围动态适应，这里先进行一个重新检测进行调试
                    // TODO:内录，抓一点场上的视觉数据
                    detect_frame_count_ = 0;
                    std::cout << "no target,try again" << std::endl;
                    continue;
                }
                for (size_t i = 0; i < target_data_collection_.size(); ++i) {
                    double this_score =
                        target_data_collection_[i].catch_count + 1000 / (target_data_collection_[i].max_move_dist + 1);

                    if (this_score > max_score) {
                        confirmed_id = i;
                        max_score    = this_score;
                    }
                }
            }

            {
                std::lock_guard<std::mutex> lock(camera_image_mtx_);
                target_initial_position_ = target_data_collection_[confirmed_id].latest_position;
                result_ready_            = true;
                continue;
            }
        }
    }

    std::thread identifier_thread_;
    std::atomic<bool> is_running_, stop_command_, processing_, result_ready_;
    std::condition_variable cv_;

    std::mutex thread_mtx_, camera_image_mtx_, display_image_mtx_;
    cv::Mat laetst_camera_image_;
    int detect_frame_count_;

    cv::Mat display_image_;
    std::vector<TargetData> target_data_collection_;
    cv::Point target_initial_position_;

    // Image Process Methods

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

        cv::imshow("binary", binary);
        cv::imshow("output", output);
        cv::waitKey(1);
        return possible_targets;
    }

    static void target_filter(const std::vector<cv::Point2d>& points, std::vector<TargetData>& collection) {
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
    }
};

} // namespace rmcs_core::controller::dartlauncher