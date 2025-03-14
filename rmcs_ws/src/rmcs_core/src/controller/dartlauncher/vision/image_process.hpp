
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <stdexcept>
namespace rmcs_core::controller::dartlauncher {

class ImageProcess {
public:
    static cv::Mat preprocess(const cv::Mat& input, int code) {
        cv::Mat color_mask;
        cv::cvtColor(input, color_mask, code);
        cv::Scalar lower_limit;
        cv::Scalar upper_limit;

        switch (code) {
        case cv::COLOR_RGB2HLS:
            lower_limit = cv::Scalar(45, 16, 128);
            upper_limit = cv::Scalar(75, 172, 255);
            break;

        case cv::COLOR_RGB2HSV:
            lower_limit = cv::Scalar(40, 50, 200);
            upper_limit = cv::Scalar(50, 255, 255);
            break;

        case cv::COLOR_RGB2BGR:
            lower_limit = cv::Scalar(0, 180, 0);
            upper_limit = cv::Scalar(80, 255, 80);
            break;

        default: break;
        } // 需要多次的实验看看哪种方式稳定性更佳，目前看来RGB是完全用不了的，光线变化一大直接失效

        cv::Mat process;
        cv::inRange(color_mask, lower_limit, upper_limit, process);
        // cv::imshow("range", process);

        static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
        cv::morphologyEx(process, process, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(process, process, cv::MORPH_CLOSE, kernel);

        // cv::imshow("process", process);
        // cv::waitKey(1);

        return process;
    }

    static cv::Mat pre_process_beta(const cv::Mat& input, int code) {
        cv::Mat color_mask;
        cv::cvtColor(input, color_mask, code);
        cv::Scalar lower_limit;
        cv::Scalar upper_limit;

        switch (code) {
        case cv::COLOR_RGB2HLS:
            lower_limit = cv::Scalar(45, 16, 128);
            upper_limit = cv::Scalar(75, 172, 255);
            break;

        case cv::COLOR_RGB2HSV:
            lower_limit = cv::Scalar(40, 50, 200);
            upper_limit = cv::Scalar(60, 255, 255);
            break; // 参数不保真，还没测

        default: throw std::runtime_error("ImageProcess::pre_process: Unacceptable color code"); break;
        }

        cv::Mat process;
        cv::inRange(color_mask, lower_limit, upper_limit, process);

        for (int i = 0; i < 3; i++) {
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(i * 2 + 5, i * 2 + 5));
            cv::morphologyEx(process, process, cv::MORPH_OPEN, kernel);
            cv::morphologyEx(process, process, cv::MORPH_CLOSE, kernel);
        }
        return process;
    }

    static void first_filter_beta(cv::Mat& binary, cv::Mat& display, bool show = false) {
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(binary, circles, cv::HOUGH_GRADIENT, 1.0, binary.rows / 10.0, 50, 30, 6, 100);

        for (const auto& circle : circles) {
            cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
            int radius = cvRound(circle[2]);
            cv::circle(display, center, radius, cv::Scalar(0, 255, 0), 4);
            cv::circle(display, center, 3, cv::Scalar(0, 0, 255), 3);
        }

        if (show) {
            int rows      = display.rows;
            int half_cols = display.cols / 2;
            cv::line(display, cv::Point(half_cols, 0), cv::Point(half_cols, rows), cv::Scalar(255, 0, 255), 1);
            cv::imshow("display", display);
            cv::waitKey(1);
        }
    }

    static std::vector<cv::Point> first_filter(cv::Mat& process, cv::Mat& display, bool show = false) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(process, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::Point> possible_targets;
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area <= 100) {
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

        if (show) {
            int rows      = display.rows;
            int half_cols = display.cols / 2;
            cv::line(display, cv::Point(half_cols, 0), cv::Point(half_cols, rows), cv::Scalar(255, 0, 255), 1);
            cv::imshow("display", display);
            cv::waitKey(1);
        }

        return possible_targets;
    }
};
} // namespace rmcs_core::controller::dartlauncher