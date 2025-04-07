#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

namespace rmcs_core::controller::dartlauncher {
class GuideLightTracker {
public:
    GuideLightTracker()
        : isTracked(false)
        , lostFrames(0) {}

    void init(const cv::Mat& frame, const cv::Rect2d& bbox) {
        tracker = cv::TrackerKCF::create();
        tracker->init(frame, bbox);

        kf = cv::KalmanFilter(4, 2, 0);

        kf.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);

        cv::setIdentity(kf.measurementMatrix, cv::Scalar::all(1));
        cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-2));
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));

        kf.statePost = (cv::Mat_<float>(4, 1) << bbox.x + bbox.width / 2, bbox.y + bbox.height / 2, 0, 0);

        isTracked  = true;
        lostFrames = 0;
    }

    bool update(const cv::Mat& frame, cv::Rect2i& bbox) {
        if (!isTracked)
            return false;

        bool success = tracker->update(frame, bbox);

        if (success) {
            cv::Mat measurement = (cv::Mat_<float>(2, 1) << bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);

            cv::Mat prediction = kf.predict();
            cv::Mat estimated  = kf.correct(measurement);

            lostFrames = 0;
            return true;
        } else {
            if (++lostFrames > MAX_LOST_FRAMES) {
                isTracked = false;
                return false;
            }

            cv::Mat prediction = kf.predict();

            bbox.x = prediction.at<int>(0) - bbox.width / 2;
            bbox.y = prediction.at<int>(1) - bbox.height / 2;

            bbox.x = std::max(0, std::min(bbox.x, frame.cols - bbox.width));
            bbox.y = std::max(0, std::min(bbox.y, frame.rows - bbox.height));

            return false;
        }
    }

    bool isTracking() const { return isTracked; }

private:
    cv::Ptr<cv::Tracker> tracker;
    cv::KalmanFilter kf;
    bool isTracked;
    int lostFrames;
    const int MAX_LOST_FRAMES = 10;
};

void example() {
    cv::VideoCapture cap(0);
    cv::Mat frame;
    GuideLightTracker tracker;
    cv::Rect2i bbox(300, 200, 100, 100); // 初始位置（来自检测器）

    while (cap.read(frame)) {
        bool success = tracker.update(frame, bbox);

        // 绘制跟踪结果
        cv::rectangle(frame, bbox, success ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);

        // 显示状态
        cv::putText(
            frame,
            success                ? "Tracking"
            : tracker.isTracking() ? "Predicting"
                                   : "Lost",
            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        cv::imshow("Tracking", frame);

        if (cv::waitKey(1) == 27)
            break;
    }
}
} // namespace rmcs_core::controller::dartlauncher