#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace rmcs_core::controller::dartlauncher {

struct TargetData {
    explicit TargetData(const cv::Point& init = cv::Point(-1, -1)) {
        round_count = 1;
        first_site  = init;
        latest_site = init;
    }

    int round_count;
    cv::Point first_site;
    cv::Point latest_site;
};

struct ImageData {
    cv::Mat image;
    int id;
};
} // namespace rmcs_core::controller::dartlauncher