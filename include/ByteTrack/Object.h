#pragma once

#include <opencv2/opencv.hpp>

namespace byte_track
{
struct Object
{
    cv::Rect2f rect;
    int label;
    float prob;

    Object(const cv::Rect2f &_rect,
           const int &_label,
           const float &_prob);
};
}