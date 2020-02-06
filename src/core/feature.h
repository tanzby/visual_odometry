#pragma once


#include <memory>
#include <opencv2/features2d.hpp>
#include "core/common_include.h"

namespace core {

struct Frame;
struct MapPoint;


struct Feature
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    cv::KeyPoint position;

    std::weak_ptr<Frame> frame;
    std::weak_ptr<MapPoint> map_point;

    bool is_outlier = false;
    bool is_on_left_image = true;

public:
    Feature() {}

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp): frame(frame), position(kp) {}
};

}  // namespace core
