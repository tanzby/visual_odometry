#pragma once

#include "core/camera_model.h"
#include "core/common_include.h"

namespace core {

struct MapPoint;
struct Feature;

struct Frame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long id_ = 0;
    unsigned long keyframe_id_ = 0;
    unsigned long time_stamp  = 0;

    bool is_keyframe_ = false;
    SE3 pose_;
    std::mutex pose_mutex_;
    cv::Mat left_image, right_image;

    // extracted features in left image
    std::vector<std::shared_ptr<Feature>> left_feature;
    // corresponding features in right image, set to nullptr if no corresponding
    std::vector<std::shared_ptr<Feature>> right_feature;

public:

    Frame() {}

    Frame(long id, unsigned long time_stamp, const SE3 &pose, const Mat &left,
          const Mat &right);

    // set and get pose, thread safe
    SE3 GetPose()
    {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }

    void SetPose(const SE3 &pose)
    {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }

    void SetKeyFrame();

    static Ptr CreateFrame();
};

}  // namespace core
