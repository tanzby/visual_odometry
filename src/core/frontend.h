#pragma once

#include <opencv2/features2d.hpp>

#include "core/common_include.h"
#include "core/backend.h"
#include "core/frame.h"
#include "core/map.h"

#include "viewer/viewer.h"

namespace core
{
enum class FrontendStatus { INITIALIZING, TRACKING_GOOD, TRACKING_BAD, LOST };

class Frontend
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frontend> Ptr;

    Frontend();

    bool AddFrame(Frame::Ptr frame);

    void SetMap(Map::Ptr map) { map_ = map; }

    void SetBackend(Backend::Ptr backend) { backend_ = backend; }

    void SetViewer(Viewer::Ptr viewer) { viewer_ = viewer; }

    FrontendStatus GetStatus() const { return status_; }

    void SetCameras(Camera::Ptr left, Camera::Ptr right)
    {
        camera_left_ = left;
        camera_right_ = right;
    }

private:

    bool Track();

    bool Reset();

    int EstimateCurrentPose();

    bool InsertKeyframe();

    bool StereoInit();

    bool BuildInitMap();

    bool Triangulation(const std::vector<SE3>&  poses,
                       const std::vector<Vec3>& points, Vec3& pt_world);

    int TriangulateNewPoints();

    void SetObservationsForKeyFrame();

    // data
    FrontendStatus status_ = FrontendStatus::INITIALIZING;

    Frame::Ptr current_frame_ = nullptr;  // 当前帧
    Frame::Ptr last_frame_ = nullptr;     // 上一帧
    Camera::Ptr camera_left_ = nullptr;   // 左侧相机
    Camera::Ptr camera_right_ = nullptr;  // 右侧相机

    Map::Ptr map_ = nullptr;
    Backend::Ptr backend_ = nullptr;
    Viewer::Ptr  viewer_ = nullptr;

    SE3 relative_motion_;  // 当前帧与上一帧的相对运动，用于估计当前帧pose初值

    int tracking_inliers_ = 0;  // inliers, used for testing new keyframes

    // params
    int num_features_init_;
    int num_features_tracking_good_;
    int num_features_tracking_bad_;
    int num_features_needed_for_keyframe_;
};

}  // namespace core
