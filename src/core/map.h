#pragma once

#include <utility>

#include "core/common_include.h"
#include "core/feature.h"
#include "core/frame.h"
#include "core/mappoint.h"

namespace core {

class Map
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef std::shared_ptr<Map> Ptr;

    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
    typedef std::unordered_map<unsigned long, Frame::Ptr>    KeyframesType;

    typedef std::function<void(Frame::Ptr keyframe)>         InsertKeyframeCbFunc;

    Map() {}

    void InsertKeyFrame(Frame::Ptr frame);

    void InsertMapPoint(MapPoint::Ptr map_point);

    LandmarksType GetAllMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return landmarks_;
    }

    KeyframesType GetAllKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return keyframes_;
    }

    LandmarksType GetActiveMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_landmarks_;
    }

    KeyframesType GetActiveKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }

    Frame::Ptr GetCurrentFrame(){ return current_frame_; }

    void CleanMap();

    void ClearMap();

private:

    void RemoveOldKeyframe();

    std::mutex data_mutex_;
    LandmarksType landmarks_;         // all landmarks
    LandmarksType active_landmarks_;  // active landmarks
    KeyframesType keyframes_;         // all key-frames
    KeyframesType active_keyframes_;  // all key-frames

    Frame::Ptr current_frame_ = nullptr;

    // settings
    int num_active_keyframes_ = 7;

};
}  // namespace core
