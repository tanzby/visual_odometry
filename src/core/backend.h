#pragma once

#include "core/common_include.h"
#include "core/frame.h"
#include "core/map.h"
#include "core/pose_graph.h"

namespace core
{

class Backend
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;

    Backend();

    ~Backend();

    void SetCameras(Camera::Ptr left, Camera::Ptr right)
    {
        cam_left_ = left;
        cam_right_ = right;
    }

    void SetMap(std::shared_ptr<Map> map) { map_ = map; }

    void UpdateMap();

    void Stop();

private:

    void BackendLoop();

    void SubMapOptimization();

    bool CheckLoopClosure(const Frame::Ptr& current_frame, unsigned long& from, unsigned long& to);

    void AddPoseGraph();

    void Optimize(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks);

    void SavePoseGraph();

    std::shared_ptr<PoseGraph> pose_graph_;

    std::shared_ptr<Map> map_;
    std::thread backend_thread_;
    std::thread loop_closure_thread_;
    std::mutex loop_mutex_;
    std::mutex data_mutex_;

    std::condition_variable map_update_;
    std::atomic<bool> backend_running_;

    Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
};

}  // namespace core