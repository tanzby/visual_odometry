#pragma once

#include <pangolin/pangolin.h>
#include <unordered_map>
#include <thread>

#include "core/map.h"

using namespace core;

class Viewer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer();

    void SetMap(Map::Ptr map) { map_ = map; }

    void Close();

    void AddCurrentFrame(Frame::Ptr current_frame);

    void UpdateMap();

    void Run();

private:

    void DrawFrame(Frame::Ptr frame, const float* color);

    void DrawMapPoints();

    void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

    /// plot the features in current frame into an image
    cv::Mat PlotFrameImage();

    Frame::Ptr current_frame_ = nullptr;
    Map::Ptr map_ = nullptr;

    bool viewer_running_ = true;

    std::unordered_map<unsigned long, Frame::Ptr>    draw_keyframes_;
    std::unordered_map<unsigned long, MapPoint::Ptr> draw_landmarks_;
    bool map_updated_ = false;

    std::mutex viewer_data_mutex_;
};
