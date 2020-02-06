#pragma once


#include "core/common_include.h"
#include "core/frame.h"

#include <opencv2/opencv.hpp>
#include <DBoW3/DBoW3.h>

namespace core
{

    class FeatureManager
    {
        cv::Ptr<cv::Feature2D> detector_;

        Camera::Ptr camera_left_ = nullptr;   // 左侧相机
        Camera::Ptr camera_right_ = nullptr;  // 右侧相机

        FeatureManager();

        ~FeatureManager();

    public:

        FeatureManager(FeatureManager const&) = delete;

        FeatureManager& operator=(FeatureManager const&) = delete;

        static FeatureManager& Get()
        {
            static FeatureManager instance;
            return instance;
        }

        void SetCamera(const Camera::Ptr& left, const Camera::Ptr& right);

        int DetectFeatures(const Frame::Ptr& frame);

        void ComputeDescriptor(const Frame::Ptr& frame, cv::Mat& descriptor);

        int TrackFrame(const Frame::Ptr& prev_frame, const Frame::Ptr& next_frame);

    };

}
