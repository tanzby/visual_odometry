#pragma once

#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <vector>

#include "core/frame.h"
#include "core/camera_model.h"

namespace io
{
    using Frame = core::Frame;
    using Camera = core::Camera;

    class DataLoader
    {
    protected:

        std::string dataset_path_;

        std::vector<Camera::Ptr> cameras_;

        size_t current_frame_index_ = 0;

        // rectified parameter
        const double resize_scale_ = 0.5;

    public:
        typedef std::shared_ptr<DataLoader> Ptr;

        inline double GetResizeScale()
        {
            return resize_scale_;
        }

        /**
         * @brief Reset for 'NextFrame()'
         */
        void Reset()
        {
            LOG(INFO) << "reset frame index counter";
            current_frame_index_ = 0;
        }

        /**
         * @brief Return camera models
         */
        std::vector<Camera::Ptr> GetCameraModel()
        {
            CHECK_GT(cameras_.size(), 0) << "the parameters have not been loaded successfully";
            return cameras_;
        }

        /**
         * @brief Get next frame automatically
         */
        virtual Frame::Ptr NextFrame() = 0;

    };
}

