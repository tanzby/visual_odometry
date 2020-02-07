#include <glog/logging.h>
#include <boost/format.hpp>
#include <fstream>

#include "kitti_api.h"

namespace io
{

KITTI::KITTI(const std::string& dataset_path)
{
    dataset_path_ = dataset_path;

    // read camera param
    const std::string calib_path(dataset_path_ + "/calib.txt");
    std::ifstream calib_context(calib_path);
    if (!calib_context)
    {
        LOG(ERROR) << "cannot load 'calib.txt' from " << calib_path;
        return;
    }

    // P0 - P4
    for (int i = 0; i < 4; ++i)
    {
        std::string tmp; calib_context >> tmp;
        double projection_data[12];
        for (int k = 0; k < 12; ++k)
        {
            calib_context >> projection_data[k];
        }
        Eigen::Matrix3d K;
        K << projection_data[0], projection_data[1], projection_data[2],
             projection_data[4], projection_data[5], projection_data[6],
             projection_data[8], projection_data[9], projection_data[10];

        Eigen::Vector3d t;
        t << projection_data[3], projection_data[7], projection_data[11];
        t = K.inverse() * t;

        K = K * resize_scale_;

        Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1),
                K(0, 2), K(1, 2), SE3(SO3(), t)));
        cameras_.emplace_back(new_camera);

        LOG(INFO) << "Camera " << i << " extrinsic: " << t.transpose();
    }
    calib_context.close();

    // read time stamp
    const std::string times_path(dataset_path_ + "/times.txt");
    std::ifstream times_context(times_path);
    if (!times_context)
    {
        LOG(ERROR) << "cannot load 'times.txt' from " << times_path;
        return;
    }
    while (!times_context.eof())
    {
        double timestamp;
        times_context >> timestamp;
        if (times_context.bad())
        {
            break;
        }
        timestamp_map_.emplace_back(timestamp*1e9);
    }
    times_context.close();
    current_frame_index_ = 0;
}

Frame::Ptr KITTI::NextFrame()
{
    if (current_frame_index_ >= timestamp_map_.size())
    {
        LOG(WARNING) << "Read to the end of current dataset, return empty frame";
        return nullptr;
    }

    static boost::format fmt("%s/image_%d/%06d.png");

    cv::Mat left_image  = cv::imread((fmt % dataset_path_ % 0 % current_frame_index_).str(),
                       cv::IMREAD_GRAYSCALE);
    cv::Mat right_image = cv::imread((fmt % dataset_path_ % 1 % current_frame_index_).str(),
                       cv::IMREAD_GRAYSCALE);

    if (left_image.data == nullptr || right_image.data == nullptr) // may be end of dataset
    {
        LOG(WARNING) << "cannot read images if index " << current_frame_index_;
        return nullptr;
    }

    auto new_frame = Frame::CreateFrame();

    new_frame->time_stamp = timestamp_map_.at(current_frame_index_);
    cv::resize(left_image,  new_frame->left_image, cv::Size(), resize_scale_, resize_scale_, cv::INTER_NEAREST);
    cv::resize(right_image, new_frame->right_image, cv::Size(), resize_scale_, resize_scale_, cv::INTER_NEAREST);

    ++current_frame_index_;

    return new_frame;
}

}