#include <glog/logging.h>
#include <boost/format.hpp>

#include "utils/eigen-yaml-serialization.h"
#include <opencv2/core/eigen.hpp> // could not find Eigen in this include file ??

#include "euroc_api.h"

namespace io
{

EUROC::EUROC(const std::string &root)
{
    dataset_path_ = root;

    /// read camera param
    auto read_cam_param = [](const std::string& path, cv::Mat& T_BS, cv::Mat& K, cv::Mat& D)->bool
    {
        try
        {
            YAML::Node cam0_sensor_param = YAML::LoadFile(path);
            cv::eigen2cv(cam0_sensor_param["T_BS"].as<Eigen::Matrix4d>(), T_BS);
            auto k_array = cam0_sensor_param["intrinsics"].as<std::vector<double>>();
            auto d_array = cam0_sensor_param["distortion_coefficients"].as<std::vector<double>>();
            K = cv::Mat::eye(3,3, CV_64F), D = cv::Mat::zeros(1,5, CV_64F);
            K.at<double>(0,0) = k_array[0];
            K.at<double>(1,1) = k_array[1];
            K.at<double>(0,2) = k_array[2];
            K.at<double>(1,2) = k_array[3];
            D.at<double>(0,0) = d_array[0];
            D.at<double>(0,1) = d_array[1];
            D.at<double>(0,2) = d_array[2];
            D.at<double>(0,3) = d_array[3];
            return true;
        }
        catch (...)
        {
            return false;
        }
    };

    const std::string cam0_sensor_param_path = root + "/cam0/sensor.yaml",
                      cam1_sensor_param_path = root + "/cam1/sensor.yaml";
    const std::string cam0_sensor_timestamp_path = root + "/cam0/data.csv",
                      cam1_sensor_timestamp_path = root + "/cam1/data.csv";

    cv::Size image_size(752, 480);
    cv::Mat cam0_K, cam0_D, cam1_K, cam1_D, cam0_t_bs, cam1_t_bs, t_right2left,
            R(3,3,CV_64F), T(3,1,CV_64F), R1, R2, P1, P2, Q;
    cv::Rect roi1,roi2;

    bool is_suss = read_cam_param(cam0_sensor_param_path,cam0_t_bs,cam0_K,cam0_D);
    CHECK_EQ(is_suss, true) << "Cannot not load parameters from " << cam0_sensor_param_path;

    is_suss = read_cam_param(cam1_sensor_param_path,cam1_t_bs,cam1_K,cam1_D);
    CHECK_EQ(is_suss, true) << "Cannot not load parameters from " << cam1_sensor_param_path;

    t_right2left = cam0_t_bs.inv() * cam1_t_bs;
    Eigen::Matrix4d eigen_t_right2left;
    cv::cv2eigen(t_right2left,eigen_t_right2left);

    R = t_right2left.rowRange(0, 3).colRange(0,3);
    T = t_right2left.rowRange(0, 3).colRange(3,4);

    cv::stereoRectify(cam0_K, cam0_D, cam1_K, cam1_D, image_size, R, T, R1, R2, P1, P2, Q,
                      cv::CALIB_ZERO_DISPARITY,1, image_size, &roi1, &roi2);

    cameras_.emplace_back(std::make_shared<Camera>(cam0_K.at<double>(0,0),cam0_K.at<double>(1,1),cam0_K.at<double>(0,2),
                                                   cam0_K.at<double>(1,2),SE3()));
    cameras_.emplace_back(std::make_shared<Camera>(cam1_K.at<double>(0,0),cam1_K.at<double>(1,1),cam1_K.at<double>(0,2),
                                                   cam1_K.at<double>(1,2),SE3(eigen_t_right2left)));

    cv::initUndistortRectifyMap(cam0_K, cam0_D, R1, P1, image_size, CV_16SC2,map11, map12);
    cv::initUndistortRectifyMap(cam1_K, cam1_D, R2, P2, image_size, CV_16SC2,map21, map22);


    // read timestamp
    auto read_timestamp = [&](const std::string& path, std::vector<std::pair<size_t, std::string>>& out_vec)
    {
        std::ifstream times_context(path);
        if (!times_context)
        {
            LOG(ERROR) << "cannot load 'times.txt' from " << path;
            return;
        }
        std::string tmp;
        std::getline(times_context, tmp); // skip the first line
        while (!times_context.eof())
        {
            times_context >> tmp;

            size_t timestamp_ns;
            char file_name[32];

            std::sscanf(tmp.data(), "%ld,%s", &timestamp_ns, file_name);
            if (times_context.bad())
            {
                break;
            }
            out_vec.emplace_back(timestamp_ns, std::string(file_name));
        }
        times_context.close();
    };

    read_timestamp(cam0_sensor_timestamp_path,cam0_timestamp_map_);
    read_timestamp(cam1_sensor_timestamp_path,cam1_timestamp_map_);

    while(cam0_timestamp_map_.front().first != cam1_timestamp_map_.front().first)
    {
        if (cam0_timestamp_map_.front().first < cam1_timestamp_map_.front().first)
        {
            cam0_timestamp_map_.erase(cam0_timestamp_map_.begin());
        }
        else
        {
            cam1_timestamp_map_.erase(cam1_timestamp_map_.begin());
        }
    }

    while(cam0_timestamp_map_.back().first != cam1_timestamp_map_.back().first)
    {
        if (cam0_timestamp_map_.back().first > cam1_timestamp_map_.back().first)
        {
            cam0_timestamp_map_.pop_back();
        }
        else
        {
            cam1_timestamp_map_.pop_back();
        }
    }

    CHECK_EQ(cam0_timestamp_map_.size(), cam1_timestamp_map_.size());
}

Frame::Ptr EUROC::NextFrame()
{
    if (current_frame_index_ >= cam0_timestamp_map_.size())
    {
        LOG(WARNING) << "Read to the end of current dataset, return empty frame";
        return nullptr;
    }
    static boost::format fmt("%s/cam%d/data/%s");
    auto cam0_item = cam0_timestamp_map_[current_frame_index_];
    auto cam1_item = cam1_timestamp_map_[current_frame_index_];
    auto left_image_path  = (fmt % dataset_path_ % 0 % cam0_item.second).str();
    auto right_image_path = (fmt % dataset_path_ % 1 % cam1_item.second).str();

    cv::Mat left_image  = cv::imread(left_image_path,cv::IMREAD_GRAYSCALE);
    cv::Mat right_image = cv::imread(right_image_path,cv::IMREAD_GRAYSCALE);

    CHECK(left_image.data  != nullptr) << "cannot load left image from " << left_image_path;
    CHECK(right_image.data != nullptr) << "cannot load right image from " << right_image_path;


    auto new_frame = Frame::CreateFrame();

    new_frame->time_stamp = (cam0_item.first + cam1_item.first)/2;
    cv::remap(left_image,  new_frame->left_image,  map11, map12, cv::INTER_LINEAR);
    cv::remap(right_image, new_frame->right_image, map21, map22, cv::INTER_LINEAR);

    ++current_frame_index_;

    return new_frame;
}

}