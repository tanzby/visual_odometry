#include "core/feature_manager.h"
#include "core/feature.h"
#include "core/mappoint.h"
#include "utils/config.h"

namespace core
{
    FeatureManager::FeatureManager()
    {
        detector_ = cv::GFTTDetector::create(Config::Get<int>("num_features", 400), 0.01, 10);
    }

    FeatureManager::~FeatureManager()
    {

    }

    int FeatureManager::DetectFeatures(const core::Frame::Ptr &frame)
    {
        cv::Mat mask(frame->left_image.size(), CV_8UC1, 255);
        for (auto &feat : frame->left_feature) {
            cv::rectangle(mask, feat->position.pt - cv::Point2f(10, 10),
                          feat->position.pt + cv::Point2f(10, 10), 0, CV_FILLED);
        }

        std::vector<cv::KeyPoint> keypoints;
        detector_->detect(frame->left_image, keypoints, mask);
        int cnt_detected = 0;
        for (auto &kp : keypoints) {
            frame->left_feature.push_back(
                    Feature::Ptr(new Feature(frame, kp)));
            cnt_detected++;
        }

        LOG(INFO) << "Detect " << cnt_detected << " new features";
        return cnt_detected;
    }

    int FeatureManager::TrackFrame(const Frame::Ptr& prev_frame, const Frame::Ptr& next_frame)
    {
        const Mat& next_image = prev_frame==next_frame? next_frame->right_image : next_frame->left_image;
        const Camera::Ptr used_camera = prev_frame==next_frame? camera_right_ : camera_left_;

        // use LK flow to estimate points in the right image
        std::vector<cv::Point2f> kps_prev, kps_next;
        for (auto &kp : prev_frame->left_feature)
        {
            kps_prev.emplace_back(kp->position.pt);
            auto mp = kp->map_point.lock();
            if (mp)
            {
                auto px = used_camera->world2pixel(mp->pos_, next_frame->GetPose());
                kps_next.emplace_back(px[0], px[1]);
            }
            else
            {
                kps_next.emplace_back(kp->position.pt);
            }
        }

        std::vector<uchar> status;
        Mat error;
        cv::calcOpticalFlowPyrLK(prev_frame->left_image, next_image, kps_prev, kps_next, status, error,
                                 cv::Size(11, 11), 3,
                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,0.01),
                                 cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;

        for (size_t i = 0; i < status.size(); ++i)
        {
            if (status[i])
            {
                cv::KeyPoint kp(kps_next[i], 7);
                Feature::Ptr feature(new Feature(next_frame, kp));
                if (prev_frame == next_frame)
                {
                    feature->is_on_left_image = false;
                    next_frame->right_feature.emplace_back(feature);
                }
                else
                {
                    feature->map_point = prev_frame->left_feature[i]->map_point;
                    next_frame->left_feature.emplace_back(feature);
                }

                num_good_pts++;
            }
            else if (prev_frame == next_frame)
            {
                next_frame->right_feature.emplace_back(nullptr);
            }
        }

        LOG(INFO) << "Find " << num_good_pts << " in the next image.";
        return num_good_pts;
    }

    void FeatureManager::SetCamera(const Camera::Ptr& left, const Camera::Ptr& right)
    {
        camera_left_ = left;
        camera_right_ = right;
    }

    void FeatureManager::ComputeDescriptor(const Frame::Ptr& frame, cv::Mat& descriptor)
    {
        std::vector<cv::KeyPoint> kps;

        for (auto& feat: frame->left_feature)
        {
            kps.emplace_back(feat->position);
        }
        detector_->compute(frame->left_image, kps, descriptor);
    }

}


