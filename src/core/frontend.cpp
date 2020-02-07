#include <opencv2/opencv.hpp>

#include "core/backend.h"
#include "core/feature.h"
#include "core/frontend.h"
#include "core/g2o_types.h"
#include "core/map.h"
#include "core/feature_manager.h"

#include "utils/config.h"

namespace core {

Frontend::Frontend() 
{
    num_features_tracking_good_ = Config::Get<int>("num_features_tracking_good_", 50);
    num_features_tracking_bad_ = Config::Get<int>("num_features_tracking_bad_", 30);
    num_features_init_ = Config::Get<int>("num_features_init", 100);
    num_features_needed_for_keyframe_ = Config::Get<int>("num_features_needed_for_keyframe", 80);
}

bool Frontend::AddFrame(Frame::Ptr frame) {
    current_frame_ = frame;

    switch (status_) {
        case FrontendStatus::INITIALIZING:
            StereoInit();
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            Track();
            break;
        case FrontendStatus::LOST:
            Reset();
            break;
    }

    last_frame_ = current_frame_;
    return true;
}

bool Frontend::Track()
{
    if (last_frame_) {
        current_frame_->SetPose(relative_motion_ * last_frame_->GetPose());
    }

    int num_track_last = FeatureManager::Get().TrackFrame(last_frame_, current_frame_);
    tracking_inliers_ = EstimateCurrentPose();

    if (tracking_inliers_ > num_features_tracking_good_) {
        // tracking good
        status_ = FrontendStatus::TRACKING_GOOD;
    } else if (tracking_inliers_ > num_features_tracking_bad_) {
        // tracking bad
        status_ = FrontendStatus::TRACKING_BAD;
    } else {
        // lost
        status_ = FrontendStatus::LOST;
    }

    InsertKeyframe();
    relative_motion_ = current_frame_->GetPose() * last_frame_->GetPose().inverse();

    if (viewer_) viewer_->AddCurrentFrame(current_frame_);
    return true;
}

bool Frontend::InsertKeyframe()
{
    if (tracking_inliers_ >= num_features_needed_for_keyframe_) {
        // still have enough features, don't insert keyframe
        return false;
    }
    // current frame is a new keyframe
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);

    LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
              << current_frame_->keyframe_id_;

    SetObservationsForKeyFrame();
    FeatureManager::Get().DetectFeatures(current_frame_);  // detect new features

    // track in right image
    FeatureManager::Get().TrackFrame(current_frame_, current_frame_);
    // triangulate map points
    TriangulateNewPoints();
    // update backend because we have a new keyframe
    backend_->UpdateMap();

    if (viewer_) viewer_->UpdateMap();

    return true;
}

void Frontend::SetObservationsForKeyFrame() {
    for (auto &feat : current_frame_->left_feature) {
        auto mp = feat->map_point.lock();
        if (mp) mp->AddObservation(feat);
    }
}

int Frontend::TriangulateNewPoints() {
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    SE3 current_pose_Twc = current_frame_->GetPose().inverse();
    int cnt_triangulated_pts = 0;
    for (size_t i = 0; i < current_frame_->left_feature.size(); ++i) {
        if (current_frame_->left_feature[i]->map_point.expired() &&
            current_frame_->right_feature[i] != nullptr) {
            // 左图的特征点未关联地图点且存在右图匹配点，尝试三角化
            std::vector<Vec3> points{
                camera_left_->pixel2camera(
                    Vec2(current_frame_->left_feature[i]->position.pt.x,
                         current_frame_->left_feature[i]->position.pt.y)),
                camera_right_->pixel2camera(
                    Vec2(current_frame_->right_feature[i]->position.pt.x,
                         current_frame_->right_feature[i]->position.pt.y))};
            Vec3 pworld = Vec3::Zero();

            if (Triangulation(poses, points, pworld) && pworld[2] > 0) {
                auto new_map_point = MapPoint::CreateNewMappoint();
                pworld = current_pose_Twc * pworld;
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(
                    current_frame_->left_feature[i]);
                new_map_point->AddObservation(
                    current_frame_->right_feature[i]);

                current_frame_->left_feature[i]->map_point = new_map_point;
                current_frame_->right_feature[i]->map_point = new_map_point;
                map_->InsertMapPoint(new_map_point);
                cnt_triangulated_pts++;
            }
        }
    }
    LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
    return cnt_triangulated_pts;
}

int Frontend::EstimateCurrentPose()
{
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // vertex
    VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
    vertex_pose->setId(0);
    vertex_pose->setEstimate(current_frame_->GetPose());
    optimizer.addVertex(vertex_pose);

    // K
    Mat33 K = camera_left_->K();

    // edges
    int index = 1;
    std::vector<EdgeProjectionPoseOnly *> edges;
    std::vector<Feature::Ptr> features;
    for (const auto& feat: current_frame_->left_feature)
    {
        auto mp = feat->map_point.lock();
        if (mp) 
        {
            features.push_back(feat);
            EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->pos_, K);
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement({feat->position.pt.x, feat->position.pt.y});
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
        }
    }

    // estimate the Pose the determine the outliers
    const double chi2_th = 5.991;
    int cnt_outlier = 0;
    for (int iteration = 0; iteration < 4; ++iteration)
    {
        vertex_pose->setEstimate(current_frame_->GetPose());
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        cnt_outlier = 0;

        // count the outliers
        for (size_t i = 0; i < edges.size(); ++i) {
            auto e = edges[i];
            if (features[i]->is_outlier) {
                e->computeError();
            }
            if (e->chi2() > chi2_th) {
                features[i]->is_outlier = true;
                e->setLevel(1);
                cnt_outlier++;
            } else {
                features[i]->is_outlier = false;
                e->setLevel(0);
            };

            if (iteration == 2) {
                e->setRobustKernel(nullptr);
            }
        }
    }

    LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
              << features.size() - cnt_outlier;
    // Set pose and outlier
    current_frame_->SetPose(vertex_pose->estimate());

    LOG(INFO) << "Current Pose = \n" << current_frame_->GetPose().matrix();

    for (auto &feat : features) {
        if (feat->is_outlier) {
            feat->map_point.reset();
            feat->is_outlier = false;  // maybe we can still use it in future
        }
    }
    return features.size() - cnt_outlier;
}

bool Frontend::StereoInit()
{
    int num_features_left = FeatureManager::Get().DetectFeatures(current_frame_);
    int num_coor_features = FeatureManager::Get().TrackFrame(current_frame_, current_frame_);
    if (num_coor_features < num_features_init_) {
        return false;
    }

    bool build_map_success = BuildInitMap();
    if (build_map_success) {
        status_ = FrontendStatus::TRACKING_GOOD;
        if (viewer_) {
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
        return true;
    }
    return false;
}

bool Frontend::Triangulation(const std::vector<SE3>&  poses,
     const std::vector<Vec3>& points, Vec3& pt_world)
{
    MatXX A(2 * poses.size(), 4);
    VecX  b(2 * poses.size());
    b.setZero();
    for (size_t i = 0; i < poses.size(); ++i)
    {
        Mat34 m = poses[i].matrix3x4();
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2)
    {
        return true;
    }
    return false;
}

bool Frontend::BuildInitMap()
{
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    size_t cnt_init_landmarks = 0;
    for (size_t i = 0; i < current_frame_->left_feature.size(); ++i) {
        if (current_frame_->right_feature[i] == nullptr) continue;
        // create map point from triangulation
        std::vector<Vec3> points{
            camera_left_->pixel2camera(
                Vec2(current_frame_->left_feature[i]->position.pt.x,
                     current_frame_->left_feature[i]->position.pt.y)),
            camera_right_->pixel2camera(
                Vec2(current_frame_->right_feature[i]->position.pt.x,
                     current_frame_->right_feature[i]->position.pt.y))};
        Vec3 pworld = Vec3::Zero();

        if (Triangulation(poses, points, pworld) && pworld[2] > 0)
        {
            auto new_map_point = MapPoint::CreateNewMappoint();
            new_map_point->SetPos(pworld);
            new_map_point->AddObservation(current_frame_->left_feature[i]);
            new_map_point->AddObservation(current_frame_->right_feature[i]);
            current_frame_->left_feature[i]->map_point = new_map_point;
            current_frame_->right_feature[i]->map_point = new_map_point;
            cnt_init_landmarks++;
            map_->InsertMapPoint(new_map_point);
        }
    }
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);
    backend_->UpdateMap();

    LOG(INFO) << "Initial map created with " << cnt_init_landmarks
              << " map points";

    return true;
}

bool Frontend::Reset()
{
    status_ = FrontendStatus::INITIALIZING;
    relative_motion_ = SE3();
    tracking_inliers_ = 0;
    map_->ClearMap();

    return true;
}

}  // namespace core