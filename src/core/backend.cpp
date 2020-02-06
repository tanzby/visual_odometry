#include <boost/format.hpp>
#include "core/backend.h"
#include "core/feature.h"
#include "core/g2o_types.h"
#include "core/map.h"
#include "core/mappoint.h"
#include "core/feature_manager.h"

#include "utils/config.h"

namespace core {

Backend::Backend()
{
    backend_running_.store(true);
    backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));

    pose_graph_ = std::make_shared<PoseGraph>();

}

Backend::~Backend()
{
    SavePoseGraph();
}

void Backend::UpdateMap()
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    map_update_.notify_one();
}

void Backend::Stop()
{
    backend_running_.store(false);
    map_update_.notify_one();
    backend_thread_.join();
}

void Backend::BackendLoop()
{
    while (backend_running_.load())
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.wait(lock);

        SubMapOptimization();

        AddPoseGraph();
    }
}

void Backend::SubMapOptimization()
{
    Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
    Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
    Optimize(active_kfs, active_landmarks);
}

bool Backend::CheckLoopClosure(const Frame::Ptr& current_frame, unsigned long& from, unsigned long& to)
{
    // cv::Mat descriptor;
    // FeatureManager::Get().ComputeDescriptor(current_frame, descriptor);

    return false;
}

void Backend::AddPoseGraph()
{
    static SE3 last_frame_pose = SE3();
    auto current_frame = map_->GetCurrentFrame();

    SE3 current_pose  = current_frame->GetPose().inverse();
    SE3 odom_relative_pose = last_frame_pose.inverse() * current_pose;

    Mat66 information = 10 * Mat66::Identity();

    pose_graph_->SetInitial(current_frame->keyframe_id_, current_pose);
    if (current_frame->keyframe_id_ > 0)
    {
        pose_graph_->AddEdge(current_frame->keyframe_id_-1,
                current_frame->keyframe_id_, odom_relative_pose, information);
    }

    unsigned long from = 0, to = 0;
    if(CheckLoopClosure(current_frame, from, to))
    {
        // TODO estimate from's and to's relative pose
        SE3 relative_pose = SE3();
        pose_graph_->AddEdge(from, to, relative_pose, information);
        loop_closure_thread_ = std::thread([&]()
        {
            loop_mutex_.lock();
            LOG(INFO) << "loop closure optimizing.";
            pose_graph_->Optimize(10);
            loop_mutex_.unlock();
        });
        loop_closure_thread_.detach();
    }

    last_frame_pose = current_pose;
}

void Backend::SavePoseGraph()
{
    LOG(INFO) << "saving PoseGraph";
    std::unique_lock<std::mutex> lock(loop_mutex_);
    pose_graph_->Save("test_vo.g2o");
}

void Backend::Optimize(Map::KeyframesType &keyframes,
                       Map::LandmarksType &landmarks)
{
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
        LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    std::map<unsigned long, VertexPose *> vertices;
    unsigned long max_kf_id = 0;
    for (auto &keyframe : keyframes)
    {
        auto kf = keyframe.second;
        VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
        vertex_pose->setId(kf->keyframe_id_);
        vertex_pose->setEstimate(kf->GetPose());
        optimizer.addVertex(vertex_pose);
        if (kf->keyframe_id_ > max_kf_id)
        {
            max_kf_id = kf->keyframe_id_;
        }

        vertices.insert({kf->keyframe_id_, vertex_pose});
    }

    std::unordered_map<unsigned long, VertexXYZ *> vertices_landmarks;

    // K 和左右外参
    Mat33 K = cam_left_->K();
    SE3 left_ext = cam_left_->pose();
    SE3 right_ext = cam_right_->pose();

    // edges
    int index = 1;
    double chi2_th = 5.991;  // robust kernel 阈值
    std::map<EdgeProjection *, Feature::Ptr> edges_and_features;

    for (auto &landmark : landmarks) {
        if (landmark.second->is_outlier_) continue;
        unsigned long landmark_id = landmark.second->id_;
        auto observations = landmark.second->GetObs();
        for (auto &obs : observations) {
            if (obs.lock() == nullptr) continue;
            auto feat = obs.lock();
            if (feat->is_outlier || feat->frame.lock() == nullptr) continue;

            auto frame = feat->frame.lock();
            EdgeProjection *edge = nullptr;
            if (feat->is_on_left_image) {
                edge = new EdgeProjection(K, left_ext);
            } else {
                edge = new EdgeProjection(K, right_ext);
            }

            // 如果landmark还没有被加入优化，则新加一个顶点
            if (vertices_landmarks.find(landmark_id) ==
                vertices_landmarks.end()) {
                VertexXYZ *v = new VertexXYZ;
                v->setEstimate(landmark.second->GetPosition());
                v->setId(landmark_id + max_kf_id + 1);
                v->setMarginalized(true);
                vertices_landmarks.insert({landmark_id, v});
                optimizer.addVertex(v);
            }

            edge->setId(index);
            edge->setVertex(0, vertices.at(frame->keyframe_id_));    // pose
            edge->setVertex(1, vertices_landmarks.at(landmark_id));  // landmark
            edge->setMeasurement({feat->position.pt.x, feat->position.pt.y});
            edge->setInformation(Mat22::Identity());
            auto rk = new g2o::RobustKernelHuber();
            rk->setDelta(chi2_th);
            edge->setRobustKernel(rk);
            edges_and_features.insert({edge, feat});

            optimizer.addEdge(edge);

            index++;
        }
    }

    // do optimization and eliminate the outliers
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    int cnt_outlier = 0, cnt_inlier = 0;
    int iteration = 0;
    while (iteration < 5)
    {
        cnt_outlier = 0;
        cnt_inlier = 0;
        // determine if we want to adjust the outlier threshold
        for (auto &ef : edges_and_features) {
            if (ef.first->chi2() > chi2_th) {
                cnt_outlier++;
            } else {
                cnt_inlier++;
            }
        }
        double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
        if (inlier_ratio > 0.5) {
            break;
        } else {
            chi2_th *= 2;
            iteration++;
        }
    }

    for (auto &ef : edges_and_features) {
        if (ef.first->chi2() > chi2_th) {
            ef.second->is_outlier = true;
            // remove the observation
            ef.second->map_point.lock()->RemoveObservation(ef.second);
        } else {
            ef.second->is_outlier = false;
        }
    }

    LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << "/"
              << cnt_inlier;

    // Set pose and lanrmark position
    for (auto &v : vertices) {
        keyframes.at(v.first)->SetPose(v.second->estimate());
    }
    for (auto &v : vertices_landmarks) {
        landmarks.at(v.first)->SetPos(v.second->estimate());
    }
}

}  // namespace core