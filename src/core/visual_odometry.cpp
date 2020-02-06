#include "utils/config.h"
#include "core/feature_manager.h"
#include "core/visual_odometry.h"

namespace core
{
    VisualOdometry::VisualOdometry(const std::string& config_path)
    {
        if (!Config::LoadConfig(config_path))
        {
            return;
        }

        frontend_ = Frontend::Ptr (new Frontend);
        backend_  = Backend::Ptr(new Backend);
        viewer_ = Viewer::Ptr (new Viewer);
        map_ = Map::Ptr(new Map);

        frontend_->SetMap(map_);
        frontend_->SetBackend(backend_);
        frontend_->SetViewer(viewer_);

        backend_->SetMap(map_);

        viewer_->SetMap(map_);
    }

    void VisualOdometry::SetDataLoader(const io::DataLoader::Ptr &data_loader)
    {
        // enable data loader
        data_loader_ = data_loader;

        CHECK(frontend_!= nullptr);
        CHECK(backend_ != nullptr);

        auto cameras = data_loader_->GetCameraModel();

        // set FeatureManager first
        FeatureManager::Get().SetCamera(cameras[0], cameras[1]);

        frontend_->SetCameras(cameras[0], cameras[1]);
        backend_->SetCameras(cameras[0], cameras[1]);

    }

    bool VisualOdometry::Step()
    {
        CHECK(data_loader_!= nullptr);

        Frame::Ptr new_frame = data_loader_->NextFrame();

        static size_t last_timestamp = new_frame->time_stamp;
        static size_t avg_vo_cost_time = 5e7; // 0.05s

        // simulating time gap
         std::this_thread::sleep_for(std::chrono::nanoseconds(new_frame->time_stamp - last_timestamp - avg_vo_cost_time));
         last_timestamp = new_frame->time_stamp;
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));

        CHECK(new_frame!= nullptr) << "cannot retrieve an valid frame from DataLoader";

        auto t1 = std::chrono::steady_clock::now();
        bool success = frontend_->AddFrame(new_frame);
        auto t2 = std::chrono::steady_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

        LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
        return success;
    }

    void VisualOdometry::Run()
    {
        /// to make mac happy, the Main thread is for Pangolin display.
        computing_thread_ = std::thread([&]()
        {
            run_computing_ = true;
            while (true)
            {
                LOG(INFO) << "VO is running";
                if (!Step())    break;
                if (!run_computing_) break;
            }

            backend_->Stop();

        });

        viewer_->Run();
        run_computing_ = false; // stop computing when close viewer (for now).
        computing_thread_.join();

        LOG(INFO) << "VO exit";
    }
}

