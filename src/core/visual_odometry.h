#pragma once

#include <atomic>

#include "core/frontend.h"
#include "core/backend.h"
#include "io/data_loader.h"
#include "viewer/viewer.h"

namespace core
{

class VisualOdometry
{
    io::DataLoader::Ptr data_loader_ = nullptr;
    Frontend::Ptr frontend_          = nullptr;
    Backend::Ptr backend_            = nullptr;
    Viewer::Ptr viewer_              = nullptr;
    Map::Ptr map_                    = nullptr;

    std::thread computing_thread_;
    std::atomic_bool run_computing_ = false;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef std::shared_ptr<VisualOdometry> Ptr;

    explicit VisualOdometry(const std::string& config_path);

    void SetDataLoader(const io::DataLoader::Ptr& data_loader);

    bool Step();

    void Run();

};

}