#pragma once

#include <vector>

#include "data_loader.h"

namespace io
{

class EUROC: public DataLoader
{
    std::vector<std::pair<size_t, std::string>> cam0_timestamp_map_, cam1_timestamp_map_;
    cv::Mat map11, map12, map21, map22;
    /**
     * @brief Loads the EUROC dataset from the `root` path.
     */
    explicit EUROC(const std::string& root);

public:

    EUROC() = delete;
    virtual ~EUROC(){}

    Frame::Ptr NextFrame() override;

    Ptr static Load(const std::string& root)
    {
        return std::shared_ptr<EUROC>(new EUROC(root));
    }
};

}