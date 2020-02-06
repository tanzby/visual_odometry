#pragma once

#include "data_loader.h"

namespace io
{

/**
 * @brief KittiDataLoader for loading dataset. camera 0 represents
 * the left grayscale, 1 the right grayscale, 2 the left color and
 * 3 the right color camera.
 */
class KITTI: public DataLoader
{
    /**
     * @brief Loads the dataset from the `root` path.
     */
    explicit KITTI(const std::string& root);

    std::vector<size_t> timestamp_map_;

public:

    virtual ~KITTI(){}

    KITTI() = delete;

    Frame::Ptr NextFrame() override;

    Ptr static Load(const std::string& root)
    {
        return std::shared_ptr<KITTI>(new KITTI(root));
    }

};
}