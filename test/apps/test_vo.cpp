#include "core/visual_odometry.h"
#include "io/kitti_api.h"
#include "io/euroc_api.h"

int main(int argc, char** argv)
{
    FLAGS_logtostderr = true;
    google::InitGoogleLogging(argv[0]);

    auto dataset = io::KITTI::Load("/Users/iceytan/Downloads/00");
    // auto dataset = io::EUROC::Load("/Users/iceytan/Downloads/mav0");

    const std::string config_path = "/Users/iceytan/CLionProjects/vo/config/default.yaml";
    core::VisualOdometry vo(config_path);
    vo.SetDataLoader(dataset);
    vo.Run();

    return 0;
}