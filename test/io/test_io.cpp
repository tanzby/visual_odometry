#include <gtest/gtest.h>

#include "io/euroc_api.h"
#include "io/kitti_api.h"

TEST(EurocApiTest, ReadParam)
{
    auto dataset = EUROC::Load("/Users/iceytan/Downloads/mav0");
    auto current_frame = dataset->NextFrame();
    EXPECT_TRUE(current_frame!= nullptr);
    EXPECT_EQ(current_frame->left_image.rows, 480);
    EXPECT_EQ(current_frame->left_image.cols, 756);
    EXPECT_EQ(current_frame->right_image.rows, 480);
    EXPECT_EQ(current_frame->right_image.cols, 756);
}

TEST(KittiApiTest, ReadParam)
{
    auto dataset = KITTI::Load("/Users/iceytan/Downloads/00");

    auto current_frame = dataset->NextFrame();
    auto next_frame    = dataset->NextFrame();

    EXPECT_EQ(current_frame->time_stamp,0);
    EXPECT_GT(next_frame->time_stamp,0);
    EXPECT_EQ(next_frame->left_image.rows, int(376*dataset->GetResizeScale()));
}

int main(int argc, char* argv[])
{
//    ::testing::InitGoogleTest(&argc, argv);
//    return RUN_ALL_TESTS();
    auto dataset = EUROC::Load("/Users/iceytan/Downloads/mav0");
    auto current_frame = dataset->NextFrame();
    cv::imshow("left_image", current_frame->left_image);
    cv::imshow("right_image", current_frame->right_image);
    cv::waitKey(0);
}