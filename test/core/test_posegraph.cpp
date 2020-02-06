#include <gtest/gtest.h>

#include "unsupported/Eigen/EulerAngles"
#include "core/pose_graph.h"

/**
 * @brief ref from https://github.com/jbehley/SuMa/blob/master/test/core/PosegraphTest.cpp
 */

SE3 pose(double yaw, double pitch, double roll, double x, double y, double z)
{
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    std::cout << q.x() <<" "<< q.y() <<" "<< q.z() <<" "<< q.w()  << std::endl;
    return SE3(SO3(),Vec3(x, y, z));
}

double normalize_angle(double angle)
{
    double a = angle;
    while (a < 0) a += 2 * M_PI;
    while (a > 2 * M_PI) a -= 2 * M_PI;
    return a;
}

#define ASSERT_POSE_EQ(YAW, PITCH, ROLL, X, Y, Z, POSE)                     \
  do {                                                                      \
    auto ypr = POSE.rotationMatrix().eulerAngles(2,1,0);                    \
    ASSERT_NEAR(normalize_angle(YAW), normalize_angle(ypr[0]), 0.0001);     \
    ASSERT_NEAR(normalize_angle(PITCH), normalize_angle(ypr[1]), 0.0001);   \
    ASSERT_NEAR(normalize_angle(ROLL), normalize_angle(ypr[2]), 0.0001);    \
    ASSERT_NEAR(X, POSE.translation().x(), 0.0001);                         \
    ASSERT_NEAR(Y, POSE.translation().y(), 0.0001);                         \
    ASSERT_NEAR(Z, POSE.translation().z(), 0.0001);                         \
  } while (0)


TEST(PosegraphTest, testOptimize)
{
    std::cout << pose(0,-M_PI_2, M_PI_2,0,0,0).matrix()  << std::endl;

    core::PoseGraph p_g2o;

    Mat66 info_ = 10 * Mat66::Identity();

    // initial values.
    p_g2o.SetInitial(0, pose(0, 0, 0, 0, 0, 0));

    p_g2o.SetInitial(1, pose(0, 0, 0, 2.1, 0.1, 0));
    p_g2o.SetInitial(2, pose(0, 0, 0, 3.9, 0.1, 0));
    p_g2o.SetInitial(3, pose(M_PI_2, 0, 0, 6.1, -0.1, 0));

    p_g2o.SetInitial(4, pose(M_PI_2, 0, 0, 6.1, 2.1, 0));
    p_g2o.SetInitial(5, pose(M_PI_2, 0, 0, 6.1, 3.9, 0));
    p_g2o.SetInitial(6, pose(M_PI, 0, 0, 6.1, 6.1, 0));

    p_g2o.SetInitial(7, pose(M_PI, 0, 0, 4.0, 6.1, 0));
    p_g2o.SetInitial(8, pose(M_PI, 0, 0, 2.0, 6.1, 0));
    p_g2o.SetInitial(9, pose(3*M_PI_2, 0, 0, 0.0, 6.1, 0));

    p_g2o.SetInitial(10, pose(3 * M_PI_2, 0, 0, 0.0, 3.95, 0));
    p_g2o.SetInitial(11, pose(3 * M_PI_2, 0, 0, 0.0, 2.0, 0));
    p_g2o.SetInitial(12, pose(3 * M_PI_2, 0, 0, 0.0, 0.0, 0));

    // odometry edges
    p_g2o.AddEdge(0, 1, pose(0, 0, 0, 2., 0, 0), info_);
    p_g2o.AddEdge(1, 2, pose(0, 0, 0, 2., 0, 0), info_);
    p_g2o.AddEdge(2, 3, pose(M_PI_2, 0, 0, 2., 0, 0), info_);

    p_g2o.AddEdge(3, 4, pose(0, 0, 0, 2., 0, 0), info_);
    p_g2o.AddEdge(4, 5, pose(0, 0, 0, 2., 0, 0), info_);
    p_g2o.AddEdge(5, 6, pose(M_PI_2, 0, 0, 2., 0, 0), info_);

    p_g2o.AddEdge(6, 7, pose(0, 0, 0, 2., 0, 0), info_);
    p_g2o.AddEdge(7, 8, pose(0, 0, 0, 2., 0, 0), info_);
    p_g2o.AddEdge(8, 9, pose(M_PI_2, 0, 0, 2., 0, 0), info_);

    p_g2o.AddEdge(9, 10, pose(0, 0, 0, 2., 0, 0), info_);
    p_g2o.AddEdge(10, 11, pose(0, 0, 0, 2., 0, 0), info_);
    p_g2o.AddEdge(11, 12, pose(0, 0, 0, 2., 0, 0), info_);

    // loop closure edges.
    p_g2o.AddEdge(11, 0, pose(M_PI_2, 0, 0, 2., 0, 0), info_);
    p_g2o.AddEdge(12, 0, pose(M_PI_2, 0, 0, 0, 0, 0), info_);
    p_g2o.AddEdge(11, 1, pose(M_PI_2, 0, 0, 2., 2, 0), info_);
    p_g2o.AddEdge(12, 1, pose(M_PI_2, 0, 0, 0, 2, 0), info_);

    p_g2o.Optimize(5);

    const auto & result = p_g2o.GetPoses();

    ASSERT_POSE_EQ(0, 0, 0, 0, 0, 0, result[0]);

    ASSERT_POSE_EQ(0, 0, 0, 2, 0, 0, result[1]);
    ASSERT_POSE_EQ(0, 0, 0, 4, 0, 0, result[2]);
    ASSERT_POSE_EQ(M_PI_2, 0, 0, 6, 0, 0, result[3]);

    ASSERT_POSE_EQ(M_PI_2, 0, 0, 6, 2, 0, result[4]);
    ASSERT_POSE_EQ(M_PI_2, 0, 0, 6, 4, 0, result[5]);
    ASSERT_POSE_EQ(M_PI, 0, 0, 6, 6, 0, result[6]);

    ASSERT_POSE_EQ(M_PI, 0, 0, 4, 6, 0, result[7]);
    ASSERT_POSE_EQ(M_PI, 0, 0, 2, 6, 0, result[8]);
    ASSERT_POSE_EQ(3 * M_PI_2, 0, 0, 0, 6, 0, result[9]);

    ASSERT_POSE_EQ(3 * M_PI_2, 0, 0, 0, 4, 0, result[10]);
    ASSERT_POSE_EQ(3 * M_PI_2, 0, 0, 0, 2, 0, result[11]);
    ASSERT_POSE_EQ(3 * M_PI_2, 0, 0, 0, 0, 0, result[12]);

}