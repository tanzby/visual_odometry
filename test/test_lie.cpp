#include <gtest/gtest.h>
#include <core/lie_algebra.h>

TEST(LieTest, testLogExpSE3)
{
    Eigen::VectorXd x(6);
    x << -0.7, 0.1, 0.2, 0.1, -0.5, 0.33;

    Eigen::VectorXd x_logexp = SE3::log(SE3::exp(x));

    for (uint32_t i = 0; i < 6; ++i) {
        ASSERT_NEAR(x[i], x_logexp[i], 0.0001);
    }
}

TEST(LieTest, testLogExpSO3)
{
    Eigen::Vector3d x;
    x << 0.8, -0.2, 0.56;

    Eigen::Vector3d x_logexp = SO3::log(SO3::exp(x));

    for (uint32_t i = 0; i < 3; ++i) {
        ASSERT_NEAR(x[i], x_logexp[i], 0.0001);
    }
}