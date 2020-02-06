#include <gtest/gtest.h>
#include "utils/config.h"

TEST(TestConfig, Read)
{
    bool is_succ = Config::LoadConfig("/Users/iceytan/CLionProjects/vo/test/utils/test_cofig.yaml");

    EXPECT_EQ(is_succ, true);
    EXPECT_EQ(Config::Get<int>("an_int"), 1);
    EXPECT_EQ(Config::Get<double>("a_double"), 123.456);
    EXPECT_EQ(Config::Get<std::string>("a_string"), "visual odometry");

    EXPECT_EQ(Config::Get<std::vector<int>>("an_int_array")[0], 1);
    EXPECT_EQ(Config::Get<std::vector<int>>("an_int_array")[1], 2);
    EXPECT_EQ(Config::Get<std::vector<int>>("an_int_array")[2], 3);
    EXPECT_EQ(Config::Get<std::vector<int>>("an_int_array")[3], 4);

    EXPECT_EQ(Config::Get<std::vector<double>>("a_double_array")[0], 1.2);
    EXPECT_EQ(Config::Get<std::vector<double>>("a_double_array")[1], 2.3);
    EXPECT_EQ(Config::Get<std::vector<double>>("a_double_array")[2], 3.4);
    EXPECT_EQ(Config::Get<std::vector<double>>("a_double_array")[3], 4.5);

    EXPECT_EQ(Config::Get<std::vector<std::string>>("a_string_array")[0], "visual");
    EXPECT_EQ(Config::Get<std::vector<std::string>>("a_string_array")[1], "odometry");

    auto test_mat44 = Config::Get<Eigen::Matrix4d>("an_eigen_matrix");
    for(int i = 0; i < 16; ++i) EXPECT_EQ(test_mat44(i/4,i%4), i+1.0);
}