#pragma once

#include <Eigen/Dense>

/**
 * \brief some utility functions for Lie groups.
 * adapted from https://github.com/jbehley/SuMa/blob/master/src/core/lie_algebra.h and
 * \em A tutorial on se (3) transformation parameterizations and on-manifold optimization
 */

struct SE3
{
    SE3() = delete;

    /** \brief get rotation matrix from angle-axis + translation **/
    static Eigen::Matrix4d exp(const Eigen::VectorXd& x);

    /** \brief get angle-axis + translation from rotation matrix **/
    static Eigen::VectorXd log(const Eigen::Matrix4d& M);
};

struct SO3
{
    SO3() = delete;

    /** \brief get rotation matrix from angle-axis **/
    static Eigen::Matrix3d exp(const Eigen::Vector3d& x);

    /** \brief get angle-axis from rotation matrix **/
    static Eigen::Vector3d log(const Eigen::Matrix3d& M);
};