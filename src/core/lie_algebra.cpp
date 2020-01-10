#include "lie_algebra.h"

#define eps 1e-10

Eigen::Matrix4d SE3::exp(const Eigen::VectorXd& x)
{
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();

    Eigen::Vector3d v(x[0], x[1], x[2]);
    Eigen::Vector3d omega(x[3], x[4], x[5]);

    double theta = omega.norm();
    if (theta > eps)
    {
        Eigen::Matrix3d omega_skew;
        omega_skew <<        0, -omega[2],   omega[1],
                      omega[2],         0,  -omega[0],
                     -omega[1],  omega[0],          0;

        Eigen::Matrix3d square_omega_skew = omega_skew * omega_skew;

        // Rotation
        result.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity() + sin(theta) / theta * omega_skew +
                (1.0 - cos(theta)) / (theta * theta) * square_omega_skew;

        // Translation
        Eigen::Matrix3d V = Eigen::Matrix3d::Identity() + (1.0 - cos(theta)) / (theta * theta) * omega_skew +
                (theta - sin(theta)) / (theta * theta * theta) * (square_omega_skew);

        result.block<3, 1>(0, 3) = V * v;
    }
    else result.block<3, 1>(0, 3) = v;

    return result;
}

Eigen::VectorXd SE3::log(const Eigen::Matrix4d& M)
{
    Eigen::VectorXd result = Eigen::VectorXd::Zero(6);

    Eigen::Matrix3d R = M.topLeftCorner(3, 3);
    Eigen::Vector3d t = M.topRightCorner(3, 1);

    double d = 0.5 * (R.trace() - 1.0);
    Eigen::Matrix3d omega_skew = Eigen::Matrix3d::Zero();

    if ( d < 1 - eps)
    {
        double theta = acos(d);
        omega_skew = theta / ( 2 * sin(theta) ) * ( R - R.transpose());

        result[3] = omega_skew(2, 1);
        result[4] = omega_skew(0, 2);
        result[5] = omega_skew(1, 0);
    }

    double theta = result.tail(3).norm();

    result.head(3) = t;

    if (abs(theta) > eps)
    {
        Eigen::Matrix3d V = Eigen::Matrix3d::Identity() + (1.0 - cos(theta)) / (theta * theta) * omega_skew +
                            (theta - sin(theta)) / (theta * theta * theta) * (omega_skew * omega_skew);

        result.head(3) = V.inverse() * t;
        /*
        double half_theta = 0.5 * theta;

        double alpha = -0.5;
        double beta = 1 / (theta * theta) * (1 - theta * std::cos(half_theta) / (2 * std::sin(half_theta)));
        Eigen::Matrix3d V_inv = Eigen::Matrix3d::Identity() + alpha * omega_skew + beta * (omega_skew * omega_skew);

        result.head(3) = V_inv * t;
        */
    }

    return result;
}

Eigen::Vector3d SO3::log(const Eigen::Matrix3d& M)
{
    Eigen::Vector3d result = Eigen::VectorXd::Zero(3);

    double d = 0.5 * (M.trace() - 1.0);
    Eigen::Matrix3d omega_skew = Eigen::Matrix3d::Zero();

    if ( d < 1 - eps)
    {
        double theta = acos(d);
        omega_skew = theta / ( 2 * sin(theta) ) * ( M - M.transpose());

        result[0] = omega_skew(2, 1);
        result[1] = omega_skew(0, 2);
        result[2] = omega_skew(1, 0);
    }

    return result;
}

Eigen::Matrix3d SO3::exp(const Eigen::Vector3d &x)
{
    Eigen::Matrix3d result = Eigen::Matrix3d::Identity();

    double theta = x.norm(); // l2 norm

    if (theta > eps)
    {
        Eigen::Matrix3d omega_skew;
        omega_skew <<    0, -x[2],  x[1],
                      x[2],    0,  -x[0],
                     -x[1],  x[0],     0;

        result = Eigen::Matrix3d::Identity() + sin(theta) / theta * omega_skew +
                 (1 - cos(theta)) / (theta * theta) * (omega_skew * omega_skew);
    }

    return result;
}
